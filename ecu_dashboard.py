"""
ECU Dashboard — Serial monitor for the ESP32 ECU shield.

Serial architecture:
  U0 (USB/Serial) → EXE debug terminal  (commands, data stream)
  U2 (Serial2)    → Pixhawk telemetry   (bridged as [Serial2] lines by ESP32)
  CAN / TWAI      → ESC via SN65HVD230  (raw frames streamed as CAN:RX lines)

Shield I/O covered here:
  4x MAX31855 thermocouples (SPI, CS via PCF8575)  → TC card
  5x MOSFET PWM outputs (GPIO 32/33/25/26/27)      → MOSFET card
  2x QNDB6 current sensors (GPIO 34/35)            → Current card
  Spare analog VP/VN (GPIO 36/39)                  → Analog readouts
  X9C digital pot (U/D=12, INC=13, CS on expander) → Digital Pot card
  Piezo/atomizer square wave (GPIO 15)             → Piezo card
  PCF8575 expander raw access                      → Commands tab

Build to EXE:
    pip install pyserial pyinstaller
    pyinstaller --onefile --windowed --name ECU_Dashboard ecu_dashboard.py
"""

import re
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import time

# ── Theme (Tokyo Night Dark) ──────────────────────────────────────────────────
BG       = "#13131a"
SURF     = "#1c1c28"
SURF2    = "#24283b"
BORDER   = "#2a2a3d"
TEXT     = "#c0caf5"
TEXT_DIM = "#565f89"
ACCENT   = "#7aa2f7"
GREEN    = "#9ece6a"
RED      = "#f7768e"
YELLOW   = "#e0af68"
CYAN     = "#7dcfff"
PURPLE   = "#bb9af7"
ORANGE   = "#ff9e64"

FONT_MONO    = ("Consolas", 10)
FONT_MONO_LG = ("Consolas", 13, "bold")
FONT_MONO_XL = ("Consolas", 16, "bold")
FONT_UI      = ("Segoe UI", 10)
FONT_UI_B    = ("Segoe UI", 10, "bold")
FONT_UI_SML  = ("Segoe UI", 9)
FONT_TITLE   = ("Segoe UI", 11, "bold")

BAUD         = 115200
HB_TIMEOUT_S = 3.5
LED_FLASH_MS = 180

# TC channel names (index 0-3 → TC1-TC4) — order matches shield CS1-CS4 wiring
TC_NAMES = ["TIT", "EGT", "Bearing Temp", "Coil Temp"]

# MOSFET channel names (index 0-4 → M1-M5), per verified wiring 2026-07-08:
# M1/GPIO32=Glow, M2/GPIO33=Sol 2, M3/GPIO25=Sol 1, M4/GPIO26=Pump 2, M5/GPIO27=Pump 1
MOSFET_NAMES = ["Glow Plug", "Solenoid 2", "Solenoid 1", "Fuel Pump 2", "Fuel Pump 1"]

# Hargrave microDRIVE error bitfield (esc.Status error_count, bits 0-12)
ESC_ERROR_BITS = ["OVER TEMP", "BUS OC", "PHASE OC", "OVER V", "UNDER V",
                  "RIPPLE", "SIG LOSS", "SATURATED", "MOTOR OT", "RPM LIM",
                  "ERROR", "SHORTED", "STARTUP FAIL"]

# ── LED indicator ─────────────────────────────────────────────────────────────
class LED(tk.Canvas):
    def __init__(self, parent, on_color: str, bg: str = BG, size: int = 14, **kw):
        super().__init__(parent, width=size, height=size,
                         highlightthickness=0, bg=bg, **kw)
        self._on  = on_color
        self._off = self._dim(on_color, 0.15)
        self._glo = self._dim(on_color, 0.35)
        s = size
        self._ring = self.create_oval(1, 1, s-1, s-1, fill=self._off, outline="")
        p = 3
        self._core = self.create_oval(p, p, s-p, s-p, fill=self._off, outline="")
        self._timer  = None
        self._steady = False

    def flash(self, ms: int = LED_FLASH_MS):
        self._light(True)
        if self._timer:
            self.after_cancel(self._timer)
        self._timer = self.after(ms, self._auto_off)

    def steady(self, on: bool):
        self._steady = on
        if self._timer:
            self.after_cancel(self._timer)
            self._timer = None
        self._light(on)

    def _auto_off(self):
        self._timer = None
        if not self._steady:
            self._light(False)

    def _light(self, on: bool):
        if on:
            self.itemconfig(self._ring, fill=self._glo)
            self.itemconfig(self._core, fill=self._on)
        else:
            self.itemconfig(self._ring, fill=self._off)
            self.itemconfig(self._core, fill=self._off)

    @staticmethod
    def _dim(h: str, f: float) -> str:
        return "#{:02x}{:02x}{:02x}".format(
            int(int(h[1:3],16)*f), int(int(h[3:5],16)*f), int(int(h[5:7],16)*f))


# ── Card helper ───────────────────────────────────────────────────────────────
def card(parent, title: str = "", **kw) -> tk.Frame:
    outer = tk.Frame(parent, bg=BORDER, **kw)
    if title:
        tk.Label(outer, text=f"  {title}  ", bg=SURF2, fg=ACCENT,
                 font=FONT_UI_B, anchor="w", padx=6, pady=4).pack(fill="x", side="top")
    inner = tk.Frame(outer, bg=SURF, padx=10, pady=8)
    inner.pack(fill="both", expand=True, padx=1, pady=(0, 1))
    return inner


# ── Serial worker ─────────────────────────────────────────────────────────────
class SerialWorker(threading.Thread):
    def __init__(self, port, baud, rx_q, tx_q):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.rx_q, self.tx_q = rx_q, tx_q
        self._stop = threading.Event()

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=0.05) as ser:
                self.rx_q.put(("status", f"Connected to {self.port}"))
                buf = b""
                while not self._stop.is_set():
                    while not self.tx_q.empty():
                        ser.write(self.tx_q.get_nowait())
                    chunk = ser.read(512)
                    if chunk:
                        buf += chunk
                        # Don't let binary junk grow the buffer forever if the
                        # stream never contains a newline.
                        if len(buf) > 4096:
                            buf = buf[-1024:]
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            # Keep printable ASCII only — MAVLink/noise bytes
                            # otherwise corrupt the Tk text widgets.
                            txt = "".join(
                                chr(b) if 32 <= b < 127 or b == 9 else "·"
                                for b in line).strip("· \t\r")
                            if txt:
                                self.rx_q.put(("line", txt))
        except serial.SerialException as e:
            self.rx_q.put(("error", str(e)))
        finally:
            self.rx_q.put(("status", "Disconnected"))

    def stop(self):
        self._stop.set()


# ── Quick-command catalogue ───────────────────────────────────────────────────
COMMANDS = [
    ("THERMOCOUPLE", [
        ("TIT Read",          "tc read 1"),
        ("EGT Read",          "tc read 2"),
        ("Bearing Temp Read", "tc read 3"),
        ("Coil Temp Read",    "tc read 4"),
        ("Read All TCs",      "tc read all"),
        ("TC Scan (debug)",   "tc scan"),
        ("Mode MAX31855",     "tc mode 31855"),
        ("Mode MAX6675",      "tc mode 6675"),
    ]),
    ("STREAM", [
        ("Stream ON",  "stream on"),
        ("Stream OFF", "stream off"),
        ("Rate 1 Hz",  "stream rate 1"),
        ("Rate 5 Hz",  "stream rate 5"),
        ("Rate 10 Hz", "stream rate 10"),
        ("Rate 20 Hz", "stream rate 20"),
        ("Rate 50 Hz", "stream rate 50"),
    ]),
    ("CURRENT SENSORS", [
        ("Read All",       "current all"),
        ("Battery (34)",   "current a"),
        ("Load (35)",      "current b"),
    ]),
    ("DIGITAL POT", [
        ("Position",  "pot pos"),
        ("Up 1",      "pot up 1"),
        ("Down 1",    "pot down 1"),
        ("Up 10",     "pot up 10"),
        ("Down 10",   "pot down 10"),
        ("Reset (0)", "pot reset"),
        ("Store NVM", "pot store"),
    ]),
    ("MOSFETS", [
        ("All ON",           "mosfet all on"),  ("All OFF",          "mosfet all off"),
        ("Glow Plug ON",     "mosfet 1 on"),    ("Glow Plug OFF",    "mosfet 1 off"),
        ("Solenoid 2 ON",    "mosfet 2 on"),    ("Solenoid 2 OFF",   "mosfet 2 off"),
        ("Solenoid 1 ON",    "mosfet 3 on"),    ("Solenoid 1 OFF",   "mosfet 3 off"),
        ("Fuel Pump 2 ON",   "mosfet 4 on"),    ("Fuel Pump 2 OFF",  "mosfet 4 off"),
        ("Fuel Pump 1 ON",   "mosfet 5 on"),    ("Fuel Pump 1 OFF",  "mosfet 5 off"),
    ]),
    ("ANALOG / ADC", [
        ("Analog All", "analog all"),
        ("VP",         "analog vp"),
        ("VN",         "analog vn"),
        ("A",          "analog a"),
        ("B",          "analog b"),
    ]),
    ("PIXHAWK (U2)", [
        ("Serial2 Status", "serial2 send STATUS"),
        ("Serial2 Test",   "serial2 send PING"),
        ("Baud 57600",     "serial2 baud 57600"),
        ("Baud 115200",    "serial2 baud 115200"),
        ("Hex Dump ON",    "serial2 hex on"),
        ("Hex Dump OFF",   "serial2 hex off"),
    ]),
    ("PIEZO / ATOMIZER", [
        ("Piezo ON",  "piezo on"),
        ("Piezo OFF", "piezo off"),
    ]),
    ("CAN / ESC (DroneCAN)", [
        ("ESC Telemetry", "can esc"),
        ("CAN Status",    "can status"),
        ("ESC STOP",      "can stop"),
        ("RX Print ON",   "can print on"),
        ("RX Print OFF",  "can print off"),
        ("Baud 1M (std)", "can baud 1000"),
        ("Baud 500k",     "can baud 500"),
        ("Baud 250k",     "can baud 250"),
    ]),
    ("SYSTEM", [
        ("Status",    "status"),
        ("Help",      "help"),
        ("I2C Scan",  "i2c scan"),
    ]),
]


# ── Main application ──────────────────────────────────────────────────────────
class ECUDashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ECU Dashboard")
        self.configure(bg=BG)
        self.minsize(980, 720)

        self._worker:    SerialWorker | None = None
        self._rx_q:      queue.Queue = queue.Queue()
        self._tx_q:      queue.Queue = queue.Queue()
        self._streaming  = False
        self._last_hb    = 0.0
        self._hb_alive   = False

        self._tc_vals    = ["---"] * 4
        self._curr_vals  = ["---"] * 2
        self._mosfet_duty= [0] * 5
        self._m_inhibit  = [False] * 5
        # anti-bounce: no stream sync while dragging + grace period after
        self._m_active   = [False] * 5
        self._m_guard    = [0.0] * 5
        self._pot_pos    = 0
        self._pot_inhibit= False
        self._pot_active = False
        self._pot_guard  = 0.0
        self._can_rx     = 0
        self._can_last_rx= 0

        self._history:   list[str] = []
        self._hist_idx   = -1

        self._configure_ttk()
        self._build_header()
        self._build_notebook()
        self._refresh_ports()
        self._poll()
        self._check_heartbeat()

    # ── TTK styling ──────────────────────────────────────────────────────────
    def _configure_ttk(self):
        s = ttk.Style(self)
        s.theme_use("default")
        s.configure("TNotebook", background=SURF2, borderwidth=0, tabmargins=[0,0,0,0])
        s.configure("TNotebook.Tab", background=SURF2, foreground=TEXT_DIM,
                    padding=[20, 8], font=FONT_UI_B, borderwidth=0)
        s.map("TNotebook.Tab",
              background=[("selected", BG), ("active", SURF)],
              foreground=[("selected", ACCENT), ("active", TEXT)])
        s.configure("Vertical.TScrollbar",
                    background=SURF2, troughcolor=BG,
                    arrowcolor=TEXT_DIM, borderwidth=0)

    # ── Header ───────────────────────────────────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self, bg=SURF)
        hdr.pack(fill="x")
        tk.Frame(hdr, bg=ACCENT, height=2).pack(fill="x")

        inner = tk.Frame(hdr, bg=SURF, padx=10, pady=7)
        inner.pack(fill="x")

        tk.Label(inner, text="⚡  ECU Dashboard", bg=SURF, fg=TEXT,
                 font=FONT_TITLE).pack(side="left")

        # Serial path indicator
        serlbl = tk.Frame(inner, bg=SURF)
        serlbl.pack(side="left", padx=20)
        for txt, color in [("U0", CYAN), ("→ EXE", TEXT_DIM),
                           ("  |  U2", YELLOW), ("→ Pixhawk", TEXT_DIM),
                           ("  |  CAN", ORANGE), ("→ ESC", TEXT_DIM)]:
            tk.Label(serlbl, text=txt, bg=SURF, fg=color,
                     font=FONT_UI_SML).pack(side="left")

        # LED indicators
        leds = tk.Frame(inner, bg=SURF)
        leds.pack(side="left", padx=20)
        for label, color, attr in [("TX", YELLOW, "_led_tx"),
                                    ("RX", CYAN,   "_led_rx"),
                                    ("HB", GREEN,  "_led_hb")]:
            g = tk.Frame(leds, bg=SURF)
            g.pack(side="left", padx=8)
            tk.Label(g, text=label, bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML).pack(side="left", padx=(0,3))
            led = LED(g, color, bg=SURF, size=14)
            led.pack(side="left")
            setattr(self, attr, led)

        # Right: connection controls
        conn = tk.Frame(inner, bg=SURF)
        conn.pack(side="right")

        self._stream_btn = self._hbtn(conn, "Stream OFF", self._toggle_stream,
                                       fg=YELLOW, state="disabled")
        self._stream_btn.pack(side="right", padx=(4,0))

        self._conn_btn = self._hbtn(conn, "Connect", self._toggle_connect, fg=GREEN)
        self._conn_btn.pack(side="right", padx=4)

        self._status_lbl = tk.Label(conn, text="●  Disconnected",
                                     bg=SURF, fg=RED, font=FONT_UI_SML)
        self._status_lbl.pack(side="right", padx=8)

        tk.Label(conn, text="Port:", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI).pack(side="right")
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(conn, textvariable=self._port_var,
                                       width=9, font=FONT_MONO, state="readonly")
        self._port_cb.pack(side="right", padx=(4,2))
        self._hbtn(conn, "⟳", self._refresh_ports, fg=ACCENT,
                   width=3).pack(side="right")

    def _hbtn(self, parent, text, cmd, fg=TEXT, width=10, state="normal"):
        b = tk.Button(parent, text=text, command=cmd, width=width,
                      bg=SURF2, fg=fg, font=FONT_UI, relief="flat",
                      activebackground=BORDER, activeforeground=fg,
                      cursor="hand2", state=state, padx=6, pady=2)
        b.bind("<Enter>", lambda e: b.config(bg=BORDER))
        b.bind("<Leave>", lambda e: b.config(bg=SURF2))
        return b

    # ── Notebook ─────────────────────────────────────────────────────────────
    def _build_notebook(self):
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True)

        dash = tk.Frame(nb, bg=BG)
        nb.add(dash, text="  Dashboard  ")
        self._build_dashboard(dash)

        cmds = tk.Frame(nb, bg=BG)
        nb.add(cmds, text="  Commands  ")
        self._build_commands(cmds)

    # ── Dashboard tab ────────────────────────────────────────────────────────
    def _build_dashboard(self, parent):
        # Row 1: TC + MOSFETs + Current/Analog
        row1 = tk.Frame(parent, bg=BG)
        row1.pack(fill="x", padx=8, pady=(8, 4))
        self._build_tc_card(row1)
        self._build_mosfet_card(row1)
        self._build_current_card(row1)

        # Row 2: Digital pot + Piezo/Atomizer
        row2 = tk.Frame(parent, bg=BG)
        row2.pack(fill="x", padx=8, pady=(0, 4))
        self._build_pot_card(row2)
        self._build_piezo_card(row2)

        # Row 3: CAN / ESC banner
        self._build_can_card(parent)

        # Row 3: U0 terminal + U2 Pixhawk (side by side, fills remaining space)
        self._build_terminals(parent)

    # ── TC card ──────────────────────────────────────────────────────────────
    def _build_tc_card(self, parent):
        c = card(parent, "Thermocouples")
        c.master.pack(side="left", fill="both", expand=True, padx=(0,4))
        self._tc_labels: list[tk.Label] = []
        self._tc_name_labels: list[tk.Label] = []

        for i, name in enumerate(TC_NAMES):
            row = tk.Frame(c, bg=SURF)
            row.pack(fill="x", pady=3)

            # TC channel label (short: TC1 / TC2 …)
            tk.Label(row, text=f"TC{i+1}", bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML, width=3, anchor="w").pack(side="left")

            # Named label
            nml = tk.Label(row, text=name, bg=SURF, fg=TEXT_DIM,
                           font=FONT_UI_SML, width=12, anchor="w")
            nml.pack(side="left", padx=(2, 4))
            self._tc_name_labels.append(nml)

            val = tk.Label(row, text="  ---  ", bg=SURF2, fg=GREEN,
                           font=FONT_MONO_LG, width=8, anchor="e",
                           relief="flat", padx=6, pady=2)
            val.pack(side="left")
            tk.Label(row, text="°C", bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI).pack(side="left", padx=(3,0))
            self._tc_labels.append(val)

        # Expander / I2C health line (populated from EXP/IFAIL/IREC stream keys)
        self._exp_lbl = tk.Label(c, text="", bg=SURF, fg=TEXT_DIM,
                                 font=FONT_UI_SML, anchor="w")
        self._exp_lbl.pack(fill="x", pady=(4, 0))

    # ── MOSFET card ──────────────────────────────────────────────────────────
    def _build_mosfet_card(self, parent):
        c = card(parent, "MOSFETs  (duty 0–10)")
        c.master.pack(side="left", fill="both", expand=True, padx=4)
        self._m_sliders: list[tk.Scale]  = []
        self._m_vallbls: list[tk.Label]  = []
        self._m_btns:    list[tk.Button] = []

        for i in range(5):
            row = tk.Frame(c, bg=SURF)
            row.pack(fill="x", pady=2)
            tk.Label(row, text=f"M{i+1}", bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML, width=3, anchor="w").pack(side="left")
            tk.Label(row, text=MOSFET_NAMES[i], bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML, width=12, anchor="w").pack(side="left", padx=(1,3))
            vl = tk.Label(row, text=" 0", bg=SURF, fg=ACCENT,
                          font=FONT_MONO, width=2)
            vl.pack(side="left", padx=(0,0))
            sl = tk.Scale(row, from_=0, to=10, orient="horizontal", length=120,
                          bg=SURF, fg=TEXT_DIM, troughcolor=SURF2,
                          highlightthickness=0, sliderlength=14, showvalue=False,
                          command=lambda v, idx=i, lbl=vl: self._on_slider(idx, v, lbl))
            sl.bind("<ButtonPress-1>",
                    lambda e, idx=i: self._on_m_press(idx))
            sl.bind("<ButtonRelease-1>",
                    lambda e, idx=i: self._on_m_release(idx))
            sl.pack(side="left", padx=5)
            btn = tk.Button(row, text="OFF", width=5, bg=SURF2, fg=RED,
                            font=FONT_UI, relief="flat", cursor="hand2",
                            command=lambda idx=i: self._toggle_m(idx))
            btn.bind("<Enter>", lambda e, b=btn: b.config(bg=BORDER))
            btn.bind("<Leave>", lambda e, b=btn: b.config(bg=SURF2))
            btn.pack(side="left")
            self._m_sliders.append(sl)
            self._m_vallbls.append(vl)
            self._m_btns.append(btn)

    # ── Current sensor / analog card ──────────────────────────────────────────
    def _build_current_card(self, parent):
        c = card(parent, "Current / Analog")
        c.master.pack(side="left", fill="both", expand=True, padx=(4,0))
        self._curr_labels: list[tk.Label] = []
        for name in ("Battery (34)", "Load (35)"):
            row = tk.Frame(c, bg=SURF)
            row.pack(fill="x", pady=3)
            tk.Label(row, text=name, bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML, width=11, anchor="w").pack(side="left")
            val = tk.Label(row, text="  ---  ", bg=SURF2, fg=YELLOW,
                           font=FONT_MONO_LG, width=8, anchor="e",
                           relief="flat", padx=6, pady=2)
            val.pack(side="left")
            tk.Label(row, text="A", bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI).pack(side="left", padx=(3,0))
            self._curr_labels.append(val)

        self._adc_labels: list[tk.Label] = []
        for name in ("VP (36)", "VN (39)"):
            row = tk.Frame(c, bg=SURF)
            row.pack(fill="x", pady=3)
            tk.Label(row, text=name, bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML, width=11, anchor="w").pack(side="left")
            val = tk.Label(row, text="  ---  ", bg=SURF2, fg=CYAN,
                           font=FONT_MONO_LG, width=8, anchor="e",
                           relief="flat", padx=6, pady=2)
            val.pack(side="left")
            tk.Label(row, text="V", bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI).pack(side="left", padx=(3,0))
            self._adc_labels.append(val)

    # ── Digital pot card ──────────────────────────────────────────────────────
    def _build_pot_card(self, parent):
        c = card(parent, "Digital Pot  (X9C, 0–99)")
        c.master.pack(side="left", fill="both", expand=True, padx=(0,4))

        row = tk.Frame(c, bg=SURF)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Wiper", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML, width=6, anchor="w").pack(side="left")
        self._pot_val = tk.Label(row, text="  0", bg=SURF2, fg=PURPLE,
                                 font=FONT_MONO_LG, width=5, anchor="e",
                                 relief="flat", padx=6, pady=2)
        self._pot_val.pack(side="left")

        self._pot_slider = tk.Scale(
            row, from_=0, to=99, orient="horizontal", length=160,
            bg=SURF, fg=TEXT_DIM, troughcolor=SURF2,
            highlightthickness=0, sliderlength=14, showvalue=False,
            command=self._on_pot_slider)
        self._pot_slider.pack(side="left", padx=8)
        self._pot_slider.bind("<ButtonPress-1>", self._on_pot_press)
        self._pot_slider.bind("<ButtonRelease-1>", self._on_pot_release)

        btns = tk.Frame(c, bg=SURF)
        btns.pack(fill="x", pady=(4, 0))
        for label, cmd in [("−10", "pot down 10"), ("−1", "pot down 1"),
                           ("+1", "pot up 1"),     ("+10", "pot up 10"),
                           ("Reset", "pot reset"), ("Store", "pot store")]:
            b = tk.Button(btns, text=label, width=5, bg=SURF2, fg=PURPLE,
                          font=FONT_UI, relief="flat", cursor="hand2",
                          command=lambda x=cmd: self._quick_cmd(x))
            b.bind("<Enter>", lambda e, bb=b: bb.config(bg=BORDER))
            b.bind("<Leave>", lambda e, bb=b: bb.config(bg=SURF2))
            b.pack(side="left", padx=2)

    def _on_pot_slider(self, value):
        self._pot_val.config(text=f"{int(float(value)):3d}")

    def _on_pot_press(self, _=None):
        self._pot_active = True

    def _on_pot_release(self, _=None):
        self._pot_active = False
        self._pot_guard = time.time() + 1.5
        if not self._pot_inhibit:
            self._quick_cmd(f"pot set {int(self._pot_slider.get())}")

    def _sync_pot(self, pos):
        if self._pot_active or time.time() < self._pot_guard:
            return
        self._pot_inhibit = True
        self._pot_pos = pos
        self._pot_slider.set(pos)
        self._pot_val.config(text=f"{pos:3d}")
        self._pot_inhibit = False

    # ── Piezo / atomizer card ─────────────────────────────────────────────────
    def _build_piezo_card(self, parent):
        c = card(parent, "Piezo / Atomizer  (GPIO15 square wave)")
        c.master.pack(side="left", fill="both", expand=True, padx=(4,0))

        row = tk.Frame(c, bg=SURF)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Freq", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML, width=5, anchor="w").pack(side="left")
        self._pz_freq = tk.Label(row, text="  ---  ", bg=SURF2, fg=ORANGE,
                                 font=FONT_MONO_LG, width=8, anchor="e",
                                 relief="flat", padx=6, pady=2)
        self._pz_freq.pack(side="left")
        tk.Label(row, text="Hz", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI).pack(side="left", padx=(3, 10))
        tk.Label(row, text="Duty", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML, width=5, anchor="w").pack(side="left")
        self._pz_duty = tk.Label(row, text=" --- ", bg=SURF2, fg=ORANGE,
                                 font=FONT_MONO_LG, width=5, anchor="e",
                                 relief="flat", padx=6, pady=2)
        self._pz_duty.pack(side="left")

        ctl = tk.Frame(c, bg=SURF)
        ctl.pack(fill="x", pady=(4, 0))
        for label, cmd, color in [("ON", "piezo on", GREEN),
                                  ("OFF", "piezo off", RED)]:
            b = tk.Button(ctl, text=label, width=5, bg=SURF2, fg=color,
                          font=FONT_UI, relief="flat", cursor="hand2",
                          command=lambda x=cmd: self._quick_cmd(x))
            b.bind("<Enter>", lambda e, bb=b: bb.config(bg=BORDER))
            b.bind("<Leave>", lambda e, bb=b: bb.config(bg=SURF2))
            b.pack(side="left", padx=2)

        self._pz_freq_var = tk.StringVar()
        e = tk.Entry(ctl, textvariable=self._pz_freq_var, bg="#0d0d14",
                     fg=TEXT, font=FONT_MONO, width=7, relief="flat",
                     insertbackground=TEXT, highlightthickness=1,
                     highlightbackground=BORDER, highlightcolor=ACCENT)
        e.pack(side="left", padx=(10, 2), ipady=2)
        e.bind("<Return>", lambda ev: self._send_pz_freq())
        b = tk.Button(ctl, text="Set Hz", bg=SURF2, fg=ORANGE, font=FONT_UI,
                      relief="flat", cursor="hand2", command=self._send_pz_freq)
        b.pack(side="left", padx=2)

        self._pz_duty_slider = tk.Scale(
            ctl, from_=0, to=255, orient="horizontal", length=110,
            bg=SURF, fg=TEXT_DIM, troughcolor=SURF2,
            highlightthickness=0, sliderlength=14, showvalue=False)
        self._pz_duty_slider.pack(side="left", padx=(10, 2))
        self._pz_duty_slider.bind(
            "<ButtonRelease-1>",
            lambda ev: self._quick_cmd(f"piezo duty {int(self._pz_duty_slider.get())}"))

    def _send_pz_freq(self):
        v = self._pz_freq_var.get().strip()
        if v.isdigit():
            self._quick_cmd(f"piezo freq {v}")

    # ── CAN / ESC banner ─────────────────────────────────────────────────────
    def _build_can_card(self, parent):
        c = card(parent, "ESC  —  microDRIVE LPi via DroneCAN  (1 Mbps, TX=23 RX=4)")
        c.master.pack(fill="x", padx=8, pady=(0, 4))

        inner = tk.Frame(c, bg=SURF)
        inner.pack(fill="x")

        self._can_fields: dict = {}
        specs = [
            ("RPM",       "---",     "",     CYAN),
            ("Voltage",   "---",     " V",   GREEN),
            ("Current",   "---",     " A",   YELLOW),
            ("ESC Temp",  "---",     " °C",  ORANGE),
            ("Power",     "---",     " %",   PURPLE),
            ("RX Frames", "0",       "",     TEXT_DIM),
            ("Bus",       "WAITING", "",     RED),
        ]
        for col, (label, default, unit, color) in enumerate(specs):
            inner.columnconfigure(col, weight=1)
            grp = tk.Frame(inner, bg=SURF2, padx=10, pady=8)
            grp.grid(row=0, column=col, padx=3, pady=0, sticky="nsew")
            tk.Label(grp, text=label.upper(), bg=SURF2, fg=TEXT_DIM,
                     font=FONT_UI_SML).pack(anchor="w")
            val = tk.Label(grp, text=default + unit, bg=SURF2, fg=color,
                           font=FONT_MONO_XL)
            val.pack(anchor="w", pady=(2, 0))
            unit_lbl = tk.Label(grp, text="waiting for CAN data…",
                                bg=SURF2, fg=TEXT_DIM, font=("Segoe UI", 8))
            unit_lbl.pack(anchor="w")
            self._can_fields[label] = (val, unit, color, unit_lbl)

        # ── ESC command strip: duty / rpm / brake + STOP ──
        ctl = tk.Frame(c, bg=SURF)
        ctl.pack(fill="x", pady=(8, 0))

        self._esc_cmd_lbl = tk.Label(ctl, text="CMD: OFF", bg=SURF2, fg=TEXT_DIM,
                                     font=FONT_MONO_LG, width=14, padx=6, pady=2)
        self._esc_cmd_lbl.pack(side="left", padx=(0, 12))

        def _mk_cmd(label, unit, build, color):
            tk.Label(ctl, text=label, bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML).pack(side="left", padx=(6, 2))
            var = tk.StringVar()
            e = tk.Entry(ctl, textvariable=var, bg="#0d0d14", fg=TEXT,
                         font=FONT_MONO, width=7, relief="flat",
                         insertbackground=TEXT, highlightthickness=1,
                         highlightbackground=BORDER, highlightcolor=ACCENT)
            e.pack(side="left", ipady=2)
            def send(_=None):
                v = var.get().strip()
                if v.lstrip("-").isdigit():
                    self._quick_cmd(build(v))
            e.bind("<Return>", send)
            b = tk.Button(ctl, text="Set", bg=SURF2, fg=color, font=FONT_UI,
                          relief="flat", cursor="hand2", padx=6, command=send)
            b.pack(side="left", padx=(2, 0))
            tk.Label(ctl, text=unit, bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML).pack(side="left")

        _mk_cmd("Duty",  "%",   lambda v: f"can duty {v}",  CYAN)
        _mk_cmd("RPM",   "",    lambda v: f"can rpm {v}",   GREEN)
        _mk_cmd("Brake", "%",   lambda v: f"can brake {v}", ORANGE)

        stop = tk.Button(ctl, text="■ STOP", bg="#3d1420", fg=RED,
                         font=FONT_UI_B, relief="flat", cursor="hand2",
                         padx=14, pady=2,
                         command=lambda: self._quick_cmd("can stop"))
        stop.bind("<Enter>", lambda e: stop.config(bg="#58182c"))
        stop.bind("<Leave>", lambda e: stop.config(bg="#3d1420"))
        stop.pack(side="right", padx=(12, 0))

        # ESC error flags + last raw frame lines
        self._esc_flags_lbl = tk.Label(c, text="ESC flags: —", bg=SURF,
                                       fg=TEXT_DIM, font=FONT_MONO, anchor="w")
        self._esc_flags_lbl.pack(fill="x", pady=(6, 0))
        self._can_frame_lbl = tk.Label(c, text="last frame: —", bg=SURF,
                                       fg=TEXT_DIM, font=FONT_MONO, anchor="w")
        self._can_frame_lbl.pack(fill="x")

    # ── Split terminals (U0 + U2 Pixhawk) ────────────────────────────────────
    def _build_terminals(self, parent):
        row = tk.Frame(parent, bg=BG)
        row.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # ── U0 Debug terminal (left, 65%) ──
        u0_outer = card(row, "U0  —  Debug Serial  (EXE ↔ ESP32)")
        u0_outer.master.pack(side="left", fill="both", expand=True, padx=(0, 4))

        self._terminal = scrolledtext.ScrolledText(
            u0_outer, bg="#0d0d14", fg=TEXT, font=FONT_MONO, height=10,
            state="disabled", wrap="word", relief="flat",
            insertbackground=TEXT, padx=4, pady=4)
        self._terminal.pack(fill="both", expand=True)
        self._terminal.tag_config("data",  foreground=TEXT_DIM)
        self._terminal.tag_config("error", foreground=RED)
        self._terminal.tag_config("sent",  foreground=YELLOW)

        inp = tk.Frame(u0_outer, bg=SURF)
        inp.pack(fill="x", pady=(5, 0))
        tk.Label(inp, text="›", bg=SURF, fg=ACCENT,
                 font=("Consolas", 13)).pack(side="left", padx=(0, 3))
        self._cmd_var = tk.StringVar()
        entry = tk.Entry(inp, textvariable=self._cmd_var, bg="#0d0d14",
                         fg=TEXT, font=FONT_MONO, insertbackground=TEXT,
                         relief="flat", bd=2, highlightthickness=1,
                         highlightbackground=BORDER, highlightcolor=ACCENT)
        entry.pack(side="left", fill="x", expand=True, ipady=3)
        entry.bind("<Return>", self._send_cmd)
        entry.bind("<Up>",     self._hist_up)
        entry.bind("<Down>",   self._hist_down)
        for label, cmd in [("Send", self._send_cmd), ("Clear", self._clear_u0)]:
            b = tk.Button(inp, text=label, command=cmd, bg=SURF2, fg=ACCENT,
                          font=FONT_UI, relief="flat", cursor="hand2",
                          padx=8, pady=2)
            b.bind("<Enter>", lambda e, btn=b: btn.config(bg=BORDER))
            b.bind("<Leave>", lambda e, btn=b: btn.config(bg=SURF2))
            b.pack(side="left", padx=(5, 0))

        # ── U2 Pixhawk feed (right, 35%) ──
        u2_outer = card(row, "U2  —  Pixhawk Telemetry")
        u2_outer.master.pack(side="left", fill="both", expand=False,
                              ipadx=0, padx=(4, 0))
        u2_outer.master.config(width=320)

        self._pix_term = scrolledtext.ScrolledText(
            u2_outer, bg="#0d1017", fg=CYAN, font=FONT_MONO, height=10,
            state="disabled", wrap="word", relief="flat",
            insertbackground=CYAN, padx=4, pady=4)
        self._pix_term.pack(fill="both", expand=True)
        self._pix_term.tag_config("dim", foreground=TEXT_DIM)

        tk.Label(u2_outer, text="← MAVLink / serial data from Pixhawk",
                 bg=SURF, fg=TEXT_DIM, font=("Segoe UI", 8),
                 anchor="w").pack(fill="x", pady=(4, 0))

    # ── Commands tab ─────────────────────────────────────────────────────────
    def _build_commands(self, parent):
        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0)
        vsb = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        frame = tk.Frame(canvas, bg=BG)
        win = canvas.create_window((0, 0), window=frame, anchor="nw")
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(win, width=e.width))
        frame.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind_all("<MouseWheel>",
                        lambda e: canvas.yview_scroll(-1*(e.delta//120), "units"))

        COLS = 5
        for section, commands in COMMANDS:
            hdr = tk.Frame(frame, bg=BG)
            hdr.pack(fill="x", padx=12, pady=(14, 4))
            tk.Label(hdr, text=section, bg=BG, fg=ACCENT,
                     font=FONT_UI_B).pack(side="left")
            tk.Frame(hdr, bg=BORDER, height=1).pack(
                side="left", fill="x", expand=True, padx=(8, 0), pady=6)

            grid = tk.Frame(frame, bg=BG)
            grid.pack(fill="x", padx=12, pady=(0, 4))
            for col in range(COLS):
                grid.columnconfigure(col, weight=1, minsize=130)

            for idx, (label, cmd) in enumerate(commands):
                r, col = divmod(idx, COLS)
                b = tk.Button(grid, text=label,
                              command=lambda x=cmd: self._quick_cmd(x),
                              bg=SURF, fg=TEXT, font=FONT_UI, relief="flat",
                              cursor="hand2", padx=8, pady=6,
                              activebackground=SURF2, activeforeground=ACCENT)
                b.bind("<Enter>", lambda e, btn=b: btn.config(bg=BORDER, fg=ACCENT))
                b.bind("<Leave>", lambda e, btn=b: btn.config(bg=SURF,   fg=TEXT))
                b.grid(row=r, column=col, padx=3, pady=3, sticky="ew")

        # Parameterised commands
        hdr2 = tk.Frame(frame, bg=BG)
        hdr2.pack(fill="x", padx=12, pady=(14, 4))
        tk.Label(hdr2, text="CUSTOM COMMANDS", bg=BG, fg=ACCENT,
                 font=FONT_UI_B).pack(side="left")
        tk.Frame(hdr2, bg=BORDER, height=1).pack(
            side="left", fill="x", expand=True, padx=(8, 0), pady=6)

        param = tk.Frame(frame, bg=BG)
        param.pack(fill="x", padx=12, pady=(0, 20))
        self._add_param(param, "MOSFET duty",   ["Target (1-5/all)", "Duty (0-10)"],
                         lambda v: f"mosfet {v[0]} duty {v[1]}")
        self._add_param(param, "MOSFET freq",   ["Frequency (Hz)"],
                         lambda v: f"mosfet freq {v[0]}")
        self._add_param(param, "Piezo duty",    ["Duty (0-255)"],
                         lambda v: f"piezo duty {v[0]}")
        self._add_param(param, "Piezo freq",    ["Frequency (Hz)"],
                         lambda v: f"piezo freq {v[0]}")
        self._add_param(param, "Pot set",       ["Position (0-99)"],
                         lambda v: f"pot set {v[0]}")
        self._add_param(param, "Current cal",   ["Zero (V)", "Amps per Volt"],
                         lambda v: f"current cal {v[0]} {v[1]}")
        self._add_param(param, "ESC duty",      ["Percent (-100..100)"],
                         lambda v: f"can duty {v[0]}")
        self._add_param(param, "ESC RPM",       ["RPM setpoint"],
                         lambda v: f"can rpm {v[0]}")
        self._add_param(param, "ESC brake",     ["Percent (0-100)"],
                         lambda v: f"can brake {v[0]}")
        self._add_param(param, "CAN send",      ["ID (hex)", "Data bytes (hex, spaced)"],
                         lambda v: f"can send {v[0]} {v[1]}")
        self._add_param(param, "Expander bit",  ["Bit (0-15)", "on / off"],
                         lambda v: f"i2c expander bit {v[0]} {v[1]}")
        self._add_param(param, "Serial2 send",  ["Message"],
                         lambda v: f"serial2 send {v[0]}")
        self._add_param(param, "Serial2 baud",  ["Baud rate"],
                         lambda v: f"serial2 baud {v[0]}")
        self._add_param(param, "Stream rate",   ["Rate (1-50 Hz)"],
                         lambda v: f"stream rate {v[0]}")

    def _add_param(self, parent, label, placeholders, build):
        row = tk.Frame(parent, bg=BG)
        row.pack(fill="x", pady=3)
        tk.Label(row, text=label, bg=BG, fg=TEXT_DIM,
                 font=FONT_UI, width=16, anchor="w").pack(side="left")
        entries = []
        for ph in placeholders:
            var = tk.StringVar()
            e = tk.Entry(row, textvariable=var, bg=SURF, fg=TEXT_DIM,
                         font=FONT_MONO, relief="flat", bd=1,
                         highlightthickness=1, highlightbackground=BORDER,
                         highlightcolor=ACCENT, width=18, insertbackground=TEXT)
            e.insert(0, ph)
            e.bind("<FocusIn>",  lambda ev, en=e, p=ph: (
                en.delete(0,"end") if en.get()==p else None, en.config(fg=TEXT)))
            e.bind("<FocusOut>", lambda ev, en=e, p=ph: (
                (en.insert(0, p) if not en.get() else None),
                en.config(fg=TEXT_DIM if en.get()==p else TEXT)))
            e.pack(side="left", padx=(0, 6), ipady=3)
            entries.append(var)
        b = tk.Button(row, text="Send",
                      command=lambda: self._quick_cmd(build([v.get() for v in entries])),
                      bg=SURF2, fg=ACCENT, font=FONT_UI, relief="flat",
                      cursor="hand2", padx=10, pady=2)
        b.bind("<Enter>", lambda e: b.config(bg=BORDER))
        b.bind("<Leave>", lambda e: b.config(bg=SURF2))
        b.pack(side="left")

    # ── Connection ───────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _toggle_connect(self):
        if self._worker and self._worker.is_alive():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._port_var.get()
        if not port:
            messagebox.showwarning("No Port", "Select a COM port first.")
            return
        self._rx_q = queue.Queue()
        self._tx_q = queue.Queue()
        self._worker = SerialWorker(port, BAUD, self._rx_q, self._tx_q)
        self._worker.start()
        self._conn_btn.config(text="Disconnect", fg=RED)
        self._stream_btn.config(state="normal")
        # Opening the port resets the ESP32 (~1-2s boot). New firmware streams
        # by default; for older firmware, nudge it on if no data has arrived.
        self.after(3000, self._auto_stream_check)

    def _auto_stream_check(self):
        if self._worker and self._worker.is_alive() and not self._streaming:
            self._send_raw(b"stream on\n")

    def _disconnect(self):
        if self._streaming:
            self._send_raw(b"stream off\n")
            self._streaming = False
        if self._worker:
            self._worker.stop()
            self._worker = None
        self._conn_btn.config(text="Connect", fg=GREEN)
        self._stream_btn.config(state="disabled", text="Stream OFF", fg=YELLOW)
        self._streaming = False
        self._led_hb.steady(False)
        self._set_status("Disconnected", ok=False)

    def _toggle_stream(self):
        if not self._streaming:
            self._send_raw(b"stream on\n")
            self._streaming = True
            self._stream_btn.config(text="Stream ON", fg=GREEN)
        else:
            self._send_raw(b"stream off\n")
            self._streaming = False
            self._stream_btn.config(text="Stream OFF", fg=YELLOW)

    # ── Sending ──────────────────────────────────────────────────────────────
    def _send_raw(self, data: bytes):
        if self._worker and self._worker.is_alive():
            self._tx_q.put(data)
            self._led_tx.flash()

    def _send_cmd(self, _=None):
        cmd = self._cmd_var.get().strip()
        if not cmd:
            return
        if not (self._worker and self._worker.is_alive()):
            self._log_u0("Not connected.", "error")
            return
        self._send_raw((cmd + "\n").encode())
        self._log_u0(f"  › {cmd}", "sent")
        self._history.append(cmd)
        self._hist_idx = len(self._history)
        self._cmd_var.set("")

    def _quick_cmd(self, cmd: str):
        if not (self._worker and self._worker.is_alive()):
            self._log_u0(f"Not connected — can't send: {cmd}", "error")
            return
        self._send_raw((cmd + "\n").encode())
        self._log_u0(f"  › {cmd}", "sent")

    def _hist_up(self, _=None):
        if self._history:
            self._hist_idx = max(0, self._hist_idx - 1)
            self._cmd_var.set(self._history[self._hist_idx])

    def _hist_down(self, _=None):
        if self._history:
            self._hist_idx = min(len(self._history), self._hist_idx + 1)
            self._cmd_var.set(
                self._history[self._hist_idx]
                if self._hist_idx < len(self._history) else "")

    # ── MOSFET controls ──────────────────────────────────────────────────────
    def _on_m_press(self, idx):
        self._m_active[idx] = True

    def _on_m_release(self, idx):
        self._m_active[idx] = False
        self._m_guard[idx] = time.time() + 1.5
        if not self._m_inhibit[idx]:
            self._send_raw(f"mosfet {idx+1} duty {self._m_sliders[idx].get()}\n".encode())

    def _on_slider(self, idx, value, lbl):
        duty = int(float(value))
        lbl.config(text=f"{duty:2d}")
        self._mosfet_duty[idx] = duty
        self._m_btns[idx].config(text="ON" if duty else "OFF",
                                  fg=GREEN if duty else RED)
        if not self._m_inhibit[idx]:
            self._send_raw(f"mosfet {idx+1} duty {duty}\n".encode())

    def _toggle_m(self, idx):
        self._m_guard[idx] = time.time() + 1.5
        self._m_sliders[idx].set(0 if self._mosfet_duty[idx] > 0 else 10)

    def _sync_mosfet(self, idx, duty):
        # never fight the user's drag (or the stale lines right after it)
        if self._m_active[idx] or time.time() < self._m_guard[idx]:
            return
        self._m_inhibit[idx] = True
        self._mosfet_duty[idx] = duty
        self._m_sliders[idx].set(duty)
        self._m_vallbls[idx].config(text=f"{duty:2d}")
        self._m_btns[idx].config(text="ON" if duty else "OFF",
                                  fg=GREEN if duty else RED)
        self._m_inhibit[idx] = False

    # ── Incoming data ─────────────────────────────────────────────────────────
    def _poll(self):
        try:
            while True:
                kind, text = self._rx_q.get_nowait()
                if kind == "line":
                    self._led_rx.flash()
                    self._on_line(text)
                elif kind == "status":
                    self._set_status(text, ok="Connected" in text)
                elif kind == "error":
                    self._set_status(text, ok=False)
                    self._log_u0(f"Error: {text}", "error")
        except queue.Empty:
            pass
        self.after(40, self._poll)

    def _on_line(self, line: str):
        # Heartbeat - silent, just resets timer
        if line == "HB":
            self._last_hb = time.time()
            return

        # Pixhawk / U2 bridge
        if line.startswith("[Serial2]"):
            payload = line[9:].strip()
            self._log_pix(payload)
            return  # don't echo to U0 terminal

        # Data stream
        if line.startswith("DATA:"):
            self._last_hb = time.time()
            # Sync the stream toggle with reality (firmware streams at boot)
            if not self._streaming:
                self._streaming = True
                self._stream_btn.config(text="Stream ON", fg=GREEN)
            self._parse_data(line[5:])
            self._log_u0(line, "data")
            return

        # Raw CAN frames
        if line.startswith("CAN:RX"):
            self._can_rx += 1
            self._can_frame_lbl.config(text="last frame: " + line[7:].strip(),
                                       fg=CYAN)
            self._update_can_status()
            self._log_u0(line, "data")
            return

        # "tc read" replies also update the TC card, so manual reads show up
        # even with streaming off. Matches both firmware formats:
        #   "TC2 (EGT): 26.00 C"   /  "raw: 0x... TC2 (EGT): 26.00 C"
        #   "TC1 (TIT): FAULT/NC"  /  "TC1 (TIT): OPEN (chip alive, ...)"
        m = re.search(r"TC([1-4]) \([^)]*\):\s*(.+)$", line)
        if m:
            idx = int(m.group(1)) - 1
            val = m.group(2).strip()
            fm = re.match(r"(-?\d+\.?\d*)\s*C", val)
            if fm:
                self._tc_labels[idx].config(text=f"{float(fm.group(1)):>7.2f}",
                                            fg=GREEN)
            else:
                word = val.split()[0].split("/")[0]  # OPEN / FAULT / SHORT-GND …
                if word == "NO":
                    word = "NO MOD"
                self._tc_labels[idx].config(text=f"{word[:7]:^7}", fg=RED)
            self._log_u0(line)
            return

        # Pot position echo ("Pot position: 42/99")
        if line.startswith("Pot position:"):
            try:
                self._sync_pot(int(line.split(":")[1].split("/")[0]))
            except (ValueError, IndexError):
                pass
            self._log_u0(line)
            return

        self._log_u0(line)

    # DATA: TC1=25.50,…,I_VP=1.230,…,M1=0,…
    def _parse_data(self, payload: str):
        parts = {k.strip(): v.strip()
                 for p in payload.split(",")
                 if "=" in p
                 for k, v in [p.split("=", 1)]}

        TC_FAULTS = {"nan": " FAULT ", "nc": "NO MOD ", "open": " OPEN  ",
                     "sgnd": "SHT-GND", "svcc": "SHT-VCC"}
        for i in range(4):
            v = parts.get(f"TC{i+1}", "")
            if v in TC_FAULTS:
                self._tc_labels[i].config(text=TC_FAULTS[v], fg=RED)
            elif v:
                try:
                    self._tc_labels[i].config(text=f"{float(v):>7.2f}", fg=GREEN)
                except ValueError:
                    pass

        for i, key in enumerate(("I_A", "I_B")):
            v = parts.get(key, "")
            if v:
                try:
                    self._curr_labels[i].config(text=f"{float(v):>7.3f}")
                except ValueError:
                    pass

        for i, key in enumerate(("AVP", "AVN")):
            v = parts.get(key, "")
            if v:
                try:
                    self._adc_labels[i].config(text=f"{float(v):>7.3f}")
                except ValueError:
                    pass

        v = parts.get("POT", "")
        if v:
            try:
                self._sync_pot(int(v))
            except ValueError:
                pass

        exp = parts.get("EXP", "")
        if exp:
            fails = parts.get("IFAIL", "?")
            recs = parts.get("IREC", "?")
            if exp == "1":
                if recs not in ("0", "?"):
                    self._exp_lbl.config(
                        text=f"expander OK  ·  {fails} I2C errors, {recs} auto-recoveries",
                        fg=YELLOW)
                else:
                    self._exp_lbl.config(text="expander OK", fg=TEXT_DIM)
            else:
                self._exp_lbl.config(
                    text=f"⚠ EXPANDER OFFLINE — recovering… ({fails} errors)", fg=RED)

        v = parts.get("PZF", "")
        if v:
            self._pz_freq.config(text=f"{v:>7}")
        v = parts.get("PZD", "")
        if v:
            self._pz_duty.config(text=f"{v:>4}")

        v = parts.get("CANRX", "")
        if v:
            try:
                self._can_rx = int(v)
                self._update_can_status()
            except ValueError:
                pass

        self._parse_esc(parts)

        for i in range(5):
            v = parts.get(f"M{i+1}", "")
            if v:
                try:
                    self._sync_mosfet(i, int(v))
                except ValueError:
                    pass

    def _update_can_status(self):
        val, unit, color, sub = self._can_fields["RX Frames"]
        val.config(text=str(self._can_rx))
        sub.config(text="total frames received")
        bus_val, _, _, bus_sub = self._can_fields["Bus"]
        if self._can_rx > self._can_last_rx:
            bus_val.config(text="ONLINE", fg=GREEN)
            bus_sub.config(text="receiving frames")
        self._can_last_rx = self._can_rx

    # Decoded microDRIVE telemetry from the stream (ESC_* keys)
    def _parse_esc(self, parts):
        specs = [("ESC_RPM", "RPM", "{:.0f}"), ("ESC_V", "Voltage", "{:.2f}"),
                 ("ESC_I", "Current", "{:.2f}"), ("ESC_T", "ESC Temp", "{:.1f}"),
                 ("ESC_PWR", "Power", "{:.0f}")]

        # Firmware stops sending values once telemetry is >5s old and sends
        # ESC_LOST=1 instead — blank everything so stale data never looks live.
        if parts.get("ESC_LOST"):
            for _, field, _ in specs:
                val, unit, color, sub = self._can_fields[field]
                val.config(text="---" + unit)
                sub.config(text="ESC disconnected")
            bus_val, _, _, bus_sub = self._can_fields["Bus"]
            bus_val.config(text="OFFLINE", fg=RED)
            bus_sub.config(text="telemetry lost")
            self._esc_flags_lbl.config(text="ESC flags: —", fg=TEXT_DIM)
            return

        got_any = False
        for key, field, fmt in specs:
            v = parts.get(key, "")
            if not v:
                continue
            try:
                val, unit, color, sub = self._can_fields[field]
                val.config(text=fmt.format(float(v)) + unit)
                sub.config(text="live from ESC")
                got_any = True
            except ValueError:
                pass

        v = parts.get("ESC_AGE", "")
        if v:
            try:
                age = int(v)
                bus_val, _, _, bus_sub = self._can_fields["Bus"]
                if age < 1500:
                    bus_val.config(text="ESC OK", fg=GREEN)
                    bus_sub.config(text="telemetry fresh")
                else:
                    bus_val.config(text="STALE", fg=YELLOW)
                    bus_sub.config(text=f"last data {age/1000:.1f}s ago")
            except ValueError:
                pass

        v = parts.get("ESC_ERR", "")
        if v:
            try:
                flags = int(v, 16)
                if flags == 0:
                    self._esc_flags_lbl.config(text="ESC flags: OK", fg=GREEN)
                else:
                    names = [n for b, n in enumerate(ESC_ERROR_BITS)
                             if flags & (1 << b)]
                    self._esc_flags_lbl.config(
                        text=f"ESC flags: 0x{flags:X}  " + ", ".join(names), fg=RED)
            except ValueError:
                pass

        extra = []
        if parts.get("ESC_IN"):  extra.append(f"in {parts['ESC_IN']}%")
        if parts.get("ESC_OUT"): extra.append(f"out {parts['ESC_OUT']}%")
        if parts.get("ESC_MT"):  extra.append(f"motor {parts['ESC_MT']}°C")
        if extra and got_any:
            val, unit, color, sub = self._can_fields["Power"]
            sub.config(text="  ".join(extra))

        # Node heartbeat / anonymous-frame diagnosis (why telemetry may be absent)
        NODE_MODES = {"0": "OPERATIONAL", "1": "INITIALIZING", "2": "MAINTENANCE",
                      "3": "SW UPDATE", "7": "OFFLINE"}
        bus_val, _, _, bus_sub = self._can_fields["Bus"]
        if parts.get("ANON"):
            bus_val.config(text="NO ID", fg=RED)
            bus_sub.config(text=f"ESC awaiting node-ID allocation ({parts['ANON']} reqs)")
        elif parts.get("NS_ID") and not parts.get("ESC_AGE"):
            m = NODE_MODES.get(parts.get("NS_MODE", ""), "?")
            bus_val.config(text=f"NODE {parts['NS_ID']}", fg=YELLOW)
            bus_sub.config(text=f"{m}, no esc.Status telemetry yet")

        mode = parts.get("ESC_CMODE", "")
        if mode:
            cval = parts.get("ESC_CVAL", "0")
            if mode == "OFF":
                self._esc_cmd_lbl.config(text="CMD: OFF", fg=TEXT_DIM)
            elif mode == "RPM":
                self._esc_cmd_lbl.config(text=f"CMD: {cval} rpm", fg=GREEN)
            elif mode == "BRK":
                self._esc_cmd_lbl.config(text=f"CMD: BRAKE {cval}%", fg=ORANGE)
            else:
                self._esc_cmd_lbl.config(text=f"CMD: DUTY {cval}%", fg=CYAN)

    # ── Heartbeat watchdog ────────────────────────────────────────────────────
    def _check_heartbeat(self):
        if self._worker and self._worker.is_alive():
            alive = (time.time() - self._last_hb) < HB_TIMEOUT_S
            if alive != self._hb_alive:
                self._hb_alive = alive
                self._led_hb.steady(alive)
        else:
            if self._hb_alive:
                self._hb_alive = False
                self._led_hb.steady(False)
        self.after(500, self._check_heartbeat)

    # ── Terminal helpers ──────────────────────────────────────────────────────
    def _log_u0(self, text: str, tag: str = ""):
        self._terminal.config(state="normal")
        self._terminal.insert("end", text + "\n", tag if tag else ())
        # Trim so a 50 Hz stream can't grow the widget unbounded over a run
        if float(self._terminal.index("end-1c").split(".")[0]) > 3000:
            self._terminal.delete("1.0", "500.0")
        self._terminal.see("end")
        self._terminal.config(state="disabled")

    def _log_pix(self, text: str):
        self._pix_term.config(state="normal")
        ts = time.strftime("%H:%M:%S")
        self._pix_term.insert("end", f"[{ts}] ", "dim")
        self._pix_term.insert("end", text + "\n")
        if float(self._pix_term.index("end-1c").split(".")[0]) > 3000:
            self._pix_term.delete("1.0", "500.0")
        self._pix_term.see("end")
        self._pix_term.config(state="disabled")

    def _clear_u0(self):
        self._terminal.config(state="normal")
        self._terminal.delete("1.0", "end")
        self._terminal.config(state="disabled")

    def _set_status(self, text: str, ok: bool = True):
        self._status_lbl.config(text=f"●  {text}", fg=GREEN if ok else RED)


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = ECUDashboard()
    app.mainloop()
