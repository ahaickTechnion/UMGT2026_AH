"""
UMGT Engine Dashboard — engine-run GUI modeled on the LabVIEW
umgt_7thJuly2026.vi front panel, talking to the ESP32 ECU over USB serial.

LabVIEW → ECU mapping:
  ESC PWM 1500-2000us + MotorTone RPM   → DroneCAN duty + esc.Status telemetry
  NI PID Advanced (RPM → throttle)      → firmware governor (gov on/sp/gains)
  Ramp Up/Pause/Ramp Down + Ramp Rate   → firmware ramp machine
  NE-50X / Bronkhorst / Alicat fuel     → 2x KNF pumps on MOSFET PWM (pump 1/2)
  Fuel Shut Off                         → fuel cut latch (pumps forced 0)
  DAQmx thermocouples                   → 4x MAX31855 (TIT, glow, bearing, coil)
  Record + Write Delimited Spreadsheet  → CSV logging in this exe
  (new) auto start/shutdown sequencer + failsafes with editable parameters

Build: pyinstaller --onefile --windowed --name UMGT_Engine engine_dashboard.py
"""

import json
import os
import re
import time
import queue
import threading
import collections
import csv
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import serial
import serial.tools.list_ports

# ── Theme ─────────────────────────────────────────────────────────────────────
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
FONT_MONO_MD = ("Consolas", 12, "bold")
FONT_MONO_LG = ("Consolas", 15, "bold")
FONT_MONO_XL = ("Consolas", 22, "bold")
FONT_UI      = ("Segoe UI", 10)
FONT_UI_B    = ("Segoe UI", 10, "bold")
FONT_UI_SML  = ("Segoe UI", 9)
FONT_TITLE   = ("Segoe UI", 11, "bold")

BAUD         = 115200
HB_TIMEOUT_S = 3.5

PARAMS_FILE  = os.path.join(os.path.expanduser("~"), ".umgt_engine_params.json")

# Engine parameter metadata: (name, label, unit) — mirrors firmware "eng params"
PARAM_META = [
    ("glow_temp",     "Glow temp to proceed",   "°C"),
    ("glow_time",     "Glow preheat max",       "s"),
    ("spool_duty",    "Spool throttle",         "%"),
    ("spool_rpm",     "Spool RPM target",       "rpm"),
    ("spool_time",    "Spool timeout",          "s"),
    ("ign_pump",      "Ignition fuel start",    "ml/min"),
    ("ign_ramp",      "Ignition fuel ramp",     "ml/min/s"),
    ("ign_pump_max",  "Ignition fuel max",      "ml/min"),
    ("lightoff_tit",  "Light-off TIT (abs)",    "°C"),
    ("ign_timeout",   "Ignition timeout",       "s"),
    ("warmup_rpm",    "Warmup idle RPM",        "rpm"),
    ("glow_off_tit",  "Glow off above TIT",     "°C"),
    ("tit_max",       "TIT max (fault)",        "°C"),
    ("coil_max",      "Coil max (fault)",       "°C"),
    ("bearing_max",   "Bearing max (fault)",    "°C"),
    ("max_rpm",       "Max RPM (fault)",        "rpm"),
    ("flameout_tit",  "Flameout TIT floor",     "°C"),
    ("esc_loss",      "ESC loss timeout",       "s"),
    ("cool_duty",     "Cooldown throttle",      "%"),
    ("cool_tit",      "Cooldown until TIT",     "°C"),
    ("cool_time",     "Cooldown max time",      "s"),
    ("pump1_cal",     "Pump 1 flow @100%",      "ml/min"),
    ("pump2_cal",     "Pump 2 flow @100%",      "ml/min"),
    ("fuel_density",  "Fuel density",           "g/ml"),
    ("gov_kc",        "PID Kc",                 "%/rpm"),
    ("gov_ti",        "PID Ti",                 "min"),
    ("gov_td",        "PID Td",                 "min"),
    ("gov_min",       "PID output min",         "%"),
    ("gov_max",       "PID output max",         "%"),
    ("gov_slew",      "Throttle slew limit",    "%/s"),
    ("ramp_rate",     "Ramp rate",              "%/s"),
    ("batt_min",      "Battery 0% voltage",     "V"),
    ("batt_max",      "Battery 100% voltage",   "V"),
    ("sol_duty",      "Solenoid ON duty",       "%"),
]

# Signals that any chart can plot (name, color). The value for each is computed
# fresh every DATA frame in _parse_data → self._sig.
CHART_SIGNALS = [
    ("RPM",              CYAN),
    ("ESC Power [W]",    GREEN),
    ("TIT [°C]",         ORANGE),
    ("EGT / Glow [°C]",  YELLOW),
    ("Bearing [°C]",     GREEN),
    ("Coil [°C]",        PURPLE),
    ("DC Voltage [V]",   GREEN),
    ("DC Current [A]",   YELLOW),
    ("ESC Temp [°C]",    ORANGE),
    ("Battery [%]",      GREEN),
    ("Bus Current [A]",  YELLOW),
    ("Sensor CM [V]",    CYAN),
]

# Signals plotted on the Pixhawk page (fed from the PX_* stream keys)
PX_SIGNALS = [
    ("Altitude AMSL [m]",  GREEN),
    ("Altitude rel [m]",   CYAN),
    ("Pressure [hPa]",     YELLOW),
    ("Baro Temp [°C]",     ORANGE),
    ("Climb [m/s]",        PURPLE),
    ("Ground Speed [m/s]", ACCENT),
    ("Fuel Flow [ml/min]", CYAN),
    ("Throttle [%]",     ACCENT),
]
SIGNAL_COLOR = {name: color for name, color in CHART_SIGNALS}
SIGNAL_NAMES = [name for name, _ in CHART_SIGNALS]

ENG_STATE_COLORS = {
    "OFF": TEXT_DIM, "MANUAL": CYAN, "PRECHECK": YELLOW, "GLOW": ORANGE,
    "SPOOL": ORANGE, "IGNITION": ORANGE, "WARMUP": YELLOW,
    "RUNNING": GREEN, "COOLDOWN": CYAN, "FAULT": RED,
}

CSV_FIELDS = ["time", "elapsed_s", "state", "fault", "rpm", "tit_c", "glow_c",
              "bearing_c", "coil_c", "esc_v", "esc_i", "esc_w", "esc_temp_c",
              "throttle_pct", "gov_on", "gov_sp", "pump1_pct", "pump2_pct",
              "fuel_mlmin", "glow_on", "battery_pct", "bus_current_a", "sensor_cm_v"]


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
                        if len(buf) > 4096:
                            buf = buf[-1024:]
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
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


# ── Strip chart (LabVIEW-style waveform chart) ───────────────────────────────
class StripChart(tk.Canvas):
    def __init__(self, parent, title, color, span_s=120, height=150, **kw):
        super().__init__(parent, bg="#0d0d14", highlightthickness=1,
                         highlightbackground=BORDER, height=height, **kw)
        self.title = title
        self.color = color
        self.span = span_s
        self.data = collections.deque()   # (t, value)
        self.bind("<Configure>", lambda e: self.redraw())

    def set_signal(self, title, color):
        self.title = title
        self.color = color
        self.data.clear()
        self.redraw()

    def add(self, value):
        now = time.time()
        self.data.append((now, value))
        cutoff = now - self.span
        while self.data and self.data[0][0] < cutoff:
            self.data.popleft()

    def redraw(self):
        self.delete("all")
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 40 or h < 30:
            return
        self.create_text(8, 10, text=self.title, fill=TEXT_DIM,
                         font=FONT_UI_SML, anchor="w")
        # keep only finite values (drop NaN and +/-inf)
        pts = [(t, v) for t, v in self.data
               if v == v and v not in (float("inf"), float("-inf"))]
        if len(pts) < 2:
            return
        vmin = min(v for _, v in pts)
        vmax = max(v for _, v in pts)
        if vmax - vmin < 1e-9:
            vmax = vmin + 1
        pad = (vmax - vmin) * 0.10
        vmin -= pad
        vmax += pad
        t1 = pts[-1][0]
        t0 = t1 - self.span
        top, bot = 22, h - 6
        # gridlines + labels
        for frac in (0.0, 0.5, 1.0):
            y = bot - frac * (bot - top)
            self.create_line(2, y, w - 2, y, fill="#20202e")
            val = vmin + frac * (vmax - vmin)
            self.create_text(w - 4, y - 7, text=f"{val:.6g}", fill=TEXT_DIM,
                             font=("Consolas", 8), anchor="e")
        coords = []
        for t, v in pts:
            x = 2 + (t - t0) / self.span * (w - 4)
            y = bot - (v - vmin) / (vmax - vmin) * (bot - top)
            coords += [x, y]
        if len(coords) >= 4:
            self.create_line(*coords, fill=self.color, width=2)
        self.create_text(8, h - 12, text=f"{pts[-1][1]:.6g}", fill=self.color,
                         font=FONT_MONO_MD, anchor="w")


# ── Card helper ───────────────────────────────────────────────────────────────
def card(parent, title=""):
    outer = tk.Frame(parent, bg=BORDER)
    if title:
        tk.Label(outer, text=f"  {title}  ", bg=SURF2, fg=ACCENT,
                 font=FONT_UI_B, anchor="w", padx=6, pady=3).pack(fill="x")
    inner = tk.Frame(outer, bg=SURF, padx=8, pady=6)
    inner.pack(fill="both", expand=True, padx=1, pady=(0, 1))
    return inner


# ── Main application ──────────────────────────────────────────────────────────
class EngineDashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("UMGT Engine Control")
        self.configure(bg=BG)
        self.minsize(1040, 660)   # fits laptop screens; drag the column sashes to fit

        self._worker = None
        self._rx_q = queue.Queue()
        self._tx_q = queue.Queue()
        self._last_hb = 0.0

        self._params = {name: None for name, _, _ in PARAM_META}
        self._param_entries = {}
        self._param_widgets = {}
        self._load_local_params()

        self._gov_on = False
        self._fuel_cut = False
        self._slider_inhibit = False
        self._eng_state = "OFF"
        self._connect_t = None
        self._data_seen = False

        # Anti-bounce: while the user is dragging a control (and for a grace
        # period after release, until the ECU echoes the new value back) the
        # live stream must not sync that control.
        self._speed_active = False
        self._speed_guard = 0.0

        # latest values for CSV logging
        self._latest = {}
        self._recording = False
        self._csv_file = None
        self._csv_writer = None
        self._rec_t0 = 0

        self._build_header()
        self._build_body()
        self._refresh_ports()
        self._poll()
        self._tick_1s()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Header ────────────────────────────────────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self, bg=SURF)
        hdr.pack(fill="x")
        tk.Frame(hdr, bg=ACCENT, height=2).pack(fill="x")
        inner = tk.Frame(hdr, bg=SURF, padx=10, pady=6)
        inner.pack(fill="x")

        tk.Label(inner, text="🔥  UMGT Engine Control", bg=SURF, fg=TEXT,
                 font=FONT_TITLE).pack(side="left")

        # Engine state banner
        self._state_lbl = tk.Label(inner, text="OFF", bg=SURF2, fg=TEXT_DIM,
                                   font=FONT_MONO_LG, padx=16, pady=2)
        self._state_lbl.pack(side="left", padx=16)
        self._fault_lbl = tk.Label(inner, text="", bg=SURF, fg=RED,
                                   font=FONT_UI_B)
        self._fault_lbl.pack(side="left", padx=4)

        # Right side: connection
        conn = tk.Frame(inner, bg=SURF)
        conn.pack(side="right")
        self._abort_btn = tk.Button(conn, text="⛔ ABORT ALL", bg="#3d1420",
                                    fg=RED, font=FONT_UI_B, relief="flat",
                                    cursor="hand2", padx=12, pady=3,
                                    command=lambda: self._send("eng abort"))
        self._abort_btn.pack(side="right", padx=(10, 0))

        self._rec_btn = self._hbtn(conn, "● Record", self._toggle_record,
                                   fg=RED, width=10)
        self._rec_btn.pack(side="right", padx=4)

        self._conn_btn = self._hbtn(conn, "Connect", self._toggle_connect,
                                    fg=GREEN)
        self._conn_btn.pack(side="right", padx=4)
        self._status_lbl = tk.Label(conn, text="●  Disconnected", bg=SURF,
                                    fg=RED, font=FONT_UI_SML)
        self._status_lbl.pack(side="right", padx=8)
        self._port_var = tk.StringVar()
        self._port_cb = ttk.Combobox(conn, textvariable=self._port_var,
                                     width=9, font=FONT_MONO, state="readonly")
        self._port_cb.pack(side="right", padx=(4, 2))
        self._hbtn(conn, "⟳", self._refresh_ports, fg=ACCENT,
                   width=3).pack(side="right")

        self._elapsed_lbl = tk.Label(conn, text="t = 0 s", bg=SURF, fg=CYAN,
                                     font=FONT_MONO_MD)
        self._elapsed_lbl.pack(side="right", padx=14)

    def _hbtn(self, parent, text, cmd, fg=TEXT, width=10):
        b = tk.Button(parent, text=text, command=cmd, width=width, bg=SURF2,
                      fg=fg, font=FONT_UI, relief="flat", cursor="hand2",
                      activebackground=BORDER, activeforeground=fg,
                      padx=6, pady=2)
        b.bind("<Enter>", lambda e: b.config(bg=BORDER))
        b.bind("<Leave>", lambda e: b.config(bg=SURF2))
        return b

    def _sbtn(self, parent, text, cmd, fg=TEXT, padx=8, font=None):
        b = tk.Button(parent, text=text, command=cmd, bg=SURF2, fg=fg,
                      font=font or FONT_UI, relief="flat", cursor="hand2",
                      activebackground=BORDER, padx=padx, pady=3)
        b.bind("<Enter>", lambda e: b.config(bg=BORDER))
        b.bind("<Leave>", lambda e: b.config(bg=SURF2))
        return b

    # ── Body layout ───────────────────────────────────────────────────────────
    def _build_body(self):
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=6, pady=6)

        # Engine Data is a full-width status strip along the very bottom, kept
        # OUTSIDE the notebook so it stays visible on every page. (Built
        # bottom-first so it reserves its height before the pages claim the rest.)
        bottom = tk.Frame(body, bg=BG)
        bottom.pack(side="bottom", fill="x")

        self._configure_ttk()
        nb = ttk.Notebook(body)
        nb.pack(side="top", fill="both", expand=True)

        page_eng = tk.Frame(nb, bg=BG)
        page_px  = tk.Frame(nb, bg=BG)
        nb.add(page_eng, text="  Engine  ")
        nb.add(page_px,  text="  Pixhawk  ")

        # Three columns split by DRAGGABLE dividers (a PanedWindow). Grab either
        # sash — the cursor turns into a ↔ — and drag to give a column more or
        # less room. Lets you rebalance on small laptop screens where the fixed
        # layout would otherwise squash the buttons.
        top = tk.PanedWindow(page_eng, orient="horizontal", bg="#39406a",
                             sashwidth=8, sashrelief="flat", bd=0,
                             sashpad=0, opaqueresize=True)
        top.pack(side="top", fill="both", expand=True)

        left = tk.Frame(top, bg=BG)       # graphs
        mid = tk.Frame(top, bg=BG)        # ESC + fuel controls
        right = tk.Frame(top, bg=BG)      # sequencer + params + log
        # Charts pane absorbs spare space on resize; the two control columns keep
        # their width (and their buttons legible) unless you drag a sash.
        top.add(left,  minsize=240, width=560, stretch="always")
        top.add(mid,   minsize=300, width=360, stretch="never")
        top.add(right, minsize=330, width=470, stretch="never")

        self._build_graphs(left)
        self._build_esc_panel(mid)
        self._build_fuel_panel(mid)
        self._build_current_cal(mid)
        self._build_seq_panel(right)
        self._build_param_panel(right)
        self._build_esc_params(right)
        self._build_log(right)
        self._build_pixhawk_page(page_px)
        self._build_gauges(bottom)        # spans the full window width

    def _configure_ttk(self):
        s = ttk.Style(self)
        s.theme_use("default")
        s.configure("TNotebook", background=BG, borderwidth=0, tabmargins=[0, 0, 0, 0])
        s.configure("TNotebook.Tab", background=SURF2, foreground=TEXT_DIM,
                    padding=[22, 7], font=FONT_UI_B, borderwidth=0)
        s.map("TNotebook.Tab",
              background=[("selected", SURF), ("active", SURF)],
              foreground=[("selected", ACCENT), ("active", TEXT)])

    # ── Pixhawk / MAVLink page ───────────────────────────────────────────────
    # Cube Orange on TELEM1 → J21 (ESP32 Serial2, GPIO16/17) at 57600 8N1.
    # The ESP32 does the MAVLink decoding and republishes the values as PX_* keys
    # in the DATA stream, so this page is a pure view over those keys.
    def _build_pixhawk_page(self, parent):
        wrap = tk.Frame(parent, bg=BG)
        wrap.pack(fill="both", expand=True)

        col_l = tk.Frame(wrap, bg=BG)
        col_l.pack(side="left", fill="both", expand=True)
        col_r = tk.Frame(wrap, bg=BG)
        col_r.pack(side="left", fill="both", expand=True, padx=(8, 0))

        # ---- link status -----------------------------------------------------
        c = card(col_l, "MAVLink Link  (Cube Orange · TELEM1 → J21 · Serial2)")
        c.master.pack(fill="x")

        srow = tk.Frame(c, bg=SURF)
        srow.pack(fill="x", pady=(0, 6))
        self._px_link = tk.Label(srow, text="● NO LINK", bg=SURF, fg=RED,
                                 font=("Segoe UI", 13, "bold"))
        self._px_link.pack(side="left")
        self._px_sub = tk.Label(srow, text="waiting for heartbeat…", bg=SURF,
                                fg=TEXT_DIM, font=FONT_UI_SML)
        self._px_sub.pack(side="left", padx=10)

        self._px_stats = {}
        grid = tk.Frame(c, bg=SURF)
        grid.pack(fill="x")
        for i, (key, label) in enumerate((("raw", "Raw bytes in"), ("rx", "Frames OK"),
                                          ("bad", "Bad CRC"), ("tx", "Frames Sent"),
                                          ("hb", "HB age"))):
            grid.columnconfigure(i, weight=1)
            box = tk.Frame(grid, bg=SURF2, padx=8, pady=4)
            box.grid(row=0, column=i, sticky="nsew", padx=2)
            tk.Label(box, text=label, bg=SURF2, fg=TEXT_DIM,
                     font=FONT_UI_SML).pack(anchor="w")
            v = tk.Label(box, text="---", bg=SURF2, fg=CYAN, font=FONT_MONO_MD,
                         width=9, anchor="w")
            v.pack(anchor="w")
            self._px_stats[key] = v

        # ---- barometer + altitude -------------------------------------------
        c2 = card(col_l, "Barometer / Altitude  (from Pixhawk)")
        c2.master.pack(fill="x", pady=(6, 0))
        self._px_vals = {}
        specs = [
            ("press", "Pressure",        "hPa"),
            ("ptemp", "Baro Temp",       "°C"),
            ("alt",   "Altitude AMSL",   "m"),
            ("ralt",  "Altitude (rel)",  "m"),
            ("climb", "Climb Rate",      "m/s"),
            ("gs",    "Ground Speed",    "m/s"),
            ("hdg",   "Heading",         "°"),
            ("vbat",  "Pixhawk Batt",    "V"),
        ]
        g2 = tk.Frame(c2, bg=SURF)
        g2.pack(fill="x")
        for i, (key, label, unit) in enumerate(specs):
            r, col = divmod(i, 4)
            g2.columnconfigure(col, weight=1)
            box = tk.Frame(g2, bg=SURF2, padx=8, pady=5)
            box.grid(row=r, column=col, sticky="nsew", padx=2, pady=2)
            tk.Label(box, text=label, bg=SURF2, fg=TEXT_DIM,
                     font=FONT_UI_SML).pack(anchor="w")
            v = tk.Label(box, text="---", bg=SURF2, fg=GREEN, font=FONT_MONO_LG,
                         width=8, anchor="w")
            v.pack(anchor="w")
            tk.Label(box, text=unit, bg=SURF2, fg=TEXT_DIM,
                     font=("Segoe UI", 8)).pack(anchor="w")
            self._px_vals[key] = v

        # ---- charts ----------------------------------------------------------
        c3 = card(col_l, "Trend  (120 s)")
        c3.master.pack(fill="both", expand=True, pady=(6, 0))
        self._px_charts = []
        px_color = dict(PX_SIGNALS)
        px_names = [s for s, _ in PX_SIGNALS]
        for var_default in ("Altitude AMSL [m]", "Pressure [hPa]"):
            hdr = tk.Frame(c3, bg=SURF)
            hdr.pack(fill="x")
            var = tk.StringVar(value=var_default)
            cb = ttk.Combobox(hdr, textvariable=var, state="readonly", width=20,
                              values=px_names, font=FONT_UI_SML)
            cb.pack(side="left", padx=2, pady=1)
            ch = StripChart(c3, var_default, px_color.get(var_default, GREEN),
                            height=130)
            ch.pack(fill="both", expand=True)
            cb.bind("<<ComboboxSelected>>",
                    lambda e, c=ch, v=var: c.set_signal(
                        v.get(), px_color.get(v.get(), GREEN)))
            self._px_charts.append((ch, var))

        # ---- control / setup -------------------------------------------------
        c4 = card(col_r, "Link Control")
        c4.master.pack(fill="x")
        brow = tk.Frame(c4, bg=SURF)
        brow.pack(fill="x")
        self._sbtn(brow, "Request streams", lambda: self._send("px req"),
                   fg=GREEN).pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(brow, "Status → log", lambda: self._send("px status"),
                   fg=CYAN).pack(side="left", fill="x", expand=True, padx=2)
        brow2 = tk.Frame(c4, bg=SURF)
        brow2.pack(fill="x", pady=(4, 0))
        self._sbtn(brow2, "\U0001F50A Beep Cube", lambda: self._send("px beep"),
                   fg=GREEN).pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(brow2, "Send heartbeat", lambda: self._send("px hb"),
                   fg=CYAN).pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(brow2, "Reset counters", lambda: self._send("px reset"),
                   fg=YELLOW).pack(side="left", fill="x", expand=True, padx=2)
        tk.Label(c4, text="Beep Cube plays the Pixhawk buzzer — proves the "
                          "ECU→Cube link works, not just Cube→ECU.",
                 bg=SURF, fg=TEXT_DIM, font=FONT_UI_SML, justify="left").pack(
                     anchor="w", pady=(4, 0))

        baudrow = tk.Frame(c4, bg=SURF)
        baudrow.pack(fill="x", pady=(6, 0))
        tk.Label(baudrow, text="Baud", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML).pack(side="left")
        self._px_baud = tk.StringVar(value="57600")
        ttk.Combobox(baudrow, textvariable=self._px_baud, state="readonly", width=9,
                     values=["57600", "115200", "921600", "38400", "19200"]
                     ).pack(side="left", padx=4)
        self._sbtn(baudrow, "Apply",
                   lambda: self._send(f"px baud {self._px_baud.get()}"),
                   fg=GREEN).pack(side="left", padx=2)
        tk.Label(c4, text="Cube TELEM1 default is 57600. If there's no link, check "
                          "GND/RX↔TX are crossed and\nset BRD_SER1_RTSCTS=0 (no flow "
                          "control on a 3-wire hookup).",
                 bg=SURF, fg=TEXT_DIM, font=FONT_UI_SML, justify="left").pack(
                     anchor="w", pady=(6, 0))

        # ---- generator telemetry we publish upward ---------------------------
        c5 = card(col_r, "Generator Telemetry → Pixhawk")
        c5.master.pack(fill="x", pady=(6, 0))
        tk.Label(c5, text="Sent at 2 Hz as GENERATOR_STATUS (bus V/A, power, RPM,\n"
                          "temps) plus NAMED_VALUE_FLOAT: TIT, FUELFLOW, BUSCUR,\n"
                          "SHAFTRPM — visible in Mission Planner and logged.",
                 bg=SURF, fg=TEXT_DIM, font=FONT_UI_SML, justify="left").pack(anchor="w")
        grow = tk.Frame(c5, bg=SURF)
        grow.pack(fill="x", pady=(6, 0))
        self._sbtn(grow, "TX ON", lambda: self._send("px gen on"),
                   fg=GREEN).pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(grow, "TX OFF", lambda: self._send("px gen off"),
                   fg=RED).pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(grow, "Send once", lambda: self._send("px gen"),
                   fg=CYAN).pack(side="left", fill="x", expand=True, padx=2)

        # ---- diagnostics -----------------------------------------------------
        c6 = card(col_r, "Diagnostics")
        c6.master.pack(fill="x", pady=(6, 0))
        drow = tk.Frame(c6, bg=SURF)
        drow.pack(fill="x")
        self._sbtn(drow, "Parser ON", lambda: self._send("px on"),
                   fg=GREEN).pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(drow, "Parser OFF", lambda: self._send("px off"),
                   fg=TEXT_DIM).pack(side="left", fill="x", expand=True, padx=2)
        drow2 = tk.Frame(c6, bg=SURF)
        drow2.pack(fill="x", pady=(4, 0))
        self._sbtn(drow2, "Raw echo ON", lambda: self._send("px raw on"),
                   fg=YELLOW).pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(drow2, "Raw echo OFF", lambda: self._send("px raw off"),
                   fg=TEXT_DIM).pack(side="left", fill="x", expand=True, padx=2)
        tk.Label(c6, text="Raw echo dumps binary MAVLink to the log — noisy, and it\n"
                          "will disturb the data stream. Use only to prove bytes arrive.",
                 bg=SURF, fg=TEXT_DIM, font=FONT_UI_SML, justify="left").pack(
                     anchor="w", pady=(6, 0))

    def _update_pixhawk(self, parts, fget):
        """Refresh the Pixhawk page from the PX_* keys in a DATA line."""
        online = parts.get("PX_OK") == "1"
        if online:
            self._px_link.config(text="● LINK UP", fg=GREEN)
            sysid = parts.get("PX_SYS", "?")
            ver   = parts.get("PX_VER", "?")
            self._px_sub.config(text=f"heartbeat from system {sysid}, MAVLink v{ver}")
        else:
            self._px_link.config(text="● NO LINK", fg=RED)
            self._px_sub.config(text="no heartbeat — check wiring, baud, TELEM1")

        # Raw byte count is the wiring verdict: 0 = nothing on the RX pin at all.
        raw = parts.get("PX_RAW", "")
        self._px_stats["raw"].config(
            text=raw or "---",
            fg=RED if raw == "0" else GREEN)
        self._px_stats["rx"].config(text=parts.get("PX_RX", "---"))
        bad = parts.get("PX_BAD", "0")
        self._px_stats["bad"].config(text=bad, fg=RED if bad not in ("0", "") else CYAN)
        self._px_stats["tx"].config(text=parts.get("PX_TX", "---"))
        hb = parts.get("PX_HB", "")
        self._px_stats["hb"].config(text=f"{hb} ms" if hb else "---")

        vals = {
            "press": fget("PX_PRESS"), "ptemp": fget("PX_PTEMP"),
            "alt":   fget("PX_ALT"),   "ralt":  fget("PX_RALT"),
            "climb": fget("PX_CLIMB"), "gs":    fget("PX_GS"),
            "hdg":   fget("PX_HDG"),   "vbat":  fget("PX_VBAT"),
        }
        fmt = {"press": "{:.2f}", "ptemp": "{:.1f}", "alt": "{:.2f}",
               "ralt": "{:.2f}", "climb": "{:.2f}", "gs": "{:.2f}",
               "hdg": "{:.0f}", "vbat": "{:.2f}"}
        for k, v in vals.items():
            self._px_vals[k].config(text=fmt[k].format(v) if v == v else "---")

        self._px_sig = {
            "Altitude AMSL [m]": vals["alt"], "Altitude rel [m]": vals["ralt"],
            "Pressure [hPa]": vals["press"],  "Baro Temp [°C]": vals["ptemp"],
            "Climb [m/s]": vals["climb"],     "Ground Speed [m/s]": vals["gs"],
        }
        for chart, var in self._px_charts:
            chart.add(self._px_sig.get(var.get(), float("nan")))

    # ── Graphs — 4 charts, each plots any signal you pick from its dropdown ──
    def _build_graphs(self, parent):
        g = card(parent, "Charts  (120 s)  —  pick a signal per chart")
        g.master.pack(fill="both", expand=True)
        grid = tk.Frame(g, bg=SURF)
        grid.pack(fill="both", expand=True)
        for c in (0, 1):
            grid.columnconfigure(c, weight=1)
        for r in (0, 1):
            grid.rowconfigure(r, weight=1)
        self._charts = []   # list of (StripChart, StringVar)
        defaults = ["RPM", "ESC Power [W]", "TIT [°C]", "Coil [°C]"]
        for idx, dflt in enumerate(defaults):
            r, c = divmod(idx, 2)
            self._make_chart(grid, r, c, dflt)

    def _make_chart(self, grid, r, c, default_signal):
        frame = tk.Frame(grid, bg=SURF)
        frame.grid(row=r, column=c, sticky="nsew", padx=2, pady=2)
        hdr = tk.Frame(frame, bg=SURF)
        hdr.pack(fill="x")
        var = tk.StringVar(value=default_signal)
        cb = ttk.Combobox(hdr, textvariable=var, state="readonly",
                          values=SIGNAL_NAMES, width=16, font=FONT_UI_SML)
        cb.pack(side="left", padx=2, pady=1)
        chart = StripChart(frame, default_signal,
                           SIGNAL_COLOR.get(default_signal, CYAN))
        chart.pack(fill="both", expand=True)
        cb.bind("<<ComboboxSelected>>",
                lambda e, ch=chart, v=var: ch.set_signal(
                    v.get(), SIGNAL_COLOR.get(v.get(), CYAN)))
        self._charts.append((chart, var))

    # ── Gauges row ────────────────────────────────────────────────────────────
    def _gauge(self, parent, label, unit, color, warn=None):
        # pack_propagate(False) + fixed-width labels so the frame keeps a constant
        # size no matter what text lands in it — otherwise a value gaining a "-"
        # sign or an extra digit resizes the gauge and reflows (shakes) the whole
        # row. The value font is monospace, so width= in chars == fixed pixels.
        grp = tk.Frame(parent, bg=SURF2, padx=8, pady=5)
        grp.pack_propagate(False)
        grp.config(height=68)
        tk.Label(grp, text=label, bg=SURF2, fg=TEXT_DIM,
                 font=FONT_UI_SML, anchor="w").pack(anchor="w", fill="x")
        val = tk.Label(grp, text="---", bg=SURF2, fg=color, font=FONT_MONO_LG,
                       width=8, anchor="w")
        val.pack(anchor="w")
        unit_lbl = tk.Label(grp, text=unit, bg=SURF2, fg=TEXT_DIM,
                            font=("Segoe UI", 8), width=16, anchor="w")
        unit_lbl.pack(anchor="w")
        grp.base_color = color
        grp.warn = warn
        grp.unit_lbl = unit_lbl   # so current gauges can append raw volts
        return grp, val

    def _build_gauges(self, parent):
        c = card(parent, "Engine Data")
        c.master.pack(fill="x", pady=(6, 0))
        row = tk.Frame(c, bg=SURF)
        row.pack(fill="x")
        self._gauges = {}
        specs = [
            ("rpm",     "Shaft Speed",        "RPM", CYAN,   "max_rpm"),
            ("tit",     "Turbine Inlet Temp", "°C",  ORANGE, "tit_max"),
            ("glow_t",  "Glow Area Temp",     "°C",  YELLOW, None),
            ("bearing", "Bearing Temp",       "°C",  GREEN,  "bearing_max"),
            ("coil",    "Coil Temp",          "°C",  PURPLE, "coil_max"),
            ("esc_v",   "DC Voltage",         "V",   GREEN,  None),
            ("esc_i",   "DC Current",         "A",   YELLOW, None),
            ("esc_w",   "DC Power",           "W",   PURPLE, None),
            ("batt",    "Battery",            "%",   GREEN,  None),
            ("esc_t",   "ESC Temp",           "°C",  ORANGE, None),
            ("fuel",    "Fuel Flow",          "ml/min", CYAN, None),
            ("cur34",   "Bus Current",        "A",   YELLOW, None),
            ("cur35",   "Sensor CM",          "V",   CYAN,   None),
        ]
        for i, (key, label, unit, color, warn) in enumerate(specs):
            row.columnconfigure(i, weight=1)
            grp, val = self._gauge(row, label, unit, color, warn)
            grp.grid(row=0, column=i, sticky="nsew", padx=2)
            self._gauges[key] = (grp, val)

    # ── ESC / throttle panel ─────────────────────────────────────────────────
    def _build_esc_panel(self, parent):
        c = card(parent, "ESC  —  Throttle / RPM Governor")
        c.master.pack(fill="y", expand=True)

        top = tk.Frame(c, bg=SURF)
        top.pack(fill="both", expand=True)

        # big vertical speed slider (LabVIEW 'speed')
        sl_frame = tk.Frame(top, bg=SURF)
        sl_frame.pack(side="left", fill="y", padx=(0, 8))
        tk.Label(sl_frame, text="speed", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML).pack()
        self._speed_slider = tk.Scale(
            sl_frame, from_=100, to=0, orient="vertical", length=330,
            bg=SURF, fg=TEXT, troughcolor=SURF2, highlightthickness=0,
            sliderlength=22, width=26, font=FONT_MONO,
            command=self._on_speed_move)
        self._speed_slider.pack(fill="y", expand=True)
        self._speed_slider.bind("<ButtonPress-1>", self._on_speed_press)
        self._speed_slider.bind("<ButtonRelease-1>", self._on_speed_release)
        self._speed_mode_lbl = tk.Label(sl_frame, text="throttle %", bg=SURF,
                                        fg=CYAN, font=FONT_UI_SML)
        self._speed_mode_lbl.pack()

        ctl = tk.Frame(top, bg=SURF)
        ctl.pack(side="left", fill="both", expand=True)

        # ARM/DISARM — the microDRIVE refuses to spin without an arming
        # broadcast (CAN_ARM_CHK_EN). Arm first, then throttle.
        self._armed = False
        self._arm_btn = tk.Button(ctl, text="ESC: DISARMED", bg=SURF2,
                                  fg=TEXT_DIM, font=FONT_UI_B, relief="flat",
                                  cursor="hand2", pady=6,
                                  command=self._toggle_arm)
        self._arm_btn.pack(fill="x", pady=(0, 4))

        # PID CONTROL toggle + gains (LabVIEW cluster: Kc, Ti min, Td min)
        self._pid_btn = tk.Button(ctl, text="PID CONTROL\nOFF", bg=SURF2,
                                  fg=TEXT_DIM, font=FONT_UI_B, relief="flat",
                                  cursor="hand2", pady=6,
                                  command=self._toggle_pid)
        self._pid_btn.pack(fill="x", pady=(0, 4))

        gains = tk.Frame(ctl, bg=SURF)
        gains.pack(fill="x")
        self._gain_vars = {}
        for key, lbl in (("gov_kc", "Kc"), ("gov_ti", "Ti (min)"),
                         ("gov_td", "Td (min)")):
            row = tk.Frame(gains, bg=SURF)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=lbl, bg=SURF, fg=TEXT_DIM, font=FONT_UI_SML,
                     width=7, anchor="w").pack(side="left")
            var = tk.StringVar()
            tk.Entry(row, textvariable=var, bg="#0d0d14", fg=TEXT, width=9,
                     font=FONT_MONO, relief="flat", insertbackground=TEXT,
                     highlightthickness=1, highlightbackground=BORDER,
                     highlightcolor=ACCENT).pack(side="left", ipady=1)
            self._gain_vars[key] = var
        self._sbtn(gains, "Send gains", self._send_gains,
                   fg=ACCENT).pack(fill="x", pady=3)

        # governor setpoint display
        self._gsp_lbl = tk.Label(ctl, text="SP: --- rpm", bg=SURF2, fg=GREEN,
                                 font=FONT_MONO_MD, pady=2)
        self._gsp_lbl.pack(fill="x", pady=2)
        self._thr_lbl = tk.Label(ctl, text="THR: 0.0 %", bg=SURF2, fg=CYAN,
                                 font=FONT_MONO_MD, pady=2)
        self._thr_lbl.pack(fill="x", pady=2)

        # Ramp controls (LabVIEW Ramp Up / Pause / Ramp Down + rate + Max RPM)
        ramp = tk.Frame(ctl, bg=SURF)
        ramp.pack(fill="x", pady=(6, 0))
        rrow = tk.Frame(ramp, bg=SURF)
        rrow.pack(fill="x")
        self._sbtn(rrow, "▲ Ramp Up", lambda: self._send("ramp up"),
                   fg=GREEN, padx=4).pack(side="left", fill="x", expand=True)
        self._sbtn(rrow, "❚❚ Pause", lambda: self._send("ramp pause"),
                   fg=YELLOW, padx=4).pack(side="left", fill="x", expand=True)
        self._sbtn(rrow, "▼ Ramp Dn", lambda: self._send("ramp down"),
                   fg=ORANGE, padx=4).pack(side="left", fill="x", expand=True)
        rrow2 = tk.Frame(ramp, bg=SURF)
        rrow2.pack(fill="x", pady=2)
        tk.Label(rrow2, text="Ramp rate %/s", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML).pack(side="left")
        self._ramp_rate_var = tk.StringVar(value="5")
        e = tk.Entry(rrow2, textvariable=self._ramp_rate_var, width=6,
                     bg="#0d0d14", fg=TEXT, font=FONT_MONO, relief="flat",
                     insertbackground=TEXT, highlightthickness=1,
                     highlightbackground=BORDER, highlightcolor=ACCENT)
        e.pack(side="left", padx=4, ipady=1)
        e.bind("<Return>", lambda ev: self._send(
            f"ramp rate {self._ramp_rate_var.get()}"))
        self._ramp_lbl = tk.Label(rrow2, text="ramp: OFF", bg=SURF,
                                  fg=TEXT_DIM, font=FONT_UI_SML)
        self._ramp_lbl.pack(side="right")

        # STOP MOTOR (LabVIEW)
        stop = tk.Button(ctl, text="■ STOP MOTOR", bg="#3d1420", fg=RED,
                         font=FONT_UI_B, relief="flat", cursor="hand2", pady=8,
                         command=self._stop_motor)
        stop.bind("<Enter>", lambda e: stop.config(bg="#58182c"))
        stop.bind("<Leave>", lambda e: stop.config(bg="#3d1420"))
        stop.pack(fill="x", pady=(8, 0))

    # ── Fuel + glow panel ────────────────────────────────────────────────────
    def _build_fuel_panel(self, parent):
        c = card(parent, "Fuel  —  KNF Pumps + Glow")
        c.master.pack(fill="x", pady=(6, 0))

        pumps = tk.Frame(c, bg=SURF)
        pumps.pack(fill="x")
        self._pump_sliders = []
        self._pump_lbls = []
        for i in (0, 1):
            col = tk.Frame(pumps, bg=SURF)
            col.pack(side="left", expand=True, fill="x", padx=4)
            tk.Label(col, text=f"Pump {i+1}", bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML).pack()
            sl = tk.Scale(col, from_=100, to=0, orient="vertical", length=120,
                          bg=SURF, fg=TEXT, troughcolor=SURF2,
                          highlightthickness=0, sliderlength=16, width=20,
                          font=("Consolas", 8))
            sl.pack()
            sl.bind("<ButtonRelease-1>",
                    lambda ev, idx=i: self._on_pump_release(idx))
            lbl = tk.Label(col, text="0.0 %", bg=SURF2, fg=CYAN,
                           font=FONT_MONO, padx=4)
            lbl.pack(pady=2)
            self._pump_sliders.append(sl)
            self._pump_lbls.append(lbl)

        prow = tk.Frame(c, bg=SURF)
        prow.pack(fill="x", pady=2)
        self._sbtn(prow, "Pumps STOP", lambda: self._send("pump stop"),
                   fg=YELLOW).pack(side="left", fill="x", expand=True, padx=2)

        # FUEL SHUT OFF latch (LabVIEW)
        self._fuelcut_btn = tk.Button(
            c, text="FUEL SHUT OFF", bg="#3d1420", fg=RED, font=FONT_UI_B,
            relief="flat", cursor="hand2", pady=6, command=self._toggle_fuelcut)
        self._fuelcut_btn.pack(fill="x", pady=(4, 2))

        grow = tk.Frame(c, bg=SURF)
        grow.pack(fill="x", pady=(4, 0))
        tk.Label(grow, text="Glow plug", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML).pack(side="left")
        self._glow_lamp = tk.Label(grow, text="●", bg=SURF, fg=TEXT_DIM,
                                   font=("Segoe UI", 14))
        self._glow_lamp.pack(side="left", padx=4)
        self._sbtn(grow, "ON", lambda: self._send("glow on"),
                   fg=GREEN, padx=10).pack(side="right", padx=2)
        self._sbtn(grow, "OFF", lambda: self._send("glow off"),
                   fg=RED, padx=10).pack(side="right", padx=2)

        # Solenoids (M3/GPIO25 = sol 1, M2/GPIO33 = sol 2)
        self._sol_lamps = []
        for i in (1, 2):
            srow = tk.Frame(c, bg=SURF)
            srow.pack(fill="x", pady=(3, 0))
            tk.Label(srow, text=f"Solenoid {i}", bg=SURF, fg=TEXT_DIM,
                     font=FONT_UI_SML).pack(side="left")
            lamp = tk.Label(srow, text="●", bg=SURF, fg=TEXT_DIM,
                            font=("Segoe UI", 14))
            lamp.pack(side="left", padx=4)
            self._sol_lamps.append(lamp)
            self._sbtn(srow, "ON", lambda n=i: self._send(f"sol {n} on"),
                       fg=GREEN, padx=10).pack(side="right", padx=2)
            self._sbtn(srow, "OFF", lambda n=i: self._send(f"sol {n} off"),
                       fg=RED, padx=10).pack(side="right", padx=2)

    # ── Current sensor calibration ───────────────────────────────────────────
    def _build_current_cal(self, parent):
        c = card(parent, "Current Sensor Cal  (SSA-2 differential, GPIO34/35)")
        c.master.pack(fill="x", pady=(6, 0))

        # live readout: differential volts (the signal) and common-mode (health)
        vrow = tk.Frame(c, bg=SURF)
        vrow.pack(fill="x")
        self._cur_vlbl = {}
        for key, title in (("34", "Differential V"), ("35", "Common-mode V")):
            box = tk.Frame(vrow, bg=SURF2, padx=8, pady=3)
            box.pack(side="left", expand=True, fill="x", padx=2)
            tk.Label(box, text=title, bg=SURF2, fg=TEXT_DIM,
                     font=FONT_UI_SML).pack(anchor="w")
            lbl = tk.Label(box, text="--- V", bg=SURF2, fg=CYAN,
                           font=FONT_MONO_MD)
            lbl.pack(anchor="w")
            self._cur_vlbl[key] = lbl

        # zero-volts and amps-per-volt fields (firmware cal is shared by both)
        crow = tk.Frame(c, bg=SURF)
        crow.pack(fill="x", pady=(4, 0))
        tk.Label(crow, text="Zero V", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML).pack(side="left")
        self._cal_zero = tk.StringVar(value="0")
        tk.Entry(crow, textvariable=self._cal_zero, width=7, bg="#0d0d14",
                 fg=TEXT, font=FONT_MONO, relief="flat", insertbackground=TEXT,
                 highlightthickness=1, highlightbackground=BORDER,
                 highlightcolor=ACCENT).pack(side="left", padx=3, ipady=1)
        tk.Label(crow, text="A / V", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML).pack(side="left")
        self._cal_apv = tk.StringVar(value="80")
        tk.Entry(crow, textvariable=self._cal_apv, width=7, bg="#0d0d14",
                 fg=TEXT, font=FONT_MONO, relief="flat", insertbackground=TEXT,
                 highlightthickness=1, highlightbackground=BORDER,
                 highlightcolor=ACCENT).pack(side="left", padx=3, ipady=1)
        self._sbtn(crow, "Apply", self._apply_cal,
                   fg=GREEN).pack(side="left", padx=3)

        # helpers: zero-now trims residual differential offset at 0 A; the model
        # presets set A/V per the SSA-2 variant (zero is always 0 for a shunt).
        brow = tk.Frame(c, bg=SURF)
        brow.pack(fill="x", pady=(4, 0))
        self._sbtn(brow, "Zero now (@0A)", self._zero_now,
                   fg=YELLOW).pack(side="left", fill="x", expand=True, padx=2)
        prow = tk.Frame(c, bg=SURF)
        prow.pack(fill="x", pady=(4, 0))
        for name, apv in (("100A", 80), ("250A", 200), ("500A", 400), ("1000A", 800)):
            self._sbtn(prow, name, lambda a=apv: self._preset_cal(0, a),
                       fg=CYAN).pack(side="left", fill="x", expand=True, padx=2)

    def _apply_cal(self):
        z = self._cal_zero.get().strip()
        a = self._cal_apv.get().strip()
        if z and a:
            self._send(f"current cal {z} {a}")

    def _preset_cal(self, zero, apv):
        self._cal_zero.set(str(zero))
        self._cal_apv.set(str(apv))
        self._apply_cal()

    def _zero_now(self):
        # capture the present differential voltage as the 0 A offset (should be
        # near 0 — trims the sensor's residual DC offset with no current flowing)
        v = getattr(self, "_last_diffv", None)
        if v is not None and v == v:
            self._cal_zero.set(f"{v:.4f}")
            self._apply_cal()

    # ── Sequencer panel ──────────────────────────────────────────────────────
    def _build_seq_panel(self, parent):
        c = card(parent, "Engine Sequencer")
        c.master.pack(fill="x")

        row = tk.Frame(c, bg=SURF)
        row.pack(fill="x")
        start = tk.Button(row, text="▶ AUTO START", bg="#14321d", fg=GREEN,
                          font=FONT_UI_B, relief="flat", cursor="hand2",
                          padx=10, pady=8, command=self._auto_start)
        start.bind("<Enter>", lambda e: start.config(bg="#1d4629"))
        start.bind("<Leave>", lambda e: start.config(bg="#14321d"))
        start.pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(row, "■ STOP\n(cooldown)", lambda: self._send("eng stop"),
                   fg=YELLOW, padx=8).pack(side="left", fill="x", expand=True, padx=2)

        row2 = tk.Frame(c, bg=SURF)
        row2.pack(fill="x", pady=(4, 0))
        self._sbtn(row2, "MANUAL MODE", lambda: self._send("eng manual"),
                   fg=CYAN).pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(row2, "RESET FAULT", lambda: self._send("eng reset"),
                   fg=ORANGE).pack(side="left", fill="x", expand=True, padx=2)

        # sequence progress strip
        self._seq_lbls = {}
        strip = tk.Frame(c, bg=SURF)
        strip.pack(fill="x", pady=(6, 0))
        for s in ("PRECHECK", "GLOW", "SPOOL", "IGNITION", "WARMUP",
                  "RUNNING"):
            l = tk.Label(strip, text=s, bg=SURF2, fg=TEXT_DIM,
                         font=("Segoe UI", 8, "bold"), padx=4, pady=2)
            l.pack(side="left", fill="x", expand=True, padx=1)
            self._seq_lbls[s] = l

    # ── Parameters panel ─────────────────────────────────────────────────────
    def _build_param_panel(self, parent):
        c = card(parent, "State Machine / Limits / PID Parameters")
        c.master.pack(fill="both", expand=True, pady=(6, 0))

        canvas = tk.Canvas(c, bg=SURF, highlightthickness=0, width=340)
        vsb = ttk.Scrollbar(c, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        frame = tk.Frame(canvas, bg=SURF)
        win = canvas.create_window((0, 0), window=frame, anchor="nw")
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfig(win, width=e.width))
        frame.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

        for name, label, unit in PARAM_META:
            row = tk.Frame(frame, bg=SURF)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=label, bg=SURF, fg=TEXT_DIM, font=FONT_UI_SML,
                     width=22, anchor="w").pack(side="left")
            var = tk.StringVar()
            e = tk.Entry(row, textvariable=var, width=9, bg="#0d0d14",
                         fg=TEXT, font=FONT_MONO, relief="flat",
                         insertbackground=TEXT, highlightthickness=1,
                         highlightbackground=BORDER, highlightcolor=ACCENT)
            e.pack(side="left", ipady=1)
            e.bind("<Return>", lambda ev, n=name: self._send_param(n))
            tk.Label(row, text=unit, bg=SURF, fg=TEXT_DIM,
                     font=("Segoe UI", 8), width=6,
                     anchor="w").pack(side="left", padx=2)
            self._param_entries[name] = var
            self._param_widgets[name] = e

        # 2x2 grid so all four buttons fit the panel width (a single row clips
        # the last button off the right edge).
        # Apply = live to ECU RAM; Save to ECU = persist to ECU flash (survives
        # reboot); Read = pull ECU's live values; Save local = JSON on this PC.
        btns = tk.Frame(c.master, bg=SURF)
        btns.pack(fill="x", side="bottom", pady=2, padx=2)
        for i in (0, 1):
            btns.columnconfigure(i, weight=1)
        specs = [
            ("Apply all (live)", self._send_all_params, GREEN),
            ("Save to ECU", self._save_to_ecu, YELLOW),
            ("Read from ECU", lambda: self._send("eng params"), ACCENT),
            ("Save local", self._save_local_params, CYAN),
        ]
        for idx, (label, cmd, color) in enumerate(specs):
            r, c2 = divmod(idx, 2)
            self._sbtn(btns, label, cmd, fg=color).grid(
                row=r, column=c2, sticky="ew", padx=2, pady=1)

    # ── ESC parameters (DroneCAN GetSet) ─────────────────────────────────────
    def _build_esc_params(self, parent):
        c = card(parent, "ESC Parameters  (DroneCAN, names are CASE-SENSITIVE)")
        c.master.pack(fill="x", pady=(6, 0))

        row = tk.Frame(c, bg=SURF)
        row.pack(fill="x")
        tk.Label(row, text="Name", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML).pack(side="left")
        self._escp_name = tk.StringVar(value="BUS_CUR_LIM")
        tk.Entry(row, textvariable=self._escp_name, width=18, bg="#0d0d14",
                 fg=TEXT, font=FONT_MONO, relief="flat", insertbackground=TEXT,
                 highlightthickness=1, highlightbackground=BORDER,
                 highlightcolor=ACCENT).pack(side="left", padx=4, ipady=1)
        tk.Label(row, text="Value", bg=SURF, fg=TEXT_DIM,
                 font=FONT_UI_SML).pack(side="left")
        self._escp_val = tk.StringVar()
        tk.Entry(row, textvariable=self._escp_val, width=10, bg="#0d0d14",
                 fg=TEXT, font=FONT_MONO, relief="flat", insertbackground=TEXT,
                 highlightthickness=1, highlightbackground=BORDER,
                 highlightcolor=ACCENT).pack(side="left", padx=4, ipady=1)

        def escp_get():
            n = self._escp_name.get().strip()
            if n:
                self._send(f"can param get {n}")

        def escp_set():
            n = self._escp_name.get().strip()
            v = self._escp_val.get().strip()
            if n and v:
                self._send(f"can param set {n} {v}")

        # Get/Set/List on one row, Save/Restart on the next — five across a
        # narrow panel clips the rightmost buttons.
        row2 = tk.Frame(c, bg=SURF)
        row2.pack(fill="x", pady=(3, 0))
        self._sbtn(row2, "Get", escp_get, fg=ACCENT).pack(
            side="left", fill="x", expand=True, padx=2)
        self._sbtn(row2, "Set", escp_set, fg=GREEN).pack(
            side="left", fill="x", expand=True, padx=2)
        self._sbtn(row2, "List all", lambda: self._send("can param list"),
                   fg=CYAN).pack(side="left", fill="x", expand=True, padx=2)
        row3 = tk.Frame(c, bg=SURF)
        row3.pack(fill="x", pady=(3, 0))
        self._sbtn(row3, "Save NVM", lambda: self._send("can save"),
                   fg=YELLOW).pack(side="left", fill="x", expand=True, padx=2)
        self._sbtn(row3, "Restart ESC", lambda: self._send("can restart"),
                   fg=ORANGE).pack(side="left", fill="x", expand=True, padx=2)

    # ── Event log / terminal ─────────────────────────────────────────────────
    def _build_log(self, parent):
        c = card(parent, "Events / Terminal")
        c.master.pack(fill="both", expand=True, pady=(6, 0))
        self._term = scrolledtext.ScrolledText(
            c, bg="#0d0d14", fg=TEXT, font=("Consolas", 9), height=8,
            state="disabled", wrap="word", relief="flat", padx=4, pady=4)
        self._term.pack(fill="both", expand=True)
        self._term.tag_config("event", foreground=YELLOW)
        self._term.tag_config("fault", foreground=RED)
        self._term.tag_config("sent", foreground=CYAN)
        self._term.tag_config("data", foreground="#3a3f58")

        inp = tk.Frame(c, bg=SURF)
        inp.pack(fill="x", pady=(3, 0))
        self._cmd_var = tk.StringVar()
        e = tk.Entry(inp, textvariable=self._cmd_var, bg="#0d0d14", fg=TEXT,
                     font=FONT_MONO, relief="flat", insertbackground=TEXT,
                     highlightthickness=1, highlightbackground=BORDER,
                     highlightcolor=ACCENT)
        e.pack(side="left", fill="x", expand=True, ipady=2)
        e.bind("<Return>", self._send_typed)

    # ── Connection ────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _toggle_connect(self):
        if self._worker and self._worker.is_alive():
            self._worker.stop()
            self._worker = None
            self._conn_btn.config(text="Connect", fg=GREEN)
            self._status_lbl.config(text="●  Disconnected", fg=RED)
        else:
            port = self._port_var.get()
            if not port:
                messagebox.showwarning("No Port", "Select a COM port first.")
                return
            self._rx_q = queue.Queue()
            self._tx_q = queue.Queue()
            self._worker = SerialWorker(port, BAUD, self._rx_q, self._tx_q)
            self._worker.start()
            self._conn_btn.config(text="Disconnect", fg=RED)
            self._connect_t = time.time()
            self._data_seen = False
            # ESP32 resets on connect; after boot enable the stream + pull params
            self.after(3000, lambda: self._send("stream on"))
            self.after(3200, lambda: self._send("eng params"))
            # fallback: if no DATA has arrived a few seconds in, force it on
            self.after(6000, self._ensure_stream)

    def _ensure_stream(self):
        if self._worker and self._worker.is_alive() and not self._data_seen:
            self._log("no data yet — sending 'stream on'", "event")
            self._send("stream on")

    def _send(self, cmd):
        if self._worker and self._worker.is_alive():
            self._tx_q.put((cmd + "\n").encode())
            self._log(f"› {cmd}", "sent")
        else:
            self._log("not connected", "fault")

    def _send_typed(self, _=None):
        cmd = self._cmd_var.get().strip()
        if cmd:
            self._send(cmd)
            self._cmd_var.set("")

    # ── Control actions ───────────────────────────────────────────────────────
    def _toggle_arm(self):
        if self._armed:
            self._send("can disarm")
        else:
            if messagebox.askokcancel(
                    "ARM ESC",
                    "Arm the ESC?\n\nThe motor will respond to throttle "
                    "commands after arming. Clear the area."):
                self._send("can arm")

    def _toggle_pid(self):
        self._send("gov off" if self._gov_on else "gov on")

    def _send_gains(self):
        kc = self._gain_vars["gov_kc"].get().strip()
        ti = self._gain_vars["gov_ti"].get().strip()
        td = self._gain_vars["gov_td"].get().strip()
        if kc and ti and td:
            self._send(f"gov gains {kc} {ti} {td}")

    def _on_speed_move(self, v):
        pass  # live label only; send on release

    def _on_speed_press(self, _=None):
        self._speed_active = True

    def _on_speed_release(self, _=None):
        self._speed_active = False
        # keep the stream from snapping the knob back until the ECU echoes
        # the new value (a couple of stale DATA lines are always in flight)
        self._speed_guard = time.time() + 1.5
        if self._slider_inhibit:
            return
        v = self._speed_slider.get()
        if self._gov_on:
            self._send(f"gov sp {int(v)}")
        else:
            self._send(f"thr {v}")

    def _stop_motor(self):
        self._send("ramp off")
        self._send("gov off")
        self._send("thr 0")

    def _on_pump_release(self, idx):
        v = self._pump_sliders[idx].get()
        self._send(f"pump {idx+1} {v}")

    def _toggle_fuelcut(self):
        self._send("fuel cut off" if self._fuel_cut else "fuel cut on")

    def _auto_start(self):
        if messagebox.askokcancel(
                "AUTO START",
                "Run the automatic start sequence?\n\n"
                "glow → spool → fuel → light-off → warmup\n"
                "Check fuel lines, glow plug and clear area first."):
            self._send("eng start")

    # ── Parameters ────────────────────────────────────────────────────────────
    def _send_param(self, name):
        v = self._param_entries[name].get().strip()
        if v:
            self._send(f"eng set {name} {v}")

    def _send_all_params(self):
        for name, _, _ in PARAM_META:
            v = self._param_entries[name].get().strip()
            if v:
                self._send(f"eng set {name} {v}")

    def _save_to_ecu(self):
        # push current field values live, then persist them to ECU flash
        self._send_all_params()
        self.after(300, lambda: self._send("eng save"))

    def _save_local_params(self):
        data = {n: self._param_entries[n].get() for n, _, _ in PARAM_META
                if self._param_entries[n].get().strip()}
        try:
            with open(PARAMS_FILE, "w") as f:
                json.dump(data, f, indent=1)
            self._log(f"params saved to {PARAMS_FILE}", "event")
        except OSError as e:
            self._log(f"param save failed: {e}", "fault")

    def _load_local_params(self):
        try:
            with open(PARAMS_FILE) as f:
                self._params_saved = json.load(f)
        except (OSError, ValueError):
            self._params_saved = {}

    # ── Recording ────────────────────────────────────────────────────────────
    def _toggle_record(self):
        if self._recording:
            self._recording = False
            if self._csv_file:
                self._csv_file.close()
                self._csv_file = None
            self._rec_btn.config(text="● Record", fg=RED)
            self._log("recording stopped", "event")
        else:
            path = filedialog.asksaveasfilename(
                defaultextension=".csv",
                initialfile=time.strftime("engine_%Y%m%d_%H%M%S.csv"),
                filetypes=[("CSV", "*.csv")])
            if not path:
                return
            try:
                self._csv_file = open(path, "w", newline="")
            except OSError as e:
                self._log(f"cannot open log file: {e}", "fault")
                return
            self._csv_writer = csv.DictWriter(self._csv_file,
                                              fieldnames=CSV_FIELDS,
                                              extrasaction="ignore")
            self._csv_writer.writeheader()
            self._rec_t0 = time.time()
            self._recording = True
            self._rec_btn.config(text="■ Recording", fg=GREEN)
            self._log(f"recording to {path}", "event")

    def _record_row(self):
        if not (self._recording and self._csv_writer):
            return
        row = dict(self._latest)
        row["time"] = time.strftime("%H:%M:%S")
        row["elapsed_s"] = f"{time.time() - self._rec_t0:.2f}"
        self._csv_writer.writerow(row)

    # ── Incoming data ─────────────────────────────────────────────────────────
    def _poll(self):
        # Each item is processed in its own guard, and the reschedule lives in
        # finally — a single malformed line can never kill the ingestion loop
        # (which used to freeze the whole UI, charts included).
        try:
            while True:
                try:
                    kind, text = self._rx_q.get_nowait()
                except queue.Empty:
                    break
                try:
                    if kind == "line":
                        self._on_line(text)
                    elif kind == "status":
                        ok = "Connected" in text
                        self._status_lbl.config(text=f"●  {text}",
                                                fg=GREEN if ok else RED)
                    elif kind == "error":
                        self._log(f"serial error: {text}", "fault")
                except Exception as e:
                    self._log(f"parse error: {e}", "fault")
        finally:
            self.after(40, self._poll)

    def _on_line(self, line):
        if line == "HB":
            self._last_hb = time.time()
            return
        if line.startswith("DATA:"):
            self._last_hb = time.time()
            self._data_seen = True
            self._parse_data(line[5:])
            return
        if line.startswith("BOOT reset reason:"):
            # The ESP32 just (re)booted while we were connected — a mid-run
            # reset means it dropped out (brownout etc.), taking sensors/CAN
            # with it. Surface it loudly.
            tag = "fault" if "BROWNOUT" in line or "PANIC" in line else "event"
            self._log("⚠ ESP32 REBOOTED — " + line, tag)
            return
        if line.startswith("ENG:FAULT="):
            self._log(line, "fault")
            return
        if line.startswith("ENG:"):
            self._log(line, "event")
            return
        if line.startswith("PARAM:"):
            m = re.match(r"PARAM:(\w+)=([-\d.]+)", line)
            if m and m.group(1) in self._param_entries:
                name = m.group(1)
                # don't clobber a field the user is currently typing in
                if self.focus_get() is not self._param_widgets.get(name):
                    self._param_entries[name].set(f"{float(m.group(2)):g}")
            return
        if line.startswith("CAN:RX"):
            return  # too chatty for the engine log
        if line.startswith("PARAMESC:"):
            self._log(line, "event")   # highlight ESC param responses
            return
        self._log(line)

    def _parse_data(self, payload):
        parts = {}
        for p in payload.split(","):
            if "=" in p:
                k, v = p.split("=", 1)
                parts[k.strip()] = v.strip()

        def fget(key):
            try:
                return float(parts.get(key, ""))
            except ValueError:
                return float("nan")

        # thermocouples: TC1 TIT, TC2 glow, TC3 bearing, TC4 coil
        tc = []
        for i in range(1, 5):
            v = parts.get(f"TC{i}", "")
            try:
                tc.append(float(v))
            except ValueError:
                tc.append(float("nan"))
        rpm   = fget("ESC_RPM")
        esc_v = fget("ESC_V")
        esc_i = fget("ESC_I")
        esc_t = fget("ESC_T")
        power = esc_v * esc_i if esc_v == esc_v and esc_i == esc_i else float("nan")
        thr   = fget("THR")
        gsp   = fget("GSP")
        p1    = fget("P1")
        p2    = fget("P2")
        ff    = fget("FF")
        # Single Bourns SSA-2 differential shunt across GPIO34/35.
        bus_i  = fget("I_A")   # bus current (A), from OUTP-OUTN differential
        diffv  = fget("I_AV")  # differential volts (OUTP-OUTN)
        cmv    = fget("I_BV")  # common-mode volts (~1.44 V = sensor alive)

        # battery % from voltage using local param fields
        try:
            bmin = float(self._param_entries["batt_min"].get() or 21)
            bmax = float(self._param_entries["batt_max"].get() or 25.2)
        except ValueError:
            bmin, bmax = 21.0, 25.2
        batt = (esc_v - bmin) / (bmax - bmin) * 100 if esc_v == esc_v and bmax > bmin else float("nan")

        self._set_gauge("rpm", rpm, "{:.0f}")
        self._set_gauge("tit", tc[0], "{:.1f}", parts.get("TC1", ""))
        self._set_gauge("glow_t", tc[1], "{:.1f}", parts.get("TC2", ""))
        self._set_gauge("bearing", tc[2], "{:.1f}", parts.get("TC3", ""))
        self._set_gauge("coil", tc[3], "{:.1f}", parts.get("TC4", ""))
        self._set_gauge("esc_v", esc_v, "{:.2f}")
        self._set_gauge("esc_i", esc_i, "{:.2f}")
        self._set_gauge("esc_w", power, "{:.1f}")
        self._set_gauge("batt", batt, "{:.0f}")
        self._set_gauge("esc_t", esc_t, "{:.1f}")
        self._set_gauge("fuel", ff, "{:.1f}")
        self._set_gauge("cur34", bus_i, "{:.2f}")
        self._set_gauge("cur35", cmv, "{:.3f}")
        # under the bus-current gauge show the differential volts; under the CM
        # gauge note the ~1.44 V nominal. Both also feed the cal panel readouts.
        if diffv == diffv:
            self._gauges["cur34"][0].unit_lbl.config(text=f"A   ({diffv:+.3f} V)")
            self._cur_vlbl["34"].config(text=f"{diffv:+.3f} V")
            self._last_diffv = diffv
        if cmv == cmv:
            self._gauges["cur35"][0].unit_lbl.config(text="V  (~1.44 nom)")
            self._cur_vlbl["35"].config(text=f"{cmv:.3f} V")

        # Feed every chart from the signal registry (each chart plots whatever
        # signal its dropdown selects).
        self._sig = {
            "RPM": rpm, "ESC Power [W]": power, "TIT [°C]": tc[0],
            "EGT / Glow [°C]": tc[1], "Bearing [°C]": tc[2], "Coil [°C]": tc[3],
            "DC Voltage [V]": esc_v, "DC Current [A]": esc_i,
            "ESC Temp [°C]": esc_t, "Battery [%]": batt,
            "Bus Current [A]": bus_i, "Sensor CM [V]": cmv,
            "Fuel Flow [ml/min]": ff, "Throttle [%]": thr,
        }
        for chart, var in self._charts:
            chart.add(self._sig.get(var.get(), float("nan")))

        # Pixhawk page (guarded: a failure here must not kill the data loop)
        try:
            self._update_pixhawk(parts, fget)
        except Exception:
            pass

        # engine state
        st = parts.get("ENG", "")
        if st:
            self._eng_state = st
            self._state_lbl.config(text=st,
                                   fg=ENG_STATE_COLORS.get(st, TEXT))
            for name, lbl in self._seq_lbls.items():
                if name == st:
                    lbl.config(bg=ENG_STATE_COLORS.get(st, SURF2), fg="#0d0d14")
                else:
                    lbl.config(bg=SURF2, fg=TEXT_DIM)
        flt = parts.get("FLT", "NONE")
        self._fault_lbl.config(text="" if flt == "NONE" else f"FAULT: {flt}")

        glw = parts.get("GLW", "")
        if glw:
            self._glow_lamp.config(fg=ORANGE if glw == "1" else TEXT_DIM)

        for i, key in enumerate(("SOL1", "SOL2")):
            v = parts.get(key, "")
            if v:
                self._sol_lamps[i].config(fg=GREEN if v == "1" else TEXT_DIM)

        arm = parts.get("ARM", "")
        if arm:
            armed = arm == "1"
            if armed != self._armed:
                self._armed = armed
            self._arm_btn.config(
                text="ESC: ARMED" if armed else "ESC: DISARMED",
                fg="#0d0d14" if armed else TEXT_DIM,
                bg=RED if armed else SURF2)

        gov = parts.get("GOV", "")
        if gov:
            new_gov = gov == "1"
            if new_gov != self._gov_on:
                self._gov_on = new_gov
                self._sync_speed_slider_mode()
            self._pid_btn.config(
                text=f"PID CONTROL\n{'ON' if self._gov_on else 'OFF'}",
                fg=GREEN if self._gov_on else TEXT_DIM)
        if gsp == gsp:
            self._gsp_lbl.config(text=f"SP: {gsp:.0f} rpm")
        if thr == thr:
            self._thr_lbl.config(text=f"THR: {thr:.1f} %")
        if self._gov_on:
            if gsp == gsp:
                self._sync_slider(gsp)
        elif thr == thr:
            self._sync_slider(thr)

        fcut = parts.get("FCUT", "")
        if fcut:
            self._fuel_cut = fcut == "1"
            self._fuelcut_btn.config(
                text="FUEL SHUT OFF — ENGAGED" if self._fuel_cut else "FUEL SHUT OFF",
                bg="#7a1f2f" if self._fuel_cut else "#3d1420")

        rmp = parts.get("RMP", "")
        if rmp:
            self._ramp_lbl.config(text=f"ramp: {rmp}",
                                  fg=GREEN if rmp == "UP" else
                                  ORANGE if rmp == "DOWN" else
                                  YELLOW if rmp == "PAUSE" else TEXT_DIM)

        if p1 == p1:
            self._pump_lbls[0].config(text=f"{p1:.1f} %")
        if p2 == p2:
            self._pump_lbls[1].config(text=f"{p2:.1f} %")

        # stash for CSV
        self._latest = {
            "state": st, "fault": flt,
            "rpm": f"{rpm:.0f}" if rpm == rpm else "",
            "tit_c": parts.get("TC1", ""), "glow_c": parts.get("TC2", ""),
            "bearing_c": parts.get("TC3", ""), "coil_c": parts.get("TC4", ""),
            "esc_v": parts.get("ESC_V", ""), "esc_i": parts.get("ESC_I", ""),
            "esc_w": f"{power:.2f}" if power == power else "",
            "esc_temp_c": parts.get("ESC_T", ""),
            "throttle_pct": parts.get("THR", ""), "gov_on": parts.get("GOV", ""),
            "gov_sp": parts.get("GSP", ""), "pump1_pct": parts.get("P1", ""),
            "pump2_pct": parts.get("P2", ""), "fuel_mlmin": parts.get("FF", ""),
            "glow_on": parts.get("GLW", ""),
            "battery_pct": f"{batt:.1f}" if batt == batt else "",
            "bus_current_a": parts.get("I_A", ""), "sensor_cm_v": parts.get("I_BV", ""),
        }
        self._record_row()

    def _set_gauge(self, key, value, fmt, raw=""):
        grp, lbl = self._gauges[key]
        if value != value:   # NaN
            txt = raw.upper()[:7] if raw and not _is_float(raw) else "---"
            lbl.config(text=txt, fg=RED if txt not in ("---",) else grp.base_color)
            return
        # warning colors like the LabVIEW color boxes
        color = grp.base_color
        if grp.warn:
            try:
                limit = float(self._param_entries[grp.warn].get())
                if value >= limit:
                    color = RED
                elif value >= 0.9 * limit:
                    color = YELLOW
            except (ValueError, KeyError):
                pass
        lbl.config(text=fmt.format(value), fg=color)

    def _sync_slider(self, value):
        if self._speed_active or time.time() < self._speed_guard:
            return   # user owns the knob right now
        self._slider_inhibit = True
        try:
            self._speed_slider.set(value)
        finally:
            self._slider_inhibit = False

    def _sync_speed_slider_mode(self):
        self._slider_inhibit = True
        try:
            if self._gov_on:
                try:
                    mx = float(self._param_entries["max_rpm"].get() or 120000)
                except ValueError:
                    mx = 120000
                self._speed_slider.config(from_=mx, to=0, resolution=100)
                self._speed_mode_lbl.config(text="RPM setpoint", fg=GREEN)
            else:
                self._speed_slider.config(from_=100, to=0, resolution=1)
                self._speed_mode_lbl.config(text="throttle %", fg=CYAN)
        finally:
            self._slider_inhibit = False

    # ── Periodic ──────────────────────────────────────────────────────────────
    def _tick_1s(self):
        try:
            self._tick_1s_body()
        except Exception as e:
            self._log(f"tick error: {e}", "fault")
        finally:
            self.after(1000, self._tick_1s)

    def _tick_1s_body(self):
        for chart, _ in self._charts:
            try:
                chart.redraw()
            except Exception:
                pass   # one bad chart must not stop the others
        if self._connect_t and self._worker and self._worker.is_alive():
            self._elapsed_lbl.config(
                text=f"t = {int(time.time() - self._connect_t)} s")
        # Heartbeat watchdog — must restore "Connected" once data flows again,
        # otherwise the boot-window "No heartbeat!" sticks forever.
        if self._worker and self._worker.is_alive():
            alive = (time.time() - self._last_hb) < HB_TIMEOUT_S
            if alive:
                self._status_lbl.config(text="●  Connected", fg=GREEN)
            else:
                self._status_lbl.config(text="●  No heartbeat!", fg=YELLOW)

    def _log(self, text, tag=""):
        self._term.config(state="normal")
        ts = time.strftime("%H:%M:%S")
        self._term.insert("end", f"[{ts}] {text}\n", tag if tag else ())
        if float(self._term.index("end-1c").split(".")[0]) > 2000:
            self._term.delete("1.0", "400.0")
        self._term.see("end")
        self._term.config(state="disabled")

    def _on_close(self):
        if self._recording and self._csv_file:
            self._csv_file.close()
        if self._worker and self._worker.is_alive():
            self._worker.stop()
        self.destroy()


def _is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


if __name__ == "__main__":
    app = EngineDashboard()
    app.mainloop()
