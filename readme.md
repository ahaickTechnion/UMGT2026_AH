# UMGT ECU 2026

ESP32 DevKitC-32 + ESPSheild carrier board. Firmware lives in `UMGT/src/main.ino.cpp`
(PlatformIO, `pio run -e esp32dev`). Two GUIs:

- `ecu_dashboard.py` → `dist/ECU_Dashboard.exe` — low-level I/O test bench
- `engine_dashboard.py` → `dist/UMGT_Engine.exe` — **engine-run GUI**, ported
  from the LabVIEW `umgt_7thJuly2026.vi` test stand (see `LABVIEW_ANALYSIS.md`)

Build either exe: `pyinstaller --onefile --windowed --name <Name> <file>.py`.

All pin assignments below are verified against the KiCad netlist
(`UMGT/ESPSheild-KiCAD/ESPSheild.kicad_pcb`).

---

## ESP32 Pin Summary (verified)

| GPIO | Net | Function |
|------|-----|----------|
| GPIO1/3 | U0_TXD/U0_RXD | Serial 1 / USB console → EXE dashboard (J20) |
| GPIO2  | D2   | Extra I/O (J22) |
| GPIO4  | CRX  | **CAN RX** (SN65HVD230, J12) |
| GPIO5  | D5   | Extra I/O (J22) |
| GPIO12 | D12  | X9C digital pot **U/D** (J16) |
| GPIO13 | D13  | X9C digital pot **INC** (J16) |
| GPIO14 | D14  | Extra I/O (J22) |
| GPIO15 | D15  | Piezo / atomizer square wave (J19) |
| GPIO16 | U2_RXD | Serial 2 RX → Pixhawk (J21) |
| GPIO17 | U2_TXD | Serial 2 TX → Pixhawk (J21) |
| GPIO18 | SCK  | SPI clock (thermocouples) |
| GPIO19 | MISO | SPI MISO (thermocouples) |
| GPIO21 | SDA  | I2C data (PCF8575 expander @ 0x20) |
| GPIO22 | SCL  | I2C clock |
| GPIO23 | CTX  | **CAN TX** (no SPI MOSI on this board!) |
| GPIO25/26/27/32/33 | D25… | MOSFET PWM (J9, J10, J11, J7, J8) |
| GPIO34 | D34  | Analog: QNDB6 current sensor A (Battery) |
| GPIO35 | D35  | Analog: QNDB6 current sensor B (Load) |
| GPIO36/39 | VP/VN | Spare analog inputs (J24) |

## Thermocouples (MAX31855, SPI read-only)

Chip selects are **all on the PCF8575 expander** (active LOW).
Expander state bit = 8 + port number (P10 = bit 8 … P17 = bit 15).

| Connector | Net | Expander port | State bit | Channel |
|-----------|-----|---------------|-----------|---------|
| J3 | CS1 | P12 | 10 | TC1 — TIT |
| J4 | CS2 | P13 | 11 | TC2 — EGT |
| J5 | CS3 | P14 | 12 | TC3 — Bearing |
| J6 | CS4 | P15 | 13 | TC4 — Coil |

## Digital pot (X9C103S/X9C503S, J16)

| J16 pin | Signal | Driven by |
|---------|--------|-----------|
| 1 | GND | — |
| 2 | U/D | GPIO12 |
| 3 | INC | GPIO13 (wiper steps on falling edge) |
| 4 | CS  | Expander P16 (state bit 14, active LOW) |
| 5 | 3.3V | — |

Firmware resets the wiper to position 0 at boot and tracks it 0–99.

## MOSFET outputs (J7–J11, 2-pin GND+signal)

Verified wiring (2026-07-08):

| # | Connector | GPIO | Load |
|---|-----------|------|------|
| 1 | J7  | 32 | **Glow plug** (on/off) |
| 2 | J8  | 33 | **Solenoid 2** |
| 3 | J9  | 25 | **Solenoid 1** |
| 4 | J10 | 26 | **Fuel pump 2** (KNF 1.4-M) |
| 5 | J11 | 27 | **Fuel pump 1** (KNF 1.4-M) |

PWM: LEDC channels 2–6 (piezo has channel 0 on its own timer), duty 0–10,
shared frequency default 10 kHz.

## CAN / ESC (SN65HVD230, J12 / passthrough J18)

TWAI driver, TX=GPIO23, RX=GPIO4, default **1 Mbps** (DroneCAN standard).
The ESC is a **Hargrave microDRIVE LPi speaking DroneCAN (UAVCAN v0)**.
Firmware decodes:

- `uavcan.equipment.esc.Status` (1034, 10 Hz, multi-frame): RPM, bus voltage,
  bus current, bridge temperature, power %, plus Hargrave's custom 13-bit
  error field (over-temp, over/under-volt, sig loss, saturation, etc.)
- `uavcan.equipment.esc.StatusExtended` (1036, 1 Hz): input/output %,
  motor temperature, status flags

Decoded values stream as `ESC_RPM/ESC_V/ESC_I/ESC_T/ESC_PWR/ESC_ERR/ESC_AGE/
ESC_IN/ESC_OUT/ESC_MT` keys and fill the ESC banner in the exe. `can esc`
prints them on demand with error names spelled out.

ESC control (all repeated at 50 Hz as node 100, array padded to the ESC's
reported index; `can stop` ends any mode with an explicit zero):

- `can duty <-100..100>` — duty cycle via `esc.RawCommand` (1030). Negative
  duty needs the ESC's **Reversible** drive mode (Normal mode clamps to 0).
- `can rpm <setpoint>` — closed-loop RPM via `esc.RPMCommand` (1031); the ESC
  clamps to its configured min/max RPM (needs correct pole-pair setting).
- `can brake <0-100>` — regen braking as negative duty (Reversible mode);
  braking torque is capped by the ESC's bus/phase current-limit settings.

The commanded mode/value streams back as `ESC_CMODE`/`ESC_CVAL` so the exe
always shows what is being commanded. The codec (Status, StatusExtended RX;
RawCommand, RPMCommand TX, negatives included) was verified bit-exact against
the reference `dronecan` python library.

The ECU is also a **dynamic node ID allocation server** (the role an autopilot
normally plays): an ESC with no node ID sends anonymous allocation requests,
the ECU collects its 16-byte unique ID over the 3-stage exchange and grants it
an ID (its preference, else 125). The ECU broadcasts its own NodeStatus at
1 Hz as node 100. Decoded NodeStatus heartbeats and anonymous-request counts
stream as `NS_ID/NS_MODE/NS_HP/NS_UP` and `ANON`, and the exe Bus tile shows
the allocation state (NO ID → NODE n → ESC OK). The allocation wire format
(echo + multi-frame CRC grant) was verified against pydronecan.

## Current sensors (QNDB6 hall, 100A / 5V output)

GPIO34 = Battery, GPIO35 = Load. Default calibration 20 A/V, zero at 0 V
(`current cal <zeroV> <A_per_V>` to adjust). Note: the ESP32 ADC clips at
~3.3 V ≈ 66 A unless a divider is added.

## Expander extra I/O

P00–P07 (state bits 0–7) are broken out on J13/J17 "EXTRA IO EXPAND" and are
controllable with `i2c expander bit <n> on|off`.

---

## Serial command set (115200 baud on USB)

```
status                          full status dump
tc read <1-4|all>               thermocouples (1=TIT 2=EGT 3=Bearing 4=Coil)
tc scan                         probe all 16 expander CS bits (find wiring/CS issues)
tc mode 31855|6675              select thermocouple chip decode
current all|a|b                 current sensors in amps
current cal <zeroV> <A_per_V>   calibration
pot pos|up <n>|down <n>|set <0-99>|reset|store
can [status|esc]                bus status + decoded microDRIVE telemetry
can duty <-100..100>            duty command (RawCommand 1030, 50 Hz)
can rpm <setpoint>              closed-loop RPM (RPMCommand 1031, 50 Hz)
can brake <0-100>               regen brake = negative duty (Reversible mode)
can stop                        stop any ESC command, send zero
can param list                  enumerate all ESC settings (99 on microDRIVE LPi)
can param get|set <NAME> [v]    by name — CASE-SENSITIVE (e.g. BUS_CUR_LIM)
can param geti|seti <idx> [v]   by index (typed automatically after a read)
can save                        persist ESC params to NVM (microDRIVE reboots itself)
can restart                     reboot ESC (clears latched ERROR flags)
can baud <125|250|500|1000> | can send <id> <hex bytes> | can print on|off
mosfet all|<1-5> on|off|duty <0-10>   ·   mosfet freq <hz>
piezo on|off|duty <0-255>|freq <hz>
analog all|vp|vn|a|b            raw ADC + volts
i2c scan | i2c expander set <hex> | i2c expander bit <n> on|off
serial2 send <text>             forward to Pixhawk UART
serial2 baud <rate>             change Serial2 baud (Pixhawk telem is usually 57600)
serial2 hex on|off              hex-dump Serial2 traffic (MAVLink is binary)
stream on|off                   telemetry stream (default 10 Hz)
stream rate <1-50>              stream rate in Hz; TCs sample at a fixed 10 Hz
                                (MAX31855 converts internally every ~100 ms)
```

### Engine control (ported from LabVIEW + designed sequencer)

```
eng start                       auto sequence: PRECHECK→GLOW→SPOOL→IGNITION→WARMUP→RUNNING
eng stop                        normal shutdown: fuel+glow off → COOLDOWN spin → OFF
eng abort | eng reset           immediate kill (latched FAULT) / clear fault
eng manual                      manual mode (direct pump/glow/throttle, failsafes live)
eng status | eng params         status dump / list parameters (PARAM:name=value)
eng set <name> <value>          edit any sequence/limit/PID parameter live
gov on|off | gov sp <rpm>       RPM governor (NI PID Advanced form, out 15-100%)
gov gains <kc> <ti_min> <td_min>
thr <0-100>                     manual engine throttle (slew via governor off)
ramp up|down|pause|off          LabVIEW-style throttle ramp; ramp rate <pct/s>
pump <1|2> <0-100>              independent KNF fuel pumps, fine PWM; pump stop
glow on|off                     glow plug on MOSFET M1 (GPIO32)
sol <1|2> on|off                solenoid 1 (M3/GPIO25) / 2 (M2/GPIO33)
fuel cut on|off                 fuel shutoff latch (pumps forced to 0)
```

Failsafes (always active when anything is live, all editable):
TIT/coil/bearing over-temp, overspeed, flameout (lit + fuel + TIT below
floor), ESC telemetry loss, TIT thermocouple loss mid-sequence. Any trip →
everything safe + latched FAULT.

Engine stream keys: `ENG,FLT,GLW,P1,P2,FF,GOV,GSP,THR,FCUT,RMP`; state
transitions and faults print as `ENG:STATE=…` / `ENG:FAULT=…` events.

Actuators: glow=M1(32), sol2=M2(33), sol1=M3(25), pump2=M4(26), pump1=M5(27),
ESC=DroneCAN. Sensors: TC1=TIT, TC2=glow area, TC3=bearing, TC4=coil,
RPM/V/I from ESC.

Stream line format:

```
DATA:TC1=..,TC2=..,TC3=..,TC4=..,I_A=..,I_B=..,AVP=..,AVN=..,POT=..,PZF=..,PZD=..,CANRX=..,M1=..,..,M5=..
```

`HB` is emitted every second as a heartbeat; Pixhawk traffic is bridged to the
console as `[Serial2] …` lines.
