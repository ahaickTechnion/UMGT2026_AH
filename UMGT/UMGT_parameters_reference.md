# UMGT Engine Dashboard — Parameter Reference

Two completely separate sets of values reach this GUI, and mixing them up is the
single most common source of confusion:

| | **ECU internal values** | **ESC parameters** |
|---|---|---|
| Who owns them | the ESP32 (UMGT ECU) | the Hargrave microDRIVE LPi |
| How they arrive | `DATA:` stream, 10 Hz | DroneCAN request/response |
| Panel | *ECU Internal Values* | *ESC Parameters (DroneCAN)* |
| Read with | `Read` (from the last DATA frame) | `Get` → `can param get <NAME>` |
| Needs a working CAN bus | no | **yes** — dead while `CANRX=0` |

`THR` is an ECU value, not an ESC parameter. Asking the ESC for it over
DroneCAN gets no reply, because the ESC has never heard of it.

---

## 1. ECU internal values (the `DATA:` frame)

Sources: repo readme (pin table, MOSFET map, CAN section, engine stream keys)
and `_parse_data()` in `engine_dashboard.py`. Entries marked **(?)** are
inferred — the readme names the signal but not its exact semantics. Confirm
those against `UMGT/src/main.ino.cpp` before trusting them for anything
safety-related.

### Engine state machine

| Key | Unit | R/W | Meaning |
|---|---|---|---|
| `ENG` | — | R | Engine state: `OFF` / `PRECHECK` / `GLOW` / `SPOOL` / `IGNITION` / `WARMUP` / `RUNNING` / `COOLDOWN` / `FAULT` / `MANUAL`. Transitions also print as `ENG:STATE=…` |
| `FLT` | — | R | Latched fault code, `NONE` when clear. Set by the failsafes: TIT/coil/bearing over-temp, overspeed, flameout, ESC telemetry loss, TIT thermocouple loss. Clear with `eng reset` / RESET FAULT |
| `ARM` | — | R | ESC armed. `1` = the ESC will act on throttle. The microDRIVE refuses to spin without the arming broadcast |
| `FCUT` | — | R | Fuel-cut latch (`fuel cut on\|off`). `1` = both pumps forced to 0 regardless of demand |
| `RMP` | — | R | Throttle ramp machine: `OFF` / `UP` / `DOWN` / `PAUSE`, stepped at the ramp rate in %/s |

### Commanded outputs — writable

| Key | Unit | Write sends | Meaning |
|---|---|---|---|
| `THR` | % | `thr <v>` | Commanded engine throttle, 0–100 |
| `GOV` | — | `gov on` / `gov off` | RPM governor. `1` = the PID owns the throttle (output clamped 15–100 %) and `GSP` is the target |
| `GSP` | rpm | `gov sp <v>` | Governor RPM setpoint — only meaningful while `GOV=1` |
| `P1` | % | `pump 1 <v>` | Fuel pump 1 duty — KNF 1.4-M on M5 / GPIO27 |
| `P2` | % | `pump 2 <v>` | Fuel pump 2 duty — KNF 1.4-M on M4 / GPIO26 |
| `GLW` | — | `glow on` / `glow off` | Glow plug — M1 / GPIO32 |
| `SOL1` | % | `sol 1 on\|off` | Solenoid 1 — M3 / GPIO25 |
| `SOL2` | % | `sol 2 on\|off` | Solenoid 2 — M2 / GPIO33 |
| `FF` | ml/min | read-only | Fuel flow, derived from pump duty and the `pump1_cal` / `pump2_cal` parameters |

`THR`, `GSP`, `P1` and `P2` act on the engine immediately, so the GUI asks for
confirmation before sending.

### MOSFET raw duties

Duty is the raw LEDC value **0–10**, not a percentage. These are the same five
outputs as `GLW` / `SOL1` / `SOL2` / `P1` / `P2`, seen at the driver level.

| Key | Connector | GPIO | Load |
|---|---|---|---|
| `M1` | J7 | 32 | Glow plug |
| `M2` | J8 | 33 | Solenoid 2 |
| `M3` | J9 | 25 | Solenoid 1 |
| `M4` | J10 | 26 | Fuel pump 2 |
| `M5` | J11 | 27 | Fuel pump 1 |

In the frame you sent, `M3=10` — solenoid 1 sitting at full duty.

### Thermocouples (MAX31855, chip selects on the PCF8575 expander)

| Key | Connector | CS | Meaning |
|---|---|---|---|
| `TC1` | J3 | CS1 / P12 | Turbine inlet temperature (TIT), °C |
| `TC2` | J4 | CS2 / P13 | The readme's pin table calls this **EGT**, the engine section calls it **glow area** — same probe, two names |
| `TC3` | J5 | CS3 / P14 | Bearing temperature, °C |
| `TC4` | J6 | CS4 / P15 | Coil temperature, °C |

Instead of a number these read `nc` (no probe), `open` (open circuit) or
`sgnd` (shorted to ground). Your frame shows `TC1=nc`, `TC2=open`,
`TC3=open`, `TC4=sgnd` — no probes currently connected.

### ESC telemetry over DroneCAN

| Key | Unit | Source |
|---|---|---|
| `ESC_RPM` | rpm | `esc.Status` (1034, 10 Hz) |
| `ESC_V` | V | Bus voltage — also drives the battery % gauge |
| `ESC_I` | A | Bus current |
| `ESC_T` | °C | Bridge temperature |
| `ESC_PWR` | % | Power |
| `ESC_ERR` | — | Hargrave's custom 13-bit error field: over-temp, over/under-volt, signal loss, saturation… `can esc` spells the bits out |
| `ESC_AGE` | — | Age of the newest telemetry; rising means the stream stopped |
| `ESC_IN` | % | Input, from `esc.StatusExtended` (1036, 1 Hz) |
| `ESC_OUT` | % | Output, from StatusExtended |
| `ESC_MT` | °C | Motor temperature, from StatusExtended |
| `ESC_CMODE` | — | What the ECU is commanding: `DUTY` (RawCommand 1030), `RPM` (RPMCommand 1031), `BRAKE`, or `OFF` |
| `ESC_CVAL` | — | The value being sent in that mode, repeated at 50 Hz |
| `CANRX` | — | **Frames received from the ESC.** `0` = nothing arriving |

`CANRX=0` is the master switch for the whole ESC side: no telemetry, no
parameter access, no arming. Check bus wiring, 120 Ω termination at both ends,
bitrate (default 1 Mbps) and ESC power before anything else.

### DroneCAN node allocation

The ECU is the dynamic node-ID allocation server (node 100), the role an
autopilot normally plays.

| Key | Meaning |
|---|---|
| `NS_ID` / `NS_MODE` / `NS_HP` / `NS_UP` | Node ID, mode, health and uptime from a decoded NodeStatus heartbeat |
| `ANON` | Count of anonymous allocation requests — an ESC with no ID asking to be given one |

### Current sensors and analog inputs

| Key | Unit | Meaning |
|---|---|---|
| `I_A` | A | Current sensor **A — Battery**, GPIO34 |
| `I_B` | A | Current sensor **B — Load**, GPIO35 |
| `I_AV` **(?)** | V | Sensor A raw volts |
| `I_BV` **(?)** | V | Sensor B raw volts |
| `AVP` | V | Spare analog input **VP / GPIO36** (`analog vp`) |
| `AVN` | V | Spare analog input **VN / GPIO39** (`analog vn`) |

> **Discrepancy worth resolving.** The readme describes two independent QNDB6
> hall sensors on GPIO34/35 (20 A/V default, ADC clips near 66 A). The GUI's
> Current Sensor Cal panel and the comments in `_parse_data` instead describe a
> single Bourns SSA-2 differential shunt across the same two pins, where `I_AV`
> is the differential voltage and `I_BV` the common-mode (~1.44 V nominal =
> sensor alive). Both can't be right. Your frame shows `I_A` and `I_B`
> identical at −0.205 — consistent with the differential reading, not with two
> independent sensors. Worth confirming which board is fitted, because the
> calibration meaning of `current cal <zeroV> <A_per_V>` differs between them.

### Other on-board hardware

| Key | Unit | Meaning |
|---|---|---|
| `POT` | 0–99 | X9C digital pot wiper position. Reset to 0 at boot; stepped with `pot up\|down\|set` |
| `PZF` | Hz | Piezo / atomizer square-wave frequency — GPIO15 / J19 |
| `PZD` | 0–255 | Piezo / atomizer duty |
| `EXP` **(?)** | — | PCF8575 expander state. That expander also carries every thermocouple chip select, so a bad value here takes the TCs with it |
| `IFAIL` **(?)** | — | Peripheral failure counter, almost certainly the I2C/expander bus |
| `IREC` **(?)** | — | Peripheral recovery counter, paired with `IFAIL` |

> Your frame shows `IFAIL=510` with `IREC=0`. If that is an I2C error count
> that never recovered, it fits the four failed thermocouple reads —
> worth checking whether the expander at 0x20 answers `i2c scan`.

### Pixhawk / MAVLink bridge (Cube Orange, TELEM1 → J21 → Serial2)

| Key | Meaning |
|---|---|
| `PX_OK` | Heartbeat present. `1` = link up |
| `PX_SYS` / `PX_VER` | Autopilot system ID and MAVLink version |
| `PX_RAW` | **Raw bytes on the RX pin.** `0` = nothing wired or wrong baud (telemetry is usually 57600). Check this first |
| `PX_RX` | Messages successfully decoded |
| `PX_BAD` | Checksum failures — non-zero suggests baud mismatch or noise |
| `PX_TX` | Messages sent to the autopilot |
| `PX_HB` | Milliseconds since the last heartbeat |
| `PX_FIX` | GPS fix: 0 none, 1 no fix, 2 = 2D, 3 = 3D, 4 DGPS, 5 RTK float, 6 RTK fixed, 8 static |
| `PX_SATS` | Satellites used in the fix |
| `PX_HDOP` | Horizontal dilution of precision — under 2.0 is healthy |

---

## 2. Engine parameters (`eng params`)

These are the ECU's editable settings — the table on the Engine page, and also
readable and writable by name from the ECU Internal Values panel. They arrive
as `PARAM:name=value` lines in response to `eng params`.

- **Write** sends `eng set <name> <value>`, which changes live RAM only.
- **Save to ECU** then issues `eng save`, which persists to ESP32 flash.
- `eng defaults` clears them back to firmware defaults.

They cover the start sequence (glow temperature and time, spool duty/RPM and
timeout, ignition fuel start/ramp/max, light-off TIT, ignition timeout, warmup
idle RPM), the failsafe limits (TIT/coil/bearing maxima, max RPM, flameout TIT
floor, ESC loss timeout), cooldown behaviour, pump and fuel calibration,
governor PID gains and limits, ramp rate, and battery voltage endpoints. The
full list with labels and units is `PARAM_META` at the top of
`engine_dashboard.py`.

---

## 3. ESC parameters (DroneCAN)

**I can't give you a per-parameter table for these, and I'd be inventing it if
I tried.** The names live inside the Hargrave microDRIVE LPi's own firmware,
not in this repository — the readme only tells us there are about 99 of them
and gives `BUS_CUR_LIM` as an example. Nothing in `engine_dashboard.py`
enumerates them either; the GUI just passes names through to the ECU.

What is documented:

| Button | Command | Notes |
|---|---|---|
| Get | `can param get <NAME>` | Names are **CASE-SENSITIVE** — `BUS_CUR_LIM`, not `bus_cur_lim` |
| Set | `can param set <NAME> <value>` | Live only until saved |
| List all | `can param list` | Enumerates the real set — the only authoritative source |
| Save NVM | `can save` | Persists to the ESC; **the microDRIVE reboots itself afterwards** |
| Restart ESC | `can restart` | Reboots the ESC and clears latched ERROR flags |

The firmware also supports access by index — `can param geti|seti <idx>` —
which the ECU types automatically after a read.

Two constraints worth remembering: nothing in this panel works while
`CANRX=0`, and changes are lost on an ESC power-cycle unless you Save NVM.

**If you run List all and paste the output to me, I'll document every one of
them properly and add the descriptions to the GUI's help.**

---

## In-app help

Both panels now have a **?** button.

- *ECU Internal Values* — with a name in the box, `?` prints that key's
  description, unit and whether it's writable. With the box empty it prints
  the whole table for the keys in the current DATA frame. `Read` also prints a
  one-line reminder under the value.
- *ESC Parameters* — `?` prints how DroneCAN parameter access works and what
  each button sends.
