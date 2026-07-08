# LabVIEW `umgt_7thJuly2026.vi` — reverse-engineering notes

Extracted with pylabview from `UMGT/Labview/7thJuly2026/D/!!!Main - Alicat MFC
and pwmESC Folder_1/umgt_7thJuly2026.vi` (front panel heap, block diagram
heap, link info). This is the spec for the ECU engine firmware + engine exe.

## What the VI actually is

A **manual turbogenerator test-stand GUI** — parallel while-loops, no
automated start/stop state machine. The operator manually ramps the ESC,
enables fuel, and watches gauges. Sequencing/failsafes must be designed new
for the ECU (they do not exist in the LabVIEW).

## Front panel — engine-relevant controls (with LabVIEW labels)

Readouts:
- Turbine Inlet Temperature [K] — with color-box warning thresholds
- Near Glow plug Temperature [K] — color thresholds
- Bearing Temperature [degC]
- Generator Coil Temperature [degC] — color thresholds
- Shaft Speed [RPM] (from "MotorTone" tone-frequency measurement)
- DC Voltage [V], DC Current [A], DC Power [W], Battery % (color-coded,
  min/max scaling switched by the 1500 W / 300 W mode)
- Compressor Outlet Pressure [Pa] (transducer: P = 16000*V),
  Inlet Pressure Delta [Pa], Atmospheric Pressure, Bias Voltage
- Air Flow Rate [g/s], Propane Fuel Flow Rate [g/s], total fuel flow, p+k
- Equivalence ratios: Overall Eq Ratio, Equivalence Ratio propane,
  Required Eq Ratio Propane, Required Eq Ratio Kero,
  Required Propane Fuel Flow Rate, Propane Flow Rate Setpoint
- Elapsed Time
- Graphs: RPM, Generator Power, Pressure Ratio, Coil Temperature,
  Bronkhorst Flow rate, KNF Pump flow rate, Dispensed Volume (x2)

Controls:
- speed slider (manual ESC throttle), STOP MOTOR, Stop - Actual, STOP CODE
- Ramp Up / Pause / Ramp Down buttons, Ramp Rate [%/sec], Ramp time,
  Max RPM, Ramping mode indicator
- PID CONTROL toggle + PID gains cluster (Kc, Ti min, Td min) + "PID SETP"
- Fuel Shut Off, Track Fuel Flow Rate
- 1500 W / 300 W mode toggle
- Record, save file (CSV logging via Write Delimited Spreadsheet)
- Time to wait [ms] (loop pacing), HT/LT/full (PWM pulse times)
- Tank 3 / Tank 4 level sliders (syringe pumps), RUN/PAUSE pump buttons

Rig hardware being replaced/dropped in the ECU port:
- Alicat MFC (propane), Bronkhorst FLOW-BUS (kero flow), NE-50X syringe
  pumps, DAQmx AI (temps/pressures), DAQmx CO pulse (ESC PWM), SIMLAB PSU.

## Block diagram logic decoded

- **ESC drive**: Throttle % → "% to us" → 1500..2000 µs servo pulse
  ("2000us (full with)", HT/LT counter times), with a slew limiter
  ("limiting to avoid glitching"). ECU equivalent: DroneCAN duty command.
- **PID**: NI `PID Advanced.vi` — enabled by PID CONTROL; gains from the
  cluster; dt (s) from the loop; **output clamped 15%..100% throttle**;
  setpoint wire labeled "PID SETP"; PV is the measured shaft RPM
  (MotorTone), i.e. an RPM governor commanding ESC throttle.
- **Ramping mode**: case machine driven by Ramp Up/Pause/Ramp Down with
  Ramp Rate [%/s] / Ramp time / Max RPM.
- **Air mass flow**: formula node `mdot = 0.702*sqrt(Pin)` for 1500 W mode,
  0.54 for 300 W (comment: "change mdot from 0.702 for 1500W to 0.54 for
  300W"); Pin = compressor outlet pressure.
- **Fuel stoichiometry**: propane AFR constant 15.5 ("changed from 15.67 to
  15.5"); kero AFR separate; required fuel flow = f(target eq ratio, mdot).
  "Track Fuel Flow Rate" makes the fuel setpoint follow the requirement;
  **Fuel Shut Off forces the fuel flow setpoint to zero**.
- **Kero conversion**: ml/min → g/s via density 0.84 ("ml/min to g/s
  (0.84/60)").
- **Pressure cal**: `P = 16000*V`.
- **Warning colors**: case structures recolor TIT / glow-plug temp / coil
  temp / Battery % indicators against thresholds (thresholds live in the
  unparsed dataspace — re-specified as config in the ECU port).
- **Stop - Actual**: user dialog + loop stop. STOP MOTOR zeroes throttle.

## Mapping to the ECU (per Owen's instructions)

| LabVIEW | ECU |
|---|---|
| ESC PWM 1500-2000 µs + MotorTone RPM | DroneCAN duty + esc.Status RPM/V/I/temp |
| DC V/I/P, Battery % | ESC telemetry voltage/current, computed power |
| NE-50X + Bronkhorst + Alicat fuel | 2× KNF 1.4-M pumps on MOSFET PWM (M1, M2?) |
| Glow plug (external PSU, temp readout only) | Glow plug on/off on MOSFET M5 |
| DAQmx thermocouples | 4× MAX31855: TIT, glow-area (EGT), bearing, coil |
| Pressure transducers | not fitted (VP/VN spare analog if added later) |
| 2 solenoids | not in LabVIEW; ECU has M3/M4 spare |
| PID Advanced (RPM→throttle 15-100%) | Same PID on ESP32, gains from exe |
| Manual ops only | + designed auto start/shutdown/failsafe state machine |
