#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include "driver/twai.h"

// -----------------------------------------------------------------------------
// Pin map — verified against ESPSheild-KiCAD netlist (J1/J2 DevKitC-32 headers)
// -----------------------------------------------------------------------------
// U0 (Serial / USB)  → debug terminal, EXE connects here (J20 breakout, SERIAL 1)
// U2 (Serial2)       → Pixhawk telemetry (J21, SERIAL 2)
#define DEBUG_BAUD        115200
#define ECU_SERIAL_BAUD   115200

#define PIN_UART2_RX      16   // U2_RXD (J2 pad 10)
#define PIN_UART2_TX      17   // U2_TXD (J2 pad 9)

// CAN / TWAI → ESC via SN65HVD230 on J12 (3V3, GND, CTX, CRX, CANH, CANL)
// The ESC is a Hargrave microDRIVE LPi speaking DroneCAN (UAVCAN v0),
// standard bitrate 1 Mbps.
#define PIN_CAN_TX        23   // net CTX (J2 pad 1)
#define PIN_CAN_RX        4    // net CRX (J2 pad 11)
#define CAN_DEFAULT_KBPS  1000

// DroneCAN identifiers
#define DC_NODE_ID_SELF     100   // our node id when transmitting
#define DTID_ALLOCATION     1     // uavcan.protocol.dynamic_node_id.Allocation
#define SIG_ALLOCATION      0x0B2A812620A11D40ULL
#define DNA_DEFAULT_NODE_ID 125   // handed to the ESC if it has no preference
#define DTID_NODE_STATUS    341   // uavcan.protocol.NodeStatus (1 Hz heartbeat)
#define DTID_ESC_RAWCMD     1030  // uavcan.equipment.esc.RawCommand (duty)
#define DTID_ESC_RPMCMD     1031  // uavcan.equipment.esc.RPMCommand (closed loop)
#define DTID_ESC_STATUS     1034  // uavcan.equipment.esc.Status (10 Hz from ESC)
#define SIG_ESC_STATUS      0xA9AF28AEA2FBB254ULL
#define DTID_ESC_STATUS_EXT 1036  // uavcan.equipment.esc.StatusExtended (1 Hz)

// ESC telemetry older than this is dead — omit it from the stream so stale
// values can never masquerade as live readings on the dashboard.
#define ESC_TELEM_TIMEOUT_MS 5000

// SPI (thermocouples, read-only — no MOSI on this board, GPIO23 is CAN TX!)
#define PIN_SPI_SCK       18   // net SCK
#define PIN_SPI_MISO      19   // net MISO

// I2C (PCF8575 expander on J15)
#define PIN_I2C_SDA       21   // net SDA
#define PIN_I2C_SCL       22   // net SCL

// MOSFET PWM outputs (J7..J11: fuel pump, cooling pump, fuel sol, cool sol, glow)
#define PIN_MOSFET_A      32   // J7
#define PIN_MOSFET_B      33   // J8
#define PIN_MOSFET_C      25   // J9
#define PIN_MOSFET_D      26   // J10
#define PIN_MOSFET_E      27   // J11

// X9C digital pot on J16 (GND, U/D, INC, CS, VCC) — "VOLTAGE CONTROL BIT"
#define PIN_POT_UD        12   // net D12 → X9C U/D
#define PIN_POT_INC       13   // net D13 → X9C INC (wiper moves on falling edge)
#define POT_STEPS         100  // X9C103S/X9C503S: 100 wiper positions (0-99)

// Piezo / atomizer driver square wave on J19 — "FREQUENCY CONTROL PWM SQUARE"
#define PIN_PIEZO         15   // net D15

// Analog inputs (J24: VP, VN, D34, D35) — "ANALOG IN"
#define PIN_ANALOG_VP     36   // spare analog
#define PIN_ANALOG_VN     39   // spare analog
#define PIN_ANALOG_A      34   // QNDB6 hall current sensor (Battery)
#define PIN_ANALOG_B      35   // QNDB6 hall current sensor (Load)

// Spare GPIO on J22 "EXTRA IO": D5, D2, D14 (unused by firmware)

// -----------------------------------------------------------------------------
// PCF8575 expander (I2C 0x20). High byte = P17..P10 → state bits 15..8.
// J14 row wiring (from PCB netlist + module silkscreen):
//   P12 = CS1 = TC1 (TIT)     → bit 10
//   P13 = CS2 = TC2 (EGT)     → bit 11
//   P14 = CS3 = TC3 (Bearing) → bit 12
//   P15 = CS4 = TC4 (Coil)    → bit 13
//   P16 = CS5 = X9C pot CS    → bit 14
// P00..P07 (bits 0-7) are broken out on J13/J17 "EXTRA IO EXPAND".
// All CS lines are active LOW.
// -----------------------------------------------------------------------------
#define I2C_EXPANDER_ADDR     0x20
#define EXP_BIT_TC1       10
#define EXP_BIT_TC2       11
#define EXP_BIT_TC3       12
#define EXP_BIT_TC4       13
#define EXP_BIT_POT_CS    14

static const uint8_t tcExpBits[4] = { EXP_BIT_TC1, EXP_BIT_TC2, EXP_BIT_TC3, EXP_BIT_TC4 };
static const char   *tcNames[4]   = { "TIT", "EGT", "Bearing", "Coil" };

// -----------------------------------------------------------------------------
// PWM (LEDC). Channels 0/1 share timer0, 2/3 timer1, 4/5 timer2, 6/7 timer3.
// Piezo gets channel 0 (timer0 alone). MOSFETs share one frequency so they can
// share timers: channels 2..6. Never mix piezo and MOSFETs on the same timer.
// -----------------------------------------------------------------------------
#define PWM_CHANNEL        0
#define PWM_RESOLUTION     8
#define PWM_DEFAULT_FREQ   2000

#define MOSFET_PWM_FREQ    10000
#define MOSFET_PWM_RESOLUTION 8

// -----------------------------------------------------------------------------
// Current sensor calibration — QNDB6 hall sensor, 100A / 5V output variant.
// Output is 0V at 0A, 5V at 100A → 20 A per volt, zero offset 0V.
// NOTE: ESP32 ADC tops out ~3.3V, so readings clip above ~66A unless a divider
// is added. Adjust at runtime with: current cal <zero_volts> <amps_per_volt>
// -----------------------------------------------------------------------------
static float currentZeroVolts = 0.0f;
static float currentAmpsPerVolt = 20.0f;

// Telemetry rates. The MAX31855 converts internally every 70-100ms, so TCs are
// sampled at a fixed 10 Hz (faster polling returns duplicate conversions).
// The stream rate is independent and adjustable 1-50 Hz ("stream rate <hz>").
#define TC_SAMPLE_MS         100
#define STREAM_DEFAULT_HZ    10
static uint32_t streamIntervalMs = 1000 / STREAM_DEFAULT_HZ;

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------
static uint32_t mosfetFrequency = MOSFET_PWM_FREQ;

static const uint8_t mosfetPins[]     = { PIN_MOSFET_A, PIN_MOSFET_B, PIN_MOSFET_C, PIN_MOSFET_D, PIN_MOSFET_E };
static const uint8_t mosfetChannels[] = { 2, 3, 4, 5, 6 };
static const char   *mosfetNames[]    = { "FuelPump", "CoolPump", "FuelSol", "CoolSol", "GlowPlug" };
static uint8_t mosfetDuty[5] = {0};

uint32_t piezoFrequency = PWM_DEFAULT_FREQ;
uint8_t  piezoDuty      = 0;
uint16_t expanderState  = 0xFFFF;  // all HIGH = all CS deasserted
bool     expanderOk     = false;
uint8_t  expanderAddr   = I2C_EXPANDER_ADDR;

// I2C health tracking / self-healing. EMI events (ESC/MOSFET switching, ground
// bounce) can corrupt a transaction and leave the bus or the PCF8575 wedged —
// detect it, clock the bus free, and rewrite the expander state automatically.
static uint32_t      i2cFailCount    = 0;
static uint32_t      i2cRecoverCount = 0;
static unsigned long i2cLastRecoverMs = 0;
static bool          tcEverValid     = false;
#define I2C_CLOCK_HZ  100000   // 100 kHz: much better noise margin than 400 kHz
String   commandBuffer  = "";
// Stream on by default: the DevKit auto-resets whenever the EXE opens the COM
// port, so telemetry must flow without a handshake.
bool     streamEnabled  = true;

// TC decode mode: MAX31855 (32-bit, signed 14-bit) or MAX6675 (16-bit, unsigned 12-bit)
static bool tcMode6675 = false;

// Serial2 (Pixhawk) bridge — non-blocking, binary-safe
static uint8_t  s2Buf[64];
static uint8_t  s2Len = 0;
static uint32_t s2LastByteMs = 0;
static bool     s2HexMode = false;
static uint32_t serial2Baud = ECU_SERIAL_BAUD;

// Digital pot tracked wiper position (0..POT_STEPS-1). Reset to 0 at boot.
static int potPosition = 0;

// CAN state
static bool     canOk       = false;
static uint32_t canKbps     = CAN_DEFAULT_KBPS;
static uint32_t canRxCount  = 0;
static uint32_t canTxCount  = 0;
static bool     canPrintRx  = false;  // DroneCAN is chatty; enable for debug only

// Decoded ESC telemetry (Hargrave microDRIVE via DroneCAN)
struct EscTelemetry {
  bool     seen;
  uint32_t lastMs;
  uint8_t  srcNode;
  uint32_t errorFlags;   // Hargrave custom bitfield in the error_count slot
  float    voltage;      // V
  float    current;      // A
  float    tempC;        // bridge temperature, converted from Kelvin
  int32_t  rpm;
  uint8_t  powerPct;
  uint8_t  escIndex;
  // From StatusExtended (1036)
  bool     extSeen;
  uint32_t extLastMs;
  uint8_t  inputPct;
  uint8_t  outputPct;
  int16_t  motorTempC;
  uint32_t statusFlags;
};
static EscTelemetry esc = {};

// Hargrave error_count bitfield names (bits 0-12), from the microDRIVE docs
static const char *escErrorNames[13] = {
  "OVER_TEMP", "BUS_OVERCURRENT", "PHASE_OVERCURRENT", "OVER_VOLT", "UNDER_VOLT",
  "RIPPLE", "SIGNAL_LOSS", "MOTOR_SATURATED", "MOTOR_OVER_TEMP", "RPM_LIMIT",
  "ERROR_ACTIVE", "OUTPUT_SHORTED", "STARTUP_CHECK_FAIL"
};

// Latest NodeStatus heartbeat on the bus (any node — normally just the ESC)
static struct {
  bool     seen;
  uint8_t  src;
  uint32_t uptimeSec;
  uint8_t  health;   // 0 OK, 1 WARNING, 2 ERROR, 3 CRITICAL
  uint8_t  mode;     // 0 OPERATIONAL, 1 INIT, 2 MAINTENANCE, 3 SW_UPDATE, 7 OFFLINE
  uint32_t lastMs;
} nodeHb = {};

// Anonymous frames (source node id 0) = a node begging for dynamic node ID
// allocation. If this counts up, the ESC has NO node id and will never send
// telemetry until an allocator (autopilot / GUI tool) assigns one.
static uint32_t anonFrameCount = 0;

// Dynamic node ID allocation server state. We are the only bus master, so the
// ECU plays allocator: collect the requester's 16-byte unique ID over up to
// three anonymous messages, then grant it a node ID.
static struct {
  uint8_t       uid[16];
  uint8_t       len;
  unsigned long lastMs;
  uint8_t       lastGrantedId;
  uint32_t      grants;
} dna = {};

static uint8_t allocTxTid = 0;   // transfer id counters per broadcast type
static uint8_t nsTxTid    = 0;

static const char *nodeHealthNames[] = { "OK", "WARNING", "ERROR", "CRITICAL" };
static const char *nodeModeName(uint8_t m) {
  switch (m) {
    case 0: return "OPERATIONAL";
    case 1: return "INITIALIZING";
    case 2: return "MAINTENANCE";
    case 3: return "SW_UPDATE";
    case 7: return "OFFLINE";
    default: return "UNKNOWN";
  }
}

// Multi-frame reassembly for esc.Status (110 bits = 14 bytes = 3 CAN frames)
static struct {
  bool     active;
  uint8_t  src, tid, toggleExpect, len;
  uint16_t crc;      // transfer CRC from the first frame — validated at the end
  uint8_t  buf[20];
} dcAsm = {};
static uint32_t escCrcErrors = 0;   // corrupted transfers rejected

// ESC command TX, repeated at 50 Hz while active (ESCs failsafe if it stops).
// DUTY/BRAKE use esc.RawCommand (1030); RPM uses esc.RPMCommand (1031).
// BRAKE = negative duty: in the microDRIVE's Reversible mode this regen-brakes
// with torque limited by the ESC's configured current limits.
enum EscCmdMode : uint8_t { ESC_CMD_OFF = 0, ESC_CMD_DUTY, ESC_CMD_RPM, ESC_CMD_BRAKE };
static EscCmdMode    escCmdMode  = ESC_CMD_OFF;
static int32_t       escCmdValue = 0;       // raw -8192..8191 for duty/brake, rpm for RPM
static int32_t       escCmdUser  = 0;       // user-facing value (% or rpm) for display
static uint8_t       escTxTransferId = 0;
static unsigned long escCmdLastMs = 0;

static const char *escCmdModeNames[] = { "OFF", "DUTY", "RPM", "BRK" };

// Last read thermocouple temps (NAN = not read / fault) + raw words for faults
static float    tcTemp[4] = { NAN, NAN, NAN, NAN };
static uint32_t tcRaw[4]  = { 0, 0, 0, 0 };

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
void initializePins();
void initializeSerial();
void initializeBuses();
void initializeCan();
void initializeTestController();
void ecuHeartbeat();
void sensorTask();

void processSerialCommands();
void processSerial2Bridge();
void processCanRx();
void handleLineCommand(const String &command);
String getNextToken(String &line, int &pos);

void printHelp();
void printStatus();

void handleMosfet(String &command, int &pos);
void handlePiezo(String &command, int &pos);
void handleAnalog(String &command, int &pos);
void handleThermocouple(String &command, int &pos);
void handleCurrent(String &command, int &pos);
void handleStream(String &command, int &pos);
void handleI2c(String &command, int &pos);
void handleSerial2(String &command, int &pos);
void handlePot(String &command, int &pos);
void handleCan(String &command, int &pos);

void setMosfetDuty(uint8_t index, uint8_t duty);
void setAllMosfets(uint8_t duty);
void setMosfetFrequency(uint32_t frequency);
void setPiezoFrequency(uint32_t frequency);
void setPiezoDuty(uint8_t duty);

uint32_t readMAX31855(uint8_t expanderBit);
float    decodeMAX31855Celsius(uint32_t raw);
float    decodeMAX6675Celsius(uint32_t raw);
float    decodeTc(uint32_t raw);
const char *tcFaultName(uint32_t raw);
float    tcInternalCelsius(uint32_t raw);
void     printTcLine(uint8_t i);
void     readAllThermocouples();
void     tcScan();

void potMove(int steps);          // >0 up, <0 down
void potSet(int target);
void potStore();
void potReset();

bool canStart(uint32_t kbps);
void canStop();
bool canSendFrame(uint32_t id, const uint8_t *data, uint8_t len);

uint32_t dcBits(const uint8_t *buf, uint32_t bitOff, uint8_t bitLen);
int32_t  dcBitsSigned(const uint8_t *buf, uint32_t bitOff, uint8_t bitLen);
void     dcEncodeBits(uint8_t *dst, uint32_t bitOff, uint8_t bitLen, uint32_t value);
float    half2float(uint16_t h);
void     dcHandleFrame(uint32_t id, const uint8_t *data, uint8_t dlc);
void     dcDecodeEscStatus(const uint8_t *buf, uint8_t len, uint8_t src);
void     dcDecodeEscStatusExt(const uint8_t *buf, uint8_t len, uint8_t src);
bool     dcSendRawCommand(int16_t value);
bool     dcSendRpmCommand(int32_t rpm);
void     escCommandStop();
void     canThrottleTask();
void     printEscTelemetry();
bool     dcBroadcast(uint16_t dtid, uint64_t signature, uint8_t priority,
                     uint8_t *tidCounter, const uint8_t *payload, uint8_t len);
void     dnaHandleRequest(const uint8_t *data, uint8_t dlc);
void     nodeStatusTask();

float    analogToVolts(uint16_t raw);
float    voltsToCurrentAmps(float volts);
float    readCurrentAmps(uint8_t pin);

void printAnalogValue(uint8_t pin, const char *name);
void sendStreamData();

void i2cScan();
bool expanderWrite(uint16_t state);
void expanderDetect();
void i2cBusClear();
bool expanderRecover();

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  initializeSerial();
  initializePins();
  initializeBuses();
  initializeCan();
  initializeTestController();

  Serial.println("ECU boot complete");
  Serial.printf("ESP32 WROOM-32 ECU starting, debug %u baud\n", DEBUG_BAUD);
  printHelp();
  Serial.print("> ");
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop() {
  sensorTask();
  ecuHeartbeat();
  processSerialCommands();
  processSerial2Bridge();
  processCanRx();
  canThrottleTask();
  nodeStatusTask();
  delay(1);
}

// Continuous thermocouple sampling at the MAX31855's conversion rate.
// The stream sender just reports the latest cached values, so raising the
// stream rate never adds SPI/I2C traffic.
void sensorTask() {
  static unsigned long lastTcSample = 0;
  unsigned long now = millis();
  if (now - lastTcSample >= TC_SAMPLE_MS) {
    lastTcSample = now;
    readAllThermocouples();

    // Self-healing: recover the I2C bus if the expander stopped ACKing, or if
    // every channel suddenly returns no SPI data after having worked before
    // (a wedged expander leaves all CS lines stuck, killing all 4 at once).
    bool allDead = true;
    bool anyValid = false;
    for (uint8_t i = 0; i < 4; i++) {
      if (tcRaw[i] != 0x00000000 && tcRaw[i] != 0xFFFFFFFF) allDead = false;
      if (!isnan(tcTemp[i])) anyValid = true;
    }
    if (anyValid) tcEverValid = true;
    if ((!expanderOk || (allDead && tcEverValid)) &&
        now - i2cLastRecoverMs > 1000) {
      i2cLastRecoverMs = now;
      expanderRecover();
    }
  }
}

// -----------------------------------------------------------------------------
// Serial command processing
// -----------------------------------------------------------------------------
void processSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      Serial.println();
      if (commandBuffer.length() > 0) {
        handleLineCommand(commandBuffer);
        commandBuffer = "";
      }
      Serial.print("> ");
      continue;
    }
    if (c == 8 || c == 127) {
      if (commandBuffer.length() > 0) {
        commandBuffer.remove(commandBuffer.length() - 1);
        Serial.print("\b \b");
      }
      continue;
    }
    if (c >= 32 && c <= 126) {
      commandBuffer += c;
      Serial.print(c);
    }
  }
}

void handleLineCommand(const String &command) {
  String line = command;
  line.trim();
  if (line.length() == 0) return;

  Serial.print("> ");
  Serial.println(line);

  String working = line;
  working.toLowerCase();
  int pos = 0;
  String token = getNextToken(working, pos);

  if (token == "help" || token == "?") {
    printHelp();
  } else if (token == "status") {
    printStatus();
  } else if (token == "mosfet") {
    handleMosfet(working, pos);
  } else if (token == "piezo" || token == "pwm") {
    handlePiezo(working, pos);
  } else if (token == "analog" || token == "adc") {
    handleAnalog(working, pos);
  } else if (token == "tc" || token == "thermocouple" || token == "spi") {
    handleThermocouple(working, pos);
  } else if (token == "current" || token == "amps") {
    handleCurrent(working, pos);
  } else if (token == "pot") {
    handlePot(working, pos);
  } else if (token == "stream") {
    handleStream(working, pos);
  } else if (token == "can") {
    handleCan(working, pos);
  } else if (token == "i2c") {
    handleI2c(working, pos);
  } else if (token == "serial2") {
    handleSerial2(working, pos);
  } else {
    Serial.println("Unknown command. Type help for a command list.");
  }
}

// Non-blocking Serial2 bridge. Pixhawk MAVLink is binary, so:
//  - never block waiting for a newline (that stalls the whole control loop)
//  - sanitize non-printable bytes (or hex-dump with "serial2 hex on")
//  - flush on newline, full buffer, or 50ms idle so every chunk keeps its prefix
static void s2Flush() {
  if (s2Len == 0) return;
  Serial.print("[Serial2] ");
  if (s2HexMode) {
    for (uint8_t i = 0; i < s2Len; i++) Serial.printf("%02X ", s2Buf[i]);
  } else {
    for (uint8_t i = 0; i < s2Len; i++) {
      char c = (char)s2Buf[i];
      Serial.print((c >= 32 && c <= 126) ? c : '.');
    }
  }
  Serial.println();
  s2Len = 0;
}

void processSerial2Bridge() {
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    s2LastByteMs = millis();
    if (b == '\n') { s2Flush(); continue; }
    if (b == '\r') continue;
    s2Buf[s2Len++] = b;
    if (s2Len >= sizeof(s2Buf)) s2Flush();
  }
  if (s2Len > 0 && millis() - s2LastByteMs > 50) s2Flush();
}

String getNextToken(String &line, int &pos) {
  line.trim();
  if (pos >= (int)line.length()) return String();
  int next = line.indexOf(' ', pos);
  if (next < 0) {
    String token = line.substring(pos);
    pos = line.length();
    token.trim();
    return token;
  }
  String token = line.substring(pos, next);
  pos = next + 1;
  token.trim();
  return token;
}

// -----------------------------------------------------------------------------
// Help / Status
// -----------------------------------------------------------------------------
void printHelp() {
  Serial.println("--- ESP32 ECU Terminal ---");
  Serial.println("help | ?                      - Show this help");
  Serial.println("status                        - Show current pin/sensor status");
  Serial.println("tc read <1-4>                 - Read one thermocouple (1=TIT 2=EGT 3=Bearing 4=Coil)");
  Serial.println("tc read all                   - Read all 4 thermocouples");
  Serial.println("tc scan                       - Probe all 16 expander CS bits (diagnostics)");
  Serial.println("tc mode 31855|6675            - Select thermocouple chip decode");
  Serial.println("current all                   - Read both current sensors (A)");
  Serial.println("current a|b                   - Read one current sensor (a=Battery/34, b=Load/35)");
  Serial.println("current cal <zeroV> <A_per_V> - Set current sensor calibration");
  Serial.println("pot pos                       - Show tracked digital pot position (0-99)");
  Serial.println("pot up|down <n>               - Step digital pot wiper up/down");
  Serial.println("pot set <0-99>                - Move wiper to absolute position");
  Serial.println("pot reset                     - Force wiper to 0 (100 down-steps)");
  Serial.println("pot store                     - Store wiper position to X9C NVM");
  Serial.println("can                           - CAN status + decoded ESC telemetry");
  Serial.println("can esc                       - Show microDRIVE ESC telemetry (DroneCAN)");
  Serial.println("can duty <-100..100>          - Duty cycle command (RawCommand, 50 Hz)");
  Serial.println("can rpm <setpoint>            - Closed-loop RPM command (RPMCommand, 50 Hz)");
  Serial.println("can brake <0-100>             - Regen brake = negative duty (Reversible mode)");
  Serial.println("can stop                      - Stop any ESC command, send zero");
  Serial.println("can baud <125|250|500|1000>   - Restart CAN at new bitrate (DroneCAN=1000)");
  Serial.println("can send <id> <b0> [b1..b7]   - Send raw CAN frame (hex id + hex bytes)");
  Serial.println("can print on|off              - Toggle live printing of raw RX frames");
  Serial.println("stream on|off                 - Enable/disable auto data stream");
  Serial.println("stream rate <1-50>            - Set stream rate in Hz (TCs max 10 Hz)");
  Serial.println("mosfet all duty <0-10>        - Set all MOSFET PWM level");
  Serial.println("mosfet <1-5> duty <0-10>      - Set one MOSFET PWM level");
  Serial.println("mosfet freq <hz>              - Set MOSFET PWM frequency 1-20000");
  Serial.println("mosfet all on|off             - Toggle all MOSFET outputs");
  Serial.println("mosfet <1-5> on|off           - 1=FuelPump 2=CoolPump 3=FuelSol 4=CoolSol 5=Glow");
  Serial.println("piezo on|off                  - Enable or disable atomizer square wave");
  Serial.println("piezo duty <0-255>            - Set piezo PWM duty");
  Serial.println("piezo freq <hz>               - Set piezo PWM frequency");
  Serial.println("analog all                    - Read all analog inputs (raw+volts)");
  Serial.println("analog vp|vn|a|b              - Read one analog input");
  Serial.println("i2c scan                      - Scan I2C bus for devices");
  Serial.println("i2c expander set <hex>        - Write raw state to I2C expander");
  Serial.println("i2c expander bit <n> on|off   - Toggle one expander bit");
  Serial.println("serial2 send <text>           - Send raw text to Serial2 (Pixhawk)");
  Serial.println("serial2 baud <rate>           - Change Serial2 baud (Pixhawk telem = 57600)");
  Serial.println("serial2 hex on|off            - Hex-dump Serial2 traffic (MAVLink is binary)");
}

void printStatus() {
  Serial.println("--- Status ---");

  Serial.print("Piezo: ");
  Serial.print(piezoFrequency);
  Serial.print(" Hz, duty=");
  Serial.println(piezoDuty);

  Serial.print("MOSFET freq: ");
  Serial.print(mosfetFrequency);
  Serial.println(" Hz");

  Serial.print("MOSFETs: ");
  for (uint8_t i = 0; i < 5; i++) {
    Serial.printf("%s:%u", mosfetNames[i], mosfetDuty[i]);
    if (i < 4) Serial.print(", ");
  }
  Serial.println();

  Serial.printf("Digital pot: position %d/%d\n", potPosition, POT_STEPS - 1);

  Serial.printf("Expander (0x%02X): %s, state=0x%04X\n",
                expanderAddr, expanderOk ? "OK" : "NOT RESPONDING", expanderState);
  Serial.printf("I2C health: %u failed writes, %u recoveries\n",
                i2cFailCount, i2cRecoverCount);

  Serial.printf("TC decode mode: %s\n", tcMode6675 ? "MAX6675" : "MAX31855");
  Serial.printf("Serial2: %u baud, hex=%s\n", serial2Baud, s2HexMode ? "on" : "off");

  Serial.printf("CAN: %s, %u kbps, RX=%u TX=%u\n",
                canOk ? "UP" : "DOWN", canKbps, canRxCount, canTxCount);

  // Thermocouples
  Serial.println("Thermocouples:");
  readAllThermocouples();
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print("  ");
    printTcLine(i);
  }

  // Current sensors
  Serial.printf("Current A (Battery/34): %.3f A\n", readCurrentAmps(PIN_ANALOG_A));
  Serial.printf("Current B (Load/35):    %.3f A\n", readCurrentAmps(PIN_ANALOG_B));
  Serial.printf("Current cal: zero=%.3fV, %.2f A/V\n", currentZeroVolts, currentAmpsPerVolt);

  Serial.printf("Stream: %s at %u Hz (TC sampling fixed at %u Hz)\n",
                streamEnabled ? "ON" : "OFF", 1000 / streamIntervalMs,
                1000 / TC_SAMPLE_MS);
}

// -----------------------------------------------------------------------------
// Thermocouple handler
// -----------------------------------------------------------------------------
void handleThermocouple(String &command, int &pos) {
  String action = getNextToken(command, pos);

  if (action == "scan") {
    tcScan();
    return;
  }
  if (action == "mode") {
    String sub = getNextToken(command, pos);
    if (sub == "6675")  { tcMode6675 = true;  Serial.println("TC decode mode: MAX6675");  return; }
    if (sub == "31855") { tcMode6675 = false; Serial.println("TC decode mode: MAX31855"); return; }
    Serial.printf("TC decode mode: %s (usage: tc mode 31855|6675)\n",
                  tcMode6675 ? "MAX6675" : "MAX31855");
    return;
  }
  if (action != "read") {
    Serial.println("Usage: tc read <1-4|all> | tc scan | tc mode 31855|6675");
    return;
  }

  String which = getNextToken(command, pos);

  if (which == "all") {
    readAllThermocouples();
    for (uint8_t i = 0; i < 4; i++) printTcLine(i);
    return;
  }

  int idx = which.toInt();
  if (idx >= 1 && idx <= 4) {
    uint32_t raw = readMAX31855(tcExpBits[idx - 1]);
    tcRaw[idx - 1]  = raw;
    tcTemp[idx - 1] = decodeTc(raw);
    Serial.printf("raw: 0x%08X  ", raw);
    printTcLine(idx - 1);
    return;
  }

  Serial.println("Usage: tc read <1-4> | tc read all");
}

// -----------------------------------------------------------------------------
// Current sensor handler
// -----------------------------------------------------------------------------
void handleCurrent(String &command, int &pos) {
  String input = getNextToken(command, pos);

  if (input == "all") {
    Serial.printf("A (Battery/34): %.3f A\n", readCurrentAmps(PIN_ANALOG_A));
    Serial.printf("B (Load/35):    %.3f A\n", readCurrentAmps(PIN_ANALOG_B));
    return;
  }
  if (input == "a") {
    Serial.printf("A (Battery/34): %.3f A\n", readCurrentAmps(PIN_ANALOG_A));
    return;
  }
  if (input == "b") {
    Serial.printf("B (Load/35): %.3f A\n", readCurrentAmps(PIN_ANALOG_B));
    return;
  }
  if (input == "cal") {
    String zeroTok = getNextToken(command, pos);
    String scaleTok = getNextToken(command, pos);
    if (zeroTok.length() > 0 && scaleTok.length() > 0) {
      currentZeroVolts   = zeroTok.toFloat();
      currentAmpsPerVolt = scaleTok.toFloat();
      Serial.printf("Current cal set: zero=%.3fV, %.2f A/V\n",
                    currentZeroVolts, currentAmpsPerVolt);
      return;
    }
  }

  Serial.println("Usage: current all | current a|b | current cal <zeroV> <A_per_V>");
}

// -----------------------------------------------------------------------------
// Digital pot handler (X9C on J16: U/D=GPIO12, INC=GPIO13, CS=expander P16)
// -----------------------------------------------------------------------------
void handlePot(String &command, int &pos) {
  String action = getNextToken(command, pos);

  if (action == "pos" || action.length() == 0) {
    Serial.printf("Pot position: %d/%d\n", potPosition, POT_STEPS - 1);
    return;
  }
  if (action == "up" || action == "down") {
    int n = getNextToken(command, pos).toInt();
    if (n <= 0) n = 1;
    potMove(action == "up" ? n : -n);
    Serial.printf("Pot position: %d/%d\n", potPosition, POT_STEPS - 1);
    return;
  }
  if (action == "set") {
    String value = getNextToken(command, pos);
    if (value.length() > 0) {
      potSet(constrain(value.toInt(), 0, POT_STEPS - 1));
      Serial.printf("Pot position: %d/%d\n", potPosition, POT_STEPS - 1);
      return;
    }
  }
  if (action == "reset") {
    potReset();
    Serial.println("Pot reset to 0");
    return;
  }
  if (action == "store") {
    potStore();
    Serial.printf("Pot position %d stored to NVM\n", potPosition);
    return;
  }

  Serial.println("Usage: pot pos | pot up|down <n> | pot set <0-99> | pot reset | pot store");
}

// -----------------------------------------------------------------------------
// CAN handler
// -----------------------------------------------------------------------------
void handleCan(String &command, int &pos) {
  String action = getNextToken(command, pos);

  if (action.length() == 0 || action == "status") {
    twai_status_info_t info;
    Serial.printf("CAN: %s, %u kbps, RX=%u TX=%u\n",
                  canOk ? "UP" : "DOWN", canKbps, canRxCount, canTxCount);
    if (canOk && twai_get_status_info(&info) == ESP_OK) {
      const char *state =
        info.state == TWAI_STATE_RUNNING     ? "RUNNING" :
        info.state == TWAI_STATE_BUS_OFF     ? "BUS_OFF" :
        info.state == TWAI_STATE_RECOVERING  ? "RECOVERING" : "STOPPED";
      Serial.printf("  state=%s, rx_q=%u, tx_err=%u, rx_err=%u, bus_err=%u, telem_crc_rejects=%u\n",
                    state, info.msgs_to_rx, info.tx_error_counter,
                    info.rx_error_counter, info.bus_error_count, escCrcErrors);
    }
    printEscTelemetry();
    return;
  }

  if (action == "esc") {
    printEscTelemetry();
    return;
  }

  if (action == "stop") {
    escCommandStop();
    Serial.println("ESC command OFF (zero sent)");
    return;
  }

  if (action == "throttle" || action == "duty") {
    String value = getNextToken(command, pos);
    if (value == "off") {
      escCommandStop();
      Serial.println("ESC command OFF (zero sent)");
      return;
    }
    if (value.length() > 0) {
      // Negative duty needs the ESC in Reversible mode (Normal mode clamps to 0)
      int pct = constrain(value.toInt(), -100, 100);
      escCmdUser = pct;
      escCmdValue = (int32_t)pct * 8191 / 100;
      escCmdMode = ESC_CMD_DUTY;
      Serial.printf("ESC duty %d%% (raw %d), repeating at 50 Hz — 'can stop' to stop\n",
                    pct, escCmdValue);
      return;
    }
    Serial.println("Usage: can duty <-100..100> | can stop");
    return;
  }

  if (action == "rpm") {
    String value = getNextToken(command, pos);
    if (value == "off") {
      escCommandStop();
      Serial.println("ESC command OFF (zero sent)");
      return;
    }
    if (value.length() > 0) {
      // int18 setpoint range; ESC clamps to its configured min/max rpm
      int32_t rpm = constrain(value.toInt(), -131072L, 131071L);
      escCmdUser = rpm;
      escCmdValue = rpm;
      escCmdMode = ESC_CMD_RPM;
      Serial.printf("ESC RPM setpoint %d (esc.RPMCommand at 50 Hz) — 'can stop' to stop\n", rpm);
      return;
    }
    Serial.println("Usage: can rpm <setpoint> | can stop");
    return;
  }

  if (action == "brake") {
    String value = getNextToken(command, pos);
    if (value == "off") {
      escCommandStop();
      Serial.println("ESC command OFF (zero sent)");
      return;
    }
    if (value.length() > 0) {
      // Brake = negative duty. Requires Reversible drive mode on the ESC;
      // braking torque is capped by the ESC's bus/phase current limits.
      int pct = constrain(value.toInt(), 0, 100);
      escCmdUser = pct;
      escCmdValue = -((int32_t)pct * 8191 / 100);
      escCmdMode = ESC_CMD_BRAKE;
      Serial.printf("ESC brake %d%% (raw %d) — needs Reversible mode; torque limited by ESC current limits\n",
                    pct, escCmdValue);
      return;
    }
    Serial.println("Usage: can brake <0-100> | can stop");
    return;
  }

  if (action == "baud") {
    uint32_t kbps = (uint32_t)getNextToken(command, pos).toInt();
    if (kbps == 125 || kbps == 250 || kbps == 500 || kbps == 1000) {
      canStop();
      if (canStart(kbps)) {
        Serial.printf("CAN restarted at %u kbps\n", kbps);
      } else {
        Serial.println("CAN restart FAILED");
      }
      return;
    }
    Serial.println("Usage: can baud <125|250|500|1000>");
    return;
  }

  if (action == "print") {
    String sub = getNextToken(command, pos);
    if (sub == "on" || sub == "off") {
      canPrintRx = (sub == "on");
      Serial.printf("CAN RX printing %s\n", canPrintRx ? "ON" : "OFF");
      return;
    }
  }

  if (action == "send") {
    String idTok = getNextToken(command, pos);
    if (idTok.length() > 0) {
      uint32_t id = (uint32_t)strtoul(idTok.c_str(), NULL, 16);
      uint8_t data[8];
      uint8_t len = 0;
      while (len < 8) {
        String b = getNextToken(command, pos);
        if (b.length() == 0) break;
        data[len++] = (uint8_t)strtoul(b.c_str(), NULL, 16);
      }
      if (canSendFrame(id, data, len)) {
        Serial.printf("CAN TX id=0x%X len=%u\n", id, len);
      } else {
        Serial.println("CAN TX failed (bus down or queue full)");
      }
      return;
    }
  }

  Serial.println("Usage: can [status|esc] | can duty <-100..100> | can rpm <n> | can brake <0-100> | can stop | can baud <kbps> | can send <id> <b0..> | can print on|off");
}

// -----------------------------------------------------------------------------
// Stream handler
// -----------------------------------------------------------------------------
void handleStream(String &command, int &pos) {
  String action = getNextToken(command, pos);
  if (action == "on") {
    streamEnabled = true;
    Serial.printf("Stream enabled at %u Hz\n", 1000 / streamIntervalMs);
  } else if (action == "off") {
    streamEnabled = false;
    Serial.println("Stream disabled");
  } else if (action == "rate") {
    long hz = getNextToken(command, pos).toInt();
    if (hz >= 1 && hz <= 50) {
      streamIntervalMs = 1000 / (uint32_t)hz;
      Serial.printf("Stream rate set to %ld Hz (TCs update at 10 Hz max — chip limit)\n", hz);
    } else {
      Serial.println("Usage: stream rate <1-50>");
    }
  } else {
    Serial.printf("Stream is %s at %u Hz\n",
                  streamEnabled ? "ON" : "OFF", 1000 / streamIntervalMs);
    Serial.println("Usage: stream on|off | stream rate <1-50>");
  }
}

// -----------------------------------------------------------------------------
// MOSFET handler
// -----------------------------------------------------------------------------
void handleMosfet(String &command, int &pos) {
  String target = getNextToken(command, pos);
  if (target == "freq") {
    String value = getNextToken(command, pos);
    uint32_t freq = (uint32_t)constrain(value.toInt(), 1, 20000);
    setMosfetFrequency(freq);
    Serial.printf("MOSFET frequency set to %u Hz\n", freq);
    return;
  }

  if (target == "all") {
    String action = getNextToken(command, pos);
    if (action == "on" || action == "off") {
      setAllMosfets(action == "on" ? 10 : 0);
      Serial.printf("MOSFETs all %s\n", action.c_str());
      return;
    }
    if (action == "duty") action = getNextToken(command, pos);
    if (action.length() > 0) {
      uint8_t duty = (uint8_t)constrain(action.toInt(), 0, 10);
      setAllMosfets(duty);
      Serial.printf("MOSFETs all set to %u\n", duty);
      return;
    }
  }

  int index = target.toInt();
  if (index >= 1 && index <= 5) {
    String action = getNextToken(command, pos);
    if (action == "on" || action == "off") {
      setMosfetDuty(index - 1, action == "on" ? 10 : 0);
      Serial.printf("MOSFET %d (%s) %s\n", index, mosfetNames[index - 1], action.c_str());
      return;
    }
    if (action == "duty") action = getNextToken(command, pos);
    if (action.length() > 0) {
      uint8_t duty = (uint8_t)constrain(action.toInt(), 0, 10);
      setMosfetDuty(index - 1, duty);
      Serial.printf("MOSFET %d (%s) duty set to %u\n", index, mosfetNames[index - 1], duty);
      return;
    }
  }

  Serial.println("Usage: mosfet freq <hz> | mosfet all|<1-5> on|off|duty <0-10>");
}

// -----------------------------------------------------------------------------
// Piezo handler
// -----------------------------------------------------------------------------
void handlePiezo(String &command, int &pos) {
  String action = getNextToken(command, pos);
  if (action == "on") {
    setPiezoDuty(piezoDuty > 0 ? piezoDuty : 128);
    Serial.println("Piezo enabled");
  } else if (action == "off") {
    setPiezoDuty(0);
    Serial.println("Piezo disabled");
  } else if (action == "duty") {
    uint8_t duty = (uint8_t)constrain(getNextToken(command, pos).toInt(), 0, 255);
    setPiezoDuty(duty);
    Serial.printf("Piezo duty set to %u\n", duty);
  } else if (action == "freq") {
    uint32_t freq = (uint32_t)max(10L, getNextToken(command, pos).toInt());
    setPiezoFrequency(freq);
    Serial.printf("Piezo frequency set to %u Hz\n", freq);
  } else {
    Serial.println("Usage: piezo on|off|duty <0-255>|freq <hz>");
  }
}

// -----------------------------------------------------------------------------
// Analog handler
// -----------------------------------------------------------------------------
void handleAnalog(String &command, int &pos) {
  String input = getNextToken(command, pos);
  if (input == "all") {
    printAnalogValue(PIN_ANALOG_VP, "VP");
    printAnalogValue(PIN_ANALOG_VN, "VN");
    printAnalogValue(PIN_ANALOG_A, "A");
    printAnalogValue(PIN_ANALOG_B, "B");
    return;
  }

  if (input == "vp") printAnalogValue(PIN_ANALOG_VP, "VP");
  else if (input == "vn") printAnalogValue(PIN_ANALOG_VN, "VN");
  else if (input == "a")  printAnalogValue(PIN_ANALOG_A, "A");
  else if (input == "b")  printAnalogValue(PIN_ANALOG_B, "B");
  else Serial.println("Usage: analog all | analog vp|vn|a|b");
}

// -----------------------------------------------------------------------------
// I2C handler
// -----------------------------------------------------------------------------
void handleI2c(String &command, int &pos) {
  String action = getNextToken(command, pos);
  if (action == "scan") {
    i2cScan();
    return;
  }

  if (action == "recover") {
    if (expanderRecover()) Serial.println("Expander back online");
    return;
  }

  if (action == "expander") {
    String sub = getNextToken(command, pos);
    if (sub == "set") {
      uint16_t state = (uint16_t)strtoul(getNextToken(command, pos).c_str(), NULL, 0);
      expanderState = state;
      expanderWrite(expanderState);
      Serial.printf("Expander raw state written: 0x%04X\n", expanderState);
      return;
    }
    if (sub == "bit") {
      int bitIndex = getNextToken(command, pos).toInt();
      String action2 = getNextToken(command, pos);
      if (bitIndex >= 0 && bitIndex < 16 && (action2 == "on" || action2 == "off")) {
        if (action2 == "on")  expanderState |= (1 << bitIndex);
        else                  expanderState &= ~(1 << bitIndex);
        expanderWrite(expanderState);
        Serial.printf("Expander bit %d %s\n", bitIndex, action2.c_str());
        return;
      }
    }
  }

  Serial.println("Usage: i2c scan | i2c recover | i2c expander set <hex> | i2c expander bit <n> on|off");
}

// -----------------------------------------------------------------------------
// Serial2 handler
// -----------------------------------------------------------------------------
void handleSerial2(String &command, int &pos) {
  String action = getNextToken(command, pos);
  if (action == "send") {
    String message = command.substring(pos);
    message.trim();
    if (message.length() > 0) {
      Serial2.println(message);
      Serial.printf("Sent to Serial2: %s\n", message.c_str());
      return;
    }
  }
  if (action == "baud") {
    long baud = getNextToken(command, pos).toInt();
    if (baud >= 1200 && baud <= 1000000) {
      serial2Baud = (uint32_t)baud;
      Serial2.updateBaudRate(serial2Baud);
      Serial.printf("Serial2 baud set to %u (Pixhawk telem is usually 57600)\n", serial2Baud);
      return;
    }
    Serial.println("Usage: serial2 baud <1200-1000000>");
    return;
  }
  if (action == "hex") {
    String sub = getNextToken(command, pos);
    if (sub == "on" || sub == "off") {
      s2HexMode = (sub == "on");
      Serial.printf("Serial2 hex dump %s\n", s2HexMode ? "ON" : "OFF");
      return;
    }
  }
  Serial.println("Usage: serial2 send <text> | serial2 baud <rate> | serial2 hex on|off");
}

// -----------------------------------------------------------------------------
// Thermocouple SPI reads — every CS is on the PCF8575 expander (active LOW).
// The expander I2C transaction adds ~200us of latency, fine for TC reads.
// -----------------------------------------------------------------------------
uint32_t readMAX31855(uint8_t expanderBit) {
  uint16_t assertedState = expanderState & ~(1u << expanderBit);
  expanderWrite(assertedState);
  delayMicroseconds(150);  // let CS settle after I2C

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  uint32_t value = ((uint32_t)SPI.transfer16(0) << 16) | (uint32_t)SPI.transfer16(0);
  SPI.endTransaction();

  expanderWrite(expanderState);  // deassert CS
  return value;
}

// Decode MAX31855 32-bit word to Celsius.
// Returns NAN on open/short fault or if the device is not responding.
float decodeMAX31855Celsius(uint32_t raw) {
  if (raw == 0x00000000 || raw == 0xFFFFFFFF) return NAN;  // no device
  if (raw & 0x00000007) return NAN;                         // D2:D0 fault bits set
  // Bits D[31:18]: 14-bit TC temperature, 2's complement, 0.25°C/LSB
  int16_t raw14 = (int16_t)((raw >> 18) & 0x3FFF);
  if (raw14 & 0x2000) raw14 |= 0xC000;  // sign-extend 14→16 bit
  return raw14 * 0.25f;
}

// Human-readable fault for a MAX31855 word ("" = no fault).
// OPEN = no thermocouple seen (check the screw terminals / probe wires),
// SHORT-GND / SHORT-VCC = thermocouple sheath touching ground or supply.
const char *tcFaultName(uint32_t raw) {
  if (raw == 0x00000000 || raw == 0xFFFFFFFF) return "NC";
  if (raw & 0x00000001) return "OPEN";
  if (raw & 0x00000002) return "SHORT-GND";
  if (raw & 0x00000004) return "SHORT-VCC";
  return "";
}

// MAX31855 internal (cold junction) temp, D15..D4, 12-bit signed, 0.0625°C/LSB.
// Reads ~room temperature whenever the chip itself is alive.
float tcInternalCelsius(uint32_t raw) {
  int16_t v = (int16_t)((raw >> 4) & 0x0FFF);
  if (v & 0x0800) v |= 0xF000;
  return v * 0.0625f;
}

// Decode MAX6675 16-bit word (upper 16 bits of the 32-bit SPI read).
// D15 dummy(0), D14..D3 unsigned temp 0.25°C/LSB, D2 open-TC flag, D1 id(0).
float decodeMAX6675Celsius(uint32_t raw) {
  uint16_t w = (uint16_t)(raw >> 16);
  if (w == 0x0000 || w == 0xFFFF) return NAN;  // no device
  if (w & 0x0004) return NAN;                   // thermocouple open
  return ((w >> 3) & 0x0FFF) * 0.25f;
}

float decodeTc(uint32_t raw) {
  return tcMode6675 ? decodeMAX6675Celsius(raw) : decodeMAX31855Celsius(raw);
}

void readAllThermocouples() {
  for (uint8_t i = 0; i < 4; i++) {
    tcRaw[i]  = readMAX31855(tcExpBits[i]);
    tcTemp[i] = decodeTc(tcRaw[i]);
  }
}

// Print one TC line with fault detail and cold-junction temp when faulted.
void printTcLine(uint8_t i) {
  Serial.printf("TC%u (%s): ", i + 1, tcNames[i]);
  if (!isnan(tcTemp[i])) {
    Serial.printf("%.2f C\n", tcTemp[i]);
    return;
  }
  const char *fault = tcFaultName(tcRaw[i]);
  if (strcmp(fault, "NC") == 0) {
    Serial.println("NO MODULE (no SPI data)");
  } else {
    Serial.printf("%s (chip alive, cold junction %.2f C)\n",
                  fault, tcInternalCelsius(tcRaw[i]));
  }
}

// Probe every expander bit as a chip select and show what comes back on SPI.
// A live MAX31855/MAX6675 shows a raw word that isn't all-0s or all-1s.
void tcScan() {
  Serial.printf("Scanning expander CS bits (expander 0x%02X %s)...\n",
                expanderAddr, expanderOk ? "OK" : "NOT RESPONDING");
  Serial.println("bit | raw        | as MAX31855 | fault     | internal  | as MAX6675");
  for (uint8_t bit = 0; bit < 16; bit++) {
    uint32_t raw = readMAX31855(bit);
    float t31855 = decodeMAX31855Celsius(raw);
    float t6675  = decodeMAX6675Celsius(raw);
    const char *fault = tcFaultName(raw);
    Serial.printf(" %2u | 0x%08X | ", bit, raw);
    if (isnan(t31855)) Serial.print("---       ");
    else               Serial.printf("%8.2f C", t31855);
    Serial.printf(" | %-9s | ", fault[0] ? fault : "-");
    if (strcmp(fault, "NC") == 0) Serial.print("---      ");
    else Serial.printf("%6.2f C", tcInternalCelsius(raw));
    Serial.print(" | ");
    if (isnan(t6675)) Serial.println("---");
    else              Serial.printf("%8.2f C\n", t6675);
  }
  Serial.println("Expected: TC1=bit10 TC2=bit11 TC3=bit12 TC4=bit13, pot CS=bit14");
  Serial.println("All 0x00000000 = no MISO data (CS not reaching module / no power)");
  Serial.println("All 0xFFFFFFFF = MISO floating (module missing on that CS)");
}

// -----------------------------------------------------------------------------
// X9C digital pot driver
// Wiper moves on the falling edge of INC while CS is LOW.
// Rising CS with INC HIGH stores the position to NVM; with INC LOW it doesn't.
// -----------------------------------------------------------------------------
static void potSelect() {
  expanderState &= ~(1u << EXP_BIT_POT_CS);
  expanderWrite(expanderState);
  delayMicroseconds(5);
}

static void potDeselect(bool store) {
  digitalWrite(PIN_POT_INC, store ? HIGH : LOW);
  delayMicroseconds(5);
  expanderState |= (1u << EXP_BIT_POT_CS);
  expanderWrite(expanderState);
  delayMicroseconds(store ? 25000 : 5);  // NVM store takes up to 20ms
  digitalWrite(PIN_POT_INC, HIGH);       // idle high
}

static void potPulse(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_POT_INC, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_POT_INC, LOW);   // wiper moves here
    delayMicroseconds(2);
  }
}

void potMove(int steps) {
  if (steps == 0) return;
  digitalWrite(PIN_POT_UD, steps > 0 ? HIGH : LOW);  // HIGH = up
  delayMicroseconds(5);
  potSelect();
  potPulse(abs(steps));
  potDeselect(false);
  potPosition = constrain(potPosition + steps, 0, POT_STEPS - 1);
}

void potSet(int target) {
  target = constrain(target, 0, POT_STEPS - 1);
  potMove(target - potPosition);
  potPosition = target;  // potMove clamps; make it exact
}

void potReset() {
  digitalWrite(PIN_POT_UD, LOW);  // down
  delayMicroseconds(5);
  potSelect();
  potPulse(POT_STEPS + 2);        // guaranteed to hit the bottom stop
  potDeselect(false);
  potPosition = 0;
}

void potStore() {
  potSelect();
  potDeselect(true);
}

// -----------------------------------------------------------------------------
// CAN / TWAI (SN65HVD230 on J12, TX=GPIO23 RX=GPIO4)
// -----------------------------------------------------------------------------
bool canStart(uint32_t kbps) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.rx_queue_len = 32;
  g.tx_queue_len = 8;

  twai_timing_config_t t;
  switch (kbps) {
    case 125:  t = TWAI_TIMING_CONFIG_125KBITS();  break;
    case 250:  t = TWAI_TIMING_CONFIG_250KBITS();  break;
    case 1000: t = TWAI_TIMING_CONFIG_1MBITS();    break;
    case 500:
    default:   t = TWAI_TIMING_CONFIG_500KBITS(); kbps = 500; break;
  }
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    canOk = false;
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    canOk = false;
    return false;
  }
  canKbps = kbps;
  canOk = true;
  return true;
}

void canStop() {
  if (!canOk) return;
  twai_stop();
  twai_driver_uninstall();
  canOk = false;
}

bool canSendFrame(uint32_t id, const uint8_t *data, uint8_t len) {
  if (!canOk) return false;
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = (id > 0x7FF) ? 1 : 0;
  msg.data_length_code = min<uint8_t>(len, 8);
  memcpy(msg.data, data, msg.data_length_code);
  if (twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
    canTxCount++;
    return true;
  }
  return false;
}

void processCanRx() {
  if (!canOk) return;

  // Auto-recover from bus-off
  twai_status_info_t info;
  if (twai_get_status_info(&info) == ESP_OK && info.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
  }

  twai_message_t msg;
  uint8_t drained = 0;
  while (drained < 32 && twai_receive(&msg, 0) == ESP_OK) {
    drained++;
    canRxCount++;
    if (msg.extd) dcHandleFrame(msg.identifier, msg.data, msg.data_length_code);
    if (canPrintRx) {
      Serial.printf("CAN:RX id=0x%X dlc=%u data=", msg.identifier, msg.data_length_code);
      for (uint8_t i = 0; i < msg.data_length_code; i++) {
        Serial.printf("%02X", msg.data[i]);
        if (i < msg.data_length_code - 1) Serial.print(" ");
      }
      Serial.println();
    }
  }
}

// -----------------------------------------------------------------------------
// DroneCAN (UAVCAN v0) — minimal decoder for the microDRIVE ESC telemetry and
// encoder for esc.RawCommand. Bit semantics match libcanard exactly: the wire
// is an MSB-first bit stream, scalars assemble little-endian byte-wise, and a
// trailing partial byte is right-justified on decode / left-justified on encode.
// -----------------------------------------------------------------------------
uint32_t dcBits(const uint8_t *buf, uint32_t bitOff, uint8_t bitLen) {
  uint8_t bytes[4] = {0};
  for (uint8_t i = 0; i < bitLen; i++) {
    uint32_t sb = bitOff + i;
    if ((buf[sb >> 3] >> (7 - (sb & 7))) & 1) {
      bytes[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
    }
  }
  if (bitLen & 7) bytes[bitLen >> 3] >>= (8 - (bitLen & 7));
  return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
         ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

int32_t dcBitsSigned(const uint8_t *buf, uint32_t bitOff, uint8_t bitLen) {
  uint32_t u = dcBits(buf, bitOff, bitLen);
  if (u & (1UL << (bitLen - 1))) u |= ~((1UL << bitLen) - 1);
  return (int32_t)u;
}

void dcEncodeBits(uint8_t *dst, uint32_t bitOff, uint8_t bitLen, uint32_t value) {
  uint8_t st[4] = { (uint8_t)value, (uint8_t)(value >> 8),
                    (uint8_t)(value >> 16), (uint8_t)(value >> 24) };
  if (bitLen & 7) st[bitLen >> 3] <<= (8 - (bitLen & 7));
  for (uint8_t i = 0; i < bitLen; i++) {
    uint32_t db = bitOff + i;
    if ((st[i >> 3] >> (7 - (i & 7))) & 1) dst[db >> 3] |= (uint8_t)(0x80 >> (db & 7));
    else                                   dst[db >> 3] &= (uint8_t)~(0x80 >> (db & 7));
  }
}

// CRC-16-CCITT-FALSE seeded with the DSDL data type signature (LE bytes) —
// the UAVCAN v0 multi-frame transfer CRC. Verified against pydronecan.
static uint16_t crc16Add(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

static uint16_t dcTransferCrc(uint64_t signature, const uint8_t *payload, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < 8; i++) crc = crc16Add(crc, (uint8_t)(signature >> (8 * i)));
  for (uint8_t i = 0; i < len; i++) crc = crc16Add(crc, payload[i]);
  return crc;
}

// Broadcast a UAVCAN v0 message transfer as our node, handling single- and
// multi-frame (with transfer CRC) automatically.
bool dcBroadcast(uint16_t dtid, uint64_t signature, uint8_t priority,
                 uint8_t *tidCounter, const uint8_t *payload, uint8_t len) {
  if (!canOk) return false;
  uint32_t id = ((uint32_t)priority << 24) | ((uint32_t)dtid << 8) | DC_NODE_ID_SELF;
  uint8_t tid = (*tidCounter)++ & 0x1F;

  if (len <= 7) {
    twai_message_t m = {};
    m.extd = 1;
    m.identifier = id;
    memcpy(m.data, payload, len);
    m.data[len] = (uint8_t)(0xC0 | tid);   // start + end
    m.data_length_code = len + 1;
    if (twai_transmit(&m, pdMS_TO_TICKS(5)) != ESP_OK) return false;
    canTxCount++;
    return true;
  }

  uint16_t crc = dcTransferCrc(signature, payload, len);
  uint8_t idx = 0;
  bool first = true;
  uint8_t toggle = 0;
  while (idx < len) {
    twai_message_t m = {};
    m.extd = 1;
    m.identifier = id;
    uint8_t n = 0;
    if (first) {
      m.data[n++] = (uint8_t)(crc & 0xFF);
      m.data[n++] = (uint8_t)(crc >> 8);
    }
    while (n < 7 && idx < len) m.data[n++] = payload[idx++];
    bool end = (idx >= len);
    m.data[n] = (uint8_t)((first ? 0x80 : 0) | (end ? 0x40 : 0) | (toggle << 5) | tid);
    m.data_length_code = n + 1;
    if (twai_transmit(&m, pdMS_TO_TICKS(5)) != ESP_OK) return false;
    canTxCount++;
    first = false;
    toggle ^= 1;
  }
  return true;
}

// Dynamic node ID allocation server. Requests arrive as anonymous single
// frames: uint7 preferred_node_id, bool first_part, uint8[<=6] unique_id part.
// We echo accumulated UID bytes back; once all 16 arrive, we grant an ID.
void dnaHandleRequest(const uint8_t *data, uint8_t dlc) {
  if (dlc < 2) return;
  uint8_t tail = data[dlc - 1];
  if (!(tail & 0x80) || !(tail & 0x40)) return;   // anonymous must be single frame
  uint8_t plen = dlc - 1;
  uint8_t preferred = (uint8_t)dcBits(data, 0, 7);
  bool firstPart    = dcBits(data, 7, 1) != 0;
  const uint8_t *uidPart = data + 1;
  uint8_t partLen = plen - 1;

  unsigned long now = millis();
  if (firstPart) {
    dna.len = 0;
  } else if (dna.len == 0 || now - dna.lastMs > 500) {
    return;   // follow-up without a session — stale, ignore
  }
  if (partLen > (uint8_t)(16 - dna.len)) partLen = 16 - dna.len;
  memcpy(dna.uid + dna.len, uidPart, partLen);
  dna.len += partLen;
  dna.lastMs = now;

  uint8_t payload[17];
  if (dna.len < 16) {
    // Echo what we have so far; the requester answers with its next part
    payload[0] = 0;   // node_id 0, first_part 0
    memcpy(payload + 1, dna.uid, dna.len);
    dcBroadcast(DTID_ALLOCATION, SIG_ALLOCATION, 20, &allocTxTid,
                payload, (uint8_t)(dna.len + 1));
    return;
  }

  // Full unique ID received — grant a node ID
  uint8_t assign = (preferred > 0 && preferred != DC_NODE_ID_SELF)
                     ? preferred : DNA_DEFAULT_NODE_ID;
  payload[0] = (uint8_t)(assign << 1);   // uint7 node_id + first_part=0
  memcpy(payload + 1, dna.uid, 16);
  if (dcBroadcast(DTID_ALLOCATION, SIG_ALLOCATION, 20, &allocTxTid, payload, 17)) {
    dna.grants++;
    dna.lastGrantedId = assign;
    Serial.printf("DNA: granted node ID %u (grant #%u)\n", assign, dna.grants);
  }
  dna.len = 0;
}

// Broadcast our own NodeStatus at 1 Hz — required to be a well-behaved node
// (and some allocatees ignore allocators they can't see on the bus).
void nodeStatusTask() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (!canOk || now - last < 1000) return;
  last = now;
  uint8_t p[7] = {};
  uint32_t up = now / 1000;
  p[0] = (uint8_t)up;
  p[1] = (uint8_t)(up >> 8);
  p[2] = (uint8_t)(up >> 16);
  p[3] = (uint8_t)(up >> 24);
  // health OK, mode OPERATIONAL, sub 0, vendor code 0 — all zero bits
  dcBroadcast(DTID_NODE_STATUS, 0, 24, &nsTxTid, p, 7);
}

float half2float(uint16_t h) {
  uint32_t sign = (uint32_t)(h & 0x8000) << 16;
  uint32_t exp  = (h >> 10) & 0x1F;
  uint32_t mant = h & 0x3FF;
  float out;
  if (exp == 0) {                       // zero / subnormal
    out = mant * 5.9604645e-8f;         // mant * 2^-24
    if (sign) out = -out;
    return out;
  }
  uint32_t f;
  if (exp == 31) f = sign | 0x7F800000UL | (mant << 13);          // inf/nan
  else           f = sign | ((exp - 15 + 127) << 23) | (mant << 13);
  memcpy(&out, &f, 4);
  return out;
}

// uavcan.equipment.esc.Status: u32 error_count, f16 voltage, f16 current,
// f16 temperature(K), int18 rpm, u7 power_rating_pct, u5 esc_index = 110 bits
void dcDecodeEscStatus(const uint8_t *buf, uint8_t len, uint8_t src) {
  if ((uint16_t)len * 8 < 110) return;
  esc.errorFlags = dcBits(buf, 0, 32);
  esc.voltage    = half2float((uint16_t)dcBits(buf, 32, 16));
  esc.current    = half2float((uint16_t)dcBits(buf, 48, 16));
  float kelvin   = half2float((uint16_t)dcBits(buf, 64, 16));
  esc.tempC      = (kelvin > 0.0f) ? kelvin - 273.15f : kelvin;
  esc.rpm        = dcBitsSigned(buf, 80, 18);
  esc.powerPct   = (uint8_t)dcBits(buf, 98, 7);
  esc.escIndex   = (uint8_t)dcBits(buf, 105, 5);
  esc.srcNode    = src;
  esc.seen       = true;
  esc.lastMs     = millis();
}

// uavcan.equipment.esc.StatusExtended: u7 input_pct, u7 output_pct,
// int9 motor_temperature_degC, u9 motor_angle, u19 status_flags, u5 esc_index
void dcDecodeEscStatusExt(const uint8_t *buf, uint8_t len, uint8_t src) {
  if ((uint16_t)len * 8 < 56) return;
  (void)src;
  esc.inputPct    = (uint8_t)dcBits(buf, 0, 7);
  esc.outputPct   = (uint8_t)dcBits(buf, 7, 7);
  esc.motorTempC  = (int16_t)dcBitsSigned(buf, 14, 9);
  esc.statusFlags = dcBits(buf, 32, 19);
  esc.extSeen     = true;
  esc.extLastMs   = millis();
}

// UAVCAN v0 message CAN ID: [28:24]=priority [23:8]=data type id
// [7]=service flag [6:0]=source node. Tail byte: start/end/toggle/transfer-id.
void dcHandleFrame(uint32_t id, const uint8_t *data, uint8_t dlc) {
  if ((id >> 7) & 1) return;                 // service frame — not interesting
  uint16_t dtid = (id >> 8) & 0xFFFF;
  uint8_t  src  = id & 0x7F;
  if (src == 0) {                            // anonymous = node has no ID yet
    anonFrameCount++;
    // Anonymous frame ID carries only the low 2 bits of the data type id
    if (((id >> 8) & 3) == (DTID_ALLOCATION & 3)) dnaHandleRequest(data, dlc);
    return;
  }
  if (dlc < 1) return;
  uint8_t tail   = data[dlc - 1];
  bool    start  = tail & 0x80;
  bool    end    = tail & 0x40;
  uint8_t toggle = (tail >> 5) & 1;
  uint8_t tid    = tail & 0x1F;
  uint8_t plen   = dlc - 1;

  if (dtid == DTID_ESC_STATUS) {
    if (start && end) {                      // single frame (CAN-FD case)
      dcDecodeEscStatus(data, plen, src);
      return;
    }
    if (start) {                             // first frame carries the transfer CRC
      if (plen < 3 || toggle != 0) return;
      dcAsm.active = true;
      dcAsm.src = src;
      dcAsm.tid = tid;
      dcAsm.toggleExpect = 1;
      dcAsm.len = 0;
      dcAsm.crc = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
      for (uint8_t i = 2; i < plen && dcAsm.len < sizeof(dcAsm.buf); i++)
        dcAsm.buf[dcAsm.len++] = data[i];
      return;
    }
    if (!dcAsm.active || dcAsm.src != src || dcAsm.tid != tid ||
        toggle != dcAsm.toggleExpect) {
      dcAsm.active = false;                  // lost a frame — drop the transfer
      return;
    }
    dcAsm.toggleExpect ^= 1;
    for (uint8_t i = 0; i < plen && dcAsm.len < sizeof(dcAsm.buf); i++)
      dcAsm.buf[dcAsm.len++] = data[i];
    if (end) {
      dcAsm.active = false;
      // Validate the transfer CRC — a corrupted/misassembled transfer decodes
      // into plausible-looking garbage (wrong voltage etc.), so reject it.
      if (dcTransferCrc(SIG_ESC_STATUS, dcAsm.buf, dcAsm.len) != dcAsm.crc) {
        escCrcErrors++;
        return;
      }
      dcDecodeEscStatus(dcAsm.buf, dcAsm.len, src);
    }
    return;
  }

  if (dtid == DTID_ESC_STATUS_EXT && start && end) {
    dcDecodeEscStatusExt(data, plen, src);
    return;
  }

  // uavcan.protocol.NodeStatus: u32 uptime_sec, u2 health, u3 mode, u3 sub_mode,
  // u16 vendor_specific_status_code = 56 bits, single frame at 1 Hz
  if (dtid == DTID_NODE_STATUS && start && end && plen >= 7) {
    nodeHb.uptimeSec = dcBits(data, 0, 32);
    nodeHb.health    = (uint8_t)dcBits(data, 32, 2);
    nodeHb.mode      = (uint8_t)dcBits(data, 34, 3);
    nodeHb.src       = src;
    nodeHb.seen      = true;
    nodeHb.lastMs    = millis();
  }
}

// esc.RawCommand: int14[<=20] with tail-array optimization. We fill the array
// up to the ESC's reported index so the value lands in the right slot.
bool dcSendRawCommand(int16_t value) {
  if (!canOk) return false;
  uint8_t n = (uint8_t)min((int)esc.escIndex + 1, 4);
  if (!esc.seen) n = 1;
  uint8_t payloadBytes = (uint8_t)((n * 14 + 7) / 8);

  twai_message_t msg = {};
  msg.extd = 1;
  msg.identifier = (8UL << 24) | ((uint32_t)DTID_ESC_RAWCMD << 8) | DC_NODE_ID_SELF;
  msg.data_length_code = payloadBytes + 1;
  for (uint8_t i = 0; i < n; i++) {
    uint16_t cmd = (i == esc.escIndex || !esc.seen) ? (uint16_t)value : 0;
    dcEncodeBits(msg.data, (uint32_t)i * 14, 14, cmd);
  }
  msg.data[payloadBytes] = (uint8_t)(0xC0 | (escTxTransferId++ & 0x1F));  // start+end
  if (twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
    canTxCount++;
    return true;
  }
  return false;
}

// esc.RPMCommand: int18[<=20] closed-loop RPM setpoint. Max 3 array items in a
// single CAN 2.0 frame (54 bits); ESC index above 2 would need multi-frame TX.
bool dcSendRpmCommand(int32_t rpm) {
  if (!canOk) return false;
  uint8_t n = (uint8_t)min((int)esc.escIndex + 1, 3);
  if (!esc.seen) n = 1;
  uint8_t payloadBytes = (uint8_t)((n * 18 + 7) / 8);

  twai_message_t msg = {};
  msg.extd = 1;
  msg.identifier = (8UL << 24) | ((uint32_t)DTID_ESC_RPMCMD << 8) | DC_NODE_ID_SELF;
  msg.data_length_code = payloadBytes + 1;
  for (uint8_t i = 0; i < n; i++) {
    uint32_t cmd = (i == esc.escIndex || !esc.seen)
                     ? ((uint32_t)rpm & 0x3FFFF) : 0;   // two's complement int18
    dcEncodeBits(msg.data, (uint32_t)i * 18, 18, cmd);
  }
  msg.data[payloadBytes] = (uint8_t)(0xC0 | (escTxTransferId++ & 0x1F));
  if (twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
    canTxCount++;
    return true;
  }
  return false;
}

void escCommandStop() {
  escCmdMode = ESC_CMD_OFF;
  escCmdValue = 0;
  escCmdUser = 0;
  dcSendRawCommand(0);   // explicit zero so the ESC stops immediately
}

void canThrottleTask() {
  if (escCmdMode == ESC_CMD_OFF) return;
  unsigned long now = millis();
  if (now - escCmdLastMs >= 20) {   // 50 Hz — ESCs failsafe if commands stop
    escCmdLastMs = now;
    if (escCmdMode == ESC_CMD_RPM) dcSendRpmCommand(escCmdValue);
    else                           dcSendRawCommand((int16_t)escCmdValue);
  }
}

void printEscTelemetry() {
  // Bus-level diagnosis first — this tells you WHY telemetry may be missing
  if (nodeHb.seen) {
    Serial.printf("Node %u heartbeat: %s / %s, uptime %us, last seen %lu ms ago\n",
                  nodeHb.src, nodeModeName(nodeHb.mode),
                  nodeHealthNames[nodeHb.health & 3], nodeHb.uptimeSec,
                  millis() - nodeHb.lastMs);
  }
  if (anonFrameCount > 0) {
    Serial.printf("Anonymous allocation requests seen: %u\n", anonFrameCount);
  }
  if (dna.grants > 0) {
    Serial.printf("DNA server: granted node ID %u (%u grants total)\n",
                  dna.lastGrantedId, dna.grants);
  } else if (anonFrameCount > 0) {
    Serial.println("DNA server active — allocation in progress...");
  }
  if (!nodeHb.seen && anonFrameCount == 0 && canRxCount > 0) {
    Serial.println("Frames arriving but no NodeStatus/anonymous — check 'can print on'");
  }

  if (!esc.seen) {
    Serial.println("ESC: no esc.Status telemetry received yet (bus voltage/RPM ride in it)");
    return;
  }
  Serial.printf("ESC (node %u, index %u), last seen %lu ms ago:\n",
                esc.srcNode, esc.escIndex, millis() - esc.lastMs);
  Serial.printf("  RPM: %d\n", esc.rpm);
  Serial.printf("  Voltage: %.2f V\n", esc.voltage);
  Serial.printf("  Current: %.2f A\n", esc.current);
  Serial.printf("  Bridge temp: %.1f C\n", esc.tempC);
  Serial.printf("  Power: %u %%\n", esc.powerPct);
  Serial.printf("  Error flags: 0x%04X", esc.errorFlags);
  if (esc.errorFlags == 0) {
    Serial.println(" (OK)");
  } else {
    for (uint8_t b = 0; b < 13; b++)
      if (esc.errorFlags & (1UL << b)) Serial.printf(" %s", escErrorNames[b]);
    Serial.println();
  }
  if (esc.extSeen) {
    Serial.printf("  Input/Output: %u%% / %u%%, motor temp: %d C, status: 0x%05X\n",
                  esc.inputPct, esc.outputPct, esc.motorTempC, esc.statusFlags);
  }
  Serial.printf("  Command: %s", escCmdModeNames[escCmdMode]);
  if (escCmdMode == ESC_CMD_RPM)        Serial.printf(" %d rpm\n", escCmdUser);
  else if (escCmdMode != ESC_CMD_OFF)   Serial.printf(" %d %%\n", escCmdUser);
  else                                  Serial.println();
}

// -----------------------------------------------------------------------------
// Current sensor helpers (QNDB6 hall sensors on GPIO34/35)
// -----------------------------------------------------------------------------
float analogToVolts(uint16_t raw) {
  return raw * 3.3f / 4095.0f;
}

float voltsToCurrentAmps(float volts) {
  return (volts - currentZeroVolts) * currentAmpsPerVolt;
}

float readCurrentAmps(uint8_t pin) {
  // Average a few samples — the ESP32 ADC is noisy
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += analogRead(pin);
  return voltsToCurrentAmps(analogToVolts(sum / 8));
}

// -----------------------------------------------------------------------------
// Stream data sender
// -----------------------------------------------------------------------------
void sendStreamData() {
  // TCs come from the 10 Hz sensorTask cache; everything else is read fresh.
  // DATA:TC1=..,TC2=..,TC3=..,TC4=..,I_A=..,I_B=..,AVP=..,AVN=..,POT=..,
  //      PZF=..,PZD=..,CANRX=..,M1=..,..,M5=..
  Serial.print("DATA:");

  for (uint8_t i = 0; i < 4; i++) {
    Serial.printf("TC%u=", i + 1);
    if (isnan(tcTemp[i])) {
      // Fault token instead of a bare nan so the EXE can show what's wrong:
      // nc=no module, open=thermocouple open, sgnd/svcc=shorted sheath
      const char *fault = tcFaultName(tcRaw[i]);
      if      (strcmp(fault, "NC") == 0)        Serial.print("nc");
      else if (strcmp(fault, "OPEN") == 0)      Serial.print("open");
      else if (strcmp(fault, "SHORT-GND") == 0) Serial.print("sgnd");
      else if (strcmp(fault, "SHORT-VCC") == 0) Serial.print("svcc");
      else                                      Serial.print("nan");
    } else {
      Serial.printf("%.2f", tcTemp[i]);
    }
    Serial.print(",");
  }

  Serial.printf("I_A=%.3f,", readCurrentAmps(PIN_ANALOG_A));
  Serial.printf("I_B=%.3f,", readCurrentAmps(PIN_ANALOG_B));
  Serial.printf("AVP=%.3f,", analogToVolts(analogRead(PIN_ANALOG_VP)));
  Serial.printf("AVN=%.3f,", analogToVolts(analogRead(PIN_ANALOG_VN)));
  Serial.printf("POT=%d,", potPosition);
  Serial.printf("PZF=%u,PZD=%u,", piezoFrequency, piezoDuty);
  Serial.printf("CANRX=%u,", canRxCount);
  Serial.printf("EXP=%d,IFAIL=%u,IREC=%u,", expanderOk ? 1 : 0, i2cFailCount, i2cRecoverCount);

  if (nodeHb.seen) {
    Serial.printf("NS_ID=%u,NS_MODE=%u,NS_HP=%u,NS_UP=%u,",
                  nodeHb.src, nodeHb.mode, nodeHb.health, nodeHb.uptimeSec);
  }
  if (anonFrameCount > 0) Serial.printf("ANON=%u,", anonFrameCount);

  // Only stream ESC values while they're fresh — stale numbers must never
  // look like live readings. ESC_LOST=1 tells the exe to blank the tiles.
  if (esc.seen && millis() - esc.lastMs < ESC_TELEM_TIMEOUT_MS) {
    Serial.printf("ESC_RPM=%d,ESC_V=%.2f,ESC_I=%.2f,ESC_T=%.1f,ESC_PWR=%u,",
                  esc.rpm, esc.voltage, esc.current, esc.tempC, esc.powerPct);
    Serial.printf("ESC_ERR=%X,ESC_AGE=%lu,", esc.errorFlags, millis() - esc.lastMs);
    if (esc.extSeen) {
      Serial.printf("ESC_IN=%u,ESC_OUT=%u,ESC_MT=%d,", esc.inputPct, esc.outputPct, esc.motorTempC);
    }
  } else if (esc.seen) {
    Serial.print("ESC_LOST=1,");
  }
  Serial.printf("ESC_CMODE=%s,ESC_CVAL=%d,", escCmdModeNames[escCmdMode], escCmdUser);

  for (uint8_t i = 0; i < 5; i++) {
    Serial.printf("M%u=%u", i + 1, mosfetDuty[i]);
    if (i < 4) Serial.print(",");
  }

  Serial.println();
}

// -----------------------------------------------------------------------------
// Misc helpers
// -----------------------------------------------------------------------------
void printAnalogValue(uint8_t pin, const char *name) {
  uint16_t raw = analogRead(pin);
  float voltage = analogToVolts(raw);
  Serial.printf("%s = %u (%.3f V)\n", name, raw, voltage);
}

void i2cScan() {
  Serial.println("Scanning I2C bus...");
  bool found = false;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" - Found device at 0x%02X\n", address);
      found = true;
    }
  }
  if (!found) Serial.println("No I2C devices found.");
}

bool expanderWrite(uint16_t state) {
  Wire.beginTransmission(expanderAddr);
  Wire.write(state & 0xFF);         // P07..P00
  Wire.write((state >> 8) & 0xFF);  // P17..P10
  expanderOk = (Wire.endTransmission() == 0);
  if (!expanderOk) i2cFailCount++;
  return expanderOk;
}

// Classic I2C bus-clear: if a slave is stuck holding SDA low mid-transaction,
// clock SCL until it releases, then issue a STOP and reinit the peripheral.
void i2cBusClear() {
  Wire.end();
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_I2C_SCL, HIGH);
  for (uint8_t i = 0; i < 9 && digitalRead(PIN_I2C_SDA) == LOW; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(10);
  }
  // STOP condition: SDA rises while SCL is high
  pinMode(PIN_I2C_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(10);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
}

// Full expander recovery: free the bus, re-find the chip, restore CS state.
// Returns true if the expander is talking again.
bool expanderRecover() {
  i2cRecoverCount++;
  i2cBusClear();
  expanderDetect();
  if (expanderOk) {
    expanderWrite(expanderState);   // deassert every CS again
    Serial.printf("I2C recovered (expander 0x%02X, recovery #%u, %u failed writes)\n",
                  expanderAddr, i2cRecoverCount, i2cFailCount);
  } else {
    Serial.printf("I2C recovery FAILED (recovery #%u) — check expander power/wiring\n",
                  i2cRecoverCount);
  }
  return expanderOk;
}

// PCF8575 answers at 0x20-0x27 depending on the A0-A2 solder pads.
// Try the default first, then hunt for it so a re-jumpered module still works.
void expanderDetect() {
  for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      expanderAddr = addr;
      expanderOk = true;
      if (addr != I2C_EXPANDER_ADDR) {
        Serial.printf("PCF8575 found at 0x%02X (not default 0x20)\n", addr);
      }
      return;
    }
  }
  expanderOk = false;
}

// -----------------------------------------------------------------------------
// PWM helpers
// -----------------------------------------------------------------------------
void setPiezoFrequency(uint32_t frequency) {
  piezoFrequency = frequency;
  ledcSetup(PWM_CHANNEL, piezoFrequency, PWM_RESOLUTION);
  ledcAttachPin(PIN_PIEZO, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, piezoDuty);
}

void setPiezoDuty(uint8_t duty) {
  piezoDuty = duty;
  ledcWrite(PWM_CHANNEL, piezoDuty);
}

void setMosfetDuty(uint8_t index, uint8_t duty) {
  if (index >= 5) return;
  mosfetDuty[index] = constrain(duty, 0, 10);
  uint8_t hwDuty = (uint8_t)((uint32_t)mosfetDuty[index] * 255u / 10u);
  ledcWrite(mosfetChannels[index], hwDuty);
}

void setAllMosfets(uint8_t duty) {
  for (uint8_t i = 0; i < 5; i++) setMosfetDuty(i, duty);
}

void setMosfetFrequency(uint32_t frequency) {
  mosfetFrequency = frequency;
  for (uint8_t i = 0; i < 5; i++) {
    ledcSetup(mosfetChannels[i], mosfetFrequency, MOSFET_PWM_RESOLUTION);
    ledcAttachPin(mosfetPins[i], mosfetChannels[i]);
    ledcWrite(mosfetChannels[i], mosfetDuty[i]);
  }
}

// -----------------------------------------------------------------------------
// Initialization helpers
// -----------------------------------------------------------------------------
void initializeSerial() {
  Serial.begin(DEBUG_BAUD);
  while (!Serial) { ; }
  Serial2.begin(ECU_SERIAL_BAUD, SERIAL_8N1, PIN_UART2_RX, PIN_UART2_TX);
  Serial.println("Serial ports initialized");
}

void initializePins() {
  for (uint8_t pin : mosfetPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // X9C pot control lines
  pinMode(PIN_POT_UD, OUTPUT);
  pinMode(PIN_POT_INC, OUTPUT);
  digitalWrite(PIN_POT_UD, LOW);
  digitalWrite(PIN_POT_INC, HIGH);  // INC idles high

  pinMode(PIN_PIEZO, OUTPUT);
  digitalWrite(PIN_PIEZO, LOW);

  pinMode(PIN_ANALOG_VP, INPUT);
  pinMode(PIN_ANALOG_VN, INPUT);
  pinMode(PIN_ANALOG_A, INPUT);
  pinMode(PIN_ANALOG_B, INPUT);
  // Full 0-3.3V ADC range
  analogSetPinAttenuation(PIN_ANALOG_VP, ADC_11db);
  analogSetPinAttenuation(PIN_ANALOG_VN, ADC_11db);
  analogSetPinAttenuation(PIN_ANALOG_A, ADC_11db);
  analogSetPinAttenuation(PIN_ANALOG_B, ADC_11db);
}

void initializeBuses() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  // MOSI = -1: this board has no SPI MOSI (MAX31855 is read-only, GPIO23 is CAN TX)
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, -1);
  Serial.println("I2C and SPI buses initialized");
}

void initializeCan() {
  if (canStart(CAN_DEFAULT_KBPS)) {
    Serial.printf("CAN/TWAI up at %u kbps (TX=GPIO%d RX=GPIO%d)\n",
                  canKbps, PIN_CAN_TX, PIN_CAN_RX);
  } else {
    Serial.println("CAN/TWAI init FAILED");
  }
}

void initializeTestController() {
  ledcSetup(PWM_CHANNEL, piezoFrequency, PWM_RESOLUTION);
  ledcAttachPin(PIN_PIEZO, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, piezoDuty);

  for (uint8_t i = 0; i < 5; i++) {
    ledcSetup(mosfetChannels[i], mosfetFrequency, MOSFET_PWM_RESOLUTION);
    ledcAttachPin(mosfetPins[i], mosfetChannels[i]);
    ledcWrite(mosfetChannels[i], 0);
  }

  // Expander: find it, then set all outputs HIGH (all CS deasserted)
  expanderDetect();
  if (!expanderOk || !expanderWrite(expanderState)) {
    Serial.println("WARNING: PCF8575 expander not responding (tried 0x20-0x27)");
    Serial.println("         Thermocouples and digital pot CS will not work!");
  }

  // Force the digital pot to a known position
  potReset();
  Serial.println("Digital pot reset to position 0");
}

void ecuHeartbeat() {
  static unsigned long lastHeartbeat = 0;
  static unsigned long lastStream    = 0;

  unsigned long now = millis();

  if (streamEnabled && (now - lastStream >= streamIntervalMs)) {
    lastStream = now;
    sendStreamData();
  }

  // Send a lightweight heartbeat every second so the dashboard knows the ESP
  // is alive even when streaming is disabled.
  if (now - lastHeartbeat >= 1000) {
    lastHeartbeat = now;
    Serial.println("HB");
  }
}
