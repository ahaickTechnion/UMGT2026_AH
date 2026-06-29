#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Defines
// -----------------------------------------------------------------------------
// U0 (Serial / USB)  → debug terminal, EXE connects here
// U2 (Serial2)       → Pixhawk telemetry (GPIO16 RX, GPIO17 TX)
#define DEBUG_BAUD        115200
#define ECU_SERIAL_BAUD   115200

#define PIN_UART2_RX      16   // U2 → Pixhawk RX
#define PIN_UART2_TX      17   // U2 → Pixhawk TX

// CAN / TWAI  →  ESC (RPM, voltage, power)
// Pins routed via SN65HVD230 transceiver on J12.
// Update these to match actual GPIO once schematic is confirmed.
#define PIN_CAN_TX        21   // placeholder - confirm from schematic
#define PIN_CAN_RX        22   // placeholder - confirm from schematic
// TODO: implement TWAI/CAN ESC data reading (driver_install, start, receive loop)

#define PIN_SPI_SCK       18
#define PIN_SPI_MISO      19
#define PIN_SPI_MOSI      23

#define PIN_I2C_SDA       21
#define PIN_I2C_SCL       22

#define PIN_MOSFET_A      32
#define PIN_MOSFET_B      33
#define PIN_MOSFET_C      25
#define PIN_MOSFET_D      26
#define PIN_MOSFET_E      27

#define PIN_VCTRL_1       12
#define PIN_VCTRL_2       13
#define PIN_PWM_FREQ      15

#define PIN_ANALOG_VP     36
#define PIN_ANALOG_VN     39
#define PIN_ANALOG_A      34
#define PIN_ANALOG_B      35

// TC1 CS is GPIO12 (direct). TC2-TC4 CS are via PCF857x expander bits.
#define PIN_TC1_CS        PIN_VCTRL_1

// PCF857x GPIO expander bits for TC2, TC3, TC4 chip selects (active LOW).
// Adjust these if your PCF857x wiring differs from bit 0/1/2.
#define TC2_EXP_BIT       0
#define TC3_EXP_BIT       1
#define TC4_EXP_BIT       2

#define PIN_PIEZO         PIN_PWM_FREQ

#define I2C_EXPANDER_ADDR     0x20
#define I2C_EXPANDER_ADDR_ALT 0x27

#define PWM_CHANNEL        0
#define PWM_RESOLUTION     8
#define PWM_DEFAULT_FREQ   2000

#define MOSFET_PWM_FREQ    10000
#define MOSFET_PWM_RESOLUTION 8

// Current sensor calibration (ACS712-5A defaults; adjust to match your sensor).
// zero_voltage: sensor output at 0A (typically VCC/2 = 1.65V on 3.3V supply)
// sensitivity_V_per_A: mV/A sensitivity (ACS712-5A = 185mV/A, 20A = 100mV/A)
#define CURRENT_ZERO_VOLTS   1.65f
#define CURRENT_SENS_V_PER_A 0.185f

#define STREAM_INTERVAL_MS   500

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------
static uint32_t mosfetFrequency = MOSFET_PWM_FREQ;

static const uint8_t mosfetPins[] = {
  PIN_MOSFET_A, PIN_MOSFET_B, PIN_MOSFET_C, PIN_MOSFET_D, PIN_MOSFET_E
};
static const uint8_t mosfetChannels[] = { 1, 2, 3, 4, 5 };
static uint8_t mosfetDuty[5] = {0};

static const uint8_t analogPins[] = {
  PIN_ANALOG_VP, PIN_ANALOG_VN, PIN_ANALOG_A, PIN_ANALOG_B
};
static const char *analogNames[] = { "VP", "VN", "A", "B" };

uint32_t piezoFrequency = PWM_DEFAULT_FREQ;
uint8_t  piezoDuty      = 0;
uint16_t expanderState  = 0xFFFF;  // all HIGH = all CS deasserted
String   commandBuffer  = "";
bool     streamEnabled  = false;

// Last read thermocouple temps (NAN = not read / fault)
static float tcTemp[4] = { NAN, NAN, NAN, NAN };

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
void initializePins();
void initializeSerial();
void initializeBuses();
void initializeTestController();
void ecuHeartbeat();

void processSerialCommands();
void processSerial2Bridge();
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

void setMosfetDuty(uint8_t index, uint8_t duty);
void setAllMosfets(uint8_t duty);
void setMosfetFrequency(uint32_t frequency);
void setPiezoFrequency(uint32_t frequency);
void setPiezoDuty(uint8_t duty);

uint32_t readMAX31855Direct(uint8_t csPin);
uint32_t readMAX31855Expander(uint8_t expanderBit);
float    decodeMAX31855Celsius(uint32_t raw);
float    readAllThermocouples();  // reads all 4, updates tcTemp[], returns TC1

float    analogToVolts(uint16_t raw);
float    voltsToCurrentAmps(float volts);
float    readCurrentAmps(uint8_t pin);

void printAnalogValue(uint8_t pin, const char *name);
void sendStreamData();

void i2cScan();
void expanderWrite(uint16_t state);

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  initializeSerial();
  initializePins();
  initializeBuses();
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
  ecuHeartbeat();
  processSerialCommands();
  processSerial2Bridge();
  delay(10);
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
  } else if (token == "stream") {
    handleStream(working, pos);
  } else if (token == "can") {
    Serial.println("CAN/ESC: not yet implemented - placeholder for TWAI ESC data");
  } else if (token == "i2c") {
    handleI2c(working, pos);
  } else if (token == "serial2") {
    handleSerial2(working, pos);
  } else {
    Serial.println("Unknown command. Type help for a command list.");
  }
}

void processSerial2Bridge() {
  if (!Serial2.available()) return;
  String incoming = Serial2.readStringUntil('\n');
  if (incoming.length() > 0) {
    Serial.print("[Serial2] ");
    Serial.println(incoming);
  }
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
  Serial.println("tc read <1-4>                 - Read one thermocouple (MAX31855)");
  Serial.println("tc read all                   - Read all 4 thermocouples");
  Serial.println("current all                   - Read all current sensors (A)");
  Serial.println("current vp|vn|a|b             - Read one current sensor");
  Serial.println("stream on|off                 - Enable/disable auto data stream");
  Serial.println("mosfet all duty <0-10>        - Set all MOSFET PWM level");
  Serial.println("mosfet <1-5> duty <0-10>      - Set one MOSFET PWM level");
  Serial.println("mosfet freq <hz>              - Set MOSFET PWM frequency 1-20000");
  Serial.println("mosfet all on|off             - Toggle all MOSFET outputs");
  Serial.println("mosfet <1-5> on|off           - Toggle one MOSFET output");
  Serial.println("piezo on|off                  - Enable or disable piezo PWM");
  Serial.println("piezo duty <0-255>            - Set piezo PWM duty");
  Serial.println("piezo freq <hz>               - Set piezo PWM frequency");
  Serial.println("analog all                    - Read all analog inputs (raw+volts)");
  Serial.println("analog vp|vn|a|b              - Read one analog input");
  Serial.println("i2c scan                      - Scan I2C bus for devices");
  Serial.println("i2c expander set <hex>        - Write raw state to I2C expander");
  Serial.println("i2c expander bit <n> on|off   - Toggle one expander bit");
  Serial.println("serial2 send <text>           - Send raw text to Serial2 (Pixhawk)");
  Serial.println("can                           - CAN/ESC status (placeholder)");
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
    Serial.printf("%u:%u", i + 1, mosfetDuty[i]);
    if (i < 4) Serial.print(", ");
  }
  Serial.println();

  // Thermocouples
  Serial.println("Thermocouples:");
  readAllThermocouples();
  for (uint8_t i = 0; i < 4; i++) {
    Serial.printf("  TC%u: ", i + 1);
    if (isnan(tcTemp[i])) {
      Serial.println("FAULT/NC");
    } else {
      Serial.printf("%.2f C\n", tcTemp[i]);
    }
  }

  // Current sensors
  Serial.println("Current sensors:");
  for (uint8_t i = 0; i < 4; i++) {
    float amps = readCurrentAmps(analogPins[i]);
    Serial.printf("  %s: %.3f A\n", analogNames[i], amps);
  }

  Serial.print("Stream: ");
  Serial.println(streamEnabled ? "ON" : "OFF");
}

// -----------------------------------------------------------------------------
// Thermocouple handler
// -----------------------------------------------------------------------------
void handleThermocouple(String &command, int &pos) {
  String action = getNextToken(command, pos);
  if (action != "read") {
    Serial.println("Usage: tc read <1-4> | tc read all");
    return;
  }

  String which = getNextToken(command, pos);

  if (which == "all") {
    readAllThermocouples();
    for (uint8_t i = 0; i < 4; i++) {
      Serial.printf("TC%u: ", i + 1);
      if (isnan(tcTemp[i])) {
        Serial.println("FAULT/NC");
      } else {
        Serial.printf("%.2f C\n", tcTemp[i]);
      }
    }
    return;
  }

  int idx = which.toInt();
  if (idx >= 1 && idx <= 4) {
    uint32_t raw;
    if (idx == 1) {
      raw = readMAX31855Direct(PIN_TC1_CS);
    } else {
      uint8_t bit = (idx == 2) ? TC2_EXP_BIT : (idx == 3) ? TC3_EXP_BIT : TC4_EXP_BIT;
      raw = readMAX31855Expander(bit);
    }
    float temp = decodeMAX31855Celsius(raw);
    tcTemp[idx - 1] = temp;
    Serial.printf("TC%d raw: 0x%08X  ", idx, raw);
    if (isnan(temp)) {
      Serial.println("FAULT/NC");
    } else {
      Serial.printf("%.2f C\n", temp);
    }
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
    for (uint8_t i = 0; i < 4; i++) {
      float amps = readCurrentAmps(analogPins[i]);
      Serial.printf("%s: %.3f A\n", analogNames[i], amps);
    }
    return;
  }

  int idx = -1;
  if (input == "vp") idx = 0;
  else if (input == "vn") idx = 1;
  else if (input == "a")  idx = 2;
  else if (input == "b")  idx = 3;

  if (idx >= 0) {
    float amps = readCurrentAmps(analogPins[idx]);
    Serial.printf("%s: %.3f A\n", analogNames[idx], amps);
    return;
  }

  Serial.println("Usage: current all | current vp|vn|a|b");
}

// -----------------------------------------------------------------------------
// Stream handler
// -----------------------------------------------------------------------------
void handleStream(String &command, int &pos) {
  String action = getNextToken(command, pos);
  if (action == "on") {
    streamEnabled = true;
    Serial.println("Stream enabled");
  } else if (action == "off") {
    streamEnabled = false;
    Serial.println("Stream disabled");
  } else {
    Serial.print("Stream is ");
    Serial.println(streamEnabled ? "ON" : "OFF");
    Serial.println("Usage: stream on|off");
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
      Serial.printf("MOSFET %d %s\n", index, action.c_str());
      return;
    }
    if (action == "duty") action = getNextToken(command, pos);
    if (action.length() > 0) {
      uint8_t duty = (uint8_t)constrain(action.toInt(), 0, 10);
      setMosfetDuty(index - 1, duty);
      Serial.printf("MOSFET %d duty set to %u\n", index, duty);
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
    uint32_t freq = (uint32_t)max(10, getNextToken(command, pos).toInt());
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
    for (uint8_t i = 0; i < 4; i++) printAnalogValue(analogPins[i], analogNames[i]);
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

  Serial.println("Usage: i2c scan | i2c expander set <hex> | i2c expander bit <n> on|off");
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
  Serial.println("Usage: serial2 send <text>");
}

// -----------------------------------------------------------------------------
// Thermocouple SPI reads
// -----------------------------------------------------------------------------
uint32_t readMAX31855Direct(uint8_t csPin) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);
  uint32_t value = ((uint32_t)SPI.transfer16(0) << 16) | (uint32_t)SPI.transfer16(0);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return value;
}

// Assert expander CS bit (LOW), read 32 bits, deassert (HIGH).
// The expander I2C transaction adds ~200us of latency which is fine for
// occasional thermocouple reads.
uint32_t readMAX31855Expander(uint8_t expanderBit) {
  // Assert selected CS (bit LOW), keep all other bits at current state
  uint16_t assertedState = expanderState & ~(1u << expanderBit);
  expanderWrite(assertedState);
  delayMicroseconds(150);  // let CS settle after I2C

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  uint32_t value = ((uint32_t)SPI.transfer16(0) << 16) | (uint32_t)SPI.transfer16(0);
  SPI.endTransaction();

  // Deassert CS (restore full state with this bit HIGH)
  expanderWrite(expanderState);
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

// Read all 4 thermocouples and update tcTemp[].
float readAllThermocouples() {
  tcTemp[0] = decodeMAX31855Celsius(readMAX31855Direct(PIN_TC1_CS));
  tcTemp[1] = decodeMAX31855Celsius(readMAX31855Expander(TC2_EXP_BIT));
  tcTemp[2] = decodeMAX31855Celsius(readMAX31855Expander(TC3_EXP_BIT));
  tcTemp[3] = decodeMAX31855Celsius(readMAX31855Expander(TC4_EXP_BIT));
  return tcTemp[0];
}

// -----------------------------------------------------------------------------
// Current sensor helpers
// -----------------------------------------------------------------------------
float analogToVolts(uint16_t raw) {
  return raw * 3.3f / 4095.0f;
}

float voltsToCurrentAmps(float volts) {
  return (volts - CURRENT_ZERO_VOLTS) / CURRENT_SENS_V_PER_A;
}

float readCurrentAmps(uint8_t pin) {
  uint16_t raw = analogRead(pin);
  return voltsToCurrentAmps(analogToVolts(raw));
}

// -----------------------------------------------------------------------------
// Stream data sender
// -----------------------------------------------------------------------------
void sendStreamData() {
  readAllThermocouples();

  // Format: DATA:TC1=xx.xx,TC2=xx.xx,...,I_VP=x.xxx,...,M1=x,...
  Serial.print("DATA:");

  for (uint8_t i = 0; i < 4; i++) {
    Serial.printf("TC%u=", i + 1);
    if (isnan(tcTemp[i])) Serial.print("nan");
    else Serial.printf("%.2f", tcTemp[i]);
    Serial.print(",");
  }

  for (uint8_t i = 0; i < 4; i++) {
    float amps = readCurrentAmps(analogPins[i]);
    Serial.printf("I_%s=%.3f,", analogNames[i], amps);
  }

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

void expanderWrite(uint16_t state) {
  Wire.beginTransmission(I2C_EXPANDER_ADDR);
  Wire.write(state & 0xFF);
  Wire.write((state >> 8) & 0xFF);
  Wire.endTransmission();
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

  pinMode(PIN_VCTRL_1, OUTPUT);
  pinMode(PIN_VCTRL_2, OUTPUT);
  pinMode(PIN_PWM_FREQ, OUTPUT);

  // TC1 CS high (deasserted), VCTRL_2 low
  digitalWrite(PIN_VCTRL_1, HIGH);
  digitalWrite(PIN_VCTRL_2, LOW);
  digitalWrite(PIN_PWM_FREQ, LOW);

  pinMode(PIN_ANALOG_VP, INPUT);
  pinMode(PIN_ANALOG_VN, INPUT);
  pinMode(PIN_ANALOG_A, INPUT);
  pinMode(PIN_ANALOG_B, INPUT);
}

void initializeBuses() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  Serial.println("I2C and SPI buses initialized");
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

  // TC1 CS
  pinMode(PIN_TC1_CS, OUTPUT);
  digitalWrite(PIN_TC1_CS, HIGH);

  // Expander: all outputs HIGH (all CS deasserted)
  expanderWrite(expanderState);
}

void ecuHeartbeat() {
  static unsigned long lastHeartbeat = 0;
  static unsigned long lastStream    = 0;

  unsigned long now = millis();

  if (streamEnabled && (now - lastStream >= STREAM_INTERVAL_MS)) {
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
