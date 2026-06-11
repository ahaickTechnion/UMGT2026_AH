#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// -----------------------------------------------------------------------------
// Defines
// -----------------------------------------------------------------------------
#define DEBUG_BAUD        115200
#define ECU_SERIAL_BAUD   115200

// UART2 pins for external ECU serial comms (Serial2)
#define PIN_UART2_RX      16  // U2_RXD
#define PIN_UART2_TX      17  // U2_TXD

// SPI pins used for thermocouple interface
#define PIN_SPI_SCK       18  // SCK
#define PIN_SPI_MISO      19  // MISO
#define PIN_SPI_MOSI      23  // MOSI if needed by library

// I2C pins used for GPIO expander / PCF857x
#define PIN_I2C_SDA       21  // SDA
#define PIN_I2C_SCL       22  // SCL

// MOSFET driver PWM outputs
#define PIN_MOSFET_A      32
#define PIN_MOSFET_B      33
#define PIN_MOSFET_C      25
#define PIN_MOSFET_D      26
#define PIN_MOSFET_E      27

// Voltage control bits / general outputs
#define PIN_VCTRL_1       12
#define PIN_VCTRL_2       13
#define PIN_PWM_FREQ      15

// Analog-only inputs
#define PIN_ANALOG_VP     36
#define PIN_ANALOG_VN     39
#define PIN_ANALOG_A      34
#define PIN_ANALOG_B      35

// Thermocouple chip select available directly on PIN_VCTRL_1
#define PIN_TC1_CS        PIN_VCTRL_1

// Piezo/pwm output
#define PIN_PIEZO         PIN_PWM_FREQ

// I2C expander addresses to probe
#define I2C_EXPANDER_ADDR     0x20
#define I2C_EXPANDER_ADDR_ALT 0x27

// PWM configuration for piezo testing
#define PWM_CHANNEL        0
#define PWM_RESOLUTION     8
#define PWM_DEFAULT_FREQ   2000

#define MOSFET_PWM_FREQ    10000
#define MOSFET_PWM_RESOLUTION 8

static uint32_t mosfetFrequency = MOSFET_PWM_FREQ;

static const uint8_t mosfetPins[] = {
  PIN_MOSFET_A,
  PIN_MOSFET_B,
  PIN_MOSFET_C,
  PIN_MOSFET_D,
  PIN_MOSFET_E
};

static const uint8_t mosfetChannels[] = {
  1,
  2,
  3,
  4,
  5
};

static uint8_t mosfetDuty[sizeof(mosfetPins) / sizeof(mosfetPins[0])] = {0};

static const uint8_t analogPins[] = {
  PIN_ANALOG_VP,
  PIN_ANALOG_VN,
  PIN_ANALOG_A,
  PIN_ANALOG_B
};

static const char *analogNames[] = {
  "VP",
  "VN",
  "A",
  "B"
};

uint32_t piezoFrequency = PWM_DEFAULT_FREQ;
uint8_t piezoDuty = 0;
uint16_t expanderState = 0xFFFF;
String commandBuffer = "";

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
void setMosfetDuty(uint8_t index, uint8_t duty);
void setAllMosfets(uint8_t duty);
void setMosfetFrequency(uint32_t frequency);
void handleI2c(String &command, int &pos);
void handleSerial2(String &command, int &pos);

void setPiezoFrequency(uint32_t frequency);
void setPiezoDuty(uint8_t duty);
uint16_t readThermocoupleRaw(uint8_t csPin);
void printAnalogValue(uint8_t pin, const char *name);
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
    if (c == '\r') {
      continue;
    }
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
  if (line.length() == 0) {
    return;
  }

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
  } else if (token == "i2c") {
    handleI2c(working, pos);
  } else if (token == "serial2") {
    handleSerial2(working, pos);
  } else {
    Serial.println("Unknown command. Type help for a command list.");
  }
}

void processSerial2Bridge() {
  if (!Serial2.available()) {
    return;
  }

  String incoming = Serial2.readStringUntil('\n');
  if (incoming.length() > 0) {
    Serial.print("[Serial2] ");
    Serial.println(incoming);
  }
}

String getNextToken(String &line, int &pos) {
  line.trim();
  if (pos >= line.length()) {
    return String();
  }

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

void printHelp() {
  Serial.println("--- ESP32 Terminal Test Controller ---");
  Serial.println("help | ?                     - Show this help");
  Serial.println("status                     - Show current pin/test status");
  Serial.println("mosfet all duty <0-10>      - Set all MOSFET PWM level");
  Serial.println("mosfet <1-5> duty <0-10>     - Set one MOSFET PWM level");
  Serial.println("mosfet freq <hz>            - Set MOSFET PWM frequency 1-20000");
  Serial.println("mosfet all on|off          - Toggle all MOSFET outputs");
  Serial.println("mosfet <1-5> on|off         - Toggle one MOSFET output");
  Serial.println("piezo on|off               - Enable or disable piezo PWM");
  Serial.println("piezo duty <0-255>         - Set piezo PWM duty");
  Serial.println("piezo freq <hz>            - Set piezo PWM frequency");
  Serial.println("analog all                 - Read all analog inputs");
  Serial.println("analog vp|vn|a|b           - Read a single analog input");
  Serial.println("tc read 1                  - Read raw data from thermocouple 1 (CS=GPIO12)");
  Serial.println("i2c scan                   - Scan I2C bus for devices");
  Serial.println("i2c expander set <hex>     - Write raw state to I2C expander");
  Serial.println("i2c expander bit <n> on|off - Toggle one expander bit");
  Serial.println("serial2 send <text>        - Send raw text to Serial2");
}

void printStatus() {
  Serial.println("--- Status ---");

  Serial.print("Piezo frequency: ");
  Serial.print(piezoFrequency);
  Serial.println(" Hz");

  Serial.print("Piezo duty: ");
  Serial.println(piezoDuty);

  Serial.print("MOSFET freq: ");
  Serial.print(mosfetFrequency);
  Serial.println(" Hz");

  Serial.print("MOSFET states: ");
  for (uint8_t i = 0; i < sizeof(mosfetPins); i++) {
    Serial.printf("%u:%u", i + 1, mosfetDuty[i]);
    if (i < sizeof(mosfetPins) - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();

  printAnalogValue(PIN_ANALOG_VP, "VP");
  printAnalogValue(PIN_ANALOG_VN, "VN");
  printAnalogValue(PIN_ANALOG_A, "A");
  printAnalogValue(PIN_ANALOG_B, "B");
}

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
      uint8_t duty = action == "on" ? 10 : 0;
      setAllMosfets(duty);
      Serial.printf("MOSFETs all %s\n", action.c_str());
      return;
    }
    if (action == "duty") {
      String value = getNextToken(command, pos);
      uint8_t duty = (uint8_t)constrain(value.toInt(), 0, 10);
      setAllMosfets(duty);
      Serial.printf("MOSFETs all set to %u\n", duty);
      return;
    }
    if (action.length() > 0) {
      uint8_t duty = (uint8_t)constrain(action.toInt(), 0, 10);
      setAllMosfets(duty);
      Serial.printf("MOSFETs all set to %u\n", duty);
      return;
    }
  }

  int index = target.toInt();
  if (index >= 1 && index <= (int)(sizeof(mosfetPins) / sizeof(mosfetPins[0]))) {
    String action = getNextToken(command, pos);
    if (action == "on" || action == "off") {
      uint8_t duty = action == "on" ? 10 : 0;
      setMosfetDuty(index - 1, duty);
      Serial.printf("MOSFET %d %s\n", index, action.c_str());
      return;
    }
    if (action == "duty") {
      String value = getNextToken(command, pos);
      uint8_t duty = (uint8_t)constrain(value.toInt(), 0, 10);
      setMosfetDuty(index - 1, duty);
      Serial.printf("MOSFET %d duty set to %u\n", index, duty);
      return;
    }
    if (action.length() > 0) {
      uint8_t duty = (uint8_t)constrain(action.toInt(), 0, 10);
      setMosfetDuty(index - 1, duty);
      Serial.printf("MOSFET %d duty set to %u\n", index, duty);
      return;
    }
  }

  Serial.println("Usage: mosfet freq <hz> | mosfet all duty <0-10> | mosfet <1-5> duty <0-10> | mosfet all on|off | mosfet <1-5> on|off");
}

void handlePiezo(String &command, int &pos) {
  String action = getNextToken(command, pos);
  if (action == "on") {
    setPiezoDuty(piezoDuty > 0 ? piezoDuty : 128);
    Serial.println("Piezo enabled");
  } else if (action == "off") {
    setPiezoDuty(0);
    Serial.println("Piezo disabled");
  } else if (action == "duty") {
    String value = getNextToken(command, pos);
    uint8_t duty = (uint8_t)constrain(value.toInt(), 0, 255);
    setPiezoDuty(duty);
    Serial.printf("Piezo duty set to %u\n", duty);
  } else if (action == "freq") {
    String value = getNextToken(command, pos);
    uint32_t freq = value.toInt();
    if (freq < 10) {
      freq = 10;
    }
    setPiezoFrequency(freq);
    Serial.printf("Piezo frequency set to %u Hz\n", freq);
  } else {
    Serial.println("Usage: piezo on|off|duty <0-255>|freq <hz>");
  }
}

void handleAnalog(String &command, int &pos) {
  String input = getNextToken(command, pos);
  if (input == "all") {
    printAnalogValue(PIN_ANALOG_VP, "VP");
    printAnalogValue(PIN_ANALOG_VN, "VN");
    printAnalogValue(PIN_ANALOG_A, "A");
    printAnalogValue(PIN_ANALOG_B, "B");
    return;
  }

  if (input == "vp") {
    printAnalogValue(PIN_ANALOG_VP, "VP");
  } else if (input == "vn") {
    printAnalogValue(PIN_ANALOG_VN, "VN");
  } else if (input == "a") {
    printAnalogValue(PIN_ANALOG_A, "A");
  } else if (input == "b") {
    printAnalogValue(PIN_ANALOG_B, "B");
  } else {
    Serial.println("Usage: analog all | analog vp|vn|a|b");
  }
}

void handleThermocouple(String &command, int &pos) {
  String action = getNextToken(command, pos);
  if (action == "read") {
    String which = getNextToken(command, pos);
    if (which == "1" || which == "tc1") {
      uint16_t raw = readThermocoupleRaw(PIN_TC1_CS);
      Serial.printf("TC1 raw: 0x%04X (%u)\n", raw, raw);
      return;
    }
  }

  Serial.println("Usage: tc read 1");
}

void handleI2c(String &command, int &pos) {
  String action = getNextToken(command, pos);
  if (action == "scan") {
    i2cScan();
    return;
  }

  if (action == "expander") {
    String sub = getNextToken(command, pos);
    if (sub == "set") {
      String value = getNextToken(command, pos);
      uint16_t state = (uint16_t)strtoul(value.c_str(), NULL, 0);
      expanderState = state;
      expanderWrite(expanderState);
      Serial.printf("Expander raw state written: 0x%04X\n", expanderState);
      return;
    }

    if (sub == "bit") {
      String bitText = getNextToken(command, pos);
      String action2 = getNextToken(command, pos);
      int bitIndex = bitText.toInt();
      if (bitIndex >= 0 && bitIndex < 16 && (action2 == "on" || action2 == "off")) {
        if (action2 == "on") {
          expanderState |= (1 << bitIndex);
        } else {
          expanderState &= ~(1 << bitIndex);
        }
        expanderWrite(expanderState);
        Serial.printf("Expander bit %d %s\n", bitIndex, action2.c_str());
        return;
      }
    }
  }

  Serial.println("Usage: i2c scan | i2c expander set <hex> | i2c expander bit <n> on|off");
}

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
  if (index >= sizeof(mosfetPins) / sizeof(mosfetPins[0])) {
    return;
  }
  duty = constrain(duty, 0, 10);
  mosfetDuty[index] = duty;
  uint8_t hwDuty = (uint8_t)((uint32_t)duty * 255u / 10u);
  ledcWrite(mosfetChannels[index], hwDuty);
}

void setAllMosfets(uint8_t duty) {
  for (uint8_t i = 0; i < sizeof(mosfetPins) / sizeof(mosfetPins[0]); i++) {
    setMosfetDuty(i, duty);
  }
}

void setMosfetFrequency(uint32_t frequency) {
  mosfetFrequency = frequency;
  for (uint8_t i = 0; i < sizeof(mosfetPins) / sizeof(mosfetPins[0]); i++) {
    ledcSetup(mosfetChannels[i], mosfetFrequency, MOSFET_PWM_RESOLUTION);
    ledcAttachPin(mosfetPins[i], mosfetChannels[i]);
    ledcWrite(mosfetChannels[i], mosfetDuty[i]);
  }
}

uint16_t readThermocoupleRaw(uint8_t csPin) {
  digitalWrite(csPin, HIGH);
  delayMicroseconds(1);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);
  uint16_t value = SPI.transfer16(0x0000);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  return value;
}

void printAnalogValue(uint8_t pin, const char *name) {
  uint16_t raw = analogRead(pin);
  float voltage = raw * 3.3f / 4095.0f;
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
  if (!found) {
    Serial.println("No I2C devices found.");
  }
}

void expanderWrite(uint16_t state) {
  Wire.beginTransmission(I2C_EXPANDER_ADDR);
  Wire.write(state & 0xFF);
  Wire.write((state >> 8) & 0xFF);
  Wire.endTransmission();
}

// -----------------------------------------------------------------------------
// Initialization helpers
// -----------------------------------------------------------------------------
void initializeSerial() {
  Serial.begin(DEBUG_BAUD);
  while (!Serial) {
    ;
  }

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

  for (uint8_t i = 0; i < sizeof(mosfetPins) / sizeof(mosfetPins[0]); i++) {
    ledcSetup(mosfetChannels[i], mosfetFrequency, MOSFET_PWM_RESOLUTION);
    ledcAttachPin(mosfetPins[i], mosfetChannels[i]);
    ledcWrite(mosfetChannels[i], mosfetDuty[i]);
  }

  pinMode(PIN_TC1_CS, OUTPUT);
  digitalWrite(PIN_TC1_CS, HIGH);
}

void ecuHeartbeat() {
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();
  }
}
