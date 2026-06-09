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

static const uint8_t mosfetPins[] = {
  PIN_MOSFET_A,
  PIN_MOSFET_B,
  PIN_MOSFET_C,
  PIN_MOSFET_D,
  PIN_MOSFET_E
};

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
void initializePins();
void initializeSerial();
void initializeBuses();
void ecuHeartbeat();

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  initializeSerial();
  initializePins();
  initializeBuses();

  Serial.println("ECU boot complete");
  Serial.printf("ESP32 WROOM-32 ECU starting, debug %u baud\n", DEBUG_BAUD);
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop() {
  ecuHeartbeat();

  if (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n');
    Serial.print("ECU RX: ");
    Serial.println(incoming);
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    Serial2.println(command);
  }

  delay(10);
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

  digitalWrite(PIN_VCTRL_1, LOW);
  digitalWrite(PIN_VCTRL_2, LOW);
  digitalWrite(PIN_PWM_FREQ, LOW);

  pinMode(PIN_ANALOG_VP, INPUT);
  pinMode(PIN_ANALOG_VN, INPUT);
  pinMode(PIN_ANALOG_A, INPUT);
  pinMode(PIN_ANALOG_B, INPUT);
}

void initializeBuses() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  Serial.println("I2C and SPI buses initialized");
}

void ecuHeartbeat() {
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();
    Serial.println("ECU alive");
  }
}
