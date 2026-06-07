Pin Assignments 

# ESP32 Shield Pinout README
## ESPShield - Pin Assignment Reference

---

## SPI - Thermocouples (x4)

| Connector | SCK | MISO | CS | GND | 3.3V |
|-----------|-----|------|----|-----|------|
| J3 (TC1)  | SCK | MISO | CS51 | GND | 3.3V |
| J4 (TC2)  | SCK | MISO | CS52 | GND | 3.3V |
| J5 (TC3)  | SCK | MISO | CS53 | GND | 3.3V |
| J6 (TC4)  | SCK | MISO | CS54 | GND | 3.3V |

**ESP32 SPI pins:**
- SCK -> D18 (GPIO18)
- MISO -> D19 (GPIO19)
- CS51 -> GPIO pin per voltage control bit mapping (D12)
- CS52, CS53, CS54 -> see GPIO Expander section

---

## MOSFET Drivers - PWM Output

| Connector | Signal | ESP32 Pin |
|-----------|--------|-----------|
| J7  | D32 | GPIO32 |
| J8  | D33 | GPIO33 |
| J9  | D25 | GPIO25 |
| J10 | D26 | GPIO26 |
| J11 | D27 | GPIO27 |

All MOSFET driver connectors are 2-pin (GND + signal).

---

## ESP32 DevBoard C - Full Pin Mapping (J1/J2)

### Left Side (J1)
| Pin | Signal |
|-----|--------|
| EN  | Enable |
| VP  | GPIO36 (input only) |
| VN  | GPIO39 (input only) |
| D34 | GPIO34 (input only) |
| D35 | GPIO35 (input only) |
| D32 | GPIO32 |
| D33 | GPIO33 |
| D25 | GPIO25 |
| D26 | GPIO26 |
| D27 | GPIO27 |
| D14 | GPIO14 |
| D12 | GPIO12 |
| D13 | GPIO13 |
| GND | GND |
| VIN | VIN |

### Right Side (J2)
| Pin | Signal |
|-----|--------|
| CTX | CAN TX |
| SCL | I2C SCL |
| U0_TXD | UART0 TX |
| U0_RXD | UART0 RX |
| SDA | I2C SDA |
| MISO | GPIO19 |
| SCK | GPIO18 |
| D5  | GPIO5 |
| U2_TXD | UART2 TX (GPIO17) |
| U2_RXD | UART2 RX (GPIO16) |
| CRX | CAN RX |
| D2  | GPIO2 |
| D15 | GPIO15 |
| GND | GND |
| 3.3V | 3.3V |

---

## ESP Power (J23)
| Pin | Signal |
|-----|--------|
| 1   | GND |
| 2   | VIN |

---

## CAN Bus - ESC (J12)
| Pin | Signal |
|-----|--------|
| 1   | 3.3V |
| 2   | GND |
| 3   | CTX (CAN TX) |
| 4   | CRX (CAN RX) |
| 5   | CANH |
| 6   | CANL |

## CAN Bus - Passthrough (J18)
| Pin | Signal |
|-----|--------|
| 1   | CANL |
| 2   | CANH |

---

## I2C - GPIO Expander (J13, J14, J15, J17)
Connected via SCL/SDA (GPIO22/GPIO21).

| Connector | Signals |
|-----------|---------|
| J17 (10-pin) | I2C bus pins 1-10 |
| J13 | Conn_01x10 |
| J14 | Conn_01x10 |
| J15 (CS pins) | CS5, CS4, CS3, CS2, CS1 |

CS pins routed through GPIO expander PCF857x:
- CS51, CS52, CS53, CS54 (thermocouple chip selects)
- CS5, CS4, CS3, CS2, CS1 (additional selects)

---

## Serial Ports

### Serial 1 (J20)
| Pin | Signal |
|-----|--------|
| 1   | U0_TXD |
| 2   | U0_RXD |
| 3   | GND |

### Serial 2 (J21)
| Pin | Signal |
|-----|--------|
| 1   | U2_TXD (GPIO17) |
| 2   | U2_RXD (GPIO16) |
| 3   | GND |

---

## Analog Inputs (J24)
| Pin | Signal | ESP32 Pin |
|-----|--------|-----------|
| 1   | VP     | GPIO36 |
| 2   | VN     | GPIO39 |
| 3   | D34    | GPIO34 |
| 4   | D35    | GPIO35 |

---

## Voltage Control Bit (J16)
| Pin | Signal |
|-----|--------|
| 1   | GND |
| 2   | D12 (GPIO12) |
| 3   | D13 (GPIO13) |
| 4   | CS55 |
| 5   | 3.3V |

---

## Frequency Control PWM Square Wave (J19)
| Pin | Signal | ESP32 Pin |
|-----|--------|-----------|
| 1   | GND    | GND |
| 2   | D15    | GPIO15 |

---

## Extra I/O (J22)
| Pin | Signal | ESP32 Pin |
|-----|--------|-----------|
| 1   | D5     | GPIO5 |
| 2   | D2     | GPIO2 |
| 3   | D14    | GPIO14 |

---

## ESP32 Pin Summary Table

| GPIO | Function | Peripheral |
|------|----------|------------|
| GPIO2  | D2       | Extra I/O |
| GPIO5  | D5       | Extra I/O / SPI CS |
| GPIO12 | D12      | Voltage Control Bit |
| GPIO13 | D13      | Voltage Control Bit |
| GPIO14 | D14      | Extra I/O |
| GPIO15 | D15      | Frequency Control PWM |
| GPIO16 | U2_RXD   | Serial 2 RX |
| GPIO17 | U2_TXD   | Serial 2 TX |
| GPIO18 | SCK      | SPI Clock (Thermocouples) |
| GPIO19 | MISO     | SPI MISO (Thermocouples) |
| GPIO21 | SDA      | I2C Data (GPIO Expander) |
| GPIO22 | SCL      | I2C Clock (GPIO Expander) |
| GPIO25 | D25      | MOSFET Driver PWM |
| GPIO26 | D26      | MOSFET Driver PWM |
| GPIO27 | D27      | MOSFET Driver PWM |
| GPIO32 | D32      | MOSFET Driver PWM |
| GPIO33 | D33      | MOSFET Driver PWM |
| GPIO34 | D34      | Analog In (input only) |
| GPIO35 | D35      | Analog In (input only) |
| GPIO36 | VP       | Analog In (input only) |
| GPIO39 | VN       | Analog In (input only) |
| CAN TX | CTX     | ESC CAN Bus |
| CAN RX | CRX     | ESC CAN Bus |
| UART0 TX | U0_TXD | Serial 1 TX |
| UART0 RX | U0_RXD | Serial 1 RX |

---

*Generated from ESPShield schematic. CS51-CS55 chip selects for thermocouples are routed through PCF857x I2C GPIO expander.*