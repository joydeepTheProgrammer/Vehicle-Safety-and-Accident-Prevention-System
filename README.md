# Integrated Intelligent Vehicle Safety and Accident Prevention System
## System Overview & Documentation

---

## 1. System Description

This embedded system implements a **comprehensive vehicle safety platform** on an ATmega328P microcontroller. It continuously monitors the vehicle environment using 8 safety subsystems, generating alerts, controlling actuators, and sending emergency communications via GSM.

---

## 2. Hardware Components

| # | Component | Qty | Function | Interface |
|---|-----------|-----|----------|-----------|
| U1 | ATmega328P-PU | 1 | Central MCU (16 MHz) | — |
| Y1 | 16 MHz Crystal | 1 | Clock source | XTAL1/2 |
| J1–J4 | HC-SR04 Ultrasonic | 4 | Collision detection (F/R/L/R) | GPIO |
| J5 | MPU-6050 | 1 | Crash/rollover detection | I2C (0x68) |
| J6 | MQ-3 Module | 1 | Alcohol/drunk driver detection | ADC5 |
| J7 | DHT11 | 1 | Cabin temp/humidity | 1-Wire GPIO |
| J8–J9 | IR Sensor (TCRT5000) | 2 | Lane departure detection | GPIO |
| LDR1 | Photoresistor | 1 | Auto headlight control | ADC4 |
| J10 | 16×2 LCD + PCF8574 | 1 | Status display | I2C (0x27) |
| J11 | SIM800L GSM | 1 | Emergency SMS | Soft UART |
| J12 | NEO-6M GPS | 1 | Location tracking | Soft UART |
| K1 | Relay 5V SPDT | 1 | Engine cut-off | NPN Q1 |
| K2 | Relay 5V SPDT | 1 | Brake assist | NPN Q2 |
| K3 | Relay 5V SPDT | 1 | Auto headlights | NPN Q3 |
| Q1–Q3 | 2N2222 NPN | 3 | Relay drivers | — |
| BZ1 | Passive Buzzer 5V | 1 | Audible alerts | GPIO (PB5) |
| LED1–LED5 | LEDs (Red×4, Yellow×1) | 5 | Visual warnings | GPIO |
| U2 | AMS1117-4.0 | 1 | 4V supply for SIM800L | — |
| U3 | L7805 | 1 | 5V supply for MCU | — |

---

## 3. Pin Map (ATmega328P)

### Port B (Digital)
| Pin | Signal | Direction | Description |
|-----|--------|-----------|-------------|
| PB0 | US_FRONT_TRIG | OUT | Front ultrasonic trigger |
| PB1 | US_FRONT_ECHO | IN  | Front ultrasonic echo |
| PB2 | US_REAR_TRIG  | OUT | Rear ultrasonic trigger |
| PB3 | US_REAR_ECHO  | IN  | Rear ultrasonic echo |
| PB4 | HEADLIGHT_RELAY | OUT | Headlight relay (NPN Q3 base via R8) |
| PB5 | BUZZER        | OUT | Piezo buzzer (via R9 100Ω) |
| PB6 | LED_COLLISION  | OUT | Red LED – collision warning |
| PB7 | LED_ALCOHOL    | OUT | Yellow LED – alcohol warning |

### Port C (Mixed Analog/Digital)
| Pin | Signal | Direction | Description |
|-----|--------|-----------|-------------|
| PC0 | US_LEFT_TRIG  | OUT | Left ultrasonic trigger |
| PC1 | US_LEFT_ECHO  | IN  | Left ultrasonic echo |
| PC2 | US_RIGHT_TRIG | OUT | Right ultrasonic trigger |
| PC3 | US_RIGHT_ECHO | IN  | Right ultrasonic echo |
| PC4 | IR_LEFT / LDR_ADC | IN | IR left lane sensor (digital) / LDR (ADC4) |
| PC5 | IR_RIGHT / MQ3_ADC | IN | IR right lane sensor / MQ-3 (ADC5) |
| PC4 | SDA | I/O | I2C data (shared with above; muxed in firmware) |
| PC5 | SCL | OUT | I2C clock (shared) |

### Port D (Digital)
| Pin | Signal | Direction | Description |
|-----|--------|-----------|-------------|
| PD0 | ENGINE_RELAY | OUT | Engine cut relay (NPN Q1 base via R6) |
| PD1 | BRAKE_RELAY  | OUT | Brake relay (NPN Q2 base via R7) |
| PD2 | GSM_PWR      | OUT | SIM800L PWRKEY |
| PD3 | GSM_TX       | OUT | Soft UART TX → SIM800L RX |
| PD4 | GSM_RX       | IN  | Soft UART RX ← SIM800L TX |
| PD5 | GPS_TX       | OUT | Soft UART TX → NEO-6M RX |
| PD6 | GPS_RX       | IN  | Soft UART RX ← NEO-6M TX |
| PD7 | DHT11_DATA   | I/O | DHT11 single-wire data |

---

## 4. Safety Features

### 4.1 Collision Avoidance
- **Sensors**: 4× HC-SR04 ultrasonic (front, rear, left, right)
- **Warning Zone**: 30–80 cm → audible warning beeps
- **Critical Zone**: < 30 cm → brake relay engaged, continuous alarm
- **Display**: Shows direction and approximate distance

### 4.2 Drunk Driver Detection
- **Sensor**: MQ-3 analog alcohol sensor (ADC5)
- **Warm-up**: 30 seconds required after power-on
- **Warning Level**: ADC > 300 → audible warning, LED on
- **Danger Level**: ADC > 500 → engine relay cut, emergency SMS sent

### 4.3 Crash / Rollover Detection
- **Sensor**: MPU-6050 6-axis IMU (I2C at 0x68)
- **Crash threshold**: |acceleration vector| > 3.5g
- **Rollover threshold**: Roll angular rate > 180°/s
- **Response**: Emergency SMS with GPS coordinates, brake assist engaged

### 4.4 Lane Departure Warning
- **Sensors**: 2× IR sensors (front-left, front-right)
- **Activation**: Only when GPS speed > 10 km/h (not in parking)
- **Response**: Audible warning, LED, LCD message showing drift direction

### 4.5 Driver Drowsiness Detection
- **Method**: GPS speed heuristic – speed variance < 5 km/h for 30+ seconds
- **Activation**: Speed > 15 km/h
- **Response**: After 2 consecutive stable windows → warning alert

### 4.6 Automatic Headlights
- **Sensor**: LDR + 10kΩ voltage divider (ADC4)
- **Threshold**: ADC4 > 700 (approx. lux threshold)
- **Response**: Headlight relay K3 activated

### 4.7 Cabin Overheat / Fire Detection
- **Sensor**: DHT11 (temperature, ±2°C accuracy)
- **Warning**: > 55°C → warning message
- **Danger**: > 65°C → evacuation alert, continuous critical alarm

### 4.8 GPS Emergency Beacon
- **Module**: NEO-6M GPS at 9600 baud (soft UART)
- **NMEA Parsing**: $GPRMC sentences (lat, lon, speed, time)
- **Integration**: Coordinates embedded in all emergency SMS messages

---

## 5. Timing Architecture

| Timer | Mode | Period | Use |
|-------|------|--------|-----|
| Timer0 | CTC (compare A) | 1 ms | System millisecond counter (`g_system_ms`) |
| Timer1 | Normal (free-run) | 0.5 µs/tick | Ultrasonic echo pulse width measurement |

---

## 6. Communication Protocols

| Protocol | Peripheral | Config |
|----------|-----------|--------|
| HW UART0 | Debug / PC | 9600 baud, 8N1 |
| Soft UART (PD3/PD4) | SIM800L GSM | 9600 baud, 8N1 |
| Soft UART (PD5/PD6) | NEO-6M GPS | 9600 baud, 8N1 |
| I2C / TWI (PC4/PC5) | MPU-6050 (0x68) + LCD (0x27) | 400 kHz |
| ADC (ADC4, ADC5) | LDR, MQ-3 | 125 kHz, 10-bit |
| 1-Wire GPIO (PD7) | DHT11 | Bit-banged |

---

## 7. Power Supply

```
12V Battery
    │
    ├──→ L7805 (U3) ──→ +5V (MCU, sensors, relays, LCD, GPS)
    │         │
    │         ├──→ C6 (470µF) : input filter
    │         └──→ C7 (100µF) : output filter
    │
    └──→ AMS1117-4.0 (U2) ──→ +4V (SIM800L)
              └──→ C5 (100µF tantalum) : output filter
```

> **Note**: SIM800L draws up to 2A burst current during GSM transmission.
> Use a bulk capacitor (≥1000µF) on the 4V rail to handle transients.

---

## 8. Build Instructions

### Prerequisites
- **Toolchain**: AVR-GCC 5.4+ (`avr-gcc`, `avr-objcopy`, `avr-size`)
- **Programmer**: `avrdude` with USBasp or Arduino bootloader

### Build and Flash
```bash
cd firmware/
make           # Compile to build/vehicle_safety_system.hex
make size      # Check memory usage
make flash     # Flash via USBasp
# OR
make flash_uno PORT=COM3   # Via Arduino bootloader
```

### Fuse Bits (ATmega328P)
```
Low:  0xFF  (16 MHz external crystal, full swing, no clock divide)
High: 0xDE  (Serial programming enabled, brownout at 2.7V)
Ext:  0xFF  (Default)
```
```bash
avrdude -c usbasp -p atmega328p -U lfuse:w:0xFF:m -U hfuse:w:0xDE:m
```

---

## 9. Block Diagram
<img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/489f5374-5657-4c41-a44a-805226a9c7f8" />


---

## 10. Bill of Materials

| Reference | Value | Quantity | Notes |
|-----------|-------|----------|-------|
| U1 | ATmega328P-PU | 1 | DIP-28 |
| U2 | AMS1117-4.0 | 1 | SOT-223 |
| U3 | L7805 | 1 | TO-220 |
| Y1 | 16 MHz crystal | 1 | HC-49/S |
| J1–J4 | HC-SR04 | 4 | Via 4-pin header |
| J5 | MPU-6050 module | 1 | GY-521 breakout |
| J6 | MQ-3 module | 1 | With onboard comparator |
| J7 | DHT11 | 1 | 3-pin package |
| J8–J9 | TCRT5000 IR module | 2 | Digital output |
| LDR1 | GL5528 LDR | 1 | 10–20kΩ dark resistance |
| J10 | 16×2 LCD + PCF8574 | 1 | I2C backpack included |
| J11 | SIM800L GSM | 1 | With SIM card slot |
| J12 | NEO-6M GPS | 1 | Ublox module |
| K1–K3 | 5V SPDT Relay | 3 | 10A contacts |
| Q1–Q3 | 2N2222 NPN | 3 | TO-92 |
| D1–D3 | 1N4007 | 3 | Flyback protection |
| BZ1 | Passive buzzer 5V | 1 | 3–5V, 85dB |
| LED1,3,5 | Red LED 3mm | 3 | 2V, 20mA |
| LED2 | Yellow LED 3mm | 1 | |
| LED4 | Amber LED 3mm | 1 | |
| R1 | 10kΩ | 1 | RESET pull-up |
| R2,R3 | 4.7kΩ | 2 | I2C pull-ups |
| R4 | 10kΩ | 1 | DHT11 data pull-up |
| R5 | 10kΩ | 1 | LDR voltage divider |
| R6–R8 | 1kΩ | 3 | NPN base resistors |
| R9 | 100Ω | 1 | Buzzer current limit |
| R10–R14 | 330Ω | 5 | LED current limit |
| C1,C2 | 22pF | 2 | Crystal load caps |
| C3 | 100nF | 1 | MCU decoupling |
| C4 | 10µF | 1 | MCU bulk decoupling |
| C5 | 100µF tantalum | 1 | GSM rail output |
| C6 | 470µF | 1 | 7805 input filter |
| C7 | 100µF | 1 | 7805 output filter |

---

