# VetNode: Cattle Health Monitoring System (LoRa + GSM)

An ESP32-based health monitoring system for livestock. This project uses wearables (Nodes) on cows to track vital signs and a central Gateway to aggregate data and send SMS alerts.

## 🚀 Overview

The system consists of two main components:
1. **Cow Node**: A battery-powered wearable that samples heart rate and temperature, then transmits the data via LoRa (433MHz). It uses Deep Sleep to maximize battery life.
2. **Gateway**: A mains-powered receiver that listens for LoRa packets, monitors health thresholds, and sends emergency SMS alerts or hourly summaries via a SIM800L GSM module.

## 🛠️ Hardware Requirements

- **Microcontroller**: ESP32-WROOM-32 (Node & Gateway)
- **Node Sensors**:
  - MAX30102 (Heart Rate & SpO2)
  - DS18B20 (Temperature)
- **Communication**:
  - SX1278 LoRa Module (433MHz)
  - SIM800L GSM Module (Gateway only)
- **Timekeeping**: DS3231 RTC module (Node for wake-up, Gateway for logging)

## 📌 Pin Mapping

### Node & Gateway (LoRa SPI)
| Component | Pin |
|---|---|
| SCK | 5 |
| MISO | 19 |
| MOSI | 27 |
| SS/CS | 18 |
| RST | 14 |
| DIO0 | 26 |

### Node Sensors
| Component | Pin |
|---|---|
| MAX30102 SDA | 21 |
| MAX30102 SCL | 22 |
| DS18B20 OneWire | 33 |

### Gateway GSM
| Component | Pin |
|---|---|
| TX | 17 (Serial2) |
| RX | 16 (Serial2) |

## 📦 Software Dependencies

Ensure these libraries are installed in your Arduino environment:
- [RadioLib](https://github.com/jgromes/RadioLib)
- [SparkFun MAX3010x Pulse and Proximity Sensor Library](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)
- [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)
- [OneWire](https://github.com/PaulStoffregen/OneWire)
- [RTClib](https://github.com/adafruit/RTClib)

## 📥 Getting Started

1. **Setup Node**:
   - Open `Node/Node.ino`.
   - Update `COW_ID` if necessary.
   - Flash to your wearable ESP32.
2. **Setup Gateway**:
   - Open `Gateway/Gateway.ino`.
   - Update `ADMIN_PHONE` with your mobile number.
   - Flash to your central receiver ESP32.
3. **Powering SIM800L**:
   - **Crucial**: Use a power source capable of providing 2A peak current for the SIM800L module.

## 🧪 Testing

We have provided individual test scripts in the `Tests/` directory to verify hardware modules:
- `GSM_Test.ino`: Check AT commands and network status.
- `MAX30102_Test.ino`: View raw sensor readings.
- `DS18B20_Test.ino`: Confirm temperature readings.
- `LoRa_Sender/Receiver_Test.ino`: Verify wireless range and connectivity.

## ⚠️ Safety Thresholds
The default alerting thresholds in `Gateway.ino` are:
- **Temperature**: 37.5°C to 39.5°C
- **Heart Rate**: 48 to 84 BPM
