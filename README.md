# VetNode: Cattle Health Monitoring System (LoRa + GSM)

An ESP32-based health monitoring system for livestock. This project uses wearables (Nodes) on cows to track vital signs and a central Gateway to aggregate data and send descriptive SMS alerts.

## 🚀 Key Features

- **Non-blocking Architecture**: State-machine driven GSM and sensor handling prevents UI freezing and packet loss.
- **Robust LoRa Link**: Synchronized parameters (433MHz) with auto-recovery and signal strength monitoring.
- **Descriptive Alerts**: SMS messages include real-time temperature and heart rate values for immediate diagnosis.
- **Power Management**: Automated OLED timeout and configurable sleep timers to preserve gateway battery.
- **Persistent Config**: Change alert thresholds and phone numbers via the on-device menu; settings survive reboots.
- **Live Diagnostics**: Dashboard includes "Last Seen" tickers, LoRa signal bars, and detailed Serial debugging.

## 🛠️ Hardware Requirements

- **Microcontroller**: ESP32-WROOM-32 (Node & Gateway)
- **Node Sensors**:
  - MAX30102 (Heart Rate & SpO2)
  - DS18B20 (Temperature)
- **Communication modules**:
  - SX1278 LoRa Module (433MHz)
  - SIM800L GSM Module (Gateway only)
- **Timekeeping**: DS3231 RTC module (Node)

## 📌 Pin Mapping (Standardized)

### LoRa SX1278 (SPI) - Node & Gateway
| Signal | Pin |
|---|---|
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| SS (CS) | 5 |
| RST | 14 |
| DIO0 | 2 |

### Node Sensors & LEDs
| Component | Pin |
|---|---|
| I2C SDA (MAX/RTC) | 21 |
| I2C SCL (MAX/RTC) | 22 |
| DS18B20 OneWire | 15 |
| LED Green | 32 |
| LED Red | 33 |

### Gateway GSM & UI
| Component | Pin |
|---|---|
| GSM TX | 17 (Serial2) |
| GSM RX | 16 (Serial2) |
| OLED SDA | 21 |
| OLED SCL | 22 |
| BTN MENU | 25 |
| BTN SELECT | 26 |
| BUZZER | 27 |

## 📦 Software Dependencies

Install via Arduino Library Manager:
- **RadioLib** (v7.6.0+)
- **Adafruit SH110X** & **Adafruit GFX**
- **DallasTemperature** & **OneWire**
- **RTClib** & **MAX30105**
- **Preferences** (ESP32 Built-in)

## 📥 Getting Started

1. **Setup Node**:
   - Open `Node/Node.ino`.
   - Ensure `COW_ID` is set correctly.
   - Flash to the wearable ESP32.
2. **Setup Gateway**:
   - Open `Gateway/Gateway.ino`.
   - Access the **CONFIG** menu on-device to set your `GSM NUMBER`.
   - Adjust `SMS INTERVAL` and `ALERT LIMITS` as needed.
3. **Powering SIM800L**:
   - **Crucial**: Use a dedicated 5V power source capable of 2A peaks for the SIM800L.

## 🛰️ System Architecture

### Gateway Firmware (`Gateway.ino`)
- **`handleGsmState()`**: Non-blocking state machine managing the SIM800L module boot, text mode set, and SMS transmission.
- **`queueAlert(const char *reason)`**: Centralized alert handler that validates safety thresholds, manages the SMS cooldown (1-60m), and triggers the buzzer.
- **`renderDashboard()`**: Dynamic UI engine that displays real-time cow statistics, LoRa signal bars, and the "Last Seen" ticker.
- **`scanButtons()`**: Debounced input handler that manages UI navigation and wakes the OLED display from timeout.
- **`loadConfig()` / `saveConfig()`**: Uses ESP32 `Preferences` NVS to store user settings (GSM number, limits) across reboots.

### Cow Node Firmware (`Node.ino`)
- **`readTemperature()`**: Samples the DS18B20 sensor with signal isolation (disabling LEDs during read) to prevent noise-based glitches.
- **`checkHeartRateQuick()`**: Optimized pulse-detection algorithm that processes MAX30102 IR samples in real-time.
- **`loop()` Background Tasks**:
  - Transmits a "Heartbeat" (HB) packet every 2 seconds.
  - Monitors for remote "CMD" packets from the Gateway (LED Control/Pings).
  - Handles auto-retry logic for I2C sensors if hardware connection is lost.

## ⚠️ Safety Thresholds
Default cattle health ranges:
- **Temperature**: 37.0°C - 39.5°C
- **Heart Rate**: 50 - 90 BPM
- Alerts are sent immediately on breach, followed by a configurable cooldown (default 5m).

## 🚀 Deployment Checklist
- [ ] Verify **2A power supply** for Gateway SIM800L.
- [ ] Install all dependencies via Arduino Library Manager.
- [ ] Set your phone number in the **CONFIG -> GSM NUMBER** menu on first boot.
- [ ] Ensure the DS18B20 sensor is firmly connected to Node Pin 15.
