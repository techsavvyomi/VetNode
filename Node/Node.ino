#include "MAX30105.h"
#include "heartRate.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RTClib.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>

// ==========================================
// CONFIGURATION & PIN DEFINITIONS
// ==========================================
#define COW_ID "C01"

// LoRa SX1278 (SPI)
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// Sensors (I2C)
#define I2C_SDA 21
#define I2C_SCL 22

// Sensors (OneWire)
#define ONE_WIRE_BUS 33

// LoRa Settings
#define LORA_FREQ 433.0
#define LORA_BW 125.0
#define LORA_SF 7
#define LORA_CR 5
#define LORA_SYNC 0x12

// Deep Sleep Settings
#define SLEEP_TIME_MIN 30
#define uS_TO_S_FACTOR 1000000ULL

// Retry Logic
#define MAX_RETRIES 3

// ==========================================
// SENSOR CLASS
// ==========================================
class Sensors {
public:
  MAX30105 particleSensor;
  OneWire oneWire;
  DallasTemperature tempSensor;

  Sensors() : oneWire(ONE_WIRE_BUS), tempSensor(&oneWire) {}

  bool begin() {
    Wire.begin(I2C_SDA, I2C_SCL);
    tempSensor.begin();
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
      return false;

    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    return true;
  }

  float getTemperature() {
    tempSensor.requestTemperatures();
    return tempSensor.getTempCByIndex(0);
  }

  int getHeartRate() {
    long lastBeat = 0;
    int beatAvg = 0;
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
      long irValue = particleSensor.getIR();
      if (checkForBeat(irValue)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        float bpm = 60 / (delta / 1000.0);
        if (bpm < 255 && bpm > 20)
          beatAvg = (int)bpm;
      }
    }
    return beatAvg;
  }
};

// ==========================================
// LORA COMMUNICATION CLASS
// ==========================================
class LoRaComms {
public:
  SX1278 lora = new Module(LORA_SS, LORA_DIO0, LORA_RST);

  bool begin() {
    int state = lora.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC);
    return (state == RADIOLIB_ERR_NONE);
  }

  bool transmit(String packet) {
    int retryCount = 0;
    while (retryCount < MAX_RETRIES) {
      int state = lora.transmit(packet);
      if (state == RADIOLIB_ERR_NONE)
        return true;
      retryCount++;
      delay(100);
    }
    return false;
  }
};

// ==========================================
// MAIN PROGRAM
// ==========================================
RTC_DS3231 rtc;
Sensors sensors;
LoRaComms loraComms;

void setup() {
  Serial.begin(115200);

  if (!rtc.begin())
    Serial.println("RTC fail");
  if (!sensors.begin())
    Serial.println("Sensors fail");
  if (!loraComms.begin())
    Serial.println("LoRa fail");

  DateTime now = rtc.now();
  char timestamp[25];
  sprintf(timestamp, "%04d-%02d-%02d %02d:%02d", now.year(), now.month(),
          now.day(), now.hour(), now.minute());

  float temp = sensors.getTemperature();
  int hr = sensors.getHeartRate();

  String packet = String(COW_ID) + "," + String(temp, 1) + "," + String(hr) +
                  "," + String(timestamp);
  Serial.println("Transmitting: " + packet);

  if (loraComms.transmit(packet))
    Serial.println("Sent successfully");
  else
    Serial.println("Transmission failed");

  Serial.println("Entering deep sleep...");
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_MIN * 60 * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {}
