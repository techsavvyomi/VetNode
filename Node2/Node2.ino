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
#define COW_ID "2"
#define COW_ID_NUM 2

// LoRa SX1278 (SPI)
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2

// Sensors (I2C) - MAX30102 & RTC
#define I2C_SDA 21
#define I2C_SCL 22

// Sensors (OneWire) - DS18B20
#define ONE_WIRE_BUS 15

// Status LEDs
#define LED_GREEN 32
#define LED_RED 33

// LoRa Settings
#define LORA_FREQ 433.0
#define LORA_BW 125.0
#define LORA_SF 7
#define LORA_CR 5
#define LORA_SYNC 0x12

// TDMA: stagger TX by node ID to avoid collisions
#define HB_INTERVAL_MS 2000
#define TDMA_SLOT_MS   500
#define TDMA_OFFSET    ((COW_ID_NUM - 1) * TDMA_SLOT_MS)

// Heart Rate Validation
#define HR_RATE_SIZE     4     // Rolling average window
#define HR_WARMUP_BEATS  4     // Discard first N beats (sensor settling)
#define HR_MIN_INTERVAL  300   // Fastest valid beat gap ms (200 BPM)
#define HR_MAX_INTERVAL  2000  // Slowest valid beat gap ms (30 BPM)
#define HR_OUTLIER_PCT   30    // Reject if >30% off from running avg
#define HR_STALE_MS      30000 // Clear HR after 30s with no beats
#define HR_DATA_TX_MS    5000  // Min interval between DATA packets

// ==========================================
// GLOBALS & OBJECTS
// ==========================================
MAX30105 particleSensor;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
SX1278 lora = new Module(LORA_SS, LORA_DIO0, LORA_RST);
RTC_DS3231 rtc;

bool loraOk = false;
bool max30102Ok = false;
bool rtcOk = false;

// State Control
bool remoteLedOverride = false;
bool forceBlinkTest = false;
unsigned long lastHeartbeat = 0;
unsigned long lastLedBlink = 0;
unsigned long lastSensorRetry = 0;
uint16_t lastPktSeq = 0;

// Non-blocking Heart Rate State
byte hrRates[HR_RATE_SIZE];
byte hrRateSpot = 0;
int hrBeatsValid = 0;
unsigned long hrLastBeat = 0;
int hrBeatAvg = 0;
bool hrFingerPresent = false;
unsigned long hrLastDataTx = 0;
unsigned long hrLastValidBeat = 0; // For staleness detection

// ==========================================
// LED CONTROLLER & PATTERNS
// ==========================================
void ledInit() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
}

void blinkStatus(int pin, int times, int durationMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(durationMs);
    digitalWrite(pin, LOW);
    if (i < times - 1)
      delay(durationMs);
  }
}

void ledErrorPattern() { blinkStatus(LED_RED, 3, 100); }

void ledTxWait() {
  digitalWrite(LED_GREEN, HIGH);
  delay(50);
  digitalWrite(LED_GREEN, LOW);
}

void ledCriticalFailure() {
  while (1) {
    blinkStatus(LED_RED, 2, 100);
    delay(500);
  }
}

// ==========================================
// SENSOR INIT & RETRY
// ==========================================
void attemptSensorInit() {
  delay(100);
  if (!rtcOk) {
    if (rtc.begin()) {
      Serial.println("[OK] RTC Reconnected!");
      rtcOk = true;
    }
  }

  if (!max30102Ok) {
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
      Serial.println("[OK] MAX30102 Reconnected!");
      // Tuned settings for reliable pulse detection
      particleSensor.setup(
        0x1F,  // LED brightness (power level) — moderate for finger contact
        4,     // Sample averaging — 4 samples averaged per reading
        2,     // LED mode — Red + IR (pulse oximetry mode)
        400,   // Sample rate — 400 samples/sec for good resolution
        411,   // Pulse width — 411us for max ADC resolution (18-bit)
        4096   // ADC range — 4096nA full scale
      );
      particleSensor.setPulseAmplitudeRed(0x0A);
      particleSensor.setPulseAmplitudeGreen(0);
      max30102Ok = true;
    }
  }
}

// ==========================================
// SENSOR READING BLOCK
// ==========================================
float readTemperature() {
  yield();
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  return (t == DEVICE_DISCONNECTED_C) ? 0.0 : t;
}

// Helper: compute rolling average from hrRates buffer
int computeHrAvg(int validCount) {
  int count = (validCount < HR_RATE_SIZE) ? validCount : HR_RATE_SIZE;
  if (count <= 0) return 0;
  int sum = 0;
  for (byte x = 0; x < count; x++)
    sum += hrRates[x];
  return sum / count;
}

// Non-blocking: called every loop iteration, processes one IR sample
void processHeartRate() {
  if (!max30102Ok)
    return;

  // Stale HR detection: clear if no valid beat for 30s
  if (hrBeatAvg > 0 && millis() - hrLastValidBeat > HR_STALE_MS) {
    Serial.println("[HR] Stale — clearing old reading");
    hrBeatAvg = 0;
  }

  long irValue = particleSensor.getIR();

  // Finger detection
  if (irValue < 50000) {
    if (hrFingerPresent) {
      Serial.println("[HR] Finger removed.");
      hrFingerPresent = false;
    }
    return;
  }

  // New finger placement — reset all state
  if (!hrFingerPresent) {
    Serial.println("[HR] Finger detected, tracking pulse...");
    hrFingerPresent = true;
    hrLastBeat = 0;
    hrBeatsValid = 0;
    hrRateSpot = 0;
    memset(hrRates, 0, sizeof(hrRates));
  }

  if (!checkForBeat(irValue))
    return;

  // Beat detected — quick red flash
  digitalWrite(LED_RED, HIGH);

  // First beat: just record timestamp
  if (hrLastBeat == 0) {
    hrLastBeat = millis();
    hrBeatsValid = 1;
    Serial.print("[HR] Warmup 1/");
    Serial.println(HR_WARMUP_BEATS);
    delay(10);
    digitalWrite(LED_RED, LOW);
    return;
  }

  long delta = millis() - hrLastBeat;
  hrLastBeat = millis();

  // Validate beat interval
  if (delta < HR_MIN_INTERVAL || delta > HR_MAX_INTERVAL) {
    Serial.print("[HR] Rejected: interval ");
    Serial.print(delta);
    Serial.println("ms out of range");
    delay(10);
    digitalWrite(LED_RED, LOW);
    return;
  }

  float bpm = 60000.0 / delta;
  hrBeatsValid++;
  hrLastValidBeat = millis();

  // Warmup phase: collect readings, build baseline average
  if (hrBeatsValid <= HR_WARMUP_BEATS) {
    hrRates[hrRateSpot++] = (byte)bpm;
    hrRateSpot %= HR_RATE_SIZE;
    hrBeatAvg = computeHrAvg(hrBeatsValid - 1); // -1 because beat 1 has no BPM

    Serial.print("[HR] Warmup ");
    Serial.print(hrBeatsValid);
    Serial.print("/");
    Serial.print(HR_WARMUP_BEATS);
    Serial.print(" BPM=");
    Serial.println(bpm);

    delay(10);
    digitalWrite(LED_RED, LOW);
    return;
  }

  // Post-warmup: reject outliers
  if (hrBeatAvg > 0) {
    int diff = abs((int)bpm - hrBeatAvg);
    int threshold = hrBeatAvg * HR_OUTLIER_PCT / 100;
    if (diff > threshold) {
      Serial.print("[HR] Outlier rejected: ");
      Serial.print(bpm);
      Serial.print(" vs avg ");
      Serial.println(hrBeatAvg);
      delay(10);
      digitalWrite(LED_RED, LOW);
      return;
    }
  }

  // Valid reading — store and update average
  hrRates[hrRateSpot++] = (byte)bpm;
  hrRateSpot %= HR_RATE_SIZE;
  hrBeatAvg = computeHrAvg(HR_RATE_SIZE);

  Serial.print("[HR] BPM=");
  Serial.print(bpm);
  Serial.print(" Avg=");
  Serial.println(hrBeatAvg);

  delay(10);
  digitalWrite(LED_RED, LOW);

  // Transmit DATA packet: immediately on first valid, then every 5s
  if (hrBeatAvg > 0 && millis() - hrLastDataTx > HR_DATA_TX_MS) {
    hrLastDataTx = millis();
    float currentTemp = readTemperature();

    lastPktSeq++;
    char dataPkt[64];
    snprintf(dataPkt, sizeof(dataPkt), "DATA,%u,%s,%.1f,%d", lastPktSeq,
             COW_ID, currentTemp, hrBeatAvg);
    Serial.print("[TX DATA] ");
    Serial.println(dataPkt);
    ledTxWait();
    lora.transmit(dataPkt);
    lora.startReceive();
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- VETNODE SENSOR NODE " COW_ID " ---");

  ledInit();

  // Boot indicator
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(500);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);

  tempSensor.begin();
  attemptSensorInit();

  if (!rtcOk || !max30102Ok) {
    Serial.println("[WARN] Missing I2C hardware on boot. Auto-retries enabled.");
    ledErrorPattern();
  }

  pinMode(LORA_DIO0, INPUT);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  if (lora.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, 10, 8) != RADIOLIB_ERR_NONE) {
    Serial.println("[CRIT] LoRa Init Failed!");
    ledCriticalFailure();
  } else {
    loraOk = true;
    lora.startReceive();
  }

  if (max30102Ok && rtcOk) {
    digitalWrite(LED_GREEN, HIGH);
    delay(1000);
    digitalWrite(LED_GREEN, LOW);
  }

  // TDMA: offset first heartbeat so nodes don't collide
  lastHeartbeat = millis() - HB_INTERVAL_MS + TDMA_OFFSET;
}

// ==========================================
// CONTINUOUS ACTIVE LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

  // 1. Diagnostics LED Handler (every 1.5s)
  if (now - lastLedBlink > 1500) {
    lastLedBlink = now;

    if (!remoteLedOverride && !forceBlinkTest) {
      if (loraOk && max30102Ok && rtcOk) {
        blinkStatus(LED_GREEN, 1, 50);
      } else if (!loraOk) {
        blinkStatus(LED_RED, 2, 50);
      } else {
        // Sensor error — alternating green/red
        digitalWrite(LED_GREEN, HIGH);
        delay(100);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, HIGH);
        delay(100);
        digitalWrite(LED_RED, LOW);
      }
    }

    if (forceBlinkTest) {
      for (int k = 0; k < 5; k++) {
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_RED, LOW);
        delay(50);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, HIGH);
        delay(50);
      }
      digitalWrite(LED_RED, LOW);
      forceBlinkTest = false;
    }
  }

  // 2. Auto-retry I2C if sensors dropped (every 10s)
  if (now - lastSensorRetry > 10000) {
    lastSensorRetry = now;
    if (!max30102Ok || !rtcOk)
      attemptSensorInit();
    if (!max30102Ok)
      blinkStatus(LED_RED, 1, 50);
  }

  // 3. Transmit periodic Heartbeat packets (TDMA staggered)
  if (now - lastHeartbeat >= HB_INTERVAL_MS) {
    lastHeartbeat = now;

    if (!remoteLedOverride) {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
    }

    float temp = readTemperature();

    const char *errFlag = "0";
    if (!max30102Ok && !rtcOk)
      errFlag = "ALL";
    else if (!max30102Ok)
      errFlag = "HR";
    else if (!rtcOk)
      errFlag = "RTC";

    // FORMAT: HB,SEQ,NODE_ID,TEMP,HR,ERROR_FLAG
    lastPktSeq++;
    char hb[64];
    snprintf(hb, sizeof(hb), "HB,%u,%s,%.1f,%d,%s", lastPktSeq, COW_ID, temp,
             hrBeatAvg, errFlag);

    ledTxWait();
    lora.transmit(hb);
    lora.startReceive();
  }

  // 4. Non-blocking heart rate processing (every loop iteration)
  processHeartRate();

  // 5. Listen for Gateway Commands (Non-blocking via DIO0)
  if (loraOk && digitalRead(LORA_DIO0) == HIGH) {
    char rxPacket[64];
    int state = lora.readData((uint8_t *)rxPacket, 63);
    if (state == RADIOLIB_ERR_NONE) {
      rxPacket[lora.getPacketLength()] = '\0';
      lora.startReceive();
      Serial.print("\n[RX CMD] ");
      Serial.println(rxPacket);

      if (strncmp(rxPacket, "CMD,", 4) == 0) {
        char *cmd = rxPacket + 4;
        if (strcmp(cmd, "LED_ON") == 0) {
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(LED_RED, HIGH);
          remoteLedOverride = true;
        } else if (strcmp(cmd, "LED_OFF") == 0) {
          digitalWrite(LED_RED, LOW);
          remoteLedOverride = false;
        } else if (strcmp(cmd, "BLINK") == 0) {
          forceBlinkTest = true;
        }
      } else if (strcmp(rxPacket, "PING") == 0) {
        Serial.println("[TX ACK]");
        lora.transmit("ACK");
        lora.startReceive();
      }
    }
  }
}
