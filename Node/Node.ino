#include "MAX30105.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RTClib.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>

// ==========================================
// CONFIGURATION & PIN DEFINITIONS
// ==========================================
#define COW_ID "1"

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

// ==========================================
// GLOBALS & OBJECTS
// ==========================================
MAX30105 particleSensor;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
SX1278 lora = new Module(LORA_SS, LORA_DIO0, LORA_RST);
RTC_DS3231 rtc;

bool i2cOk = false;
bool loraOk = false;
bool max30102Ok = false;
bool rtcOk = false;

// State Control
bool remoteLedOverride = false;
bool forceBlinkTest = false;
unsigned long lastHeartbeat = 0;
unsigned long lastLedBlink = 0;
unsigned long lastSensorRetry = 0;
bool led1State = false;
uint16_t lastPktSeq = 0;

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
  while (1) { // Will halt until WDT or manual reset
    digitalWrite(LED_RED, HIGH);
    delay(100);
    digitalWrite(LED_RED, LOW);
    delay(100);
    digitalWrite(LED_RED, HIGH);
    delay(100);
    digitalWrite(LED_RED, LOW);
    delay(500);
  }
}

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
      particleSensor.setup();
      particleSensor.setPulseAmplitudeRed(0x0A);
      particleSensor.setPulseAmplitudeGreen(0);
      max30102Ok = true;
    }
  }
}

// ==========================================
// HEART RATE ALGORITHM (Built-in to bypass Arduino Cloud Library conflicts)
// ==========================================
int16_t node_IR_AC_Max = 20;
int16_t node_IR_AC_Min = -20;
int16_t node_IR_AC_Signal_Current = 0;
int16_t node_IR_AC_Signal_Previous;
int16_t node_IR_AC_Signal_min = 0;
int16_t node_IR_AC_Signal_max = 0;
int16_t node_IR_Average_Estimated;
int16_t node_positiveEdge = 0;
int16_t node_negativeEdge = 0;
int32_t node_ir_avg_reg = 0;
int16_t node_cbuf[32];
uint8_t node_offset = 0;

static const uint16_t node_FIRCoeffs[12] = {172,  321,  579,  927,  1360, 1858,
                                            2390, 2916, 3391, 3768, 4012, 4096};

bool checkNodeBeat(int32_t sample) {
  bool beatDetected = false;
  node_IR_AC_Signal_Previous = node_IR_AC_Signal_Current;
  node_ir_avg_reg += (((sample << 15) - node_ir_avg_reg) >> 4);
  node_IR_Average_Estimated = node_ir_avg_reg >> 15;
  int16_t FIR_Temp = sample - node_IR_Average_Estimated;

  node_cbuf[node_offset] = FIR_Temp;
  int32_t temp_sum = 0;

  for (uint8_t i = 0; i < 12; i++) {
    int16_t index = (node_offset - i + 32) % 32;
    int16_t index2 = (node_offset - (24 - i) + 32) % 32;
    temp_sum += node_cbuf[index] * node_FIRCoeffs[i];
    temp_sum += node_cbuf[index2] * node_FIRCoeffs[i];
  }

  node_IR_AC_Signal_Current = temp_sum / 256;
  node_offset = (node_offset + 1) % 32;

  if (node_IR_AC_Signal_Previous < 0 && node_IR_AC_Signal_Current >= 0) {
    node_IR_AC_Max = node_IR_AC_Signal_max;
    node_IR_AC_Signal_max = 0;
    node_positiveEdge = 1;
    node_negativeEdge = 0;
  }
  if (node_IR_AC_Signal_Previous > 0 && node_IR_AC_Signal_Current <= 0) {
    node_IR_AC_Min = node_IR_AC_Signal_min;
    node_IR_AC_Signal_min = 0;
    node_positiveEdge = 0;
    node_negativeEdge = 1;
  }

  if (node_IR_AC_Signal_Current > node_IR_AC_Signal_max)
    node_IR_AC_Signal_max = node_IR_AC_Signal_Current;
  if (node_IR_AC_Signal_Current < node_IR_AC_Signal_min)
    node_IR_AC_Signal_min = node_IR_AC_Signal_Current;

  int16_t IR_AC_Amplitude = node_IR_AC_Max - node_IR_AC_Min;
  if (IR_AC_Amplitude > 20 && IR_AC_Amplitude < 1000) {
    if (node_IR_AC_Signal_Current == node_IR_AC_Max && node_positiveEdge == 1) {
      beatDetected = true;
      node_positiveEdge = 0;
    }
  }

  return beatDetected;
}

// ==========================================
// SENSOR READING BLOCK
// ==========================================
float readTemperature() {
  yield(); // Let background tasks settle
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  return (t == DEVICE_DISCONNECTED_C) ? 0.0 : t;
}

int checkHeartRateQuick(float currentTemp) {
  if (!max30102Ok)
    return 0;

  long irValue = particleSensor.getIR();
  if (irValue < 50000)
    return 0; // No finger detected

  Serial.println("\n[SENSOR] Finger detected, analyzing pulse...");

  const byte RATE_SIZE = 4; // Increase this for more averaging
  byte rates[RATE_SIZE];    // Array of heart rates
  byte rateSpot = 0;

  long lastBeat = 0;
  int beatAvg = 0;
  int beatsValid = 0;
  unsigned long startTime = millis();

  // Spend up to 10 seconds tracking the pulse
  while (millis() - startTime < 10000) {
    long irLocal = particleSensor.getIR();

    // User let go
    if (irLocal < 50000) {
      Serial.println("[SENSOR] Finger removed early.");
      break;
    }

    if (checkNodeBeat(irLocal)) {
      digitalWrite(LED_RED, HIGH); // Pulse flash

      if (lastBeat == 0) {
        lastBeat = millis(); // Discard the first beat, it's just the start time
                             // boundary!
      } else {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        float beatsPerMinute = 60000.0 / delta;

        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
          rates[rateSpot++] =
              (byte)beatsPerMinute; // Store this reading in the array
          rateSpot %= RATE_SIZE;    // Wrap variable

          beatsValid++;

          // Take average of readings
          beatAvg = 0;
          int numToAverage = (beatsValid < RATE_SIZE) ? beatsValid : RATE_SIZE;
          for (byte x = 0; x < numToAverage; x++)
            beatAvg += rates[x];
          beatAvg /= numToAverage;

          Serial.print("IR=");
          Serial.print(irLocal);
          Serial.print(", BPM=");
          Serial.print(beatsPerMinute);
          Serial.print(", Avg BPM=");
          Serial.println(beatAvg);
        }
      }

      delay(10);
      digitalWrite(LED_RED, LOW);
    }

    // Got enough data to declare a valid HR measurement
    if (beatsValid >= RATE_SIZE) {
      Serial.print("[SENSOR] Valid HR locked: ");
      Serial.println(beatAvg);

      // Force transmit an immediate DATA packet with HR
      lastPktSeq++;
      char dataPkt[64];
      snprintf(dataPkt, sizeof(dataPkt), "DATA,%u,%s,%.1f,%u", lastPktSeq,
               COW_ID, currentTemp, beatAvg);
      Serial.print("[TX] ");
      Serial.println(dataPkt);
      ledTxWait();
      lora.transmit(dataPkt);
      lora.startReceive(); // Immediately back to receive mode

      // Sleep node sensor logic for 3 seconds to prevent re-trigger spam
      delay(3000);
      return beatAvg;
    }
  }
  return 0;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- VETNODE SENSOR ACTIVE NODE ---");

  ledInit();

  // Indicate boot
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(500);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  i2cOk = true;

  tempSensor.begin();
  attemptSensorInit();

  if (!rtcOk || !max30102Ok) {
    Serial.println(
        "[WARN] Missing I2C hardware on boot. Auto-retries enabled.");
    ledErrorPattern();
  }

  pinMode(LORA_DIO0, INPUT); // MUST BE INPUT FOR POLLING
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  // Match Gateway params: Freq, BW, SF, CR, Sync, Pwr, Preamble
  if (lora.begin(433.0, 125.0, 7, 5, 0x12, 10, 8) != RADIOLIB_ERR_NONE) {
    Serial.println("[CRIT] LoRa Init Failed!");
    ledCriticalFailure();
  } else {
    loraOk = true;
    lora.startReceive(); // Enter continuous RX background polling mode
  }

  // Solid Green = All Systems Nominal
  if (max30102Ok && rtcOk) {
    digitalWrite(LED_GREEN, HIGH);
    delay(1000);
    digitalWrite(LED_GREEN, LOW);
  }
}

// ==========================================
// CONTINUOUS ACTIVE LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

  // 1. Diagnostics LED Handler (Blinks every 1.5 seconds)
  if (now - lastLedBlink > 1500) {
    lastLedBlink = now;
    led1State = !led1State;

    if (!remoteLedOverride && !forceBlinkTest) {
      if (loraOk && max30102Ok && rtcOk) {
        // [ALL SYSTEMS GO] -> Single short green blink
        digitalWrite(LED_GREEN, HIGH);
        delay(50);
        digitalWrite(LED_GREEN, LOW);
      } else if (!loraOk) {
        // [RADIO DEAD] -> Double RED flash
        digitalWrite(LED_RED, HIGH);
        delay(50);
        digitalWrite(LED_RED, LOW);
        delay(50);
        digitalWrite(LED_RED, HIGH);
        delay(50);
        digitalWrite(LED_RED, LOW);
      } else {
        // [SENSOR ERROR] -> Alternating Green/Red indicating radio ok but
        // hardware fault
        digitalWrite(LED_GREEN, HIGH);
        delay(100);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, HIGH);
        delay(100);
        digitalWrite(LED_RED, LOW);
      }
    }

    if (forceBlinkTest) {
      // Rapid alternating test triggered by Gateway
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

  // 1.5 Auto-retry I2C if sensors dropped
  if (now - lastSensorRetry > 10000) {
    lastSensorRetry = now;
    if (!max30102Ok || !rtcOk)
      attemptSensorInit();

    // If still bad, blink red to show distress locally
    if (!max30102Ok)
      blinkStatus(LED_RED, 1, 50);
  }

  // 2. Transmit periodic 2s Heartbeat packets
  if (now - lastHeartbeat >= 2000) {
    lastHeartbeat = now;

    // Safety: ensure LEDs are off during high-precision timing of OneWire
    if (!remoteLedOverride) {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
    }

    float temp = readTemperature();

    // Error String flag
    String errFlag = "0";
    if (!max30102Ok && !rtcOk)
      errFlag = "ALL";
    else if (!max30102Ok)
      errFlag = "HR";
    else if (!rtcOk)
      errFlag = "RTC";

    // FORMAT: HB,SEQ,NODE_ID,TEMP,ERROR_FLAG
    lastPktSeq++;
    char hb[64];
    snprintf(hb, sizeof(hb), "HB,%u,%s,%.1f,%s", lastPktSeq, COW_ID, temp,
             errFlag.c_str());

    ledTxWait();
    lora.transmit(hb);

    // Right after TX, switch to RX briefly to catch remote commands
    lora.startReceive();
  }

  // 3. Check for User Finger on HR Sensor (Generates DATA packet)
  if (max30102Ok) {
    float currentTemp = readTemperature();
    checkHeartRateQuick(currentTemp);
  }

  // 4. Listen for Gateway Commands (Non-blocking via DIO0)
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
