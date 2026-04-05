/*
 * VetNode Demo — ESP32-C3 Super Mini
 * No LoRa, no RTC, no DS18B20. Just MAX30102 + 2 LEDs.
 *
 * Hardware:
 *   Green LED  → GPIO 1
 *   Red LED    → GPIO 2
 *   MAX30102   → I2C (SDA=8, SCL=9 default C3)
 *
 * Behavior:
 *   Idle:     Packet-send simulation LED pattern
 *   Finger:   Red LED blinks while warming up (10 beats)
 *   Reading:  Measures HR, then displays value via LED protocol:
 *
 * HR Display Protocol (e.g. HR = 72):
 *   [RED x1]         ← START bit
 *   [GREEN x7]       ← tens digit (7)
 *   [RED x1]         ← separator
 *   [GREEN x2]       ← ones digit (2)
 *   [RED x2]         ← STOP bits
 *   ... pause, then repeat or keep measuring ...
 *
 * For HR = 108:
 *   [RED x1] [GREEN x1] [RED x1] [GREEN x0] [RED x1] [GREEN x8] [RED x2]
 *   (hundreds=1, tens=0 → skip green, ones=8)
 */

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// ==========================================
// PIN DEFINITIONS
// ==========================================
#define LED_GREEN 1
#define LED_RED   2
#define I2C_SDA   8
#define I2C_SCL   9

// ==========================================
// HR CONFIG
// ==========================================
#define HR_AVG_SIZE       4
#define HR_THRESHOLD      120
#define IR_FINGER_THRESH  50000
#define HR_WARMUP_BEATS   5     // reduced — 5 beats ≈ 4s at 70BPM

// ==========================================
// HR DISPLAY PROTOCOL TIMING (ms)
// ==========================================
#define DISP_BLINK_ON     200   // LED on time per blink
#define DISP_BLINK_OFF    150   // gap between blinks
#define DISP_DIGIT_GAP    400   // extra pause between digit groups
#define DISP_END_PAUSE    2000  // pause after full display before repeat

MAX30105 particleSensor;

// HR calculation
byte   rates[HR_AVG_SIZE];
byte   rateIndex    = 0;
int    beatAvg      = 0;
int    beatCount    = 0;
long   lastBeat     = 0;
bool   fingerPresent = false;
bool   warmedUp      = false;

// Idle pattern state
unsigned long idleTimer = 0;
byte          idleStep  = 0;

// Warmup blink state
unsigned long warmupTimer = 0;
bool          warmupLedOn = false;
#define WARMUP_BLINK_ON   120
#define WARMUP_BLINK_OFF  200

// HR display state machine
enum DispState { DISP_IDLE, DISP_START, DISP_DIGIT, DISP_SEP, DISP_STOP, DISP_DONE };
DispState     dispState     = DISP_IDLE;
unsigned long dispTimer     = 0;
bool          dispLedOn     = false;
byte          dispDigits[3];    // up to 3 digits (hundreds, tens, ones)
byte          dispDigitCount;   // how many digits (2 or 3)
byte          dispCurrentDigit; // which digit we're showing
byte          dispBlinksDone;   // blinks done for current digit
byte          dispStopCount;    // stop bits done
int           dispHR;           // HR value being displayed
bool          displaying   = false;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[DemoNode] Starting...");
  Serial.printf("[CONF] Green LED=GPIO%d  Red LED=GPIO%d\n", LED_GREEN, LED_RED);
  Serial.printf("[CONF] I2C SDA=GPIO%d  SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
  Serial.printf("[CONF] IR threshold=%d  Warmup beats=%d\n", IR_FINGER_THRESH, HR_WARMUP_BEATS);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[ERROR] MAX30102 not found! Check I2C wiring (SDA/SCL)");
    // Error: alternate BOTH LEDs so it's visually distinct from warmup
    while (1) {
      digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, LOW);  delay(150);
      digitalWrite(LED_RED, LOW);  digitalWrite(LED_GREEN, HIGH); delay(150);
    }
  }
  Serial.println("[OK] MAX30102 ready");

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  memset(rates, 0, sizeof(rates));
  idleTimer = millis();
}

void loop() {
  long irValue = particleSensor.getIR();
  fingerPresent = (irValue > IR_FINGER_THRESH);

  // Debug: print IR every 500ms so you can check threshold
  static unsigned long dbgTimer = 0;
  if (millis() - dbgTimer > 500) {
    Serial.printf("[DBG] IR=%ld  finger=%s  beats=%d\n", irValue, fingerPresent ? "YES" : "NO", beatCount);
    dbgTimer = millis();
  }

  if (fingerPresent) {
    // Seed lastBeat on first detection so first delta is valid
    if (lastBeat == 0) {
      lastBeat = millis();
      Serial.println("[STATE] Finger detected — starting warmup");
    }

    if (!warmedUp && beatCount < HR_WARMUP_BEATS) {
      collectBeat(irValue);
      runWarmupBlink();
    } else if (!displaying) {
      if (!warmedUp) Serial.println("[STATE] Warmup complete — measuring HR");
      warmedUp = true;
      handleHeartRate(irValue);
    } else {
      runHRDisplay();
    }
  } else {
    if (fingerPresent != false && lastBeat != 0) {
      Serial.println("[STATE] Finger removed — resetting");
    }
    resetHR();
    runIdlePattern();
  }
}

// ==========================================
// BEAT COLLECTION (during warmup)
// ==========================================
void collectBeat(long irValue) {
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    if (delta > 250 && delta < 2500) {
      int bpm = 60000 / delta;
      rates[rateIndex++ % HR_AVG_SIZE] = (byte)bpm;
      beatCount++;

      int sum = 0;
      byte count = min((int)rateIndex, HR_AVG_SIZE);
      for (byte i = 0; i < count; i++) sum += rates[i];
      beatAvg = sum / count;

      Serial.printf("[WARMUP] Beat %d/%d  BPM: %d\n", beatCount, HR_WARMUP_BEATS, bpm);

      // Quick green flash to confirm beat detected
      digitalWrite(LED_GREEN, HIGH);
      delay(50);
      digitalWrite(LED_GREEN, LOW);
    }
  }
}

// ==========================================
// WARMUP BLINK — red LED pulses while collecting
// ==========================================
void runWarmupBlink() {
  unsigned long now = millis();
  digitalWrite(LED_GREEN, LOW);

  if (warmupLedOn) {
    if (now - warmupTimer >= WARMUP_BLINK_ON) {
      digitalWrite(LED_RED, LOW);
      warmupLedOn = false;
      warmupTimer = now;
    }
  } else {
    if (now - warmupTimer >= WARMUP_BLINK_OFF) {
      digitalWrite(LED_RED, HIGH);
      warmupLedOn = true;
      warmupTimer = now;
    }
  }
}

// ==========================================
// HEART RATE MODE (after warmup)
// ==========================================
void handleHeartRate(long irValue) {
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    if (delta > 250 && delta < 2500) {
      int bpm = 60000 / delta;
      rates[rateIndex++ % HR_AVG_SIZE] = (byte)bpm;
      beatCount++;

      int sum = 0;
      byte count = min((int)rateIndex, HR_AVG_SIZE);
      for (byte i = 0; i < count; i++) sum += rates[i];
      beatAvg = sum / count;

      // Flash on beat
      if (beatAvg > HR_THRESHOLD) {
        digitalWrite(LED_RED, HIGH);
      } else {
        digitalWrite(LED_GREEN, HIGH);
      }

      Serial.printf("[HR] BPM: %d  Avg: %d\n", bpm, beatAvg);

      delay(60);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);

      // After each new beat, display the HR value on LEDs
      startHRDisplay(beatAvg);
    }
  }

  // Soft glow between beats (only if not in display mode)
  if (!displaying && beatAvg > 0) {
    if (beatAvg > HR_THRESHOLD) {
      analogWrite(LED_RED, 30);
      analogWrite(LED_GREEN, 0);
    } else {
      analogWrite(LED_GREEN, 30);
      analogWrite(LED_RED, 0);
    }
  }
}

// ==========================================
// HR LED DISPLAY PROTOCOL
// ==========================================
// Encodes HR digit-by-digit:
//   START:  1x red blink
//   DIGIT:  Nx green blinks (N = digit value, 0 = no blinks)
//   SEP:    1x red blink (between digits)
//   STOP:   2x red blinks
//
// Example HR=72:  RED(1) GREEN(7) RED(1) GREEN(2) RED(2)
// Example HR=108: RED(1) GREEN(1) RED(1) GREEN(0) RED(1) GREEN(8) RED(2)

void startHRDisplay(int hr) {
  if (hr <= 0 || hr > 250) return;

  dispHR = hr;
  displaying = true;

  // Break HR into digits
  if (hr >= 100) {
    dispDigits[0] = hr / 100;
    dispDigits[1] = (hr / 10) % 10;
    dispDigits[2] = hr % 10;
    dispDigitCount = 3;
  } else {
    dispDigits[0] = hr / 10;
    dispDigits[1] = hr % 10;
    dispDigitCount = 2;
  }

  dispCurrentDigit = 0;
  dispBlinksDone = 0;
  dispStopCount = 0;
  dispState = DISP_START;
  dispLedOn = false;
  dispTimer = millis();

  Serial.printf("[DISP] Showing HR=%d as digits: ", hr);
  for (byte i = 0; i < dispDigitCount; i++) Serial.printf("%d ", dispDigits[i]);
  Serial.println();
}

void runHRDisplay() {
  unsigned long now = millis();

  switch (dispState) {

    // ---- START BIT: 1x red blink ----
    case DISP_START:
      if (!dispLedOn) {
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GREEN, LOW);
        dispLedOn = true;
        dispTimer = now;
      }
      if (now - dispTimer >= DISP_BLINK_ON) {
        digitalWrite(LED_RED, LOW);
        dispLedOn = false;
        dispTimer = now;
        dispState = DISP_DIGIT;
        dispBlinksDone = 0;
      }
      break;

    // ---- DIGIT: Nx green blinks ----
    case DISP_DIGIT:
      if (dispCurrentDigit >= dispDigitCount) {
        // All digits done → go to STOP
        dispState = DISP_STOP;
        dispStopCount = 0;
        dispTimer = now;
        break;
      }

      // If digit is 0, skip green blinks, just pause then separator
      if (dispDigits[dispCurrentDigit] == 0) {
        if (now - dispTimer >= DISP_DIGIT_GAP) {
          dispCurrentDigit++;
          dispBlinksDone = 0;
          if (dispCurrentDigit < dispDigitCount) {
            dispState = DISP_SEP;
          }
          dispTimer = now;
        }
        break;
      }

      if (dispBlinksDone >= dispDigits[dispCurrentDigit]) {
        // Digit complete → separator or stop
        dispCurrentDigit++;
        dispBlinksDone = 0;
        if (dispCurrentDigit < dispDigitCount) {
          dispState = DISP_SEP;
          dispTimer = now;
        }
        break;
      }

      // Blink green
      if (!dispLedOn) {
        if (now - dispTimer >= DISP_BLINK_OFF) {
          digitalWrite(LED_GREEN, HIGH);
          digitalWrite(LED_RED, LOW);
          dispLedOn = true;
          dispTimer = now;
        }
      } else {
        if (now - dispTimer >= DISP_BLINK_ON) {
          digitalWrite(LED_GREEN, LOW);
          dispLedOn = false;
          dispBlinksDone++;
          dispTimer = now;
        }
      }
      break;

    // ---- SEPARATOR: 1x red blink ----
    case DISP_SEP:
      if (!dispLedOn) {
        if (now - dispTimer >= DISP_DIGIT_GAP) {
          digitalWrite(LED_RED, HIGH);
          digitalWrite(LED_GREEN, LOW);
          dispLedOn = true;
          dispTimer = now;
        }
      } else {
        if (now - dispTimer >= DISP_BLINK_ON) {
          digitalWrite(LED_RED, LOW);
          dispLedOn = false;
          dispState = DISP_DIGIT;
          dispBlinksDone = 0;
          dispTimer = now;
        }
      }
      break;

    // ---- STOP: 2x red blinks ----
    case DISP_STOP:
      if (dispStopCount >= 2) {
        dispState = DISP_DONE;
        dispTimer = now;
        break;
      }

      if (!dispLedOn) {
        if (now - dispTimer >= DISP_BLINK_OFF) {
          digitalWrite(LED_RED, HIGH);
          digitalWrite(LED_GREEN, LOW);
          dispLedOn = true;
          dispTimer = now;
        }
      } else {
        if (now - dispTimer >= DISP_BLINK_ON) {
          digitalWrite(LED_RED, LOW);
          dispLedOn = false;
          dispStopCount++;
          dispTimer = now;
        }
      }
      break;

    // ---- DONE: pause then back to measuring ----
    case DISP_DONE:
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
      if (now - dispTimer >= DISP_END_PAUSE) {
        displaying = false;
        dispState = DISP_IDLE;
      }
      break;

    case DISP_IDLE:
      break;
  }
}

// ==========================================
// IDLE PATTERN — packet-send simulation
// ==========================================
#define PKT_BLINK_ON    80
#define PKT_BLINK_OFF   60
#define PKT_BOTH_ON     120
#define PKT_INTERVAL    2000
#define PKT_STEPS       10

void runIdlePattern() {
  unsigned long now = millis();

  unsigned long wait;
  if (idleStep == PKT_STEPS - 1) {
    wait = PKT_INTERVAL;
  } else if (idleStep % 2 == 0) {
    wait = PKT_BLINK_ON;
  } else {
    wait = PKT_BLINK_OFF;
  }

  if (now - idleTimer < wait) return;
  idleTimer = now;

  switch (idleStep) {
    case 0: digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_RED, LOW);  break;
    case 1: digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_RED, LOW);  break;
    case 2: digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_RED, HIGH); break;
    case 3: digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_RED, LOW);  break;
    case 4: digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_RED, LOW);  break;
    case 5: digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_RED, LOW);  break;
    case 6: digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_RED, HIGH); break;
    case 7: digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_RED, LOW);  break;
    case 8: digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_RED, HIGH); break;
    case 9: digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_RED, LOW);  break;
  }

  idleStep = (idleStep + 1) % PKT_STEPS;
}

// ==========================================
// RESET
// ==========================================
void resetHR() {
  beatAvg   = 0;
  beatCount = 0;
  rateIndex = 0;
  lastBeat  = 0;
  warmedUp  = false;
  displaying = false;
  dispState = DISP_IDLE;
  memset(rates, 0, sizeof(rates));
  warmupTimer = 0;
  warmupLedOn = false;
}
