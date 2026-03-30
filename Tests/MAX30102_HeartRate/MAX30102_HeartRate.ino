/*
  MAX30102 Heart Rate Test with LED Debug Output
  ================================================

  Reads BPM from MAX30102 and displays it via LEDs:

  RED LED:
    - Steady ON  = no finger detected
    - Blink once = beat detected (heartbeat indicator)

  GREEN LED - Blinks out BPM digits using this protocol:
    - START:  3 rapid blinks (100ms each)
    - DIGITS: each digit blinked out (0 = 10 blinks), separated by 500ms pause
    - STOP:   1 long blink (800ms)

    Example: BPM = 72
      -> START (3 rapid blinks)
      -> 7 blinks (digit 7)
      -> 500ms pause
      -> 2 blinks (digit 2)
      -> STOP (1 long blink)

  Hardware (ESP32):
    - MAX30102: SDA=21, SCL=22 (I2C)
    - Green LED: GPIO 32
    - Red LED:   GPIO 33
*/

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// Pin definitions (matching Node.ino)
#define I2C_SDA   21
#define I2C_SCL   22
#define LED_GREEN 32
#define LED_RED   33

MAX30105 particleSensor;

// Heart rate averaging
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute = 0;
int beatAvg = 0;

// LED display timing
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 5000;  // show BPM every 5 seconds

// ==========================================
// LED HELPERS
// ==========================================
void blinkLed(int pin, int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(onMs);
    digitalWrite(pin, LOW);
    if (i < times - 1) delay(offMs);
  }
}

// Blink out a single digit (0-9) on GREEN LED
// 0 is shown as 10 blinks to distinguish from "no blinks"
void blinkDigit(int digit) {
  int count = (digit == 0) ? 10 : digit;
  blinkLed(LED_GREEN, count, 150, 150);
}

// Display a BPM value on GREEN LED using blink protocol
void displayBpmOnLed(int bpm) {
  if (bpm <= 0 || bpm > 255) return;

  Serial.print("[LED] Displaying BPM: ");
  Serial.println(bpm);

  // Extract digits
  int digits[3];
  int numDigits = 0;

  if (bpm >= 100) {
    digits[numDigits++] = bpm / 100;
    digits[numDigits++] = (bpm / 10) % 10;
    digits[numDigits++] = bpm % 10;
  } else if (bpm >= 10) {
    digits[numDigits++] = bpm / 10;
    digits[numDigits++] = bpm % 10;
  } else {
    digits[numDigits++] = bpm;
  }

  // START: 3 rapid blinks
  blinkLed(LED_GREEN, 3, 80, 80);
  delay(400);

  // DIGITS: blink each digit with pause between
  for (int i = 0; i < numDigits; i++) {
    blinkDigit(digits[i]);
    if (i < numDigits - 1) {
      delay(500);  // pause between digits
    }
  }
  delay(400);

  // STOP: 1 long blink
  digitalWrite(LED_GREEN, HIGH);
  delay(800);
  digitalWrite(LED_GREEN, LOW);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println("MAX30102 Heart Rate Test");
  Serial.println("========================");

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  // Startup indicator: alternate LEDs
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GREEN, HIGH);
    delay(200);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    delay(200);
    digitalWrite(LED_RED, LOW);
  }

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30102 not found! Check wiring.");
    // Error pattern: rapid red blink forever
    while (1) {
      blinkLed(LED_RED, 5, 100, 100);
      delay(1000);
    }
  }

  Serial.println("MAX30102 found!");
  // Success: green blink
  blinkLed(LED_GREEN, 3, 200, 200);

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.println("Place finger on sensor with steady pressure.");
  Serial.println();
  Serial.println("LED Protocol:");
  Serial.println("  RED steady  = no finger");
  Serial.println("  RED blink   = beat detected");
  Serial.println("  GREEN       = BPM digits (every 5s)");
  Serial.println("    START: 3 rapid blinks");
  Serial.println("    DIGIT: N blinks per digit (0=10 blinks)");
  Serial.println("    STOP:  1 long blink");
  Serial.println();
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  long irValue = particleSensor.getIR();

  // No finger detection
  if (irValue < 50000) {
    digitalWrite(LED_RED, HIGH);   // red ON = no finger
    digitalWrite(LED_GREEN, LOW);

    // Reset readings
    beatsPerMinute = 0;
    beatAvg = 0;

    Serial.println("No finger detected - place finger on sensor");
    delay(500);
    return;
  }

  // Finger is on sensor - turn off red
  digitalWrite(LED_RED, LOW);

  if (checkForBeat(irValue)) {
    // Beat detected - quick red flash
    digitalWrite(LED_RED, HIGH);
    delay(30);
    digitalWrite(LED_RED, LOW);

    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60.0 / (delta / 1000.0);

    if (beatsPerMinute > 20 && beatsPerMinute < 255) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }

    Serial.print("Beat! BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(" Avg=");
    Serial.println(beatAvg);
  }

  // Display BPM on green LED every DISPLAY_INTERVAL
  if (beatAvg > 20 && millis() - lastDisplayTime > DISPLAY_INTERVAL) {
    lastDisplayTime = millis();
    displayBpmOnLed(beatAvg);
  }
}
