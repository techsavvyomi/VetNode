/*
  MAX30102 Heart Rate Sensor Test
  Dumps raw IR values to Serial Plotter/Monitor.
*/

#include "MAX30105.h"
#include <Wire.h>

MAX30105 particleSensor;

#define I2C_SDA 21
#define I2C_SCL 22

void setup() {
  Serial.begin(115200);
  Serial.println("MAX30102 Test...");

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1)
      ;
  }

  particleSensor.setup(); // Configure sensor with default settings
}

void loop() {
  Serial.print(" IR: ");
  Serial.print(particleSensor.getIR());
  Serial.print(" Red: ");
  Serial.print(particleSensor.getRed());
  Serial.println();
  delay(10);
}
