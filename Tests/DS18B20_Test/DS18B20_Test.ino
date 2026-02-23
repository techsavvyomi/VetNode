/*
  DS18B20 Temperature Sensor Test
*/

#include <DallasTemperature.h>
#include <OneWire.h>

#define ONE_WIRE_BUS 33
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  Serial.println("DS18B20 Test...");
  sensors.begin();
}

void loop() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  if (tempC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" C");
  } else {
    Serial.println("Error: Could not read temperature data");
  }

  delay(2000);
}
