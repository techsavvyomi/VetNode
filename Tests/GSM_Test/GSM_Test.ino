/*
  SIM800L GSM Module Test
  Checks basic AT commands and network status.
*/

#define GSM_TX 17
#define GSM_RX 16
HardwareSerial &gsmSerial = Serial2;

void setup() {
  Serial.begin(115200);
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);

  delay(1000);
  Serial.println("GSM Test Started...");

  // Test AT
  gsmSerial.println("AT");
  waitForResponse();

  // Check Signal Quality
  gsmSerial.println("AT+CSQ");
  waitForResponse();

  // Check Network Registration
  gsmSerial.println("AT+CREG?");
  waitForResponse();

  // Check Sim Card
  gsmSerial.println("AT+CCID");
  waitForResponse();
}

void loop() {
  if (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
  if (Serial.available()) {
    gsmSerial.write(Serial.read());
  }
}

void waitForResponse() {
  unsigned long start = millis();
  while (millis() - start < 3000) {
    while (gsmSerial.available()) {
      Serial.write(gsmSerial.read());
    }
  }
  Serial.println("\n---");
}
