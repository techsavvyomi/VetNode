/*
  LoRa Simple Receiver Test
*/

#include <RadioLib.h>

// Pins for ESP32
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

SX1278 lora = new Module(LORA_SS, LORA_DIO0, LORA_RST);

void setup() {
  Serial.begin(115200);
  Serial.print("[LoRa] Initializing ... ");

  // carrier frequency: 433.0 MHz
  // bandwidth: 125.0 kHz
  // spreading factor: 7
  // coding rate: 5
  // sync word: 0x12
  int state = lora.begin(433.0, 125.0, 7, 5, 0x12);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
    while (true)
      ;
  }
}

void loop() {
  Serial.print("[LoRa] Waiting for packet ... ");

  String str;
  int state = lora.receive(str);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
    Serial.print("[LoRa] Data:\t\t");
    Serial.println(str);

    Serial.print("[LoRa] RSSI:\t\t");
    Serial.print(lora.getRSSI());
    Serial.println(" dBm");

    Serial.print("[LoRa] SNR:\t\t");
    Serial.print(lora.getSNR());
    Serial.println(" dB");

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    Serial.println("timeout!");
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
  }
}
