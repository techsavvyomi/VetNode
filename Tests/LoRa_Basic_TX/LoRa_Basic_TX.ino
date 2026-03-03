#include <RadioLib.h>
#include <SPI.h>

// ==========================================
// CONFIGURATION: Node (Transmitter) Pins
// ==========================================
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2

SX1278 lora = new Module(LORA_SS, LORA_DIO0, LORA_RST);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[NODE] Basic LoRa Transmitter Test");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  // Initialize LoRa at 433 MHz
  Serial.print("[LoRa] Initializing... ");
  int state = lora.begin(433.0, 125.0, 7, 5, 0x12);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("SUCCESS!");
  } else {
    Serial.print("FAILED, code ");
    Serial.println(state);
    while (true)
      ; // Halt
  }
}

void loop() {
  Serial.print("[LoRa] Transmitting packet... ");

  // Send random ABCD string
  String packet = "Test Data: ";
  char letters[] = "ABCD";
  for (int i = 0; i < 4; i++)
    packet += letters[random(0, 4)];

  int state = lora.transmit(packet);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("SUCCESS. Packet: " + packet);
  } else {
    Serial.print("FAILED, code ");
    Serial.println(state);
  }

  // Wait 3 seconds before next transmission
  delay(3000);
}
