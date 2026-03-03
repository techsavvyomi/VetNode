#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>

// ==========================================
// CONFIGURATION: Gateway (Receiver) Pins
// ==========================================
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2

// OLED I2C Pins
#define OLED_SDA 21
#define OLED_SCL 22

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SX1278 lora = new Module(LORA_SS, LORA_DIO0, LORA_RST);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[GATEWAY] Basic LoRa Receiver Test");

  // 1. OLED Init
  Wire.begin(OLED_SDA, OLED_SCL);
  if (display.begin(0x3C, true)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("LORA RX INIT...");
    display.display();
  } else {
    Serial.println("OLED init failed");
  }

  // 2. LoRa Init
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  Serial.print("[LoRa] Initializing... ");
  int state = lora.begin(433.0, 125.0, 7, 5, 0x12);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("SUCCESS!");
    display.println("LORA OK! Listening...");
    display.display();
  } else {
    Serial.print("FAILED, code ");
    Serial.println(state);
    display.println("LORA FAILED");
    display.display();
    while (true)
      ; // Halt
  }
}

void loop() {
  String str;
  // This blocks until a packet is received or timeout
  int state = lora.receive(str);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("\n[RX] Received packet!");
    Serial.print("Data:\t\t");
    Serial.println(str);
    Serial.print("RSSI:\t\t");
    Serial.print(lora.getRSSI());
    Serial.println(" dBm");
    Serial.print("SNR:\t\t");
    Serial.print(lora.getSNR());
    Serial.println(" dB");

    // Update OLED Display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(">>> PACKET RECEIVED");
    display.drawLine(0, 10, 128, 10, SH110X_WHITE);

    display.setCursor(0, 16);
    display.print("DATA: ");
    display.println(str);

    display.setCursor(0, 36);
    display.print("RSSI: ");
    display.print(lora.getRSSI());
    display.println(" dBm");
    display.print("SNR : ");
    display.print(lora.getSNR());
    display.println(" dB");

    display.display();

    // Flash the built-in ESP32 LED if you have one on Pin 2
    // digitalWrite(2, HIGH); delay(100); digitalWrite(2, LOW);
  } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
    // If it's not simply a timeout, log the error
    Serial.print("[LoRa] Failed to receive, code:");
    Serial.println(state);
  }
}
