#include <RadioLib.h>
#include <SPI.h>
#include <map>

// ==========================================
// CONFIGURATION & PIN DEFINITIONS
// ==========================================
// LoRa SX1278 (SPI)
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// GSM SIM800L (Serial2)
#define GSM_TX 17
#define GSM_RX 16

// Health Thresholds
#define TEMP_MAX 39.5
#define TEMP_MIN 37.5
#define HR_MAX 84
#define HR_MIN 48

// Alert Numbers
#define ADMIN_PHONE "+91XXXXXXXXXX" // reciever number

// LoRa Settings
#define LORA_FREQ 433.0
#define LORA_BW 125.0
#define LORA_SF 7
#define LORA_CR 5
#define LORA_SYNC 0x12

// Timing
#define SUMMARY_INTERVAL_MS 3600000UL // 1 Hour

// ==========================================
// GSM COMMUNICATION CLASS
// ==========================================
class GSMComms {
public:
  HardwareSerial &gsmSerial = Serial2;

  bool begin() {
    gsmSerial.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);
    gsmSerial.println("AT");
    if (waitResponse().indexOf("OK") == -1)
      return false;
    gsmSerial.println("AT+CMGF=1");
    return (waitResponse().indexOf("OK") != -1);
  }

  bool sendSMS(String number, String message) {
    gsmSerial.print("AT+CMGS=\"");
    gsmSerial.print(number);
    gsmSerial.println("\"");
    delay(100);
    gsmSerial.print(message);
    gsmSerial.write(26);
    String resp = waitResponse(10000);
    return resp.indexOf("+CMGS:") != -1;
  }

private:
  String waitResponse(unsigned long timeout = 5000) {
    String response = "";
    unsigned long start = millis();
    while (millis() - start < timeout) {
      while (gsmSerial.available())
        response += char(gsmSerial.read());
      if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1)
        break;
    }
    return response;
  }
};

// ==========================================
// MAIN PROGRAM
// ==========================================
SX1278 lora = new Module(LORA_SS, LORA_DIO0, LORA_RST);
GSMComms gsm;

struct CowData {
  float temp;
  int hr;
  String timestamp;
};

std::map<String, CowData> farmData;
unsigned long lastSummaryTime = 0;

void processPacket(String packet) {
  int comma1 = packet.indexOf(',');
  int comma2 = packet.indexOf(',', comma1 + 1);
  int comma3 = packet.indexOf(',', comma2 + 1);
  if (comma1 == -1 || comma2 == -1 || comma3 == -1)
    return;

  String id = packet.substring(0, comma1);
  float temp = packet.substring(comma1 + 1, comma2).toFloat();
  int hr = packet.substring(comma2 + 1, comma3).toInt();
  String ts = packet.substring(comma3 + 1);

  farmData[id] = {temp, hr, ts};

  String alertMsg = "ALERT Cow " + id + ": ";
  bool alert = false;
  if (temp > TEMP_MAX || temp < TEMP_MIN) {
    alert = true;
    alertMsg += "Temp=" + String(temp, 1) + "C ";
  }
  if (hr > HR_MAX || hr < HR_MIN) {
    alert = true;
    alertMsg += "HR=" + String(hr) + "BPM";
  }

  if (alert) {
    Serial.println("Emergency SMS: " + alertMsg);
    gsm.sendSMS(ADMIN_PHONE, alertMsg);
  }
}

void sendSummary() {
  String summary = "FARM SUMMARY:\n";
  for (auto const &[id, data] : farmData) {
    summary +=
        id + ": " + String(data.temp, 1) + "C, " + String(data.hr) + "BPM\n";
  }
  gsm.sendSMS(ADMIN_PHONE, summary);
}

void setup() {
  Serial.begin(115200);
  int loraState = lora.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC);
  if (loraState != RADIOLIB_ERR_NONE)
    Serial.println("LoRa fail");
  if (!gsm.begin())
    Serial.println("GSM fail");
  lastSummaryTime = millis();
}

void loop() {
  String received;
  int state = lora.receive(received);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Received: " + received);
    processPacket(received);
  }

  if (millis() - lastSummaryTime >= SUMMARY_INTERVAL_MS) {
    sendSummary();
    lastSummaryTime = millis();
  }
}
