#include "esp_task_wdt.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>

// ==========================================
// CONFIGURATION & PIN DEFINITIONS
// ==========================================
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2
#define GSM_TX 17
#define GSM_RX 16
#define OLED_SDA 21
#define OLED_SCL 22
#define BTN_MENU 25
#define BTN_SELECT 26
#define BUZZER_PIN 27
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SX1278 lora = new Module(LORA_SS, LORA_DIO0, LORA_RST);
HardwareSerial &gsmSerial = Serial2;
Preferences preferences;

bool oledOk = false, loraOk = false, gsmOk = false;

// Configs
float cfgTempHigh = 39.5, cfgTempLow = 37.0;
int cfgHrHigh = 90, cfgHrLow = 50, cfgMockRateS = 10, cfgSleepMin = 0;
int cfgOledOffSec = 30; // Default 30 seconds
char cfgGsmNum[16] = "+918898865508";
bool cfgBuzzerOn = true;
int cfgSmsIntMin = 5; // Default 5 minutes

// Stats
int mockCows = 0;
float lastCowTemp = 0.0;
int lastCowHr = 0;
int lastCowId = 0;
int lastRssi = -120, lastGsmCsq = 0, lastGsmDbm = -110;
char alertStatus[32] = "NORMAL";
unsigned long bootTime = 0;
unsigned long lastSmsTime = 0;
unsigned long lastPacketTime = 0;
unsigned long lastRecoveryBtnTime = 0;
#define SMS_COOLDOWN_MS 300000UL // 5 Minutes

int beepRem = 0;
unsigned long beepTimer = 0;
int beepDur = 0;

uint16_t gatewayPktSeq = 0;
uint32_t totalPktsLost = 0;
uint32_t totalPktsRcvd = 0;

// ==========================================
// HIERARCHICAL UI STATES
// ==========================================
enum MainState { S_DASH, S_MENU, S_EDIT, S_TEST, S_DEBUG, S_INFO, S_MONITOR };
enum SubMenu { M_NONE, M_MAIN, M_CONFIG, M_ALERTS, M_TEST, M_DEBUG };
enum GsmState {
  GSM_BOOT,
  GSM_IDLE,
  GSM_SET_TEXT_MODE,
  GSM_TX_SMS_START,
  GSM_WAIT_RESP,
  GSM_TX_BODY,
  GSM_WAIT_SMS_OK
};

MainState uiState = S_DASH;
SubMenu currentMenu = M_NONE;
GsmState gsmCommState = GSM_BOOT;
unsigned long gsmTimer = 0;
int gsmRetryCount = 0;
char gsmSmsBuffer[64] = "";
bool gsmSmsPending = false;

int menuIndex = 0;
int editDigitIndex = 0;
int editConfigId = 0; // Tracks which config we are editing
unsigned long uiTimer = 0;
bool uiTimerActive = false;
bool showSmsToast = false;
unsigned long smsToastTimer = 0;
unsigned long lastUserAction = 0;
bool isOledOff = false;

// Menu Arrays
const int NUM_MAIN = 5;
const char *menuMain[] = {"MONITOR", "TEST", "CONFIG", "DEBUG", "INFO"};

const int NUM_TEST = 6;
const char *menuTest[] = {"LORA PING",  "NODE LED ON", "NODE LED OFF",
                          "NODE BLINK", "GSM CALL",    "GSM SMS"};

const int NUM_CONFIG = 7;
const char *menuConfig[] = {"ALERTS...",    "GSM NUMBER",  "SIM RATE",
                            "SMS INTERVAL", "DISPLAY OFF", "SLEEP TIMER",
                            "BUZZER TOGGLE"};

const int NUM_ALERTS = 4;
const char *menuAlerts[] = {"TEMP HIGH", "TEMP LOW", "HR HIGH", "HR LOW"};

const int NUM_DEBUG = 3;
const char *menuDebug[] = {"RAW LORA", "RAW GSM", "SYS MEMORY"};

// Sub-states for specific tools
enum ToolState {
  T_NONE,
  T_LORA_PING,
  T_GSM_CALL,
  T_GSM_SMS,
  T_DISP_TEST,
  T_RAW_LORA,
  T_RAW_GSM,
  T_SYS_MEM,
  T_INFO,
  T_MONITOR
};
ToolState activeTool = T_NONE;

void beep(int times, int duration);
void queueAlert(const char *reason);

// ==========================================
// HELPERS
// ==========================================
void beep(int times, int duration) {
  if (!cfgBuzzerOn)
    return;
  beepRem = times * 2;
  beepDur = duration;
  beepTimer = millis();
}

void handleBeep() {
  if (beepRem <= 0)
    return;
  if (millis() - beepTimer > beepDur) {
    if (beepRem % 2 == 0)
      digitalWrite(BUZZER_PIN, HIGH);
    else
      digitalWrite(BUZZER_PIN, LOW);
    beepRem--;
    beepTimer = millis();
  }
}

void loadConfig() {
  preferences.begin("vetnode", false);
  cfgTempHigh = preferences.getFloat("th", 39.5);
  cfgTempLow = preferences.getFloat("tl", 37.0);
  cfgHrHigh = preferences.getInt("hh", 90);
  cfgHrLow = preferences.getInt("hl", 50);
  cfgMockRateS = preferences.getInt("mr", 10);
  cfgBuzzerOn = preferences.getBool("bz", true);
  cfgSleepMin = preferences.getInt("slp", 0);
  cfgSmsIntMin = preferences.getInt("si", 5);
  cfgOledOffSec = preferences.getInt("oto", 30);
  String num = preferences.getString("gsm", "+918898865508");
  num.toCharArray(cfgGsmNum, sizeof(cfgGsmNum));
  preferences.end();
}
void saveConfig() {
  preferences.begin("vetnode", false);
  preferences.putFloat("th", cfgTempHigh);
  preferences.putFloat("tl", cfgTempLow);
  preferences.putInt("hh", cfgHrHigh);
  preferences.putInt("hl", cfgHrLow);
  preferences.putInt("mr", cfgMockRateS);
  preferences.putBool("bz", cfgBuzzerOn);
  preferences.putInt("slp", cfgSleepMin);
  preferences.putInt("si", cfgSmsIntMin);
  preferences.putInt("oto", cfgOledOffSec);
  preferences.putString("gsm", String(cfgGsmNum));
  preferences.end();
}

void updateGsmSignal() {
  if (!gsmOk)
    return;
  // Send the request, help handleGsmState will catch it eventually or just read
  // next poll
  gsmSerial.println("AT+CSQ");
}

void getFormatTime(unsigned long millisSince, char *outBuf) {
  if (millisSince == 0) {
    strcpy(outBuf, "Never");
    return;
  }
  unsigned long secs = (millis() - millisSince) / 1000;
  if (secs < 60)
    snprintf(outBuf, 16, "%lus ago", secs);
  else if (secs < 3600)
    snprintf(outBuf, 16, "%lum ago", secs / 60);
  else
    snprintf(outBuf, 16, "%luh ago", secs / 3600);
}

void getUptime(char *outBuf) {
  unsigned long secs = millis() / 1000;
  int h = secs / 3600;
  int m = (secs % 3600) / 60;
  int s = secs % 60;
  snprintf(outBuf, 10, "%02d:%02d:%02d", h, m, s);
}

// ==========================================
// UI DRAWING KERNEL
// ==========================================
void drawGsmBars(int x, int y, int csq) {
  int level = 0;
  if (csq > 2 && csq <= 9)
    level = 1;
  else if (csq > 9 && csq <= 14)
    level = 2;
  else if (csq > 14 && csq <= 19)
    level = 3;
  else if (csq > 19 && csq != 99)
    level = 4;

  for (int i = 0; i < 4; i++) {
    int h = 2 + (i * 2);
    if (i < level)
      display.fillRect(x + (i * 3), y + 8 - h, 2, h, SH110X_WHITE);
    else
      display.drawRect(x + (i * 3), y + 8 - h, 2, h, SH110X_WHITE);
  }
}

void drawLoraBars(int x, int y, int rssi) {
  int level = 0;
  if (rssi > -70)
    level = 4;
  else if (rssi > -90)
    level = 3;
  else if (rssi > -110)
    level = 2;
  else if (rssi > -125)
    level = 1;

  for (int i = 0; i < 4; i++) {
    int h = 2 + (i * 2);
    if (i < level)
      display.fillRect(x + (i * 3), y + 8 - h, 2, h, SH110X_WHITE);
    else
      display.drawRect(x + (i * 3), y + 8 - h, 2, h, SH110X_WHITE);
  }
}

void renderDashboard() {
  // --- TOP BAR ---
  display.setCursor(0, 0);
  display.print("VETNODE v1.3");

  if (gsmOk)
    drawGsmBars(82, 0, lastGsmCsq);
  display.setCursor(96, 0);
  if (gsmOk && lastGsmCsq != 99) {
    display.print(lastGsmDbm);
    display.print("d");
  } else
    display.print("NoGSM");
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);

  // --- MIDDLE AREA (SENSOR DATA) ---
  if (lastCowId != 0) {
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.print("COW ID: ");
    display.setTextSize(2);
    display.print(lastCowId);

    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print("TEMP:");
    display.print(lastCowTemp);
    display.print("C");

    display.setCursor(75, 24);
    display.print("HR:");
    display.print(lastCowHr);
  } else {
    display.setCursor(0, 20);
    display.println("Waiting for telemetry...");
  }

  // --- BOTTOM BAR ---
  display.drawLine(0, 36, 128, 36, SH110X_WHITE);
  display.setCursor(0, 40);
  if (loraOk) {
    drawLoraBars(0, 40, lastRssi);
    display.setCursor(14, 40);
    display.print(lastRssi);
    display.print("dBm");
  } else {
    display.print("LORA: FAIL");
  }

  display.setCursor(70, 40);
  char lTime[16];
  getFormatTime(lastPacketTime, lTime);
  display.print("LS:");
  display.print(lTime);

  // STATUS BANNER (Inverted if Alert)
  if (strcmp(alertStatus, "NORMAL") == 0) {
    display.setCursor(0, 52);
    display.print("STATUS: ");
    display.print(alertStatus);
  } else {
    display.fillRect(0, 50, 128, 14, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    display.setCursor(2, 53);
    display.print("STATUS: ");
    display.print(alertStatus);
    display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  }

  // Tiny Icons Top Right Setup area
  if (cfgBuzzerOn)
    display.fillRect(124, 0, 4, 4, SH110X_WHITE); // Dot for buzzer

  // --- SMS TOAST ANIMATION ---
  if (showSmsToast) {
    if (millis() - smsToastTimer > 3000) {
      showSmsToast = false;
    } else {
      display.fillRect(20, 20, 88, 20, SH110X_BLACK);
      display.drawRect(20, 20, 88, 20, SH110X_WHITE);
      display.setCursor(32, 26);
      display.print("SMS SENT!");
    }
  }
}

void showBootSplash() {
  // Removed as per user request
}

void renderMenu(const char *title, const char *items[], int maxItems,
                int offset = 0) {
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);
  display.setCursor(0, 0);
  display.print(title);

  int startIdx = (menuIndex / 4) * 4;
  for (int i = 0; i < 4; i++) {
    int itemIdx = startIdx + i;
    if (itemIdx >= maxItems)
      break;
    display.setCursor(0, 14 + (i * 12));
    if (itemIdx == menuIndex) {
      display.print("> ");
      // Invert selected line
      display.setTextColor(SH110X_BLACK, SH110X_WHITE);
      display.print(items[itemIdx]);
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
    } else {
      display.print("  ");
      display.print(items[itemIdx]);
    }
  }
  display.drawLine(0, 63, 128, 63, SH110X_WHITE);
}

void renderEditPage() {
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("CONFIG: ");
  display.setCursor(0, 16);

  if (editConfigId == 0) {
    display.println("TEMP HIGH LIMIT");
    display.setTextSize(2);
    display.setCursor(10, 30);
    display.print(cfgTempHigh, 1);
    display.print("C");
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("(35.0 - 45.0)");
  } else if (editConfigId == 1) {
    display.println("TEMP LOW LIMIT");
    display.setTextSize(2);
    display.setCursor(10, 30);
    display.print(cfgTempLow, 1);
    display.print("C");
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("(32.0 - 40.0)");
  } else if (editConfigId == 2) {
    display.println("HR HIGH LIMIT");
    display.setTextSize(2);
    display.setCursor(10, 30);
    display.print(cfgHrHigh);
    display.print(" bpm");
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("(60 - 150)");
  } else if (editConfigId == 3) {
    display.println("HR LOW LIMIT");
    display.setTextSize(2);
    display.setCursor(10, 30);
    display.print(cfgHrLow);
    display.print(" bpm");
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("(30 - 80)");
  } else if (editConfigId == 4) {
    display.println("SIMULATION RATE");
    display.setTextSize(2);
    display.setCursor(10, 30);
    display.print(cfgMockRateS);
    display.print(" s");
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("(5 - 60)");
  } else if (editConfigId == 5) {
    display.println("SLEEP TIMER");
    display.setTextSize(2);
    display.setCursor(10, 30);
    display.print(cfgSleepMin);
    display.print(" m");
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("(0=Off, 5-60)");
    display.setCursor(0, 54);
    display.print("S:Next L.S:Save");
  } else if (editConfigId == 7) {
    display.println("SMS ALERT INTRVL");
    display.setTextSize(2);
    display.setCursor(10, 30);
    display.print(cfgSmsIntMin);
    display.print(" m");
    display.setCursor(0, 50);
    display.print("(1, 5, 10, 30, 60)");
  } else if (editConfigId == 8) {
    display.println("OLED AUTO-OFF");
    display.setTextSize(2);
    display.setCursor(10, 30);
    if (cfgOledOffSec == 0)
      display.print("ALWAYS ON");
    else {
      display.print(cfgOledOffSec);
      display.print(" s");
    }
    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print("0=ON, 15,30,60,300");
  }
}

void renderToolPage() {
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);

  if (activeTool == T_MONITOR) {
    display.setCursor(0, 0);
    display.println("LIVE TELEMETRY");
    display.setCursor(0, 14);
    display.print("TEMP H/L : ");
    display.print(cfgTempHigh, 1);
    display.print("/");
    display.println(cfgTempLow, 1);
    display.setCursor(0, 24);
    display.print("HR H/L   : ");
    display.print(cfgHrHigh);
    display.print("/");
    display.println(cfgHrLow);
    display.setCursor(0, 34);
    display.print("SIM RATE : ");
    display.print(cfgMockRateS);
    display.println("s");
    display.setCursor(0, 44);
    display.print("SLEEP    : ");
    display.print(cfgSleepMin);
    display.println("m");
  } else if (activeTool == T_INFO) {
    display.setCursor(0, 0);
    display.println("SYSTEM INFO");
    display.setCursor(0, 14);
    display.println("Device: VETNODE GW");
    display.setCursor(0, 24);
    display.println("FW: 1.4 PRO");
    display.setCursor(0, 34);
    display.println("Build: 03-03-2026");
    display.setCursor(0, 44);
    display.println("LoRa: 433MHz");
    display.setCursor(0, 54);
    display.println("GSM: SIM800L Series");
  } else if (activeTool == T_SYS_MEM) {
    display.setCursor(0, 0);
    display.println("MEMORY & WATCHDOG");
    display.setCursor(0, 14);
    display.print("Free Heap: ");
    display.print(ESP.getFreeHeap() / 1024);
    display.println(" KB");
    display.setCursor(0, 24);
    display.print("Uptime: ");
    char uptimeBuf[16];
    getUptime(uptimeBuf);
    display.println(uptimeBuf);
    display.setCursor(0, 36);
    display.print("Loss %: ");
    if (totalPktsRcvd + totalPktsLost > 0) {
      float loss =
          (float)totalPktsLost / (totalPktsRcvd + totalPktsLost) * 100.0;
      display.print(loss, 1);
    } else {
      display.print("0.0");
    }
    display.println(" %");
    display.setCursor(0, 48);
    display.print("Last Pkt: ");
    char pktTimeBuf[16];
    getFormatTime(lastPacketTime, pktTimeBuf);
    display.println(pktTimeBuf);
  } else if (activeTool == T_RAW_LORA) {
    display.setCursor(0, 0);
    display.println("RAW LORA STATS");
    display.setCursor(0, 16);
    display.print("RSSI : ");
    display.print(lora.getRSSI());
    display.println(" dBm");
    display.setCursor(0, 28);
    display.print("SNR  : ");
    display.print(lora.getSNR());
    display.println(" dB");
    display.setCursor(0, 40);
    display.println("BW: 125  | SF: 7");
    display.setCursor(0, 52);
    display.println("PWR: 10dBm");
  } else if (activeTool == T_RAW_GSM) {
    display.setCursor(0, 0);
    display.println("RAW GSM UART");
    display.setCursor(0, 16);
    display.print("CSQ Numeric : ");
    display.println(lastGsmCsq);
    display.setCursor(0, 26);
    display.print("CSQ (dBm)   : ");
    display.print(lastGsmDbm);
    display.println("dB");
    display.setCursor(0, 38);
    display.println("Terminal Active.");
    display.setCursor(0, 48);
    display.println("-> Send AT from PC");
  } else if (activeTool == T_LORA_PING) {
    display.setCursor(0, 0);
    display.println("TEST: LORA PING");
    display.setCursor(0, 30);
    display.println("Sending Syn/Ack...");
  } else if (activeTool == T_GSM_CALL) {
    display.setCursor(0, 0);
    display.println("TEST: GSM CALL");
    display.setCursor(0, 24);
    display.println("Calling Admin:");
    display.setCursor(0, 34);
    display.println(cfgGsmNum);
    display.setCursor(0, 50);
    display.println("L.M: Cancel/Hangup");
  } else if (activeTool == T_GSM_SMS) {
    display.setCursor(0, 0);
    display.println("TEST: GSM SMS");
    display.setCursor(0, 30);
    display.println("Sending Test SMS...");
  }
}

void updateUI() {
  if (!oledOk)
    return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (uiState == S_DASH)
    renderDashboard();
  else if (uiState == S_MENU) {
    if (currentMenu == M_MAIN)
      renderMenu("MAIN MENU", menuMain, NUM_MAIN);
    else if (currentMenu == M_TEST)
      renderMenu("TEST TOOLS", menuTest, NUM_TEST);
    else if (currentMenu == M_CONFIG)
      renderMenu("CONFIG", menuConfig, NUM_CONFIG);
    else if (currentMenu == M_ALERTS)
      renderMenu("ALERT LIMITS", menuAlerts, NUM_ALERTS);
    else if (currentMenu == M_DEBUG)
      renderMenu("DEBUG/DIAGS", menuDebug, NUM_DEBUG);
  } else if (uiState == S_EDIT)
    renderEditPage();
  else if (uiState == S_TEST || uiState == S_DEBUG || uiState == S_INFO ||
           uiState == S_MONITOR)
    renderToolPage();

  display.display();
}

// ==========================================
// INPUT CONTROLLER
// ==========================================
// M: Scroll/Increase | Long M: Back/Decrease | S: Enter/Next | Long S: Save
bool btnMenuState = true, btnSelState = true, menuHandled = false,
     selHandled = false;
unsigned long menuPressTime = 0, selPressTime = 0;

void handleLogicInput(char btn, bool isLongMode) {
  beep(1, 15);

  // DASHBOARD
  if (uiState == S_DASH && isLongMode) {
    uiState = S_MENU;
    currentMenu = M_MAIN;
    menuIndex = 0;
  }

  // MENUS
  else if (uiState == S_MENU) {
    if (btn == 'M' && !isLongMode) { // Scroll down
      if (currentMenu == M_MAIN)
        menuIndex = (menuIndex + 1) % NUM_MAIN;
      else if (currentMenu == M_TEST)
        menuIndex = (menuIndex + 1) % NUM_TEST;
      else if (currentMenu == M_CONFIG)
        menuIndex = (menuIndex + 1) % NUM_CONFIG;
      else if (currentMenu == M_ALERTS)
        menuIndex = (menuIndex + 1) % NUM_ALERTS;
      else if (currentMenu == M_DEBUG)
        menuIndex = (menuIndex + 1) % NUM_DEBUG;
    } else if (btn == 'M' && isLongMode) { // Back
      if (currentMenu == M_MAIN) {
        uiState = S_DASH;
      } else if (currentMenu == M_TEST || currentMenu == M_CONFIG ||
                 currentMenu == M_DEBUG) {
        currentMenu = M_MAIN;
        menuIndex = 0;
      } else if (currentMenu == M_ALERTS) {
        currentMenu = M_CONFIG;
        menuIndex = 0;
      }
    } else if (btn == 'S' && !isLongMode) { // Enter Selection
      if (currentMenu == M_MAIN) {
        if (menuIndex == 0) {
          uiState = S_MONITOR;
          activeTool = T_MONITOR;
        } else if (menuIndex == 1) {
          currentMenu = M_TEST;
          menuIndex = 0;
        } else if (menuIndex == 2) {
          currentMenu = M_CONFIG;
          menuIndex = 0;
        } else if (menuIndex == 3) {
          currentMenu = M_DEBUG;
          menuIndex = 0;
        } else if (menuIndex == 4) {
          uiState = S_INFO;
          activeTool = T_INFO;
        }
      } else if (currentMenu == M_TEST) {
        uiState = S_TEST;
        if (menuIndex == 0) {
          activeTool = T_LORA_PING;
          updateUI();
          if (loraOk) {
            lora.transmit("PING");
            lora.startReceive();
          }
          uiTimer = millis();
          uiTimerActive = true;
        } else if (menuIndex == 1) { // NODE LED ON
          updateUI();
          if (loraOk) {
            lora.transmit("CMD,LED_ON");
            lora.startReceive();
          }
          uiTimer = millis();
          uiTimerActive = true;
        } else if (menuIndex == 2) { // NODE LED OFF
          updateUI();
          if (loraOk) {
            lora.transmit("CMD,LED_OFF");
            lora.startReceive();
          }
          uiTimer = millis();
          uiTimerActive = true;
        } else if (menuIndex == 3) { // NODE BLINK
          updateUI();
          if (loraOk) {
            lora.transmit("CMD,BLINK");
            lora.startReceive();
          }
          uiTimer = millis();
          uiTimerActive = true;
        } else if (menuIndex == 4) { // GSM CALL
          activeTool = T_GSM_CALL;
          if (gsmOk) {
            gsmSerial.print("ATD");
            gsmSerial.print(cfgGsmNum);
            gsmSerial.println(";");
          }
        } else if (menuIndex == 5) { // GSM SMS
          if (gsmOk && !gsmSmsPending) {
            strncpy(gsmSmsBuffer, "VETNODE TEST SMS", sizeof(gsmSmsBuffer) - 1);
            gsmSmsPending = true;
            lastSmsTime = millis();
            activeTool = T_GSM_SMS;
            updateUI();
          }
          uiTimer = millis();
          uiTimerActive = true;
        }
      } else if (currentMenu == M_CONFIG) {
        if (menuIndex == 0) {
          currentMenu = M_ALERTS;
          menuIndex = 0;
        } else if (menuIndex == 1) {
          uiState = S_EDIT;
          editConfigId = 6;
          editDigitIndex = 1;
        } else if (menuIndex == 2) {
          uiState = S_EDIT;
          editConfigId = 4;
        } else if (menuIndex == 3) {
          uiState = S_EDIT;
          editConfigId = 7;
        } else if (menuIndex == 4) {
          uiState = S_EDIT;
          editConfigId = 8;
        } else if (menuIndex == 5) {
          uiState = S_EDIT;
          editConfigId = 5;
        } else if (menuIndex == 6) {
          cfgBuzzerOn = !cfgBuzzerOn;
          saveConfig();

          // Visual Save Feedback
          display.fillRect(10, 20, 108, 24, SH110X_BLACK);
          display.drawRect(10, 20, 108, 24, SH110X_WHITE);
          display.setCursor(35, 28);
          display.print("SAVED!");
          display.display();

          if (cfgBuzzerOn)
            beep(2, 50);
          else
            delay(300);
        }
      } else if (currentMenu == M_ALERTS) {
        uiState = S_EDIT;
        editConfigId = menuIndex;
      } else if (currentMenu == M_DEBUG) {
        uiState = S_DEBUG;
        if (menuIndex == 0)
          activeTool = T_RAW_LORA;
        else if (menuIndex == 1)
          activeTool = T_RAW_GSM;
        else if (menuIndex == 2)
          activeTool = T_SYS_MEM;
      }
    }
  }

  // EDIT UI
  else if (uiState == S_EDIT) {
    if (btn == 'M' && !isLongMode) { // Increase
      if (editConfigId == 0) {
        cfgTempHigh += 0.5;
        if (cfgTempHigh > 45.0)
          cfgTempHigh = 35.0;
      }
      if (editConfigId == 1) {
        cfgTempLow += 0.5;
        if (cfgTempLow > 40.0)
          cfgTempLow = 32.0;
      }
      if (editConfigId == 2) {
        cfgHrHigh += 5;
        if (cfgHrHigh > 150)
          cfgHrHigh = 60;
      }
      if (editConfigId == 3) {
        cfgHrLow += 5;
        if (cfgHrLow > 80)
          cfgHrLow = 30;
      }
      if (editConfigId == 4) {
        cfgMockRateS += 5;
        if (cfgMockRateS > 60)
          cfgMockRateS = 5;
      }
      if (editConfigId == 5) {
        cfgSleepMin += 5;
        if (cfgSleepMin > 60)
          cfgSleepMin = 0;
      }
      if (editConfigId == 6) {
        char ch = cfgGsmNum[editDigitIndex];
        if (ch >= '0' && ch < '9')
          ch++;
        else if (ch == '9')
          ch = '0';
        cfgGsmNum[editDigitIndex] = ch;
      }
      if (editConfigId == 7) {
        if (cfgSmsIntMin == 1)
          cfgSmsIntMin = 5;
        else if (cfgSmsIntMin == 5)
          cfgSmsIntMin = 10;
        else if (cfgSmsIntMin == 10)
          cfgSmsIntMin = 30;
        else if (cfgSmsIntMin == 30)
          cfgSmsIntMin = 60;
        else
          cfgSmsIntMin = 1;
      }
      if (editConfigId == 8) {
        if (cfgOledOffSec == 0)
          cfgOledOffSec = 15;
        else if (cfgOledOffSec == 15)
          cfgOledOffSec = 30;
        else if (cfgOledOffSec == 30)
          cfgOledOffSec = 60;
        else if (cfgOledOffSec == 60)
          cfgOledOffSec = 300;
        else
          cfgOledOffSec = 0;
      }
    } else if (btn == 'M' && isLongMode) { // Decrease / Back for numbers
      if (editConfigId == 0) {
        cfgTempHigh -= 0.5;
        if (cfgTempHigh < 35.0)
          cfgTempHigh = 45.0;
      }
      if (editConfigId == 1) {
        cfgTempLow -= 0.5;
        if (cfgTempLow < 32.0)
          cfgTempLow = 40.0;
      }
      if (editConfigId == 2) {
        cfgHrHigh -= 5;
        if (cfgHrHigh < 60)
          cfgHrHigh = 150;
      }
      if (editConfigId == 3) {
        cfgHrLow -= 5;
        if (cfgHrLow < 30)
          cfgHrLow = 80;
      }
      if (editConfigId == 4) {
        cfgMockRateS -= 5;
        if (cfgMockRateS < 5)
          cfgMockRateS = 60;
      }
      if (editConfigId == 5) {
        cfgSleepMin -= 5;
        if (cfgSleepMin < 0)
          cfgSleepMin = 60;
      }
    } else if (btn == 'S' && !isLongMode) { // Check digit or save
      if (editConfigId == 6) {
        editDigitIndex++;
        if (cfgGsmNum[editDigitIndex] == 0)
          editDigitIndex = 1;
      }
    } else if (btn == 'S' && isLongMode) { // Commit Save
      saveConfig();

      // Visual Save Feedback
      display.fillRect(10, 20, 108, 24, SH110X_BLACK);
      display.drawRect(10, 20, 108, 24, SH110X_WHITE);
      display.setCursor(35, 28);
      display.print("SAVED!");
      display.display();

      beep(2, 50);
      delay(300);       // Hold message briefly
      uiState = S_MENU; // Exit to parent
    }
  }

  // TOOLS
  else if (uiState == S_TEST || uiState == S_DEBUG || uiState == S_INFO ||
           uiState == S_MONITOR) {
    if (btn == 'M' && isLongMode) { // Exit tool
      if (activeTool == T_GSM_CALL) {
        if (gsmOk)
          gsmSerial.println("ATH");
      } // Hang up call
      uiState = S_MENU;
      activeTool = T_NONE;
    }
  }

  updateUI();
}

void scanButtons() {
  bool currentMenu = digitalRead(BTN_MENU),
       currentSel = digitalRead(BTN_SELECT);
  unsigned long now = millis();
  if (currentMenu == LOW && btnMenuState == HIGH) {
    lastUserAction = now;
    menuPressTime = now;
    btnMenuState = LOW;
    menuHandled = false;
    if (isOledOff) {
      display.oled_command(SH110X_DISPLAYON);
      isOledOff = false;
      menuHandled = true; // Block action on wake press
    }
  } else if (currentMenu == LOW && btnMenuState == LOW && !menuHandled) {
    if (now - menuPressTime > 800) {
      handleLogicInput('M', true);
      menuHandled = true;
    }
  } else if (currentMenu == HIGH && btnMenuState == LOW) {
    if (!menuHandled && now - menuPressTime > 50)
      handleLogicInput('M', false);
    btnMenuState = HIGH;
  }

  if (currentSel == LOW && btnSelState == HIGH) {
    lastUserAction = now;
    selPressTime = now;
    btnSelState = LOW;
    selHandled = false;
    if (isOledOff) {
      display.oled_command(SH110X_DISPLAYON);
      isOledOff = false;
      selHandled = true; // Block action on wake press
    }
  } else if (currentSel == LOW && btnSelState == LOW && !selHandled) {
    if (now - selPressTime > 800) {
      handleLogicInput('S', true);
      selHandled = true;
    }
  } else if (currentSel == HIGH && btnSelState == LOW) {
    if (!selHandled && now - selPressTime > 50)
      handleLogicInput('S', false);
    btnSelState = HIGH;
  }
}

void handleGsmState() {
  unsigned long now = millis();

  switch (gsmCommState) {
  case GSM_BOOT:
    if (now - gsmTimer > 3000) {
      Serial.println("[GSM] Sending AT (Boot)...");
      gsmSerial.print("AT\r\n");
      gsmTimer = now;
      gsmCommState = GSM_WAIT_RESP;
    }
    break;

  case GSM_IDLE:
    if (gsmSmsPending) {
      Serial.println("[GSM] SMS Pending. Setting Text Mode...");
      gsmCommState = GSM_SET_TEXT_MODE;
      gsmTimer = now;
    }
    break;

  case GSM_SET_TEXT_MODE:
    Serial.println("[GSM] AT+CMGF=1");
    gsmSerial.print("AT+CMGF=1\r\n");
    gsmTimer = now;
    gsmCommState = GSM_WAIT_RESP;
    break;

  case GSM_TX_SMS_START:
    Serial.print("[GSM] AT+CMGS to ");
    Serial.println(cfgGsmNum);
    gsmSerial.print("AT+CMGS=\"");
    gsmSerial.print(cfgGsmNum);
    gsmSerial.println("\"");
    gsmTimer = now;
    gsmCommState = GSM_WAIT_RESP;
    break;

  case GSM_WAIT_RESP:
    if (gsmSerial.available()) {
      char resp[64];
      int len = 0;
      delay(50);
      while (gsmSerial.available() && len < 63)
        resp[len++] = gsmSerial.read();
      resp[len] = '\0';

      if (strstr(resp, ">")) {
        gsmCommState = GSM_TX_BODY;
      } else if (strstr(resp, "OK")) {
        gsmOk = true;
        Serial.println("[GSM] Response: OK");
        if (gsmSmsPending)
          gsmCommState = GSM_TX_SMS_START;
        else
          gsmCommState = GSM_IDLE;
      } else if (strstr(resp, "ERROR")) {
        Serial.println("[GSM] Response: ERROR");
        gsmCommState = GSM_IDLE;
        gsmSmsPending = false;
      }
    }
    if (now - gsmTimer > 5000)
      gsmCommState = GSM_IDLE;
    break;

  case GSM_TX_BODY:
    gsmSerial.print(gsmSmsBuffer);
    gsmSerial.write(26);
    gsmTimer = now;
    gsmCommState = GSM_WAIT_SMS_OK;
    break;

  case GSM_WAIT_SMS_OK:
    if (gsmSerial.available()) {
      char resp[64]; // Increased size for safety
      int len = 0;
      while (gsmSerial.available() && len < 31)
        resp[len++] = gsmSerial.read();
      resp[len] = '\0';
      if (strstr(resp, "OK") || strstr(resp, "+CMGS:")) {
        gsmCommState = GSM_IDLE;
        gsmSmsPending = false;
        showSmsToast = true;
        smsToastTimer = millis();
      }
    }
    if (now - gsmTimer > 15000) {
      gsmCommState = GSM_IDLE;
      gsmSmsPending = false;
    }
    break;
  }
}

void setup() {
  Serial.begin(115200);
  bootTime = millis();

  // Initialize Task Watchdog (10 second timeout) for ESP32 Core 3.x
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = 10000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true,
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);

  loadConfig();
  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  if (display.begin(0x3C, true))
    oledOk = true;

  if (oledOk)
    showBootSplash();

  // Wait for modules to stabilize
  delay(1000);

  pinMode(LORA_DIO0, INPUT);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  // Match original params: Freq, BW, SF, CR, Sync, Pwr, Preamble
  if (lora.begin(433.0, 125.0, 7, 5, 0x12, 10, 8) == RADIOLIB_ERR_NONE) {
    loraOk = true;
    lora.startReceive();
  }

  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);
  // Non-blocking GSM init will happen in loop() state machine
  gsmCommState = GSM_BOOT;
  gsmTimer = millis();

  lastPacketTime = millis();
  lastUserAction = millis(); // Initialize activity
  beep(2, 50);
  updateUI();
}

unsigned long lastMockLogic = 0;
unsigned long lastDashRefresh = 0;

void loop() {
  esp_task_wdt_reset(); // Feed the Task Watchdog
  scanButtons();
  handleGsmState(); // Run the asynchronous GSM manager
  handleBeep();     // Run the asynchronous buzzer manager

  // Non-blocking UI Auto-exit timer
  if (uiTimerActive && millis() - uiTimer > 1000) {
    uiState = S_MENU;
    uiTimerActive = false;
  }

  // GSM passthrough
  if (gsmSerial.available()) {
    char c = gsmSerial.read();
    Serial.write(c);
  }
  if (Serial.available())
    gsmSerial.write(Serial.read());

  // ===============================
  // LoRa Reception & Parsing
  // ===============================
  if (loraOk && digitalRead(LORA_DIO0) == HIGH) {
    char packet[64];
    // Simple read: rely on RadioLib to manage FIFO
    int state = lora.readData((uint8_t *)packet, 63);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.print("[RX DATA] ");
      Serial.println(packet);
      lora.startReceive();

      lastPacketTime = millis();
      lastRssi = lora.getRSSI();
      lastUserAction = millis(); // Wake dashboard on packet

      if (strncmp(packet, "HB,", 3) == 0) {
        uint16_t incomingSeq = 0;
        int cowId = 0;
        float temp = 0.0;
        char errFlag[16] = "";
        // Parse CSV: HB,seq,id,temp,err
        if (sscanf(packet + 3, "%hu,%d,%f,%15[^,\r\n]", &incomingSeq, &cowId,
                   &temp, errFlag) >= 3) {
          totalPktsRcvd++;
          if (gatewayPktSeq > 0 && incomingSeq > gatewayPktSeq + 1) {
            totalPktsLost += (incomingSeq - gatewayPktSeq - 1);
          }
          gatewayPktSeq = incomingSeq;
          lastCowId = cowId;
          lastCowTemp = temp;
          snprintf(alertStatus, sizeof(alertStatus), "NORMAL");

          if (lastCowTemp >= cfgTempHigh) {
            snprintf(alertStatus, sizeof(alertStatus), "TEMP HIGH");
            char buf[48];
            snprintf(buf, sizeof(buf), "ID:%d T_HI:%.1fC (HB)", cowId, temp);
            Serial.print("[HB ALERT] ");
            Serial.println(buf);
            queueAlert(buf);
          } else if (lastCowTemp <= cfgTempLow) {
            snprintf(alertStatus, sizeof(alertStatus), "TEMP LOW");
            char buf[48];
            snprintf(buf, sizeof(buf), "ID:%d T_LO:%.1fC (HB)", cowId, temp);
            Serial.print("[HB ALERT] ");
            Serial.println(buf);
            queueAlert(buf);
          } else if (strcmp(errFlag, "0") != 0 && strlen(errFlag) > 0) {
            snprintf(alertStatus, sizeof(alertStatus), "ERR: %s", errFlag);
            char buf[48];
            snprintf(buf, sizeof(buf), "ID:%d SENSOR_ERR:%s", cowId, errFlag);
            Serial.print("[HB ERR] ");
            Serial.println(buf);
            queueAlert(buf);
          }
        }
      }

      else if (strncmp(packet, "DATA,", 5) == 0) {
        mockCows++;
        beep(2, 50);

        uint16_t incomingSeq = 0;
        int cowId = 0;
        float temp = 0.0;
        int hr = 0;
        // Parse CSV: DATA,seq,id,temp,hr
        if (sscanf(packet + 5, "%hu,%d,%f,%d", &incomingSeq, &cowId, &temp,
                   &hr) >= 4) {
          totalPktsRcvd++;
          if (gatewayPktSeq > 0 && incomingSeq > gatewayPktSeq + 1) {
            totalPktsLost += (incomingSeq - gatewayPktSeq - 1);
          }
          gatewayPktSeq = incomingSeq;
          lastCowId = cowId;
          lastCowTemp = temp;
          lastCowHr = hr;

          snprintf(alertStatus, sizeof(alertStatus), "NORMAL");
          if (lastCowTemp >= cfgTempHigh || lastCowTemp <= cfgTempLow ||
              lastCowHr >= cfgHrHigh || lastCowHr <= cfgHrLow) {
            lastUserAction = millis(); // Wake for alerts
            if (isOledOff) {
              display.oled_command(SH110X_DISPLAYON);
              isOledOff = false;
            }
          }
          char alertReason[32] = "";
          snprintf(alertReason, sizeof(alertReason), "ID:%d ", lastCowId);

          if (lastCowTemp >= cfgTempHigh) {
            snprintf(alertStatus, sizeof(alertStatus), "TEMP HIGH");
            strncat(alertReason, "T_HI ",
                    sizeof(alertReason) - strlen(alertReason) - 1);
          } else if (lastCowTemp <= cfgTempLow) {
            snprintf(alertStatus, sizeof(alertStatus), "TEMP LOW");
            strncat(alertReason, "T_LO ",
                    sizeof(alertReason) - strlen(alertReason) - 1);
          }

          if (lastCowHr >= cfgHrHigh) {
            snprintf(alertStatus, sizeof(alertStatus), "HR HIGH");
            strncat(alertReason, "HR_HI",
                    sizeof(alertReason) - strlen(alertReason) - 1);
          } else if (lastCowHr <= cfgHrLow) {
            snprintf(alertStatus, sizeof(alertStatus), "HR LOW");
            strncat(alertReason, "HR_LO",
                    sizeof(alertReason) - strlen(alertReason) - 1);
          }

          if (strcmp(alertStatus, "NORMAL") != 0) {
            char valMsg[64];
            snprintf(valMsg, sizeof(valMsg), "ID:%d %s T:%.1f HR:%d", lastCowId,
                     alertStatus, lastCowTemp, lastCowHr);
            Serial.print("[ALERT] Triggered: ");
            Serial.println(valMsg);
            queueAlert(valMsg);
          }
        }
      }
    }
  }

  // ===============================
  // UI Refresh Logic
  // ===============================
  if (uiState == S_DASH) {

    if (millis() - lastDashRefresh > 3000) {

      updateGsmSignal();

      // AUTO-RECOVERY: If lost for > 60s, try to re-init radio every 60s
      static unsigned long lastLoraRetry = 0;
      if (millis() - lastPacketTime > 60000 &&
          millis() - lastLoraRetry > 60000) {
        lastLoraRetry = millis();
        Serial.print("[LORA] Stale connection. Re-init... ");
        if (lora.begin(433.0, 125.0, 7, 5, 0x12, 10, 8) == RADIOLIB_ERR_NONE) {
          loraOk = true;
          lora.startReceive();
          Serial.println("SUCCESS");
        } else {
          Serial.println("FAILED");
        }
      }

      if (uiState == S_DASH) {
        // Show "LORA LOST" if no packet for 30 seconds
        if (millis() - lastPacketTime > 30000) {
          snprintf(alertStatus, sizeof(alertStatus), "LORA LOST");
        }
        updateUI();
        lastDashRefresh = millis();
      }
    }
  } else if (uiState == S_DEBUG && activeTool == T_SYS_MEM) {

    if (millis() - lastDashRefresh > 1000) {
      updateUI();
      lastDashRefresh = millis();
    }
  } else if (uiState == S_DEBUG && activeTool == T_RAW_LORA) {

    if (millis() - lastDashRefresh > 500) {
      updateUI();
      lastDashRefresh = millis();
    }
  }

  // ===============================
  // Power Management (Inactivity Based)
  // ===============================
  if (cfgSleepMin > 0 &&
      (millis() - lastPacketTime > (cfgSleepMin * 60000ULL))) {
    display.clearDisplay();
    display.setCursor(30, 30);
    display.println("SLEEPING...");
    display.display();
    delay(1000);
  }

  // ===============================
  // OLED Power Management
  // ===============================
  if (cfgOledOffSec > 0 && !isOledOff &&
      (millis() - lastUserAction > (cfgOledOffSec * 1000UL))) {
    display.oled_command(SH110X_DISPLAYOFF);
    isOledOff = true;
  }
}
// Helper to trigger SMS and buzzer alerts
void queueAlert(const char *reason) {
  // Always try to queue alerts if threshold breached
  if (strcmp(alertStatus, "NORMAL") != 0) {
    unsigned long cooldownMs = cfgSmsIntMin * 60000UL;
    if ((lastSmsTime == 0 || millis() - lastSmsTime > cooldownMs) &&
        !gsmSmsPending) {
      Serial.print("[ALERT] Queuing SMS: ");
      Serial.println(reason);
      beep(3, 100);
      lastSmsTime = millis();
      strncpy(gsmSmsBuffer, reason, sizeof(gsmSmsBuffer) - 1);
      gsmSmsPending = true;
    } else if (gsmSmsPending) {
      Serial.println("[ALERT] SMS already pending...");
    } else {
      Serial.println("[ALERT] Cooldown active, skipping SMS.");
    }
  }
}
