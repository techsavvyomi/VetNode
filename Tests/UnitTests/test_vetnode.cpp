// ============================================================
// VetNode Unit Tests
// Compile: g++ -std=c++11 -o test_vetnode test_vetnode.cpp && ./test_vetnode
// ============================================================
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

// ============================================================
// MINIMAL MOCKS (replicate Arduino types / funcs for host)
// ============================================================
typedef unsigned long millis_t;
static millis_t _mock_millis = 0;
millis_t millis() { return _mock_millis; }
void setMockMillis(millis_t v) { _mock_millis = v; }
int min(int a, int b) { return a < b ? a : b; }
int max(int a, int b) { return a > b ? a : b; }

// ============================================================
// EXTRACT: TDMA defines (from Node.ino)
// ============================================================
#define HB_INTERVAL_MS 2000
#define TDMA_SLOT_MS   500

int tdmaOffset(int cowIdNum) {
  return (cowIdNum - 1) * TDMA_SLOT_MS;
}

unsigned long firstHeartbeatTime(millis_t bootTime, int cowIdNum) {
  return bootTime - HB_INTERVAL_MS + tdmaOffset(cowIdNum);
}

bool shouldTransmit(millis_t now, millis_t lastHb, int cowIdNum) {
  (void)cowIdNum; // TDMA offset is baked into lastHb at boot
  return (now - lastHb) >= HB_INTERVAL_MS;
}

// ============================================================
// EXTRACT: Multi-Node tracking (from Gateway.ino)
// ============================================================
#define MAX_NODES 8
#define NODE_TIMEOUT_MS 30000

struct NodeData {
  int id;
  float temp;
  int hr;
  int rssi;
  uint16_t pktSeq;
  uint32_t pktsRcvd;
  uint32_t pktsLost;
  unsigned long lastSeen;
  char alertStatus[24];
  char errFlag[8];
  bool hasHr;
};

NodeData nodes[MAX_NODES];
int numActiveNodes = 0;

void resetNodes() {
  memset(nodes, 0, sizeof(nodes));
  numActiveNodes = 0;
}

NodeData *getNode(int cowId) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].id == cowId)
      return &nodes[i];
  }
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].id == 0) {
      memset(&nodes[i], 0, sizeof(NodeData));
      nodes[i].id = cowId;
      nodes[i].rssi = -120;
      strcpy(nodes[i].alertStatus, "NORMAL");
      numActiveNodes++;
      return &nodes[i];
    }
  }
  return nullptr;
}

int countOnlineNodes() {
  int count = 0;
  unsigned long now = millis();
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].id != 0 && (now - nodes[i].lastSeen) < NODE_TIMEOUT_MS)
      count++;
  }
  return count;
}

NodeData *getActiveNode(int index) {
  int count = 0;
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].id != 0) {
      if (count == index)
        return &nodes[i];
      count++;
    }
  }
  return nullptr;
}

// ============================================================
// EXTRACT: Alert logic (from Gateway.ino)
// ============================================================
float cfgTempHigh = 39.5, cfgTempLow = 37.0;
int cfgHrHigh = 90, cfgHrLow = 50;

// Build combined alert status for a DATA packet
void buildAlertStatus(NodeData *nd, float temp, int hr) {
  strcpy(nd->alertStatus, "NORMAL");
  char statusParts[24] = "";

  if (temp >= cfgTempHigh)
    strncat(statusParts, "T_HI ", sizeof(statusParts) - strlen(statusParts) - 1);
  else if (temp <= cfgTempLow)
    strncat(statusParts, "T_LO ", sizeof(statusParts) - strlen(statusParts) - 1);

  if (hr >= cfgHrHigh)
    strncat(statusParts, "HR_HI", sizeof(statusParts) - strlen(statusParts) - 1);
  else if (hr <= cfgHrLow)
    strncat(statusParts, "HR_LO", sizeof(statusParts) - strlen(statusParts) - 1);

  if (strlen(statusParts) > 0) {
    strncpy(nd->alertStatus, statusParts, sizeof(nd->alertStatus) - 1);
    nd->alertStatus[sizeof(nd->alertStatus) - 1] = '\0';
  }
}

// ============================================================
// EXTRACT: Packet parsing (from Gateway.ino)
// ============================================================
struct ParsedHB {
  bool valid;
  uint16_t seq;
  int cowId;
  float temp;
  char errFlag[16];
};

ParsedHB parseHBPacket(const char *packet) {
  ParsedHB result = {};
  if (strncmp(packet, "HB,", 3) != 0) return result;

  if (sscanf(packet + 3, "%hu,%d,%f,%15[^,\r\n]",
             &result.seq, &result.cowId, &result.temp, result.errFlag) >= 3) {
    result.valid = true;
  }
  return result;
}

struct ParsedDATA {
  bool valid;
  uint16_t seq;
  int cowId;
  float temp;
  int hr;
};

ParsedDATA parseDATAPacket(const char *packet) {
  ParsedDATA result = {};
  if (strncmp(packet, "DATA,", 5) != 0) return result;

  if (sscanf(packet + 5, "%hu,%d,%f,%d",
             &result.seq, &result.cowId, &result.temp, &result.hr) >= 4) {
    result.valid = true;
  }
  return result;
}

// ============================================================
// EXTRACT: Config bounds (from Gateway.ino edit handlers)
// ============================================================
float wrapTempHigh(float val, bool increase) {
  if (increase) { val += 0.5; if (val > 45.0) val = 35.0; }
  else          { val -= 0.5; if (val < 35.0) val = 45.0; }
  return val;
}

float wrapTempLow(float val, bool increase) {
  if (increase) { val += 0.5; if (val > 40.0) val = 32.0; }
  else          { val -= 0.5; if (val < 32.0) val = 40.0; }
  return val;
}

int wrapHrHigh(int val, bool increase) {
  if (increase) { val += 5; if (val > 150) val = 60; }
  else          { val -= 5; if (val < 60) val = 150; }
  return val;
}

int wrapHrLow(int val, bool increase) {
  if (increase) { val += 5; if (val > 80) val = 30; }
  else          { val -= 5; if (val < 30) val = 80; }
  return val;
}

int cycleSmsInterval(int val, bool increase) {
  if (increase) {
    if (val == 1) return 5;
    if (val == 5) return 10;
    if (val == 10) return 30;
    if (val == 30) return 60;
    return 1;
  } else {
    if (val == 60) return 30;
    if (val == 30) return 10;
    if (val == 10) return 5;
    if (val == 5) return 1;
    return 60;
  }
}

int cycleOledTimeout(int val, bool increase) {
  if (increase) {
    if (val == 0) return 15;
    if (val == 15) return 30;
    if (val == 30) return 60;
    if (val == 60) return 300;
    return 0;
  } else {
    if (val == 300) return 60;
    if (val == 60) return 30;
    if (val == 30) return 15;
    if (val == 15) return 0;
    return 300;
  }
}

// ============================================================
// EXTRACT: GSM State machine transitions
// ============================================================
enum GsmState {
  GSM_BOOT,
  GSM_IDLE,
  GSM_SET_TEXT_MODE,
  GSM_TX_SMS_START,
  GSM_WAIT_RESP,
  GSM_TX_BODY,
  GSM_WAIT_SMS_OK
};

struct GsmContext {
  GsmState state;
  GsmState prevState;
  bool smsPending;
  bool gsmOk;
};

// Simulate what happens when we get "OK" in WAIT_RESP
GsmState gsmHandleOk(GsmContext *ctx) {
  ctx->gsmOk = true;
  if (ctx->prevState == GSM_BOOT) {
    return GSM_IDLE;
  } else if (ctx->prevState == GSM_SET_TEXT_MODE && ctx->smsPending) {
    return GSM_TX_SMS_START;
  } else {
    return GSM_IDLE;
  }
}

// Simulate what happens on ERROR in WAIT_RESP
GsmState gsmHandleError(GsmContext *ctx) {
  ctx->smsPending = false;
  return GSM_IDLE;
}

// ============================================================
// EXTRACT: Signal level mapping
// ============================================================
int gsmCsqToLevel(int csq) {
  if (csq == 99 || csq <= 2) return 0;
  if (csq > 19) return 4;
  if (csq > 14) return 3;
  if (csq > 9) return 2;
  return 1;
}

int rssiToLevel(int rssi) {
  if (rssi > -70) return 4;
  if (rssi > -90) return 3;
  if (rssi > -110) return 2;
  if (rssi > -125) return 1;
  return 0;
}

// ============================================================
// EXTRACT: Display layout constants verification
// ============================================================
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define CHAR_W 6
#define CHAR_H 8

bool textFits(int startX, int numChars, int textSize) {
  return (startX + numChars * CHAR_W * textSize) <= SSD1306_WIDTH;
}

bool rectFits(int x, int y, int w, int h) {
  return (x + w) <= SSD1306_WIDTH && (y + h) <= SSD1306_HEIGHT && x >= 0 && y >= 0;
}

// ============================================================
// TEST RUNNER
// ============================================================
int passed = 0, failed = 0;

#define TEST(name) void name()
#define RUN(name) do { \
  printf("  %-50s ", #name); \
  try { name(); printf("[PASS]\n"); passed++; } \
  catch (...) { printf("[FAIL]\n"); failed++; } \
} while(0)

#define ASSERT(cond) do { if (!(cond)) { \
  printf("\n    ASSERT FAILED: %s (line %d)\n", #cond, __LINE__); \
  throw 1; } } while(0)

#define ASSERT_EQ(a, b) do { if ((a) != (b)) { \
  printf("\n    ASSERT_EQ FAILED: %s != %s (line %d)\n", #a, #b, __LINE__); \
  throw 1; } } while(0)

#define ASSERT_NEAR(a, b, eps) do { if (fabs((double)(a) - (double)(b)) > eps) { \
  printf("\n    ASSERT_NEAR FAILED: %s=%.2f != %s=%.2f (line %d)\n", \
  #a, (double)(a), #b, (double)(b), __LINE__); throw 1; } } while(0)

#define ASSERT_STR_EQ(a, b) do { if (strcmp((a), (b)) != 0) { \
  printf("\n    ASSERT_STR_EQ FAILED: \"%s\" != \"%s\" (line %d)\n", \
  (a), (b), __LINE__); throw 1; } } while(0)

// ============================================================
// TDMA TESTS
// ============================================================
TEST(test_tdma_offset_node1) {
  ASSERT_EQ(tdmaOffset(1), 0);
}

TEST(test_tdma_offset_node2) {
  ASSERT_EQ(tdmaOffset(2), 500);
}

TEST(test_tdma_offset_node3) {
  ASSERT_EQ(tdmaOffset(3), 1000);
}

TEST(test_tdma_offset_node4) {
  ASSERT_EQ(tdmaOffset(4), 1500);
}

TEST(test_tdma_no_collision_2_nodes) {
  // Node 1 at 0ms, Node 2 at 500ms — 500ms gap
  int gap = tdmaOffset(2) - tdmaOffset(1);
  ASSERT(gap >= 500);
}

TEST(test_tdma_4_nodes_fit_in_window) {
  // All 4 nodes must fit within the 2000ms HB window
  ASSERT(tdmaOffset(4) < HB_INTERVAL_MS);
}

TEST(test_tdma_first_heartbeat_node1) {
  millis_t boot = 5000;
  unsigned long lastHb = firstHeartbeatTime(boot, 1);
  // Node 1: boot - 2000 + 0 = 3000
  ASSERT_EQ(lastHb, 3000UL);
}

TEST(test_tdma_first_heartbeat_node2) {
  millis_t boot = 5000;
  unsigned long lastHb = firstHeartbeatTime(boot, 2);
  // Node 2: boot - 2000 + 500 = 3500
  ASSERT_EQ(lastHb, 3500UL);
}

TEST(test_tdma_transmit_timing) {
  // Node 1 boot at 5000, lastHb = 3000
  // At time 5000: 5000 - 3000 = 2000 >= 2000 → should TX
  setMockMillis(5000);
  ASSERT(shouldTransmit(5000, 3000, 1));
  // At time 4999: 4999 - 3000 = 1999 < 2000 → should not TX
  ASSERT(!shouldTransmit(4999, 3000, 1));
}

// ============================================================
// MULTI-NODE TRACKING TESTS
// ============================================================
TEST(test_getNode_creates_new) {
  resetNodes();
  NodeData *nd = getNode(1);
  ASSERT(nd != nullptr);
  ASSERT_EQ(nd->id, 1);
  ASSERT_EQ(numActiveNodes, 1);
  ASSERT_STR_EQ(nd->alertStatus, "NORMAL");
  ASSERT_EQ(nd->rssi, -120);
}

TEST(test_getNode_finds_existing) {
  resetNodes();
  NodeData *nd1 = getNode(1);
  nd1->temp = 38.5;
  NodeData *nd1b = getNode(1);
  ASSERT(nd1 == nd1b);
  ASSERT_NEAR(nd1b->temp, 38.5, 0.01);
  ASSERT_EQ(numActiveNodes, 1); // No duplicate
}

TEST(test_getNode_multiple_nodes) {
  resetNodes();
  NodeData *n1 = getNode(1);
  NodeData *n2 = getNode(2);
  NodeData *n3 = getNode(3);
  ASSERT(n1 != n2);
  ASSERT(n2 != n3);
  ASSERT_EQ(numActiveNodes, 3);
  ASSERT_EQ(n1->id, 1);
  ASSERT_EQ(n2->id, 2);
  ASSERT_EQ(n3->id, 3);
}

TEST(test_getNode_full_returns_null) {
  resetNodes();
  for (int i = 1; i <= MAX_NODES; i++)
    getNode(i);
  NodeData *overflow = getNode(99);
  ASSERT(overflow == nullptr);
  ASSERT_EQ(numActiveNodes, MAX_NODES);
}

TEST(test_countOnline_all_fresh) {
  resetNodes();
  setMockMillis(10000);
  NodeData *n1 = getNode(1);
  n1->lastSeen = 9000;
  NodeData *n2 = getNode(2);
  n2->lastSeen = 9500;
  ASSERT_EQ(countOnlineNodes(), 2);
}

TEST(test_countOnline_one_stale) {
  resetNodes();
  setMockMillis(60000);
  NodeData *n1 = getNode(1);
  n1->lastSeen = 59000; // 1s ago — online
  NodeData *n2 = getNode(2);
  n2->lastSeen = 10000; // 50s ago — offline (>30s)
  ASSERT_EQ(countOnlineNodes(), 1);
}

TEST(test_countOnline_all_stale) {
  resetNodes();
  setMockMillis(100000);
  NodeData *n1 = getNode(1);
  n1->lastSeen = 1000;
  ASSERT_EQ(countOnlineNodes(), 0);
}

TEST(test_getActiveNode_index) {
  resetNodes();
  getNode(5);
  getNode(10);
  NodeData *first = getActiveNode(0);
  NodeData *second = getActiveNode(1);
  ASSERT(first != nullptr);
  ASSERT(second != nullptr);
  ASSERT_EQ(first->id, 5);
  ASSERT_EQ(second->id, 10);
}

TEST(test_getActiveNode_out_of_range) {
  resetNodes();
  getNode(1);
  NodeData *nd = getActiveNode(5);
  ASSERT(nd == nullptr);
}

// ============================================================
// PACKET PARSING TESTS
// ============================================================
TEST(test_parse_hb_valid) {
  ParsedHB hb = parseHBPacket("HB,42,1,38.5,0");
  ASSERT(hb.valid);
  ASSERT_EQ(hb.seq, 42);
  ASSERT_EQ(hb.cowId, 1);
  ASSERT_NEAR(hb.temp, 38.5, 0.1);
  ASSERT_STR_EQ(hb.errFlag, "0");
}

TEST(test_parse_hb_with_error) {
  ParsedHB hb = parseHBPacket("HB,100,2,40.1,HR");
  ASSERT(hb.valid);
  ASSERT_EQ(hb.cowId, 2);
  ASSERT_STR_EQ(hb.errFlag, "HR");
}

TEST(test_parse_hb_all_error) {
  ParsedHB hb = parseHBPacket("HB,5,1,37.0,ALL");
  ASSERT(hb.valid);
  ASSERT_STR_EQ(hb.errFlag, "ALL");
}

TEST(test_parse_hb_no_error_field) {
  // sscanf should still parse 3 fields minimum
  ParsedHB hb = parseHBPacket("HB,10,3,39.0");
  ASSERT(hb.valid);
  ASSERT_EQ(hb.cowId, 3);
  ASSERT_NEAR(hb.temp, 39.0, 0.1);
}

TEST(test_parse_hb_invalid_prefix) {
  ParsedHB hb = parseHBPacket("DATA,1,1,38.0,0");
  ASSERT(!hb.valid);
}

TEST(test_parse_hb_garbage) {
  ParsedHB hb = parseHBPacket("HELLO WORLD");
  ASSERT(!hb.valid);
}

TEST(test_parse_data_valid) {
  ParsedDATA d = parseDATAPacket("DATA,55,2,38.7,72");
  ASSERT(d.valid);
  ASSERT_EQ(d.seq, 55);
  ASSERT_EQ(d.cowId, 2);
  ASSERT_NEAR(d.temp, 38.7, 0.1);
  ASSERT_EQ(d.hr, 72);
}

TEST(test_parse_data_high_hr) {
  ParsedDATA d = parseDATAPacket("DATA,1,1,39.5,120");
  ASSERT(d.valid);
  ASSERT_EQ(d.hr, 120);
}

TEST(test_parse_data_invalid_prefix) {
  ParsedDATA d = parseDATAPacket("HB,1,1,38.0,0");
  ASSERT(!d.valid);
}

TEST(test_parse_data_missing_fields) {
  ParsedDATA d = parseDATAPacket("DATA,1,2,38.0");
  ASSERT(!d.valid); // needs 4 fields
}

// ============================================================
// ALERT LOGIC TESTS
// ============================================================
TEST(test_alert_normal_range) {
  NodeData nd = {};
  buildAlertStatus(&nd, 38.0, 70);
  ASSERT_STR_EQ(nd.alertStatus, "NORMAL");
}

TEST(test_alert_temp_high) {
  NodeData nd = {};
  buildAlertStatus(&nd, 39.5, 70);
  ASSERT(strstr(nd.alertStatus, "T_HI") != nullptr);
}

TEST(test_alert_temp_low) {
  NodeData nd = {};
  buildAlertStatus(&nd, 37.0, 70);
  ASSERT(strstr(nd.alertStatus, "T_LO") != nullptr);
}

TEST(test_alert_hr_high) {
  NodeData nd = {};
  buildAlertStatus(&nd, 38.0, 90);
  ASSERT(strstr(nd.alertStatus, "HR_HI") != nullptr);
}

TEST(test_alert_hr_low) {
  NodeData nd = {};
  buildAlertStatus(&nd, 38.0, 50);
  ASSERT(strstr(nd.alertStatus, "HR_LO") != nullptr);
}

TEST(test_alert_combined_temp_high_hr_high) {
  NodeData nd = {};
  buildAlertStatus(&nd, 40.0, 95);
  ASSERT(strstr(nd.alertStatus, "T_HI") != nullptr);
  ASSERT(strstr(nd.alertStatus, "HR_HI") != nullptr);
}

TEST(test_alert_combined_temp_low_hr_low) {
  NodeData nd = {};
  buildAlertStatus(&nd, 36.0, 45);
  ASSERT(strstr(nd.alertStatus, "T_LO") != nullptr);
  ASSERT(strstr(nd.alertStatus, "HR_LO") != nullptr);
}

TEST(test_alert_temp_high_hr_low) {
  NodeData nd = {};
  buildAlertStatus(&nd, 41.0, 30);
  ASSERT(strstr(nd.alertStatus, "T_HI") != nullptr);
  ASSERT(strstr(nd.alertStatus, "HR_LO") != nullptr);
}

TEST(test_alert_boundary_just_below_high) {
  // 39.4 < 39.5 and 89 < 90 → normal
  NodeData nd = {};
  buildAlertStatus(&nd, 39.4, 89);
  ASSERT_STR_EQ(nd.alertStatus, "NORMAL");
}

TEST(test_alert_boundary_exact_low) {
  // 37.0 <= 37.0 and 50 <= 50 → both trigger
  NodeData nd = {};
  buildAlertStatus(&nd, 37.0, 50);
  ASSERT(strstr(nd.alertStatus, "T_LO") != nullptr);
  ASSERT(strstr(nd.alertStatus, "HR_LO") != nullptr);
}

// ============================================================
// CONFIG BOUNDS TESTS
// ============================================================
TEST(test_config_temp_high_increase_wraps) {
  float v = 45.0;
  v = wrapTempHigh(v, true);
  ASSERT_NEAR(v, 35.0, 0.01);
}

TEST(test_config_temp_high_decrease_wraps) {
  float v = 35.0;
  v = wrapTempHigh(v, false);
  ASSERT_NEAR(v, 45.0, 0.01);
}

TEST(test_config_temp_low_increase_wraps) {
  float v = 40.0;
  v = wrapTempLow(v, true);
  ASSERT_NEAR(v, 32.0, 0.01);
}

TEST(test_config_temp_low_decrease_wraps) {
  float v = 32.0;
  v = wrapTempLow(v, false);
  ASSERT_NEAR(v, 40.0, 0.01);
}

TEST(test_config_hr_high_increase_wraps) {
  ASSERT_EQ(wrapHrHigh(150, true), 60);
}

TEST(test_config_hr_high_decrease_wraps) {
  ASSERT_EQ(wrapHrHigh(60, false), 150);
}

TEST(test_config_hr_low_increase_wraps) {
  ASSERT_EQ(wrapHrLow(80, true), 30);
}

TEST(test_config_hr_low_decrease_wraps) {
  ASSERT_EQ(wrapHrLow(30, false), 80);
}

TEST(test_config_hr_high_normal_step) {
  ASSERT_EQ(wrapHrHigh(90, true), 95);
  ASSERT_EQ(wrapHrHigh(90, false), 85);
}

TEST(test_config_sms_interval_cycle_forward) {
  ASSERT_EQ(cycleSmsInterval(1, true), 5);
  ASSERT_EQ(cycleSmsInterval(5, true), 10);
  ASSERT_EQ(cycleSmsInterval(10, true), 30);
  ASSERT_EQ(cycleSmsInterval(30, true), 60);
  ASSERT_EQ(cycleSmsInterval(60, true), 1);
}

TEST(test_config_sms_interval_cycle_backward) {
  ASSERT_EQ(cycleSmsInterval(60, false), 30);
  ASSERT_EQ(cycleSmsInterval(30, false), 10);
  ASSERT_EQ(cycleSmsInterval(10, false), 5);
  ASSERT_EQ(cycleSmsInterval(5, false), 1);
  ASSERT_EQ(cycleSmsInterval(1, false), 60);
}

TEST(test_config_oled_timeout_cycle_forward) {
  ASSERT_EQ(cycleOledTimeout(0, true), 15);
  ASSERT_EQ(cycleOledTimeout(15, true), 30);
  ASSERT_EQ(cycleOledTimeout(30, true), 60);
  ASSERT_EQ(cycleOledTimeout(60, true), 300);
  ASSERT_EQ(cycleOledTimeout(300, true), 0);
}

TEST(test_config_oled_timeout_cycle_backward) {
  ASSERT_EQ(cycleOledTimeout(300, false), 60);
  ASSERT_EQ(cycleOledTimeout(0, false), 300);
}

// ============================================================
// GSM STATE MACHINE TESTS
// ============================================================
TEST(test_gsm_boot_ok_goes_idle) {
  GsmContext ctx = {GSM_WAIT_RESP, GSM_BOOT, false, false};
  GsmState next = gsmHandleOk(&ctx);
  ASSERT_EQ(next, (int)GSM_IDLE);
  ASSERT(ctx.gsmOk);
}

TEST(test_gsm_boot_ok_with_pending_sms_still_goes_idle) {
  // Key fix: boot OK must NOT skip to TX_SMS_START
  GsmContext ctx = {GSM_WAIT_RESP, GSM_BOOT, true, false};
  GsmState next = gsmHandleOk(&ctx);
  ASSERT_EQ(next, (int)GSM_IDLE);
}

TEST(test_gsm_text_mode_ok_with_pending_goes_to_tx) {
  GsmContext ctx = {GSM_WAIT_RESP, GSM_SET_TEXT_MODE, true, false};
  GsmState next = gsmHandleOk(&ctx);
  ASSERT_EQ(next, (int)GSM_TX_SMS_START);
}

TEST(test_gsm_text_mode_ok_without_pending_goes_idle) {
  GsmContext ctx = {GSM_WAIT_RESP, GSM_SET_TEXT_MODE, false, false};
  GsmState next = gsmHandleOk(&ctx);
  ASSERT_EQ(next, (int)GSM_IDLE);
}

TEST(test_gsm_error_clears_pending) {
  GsmContext ctx = {GSM_WAIT_RESP, GSM_SET_TEXT_MODE, true, false};
  GsmState next = gsmHandleError(&ctx);
  ASSERT_EQ(next, (int)GSM_IDLE);
  ASSERT(!ctx.smsPending);
}

TEST(test_gsm_error_from_boot) {
  GsmContext ctx = {GSM_WAIT_RESP, GSM_BOOT, false, false};
  GsmState next = gsmHandleError(&ctx);
  ASSERT_EQ(next, (int)GSM_IDLE);
}

// ============================================================
// SIGNAL LEVEL TESTS
// ============================================================
TEST(test_gsm_csq_levels) {
  ASSERT_EQ(gsmCsqToLevel(0), 0);
  ASSERT_EQ(gsmCsqToLevel(2), 0);
  ASSERT_EQ(gsmCsqToLevel(5), 1);
  ASSERT_EQ(gsmCsqToLevel(12), 2);
  ASSERT_EQ(gsmCsqToLevel(17), 3);
  ASSERT_EQ(gsmCsqToLevel(25), 4);
  ASSERT_EQ(gsmCsqToLevel(99), 0); // Unknown
}

TEST(test_rssi_levels) {
  ASSERT_EQ(rssiToLevel(-130), 0);
  ASSERT_EQ(rssiToLevel(-120), 1);
  ASSERT_EQ(rssiToLevel(-100), 2);
  ASSERT_EQ(rssiToLevel(-80), 3);
  ASSERT_EQ(rssiToLevel(-60), 4);
}

// ============================================================
// DISPLAY LAYOUT TESTS
// ============================================================
TEST(test_layout_top_bar_text_fits) {
  // "VETNODE" = 7 chars at x=0
  ASSERT(textFits(0, 7, 1));
  // Node count "8/8" = 3 chars at x=44
  ASSERT(textFits(44, 3, 1));
  // "NoGSM" = 5 chars at x=80
  ASSERT(textFits(80, 5, 1));
}

TEST(test_layout_buzzer_icon_fits) {
  // fillRect(123, 1, 4, 4)
  ASSERT(rectFits(123, 1, 4, 4));
}

TEST(test_layout_cow_id_size2_fits) {
  // "COW " at size 1 = 24px, then 2-digit ID at size 2 = 24px → 48px
  ASSERT(textFits(0, 4, 1));  // "COW "
  ASSERT(textFits(24, 2, 2)); // ID at size 2
}

TEST(test_layout_online_badge_fits) {
  // fillRect(48, 11, 38, 10) → 48+38=86
  ASSERT(rectFits(48, 11, 38, 10));
  // OFFLINE drawRect(48, 11, 44, 10) → 48+44=92
  ASSERT(rectFits(48, 11, 44, 10));
}

TEST(test_layout_status_banner_fits) {
  // fillRect(0, 49, 128, 15) → Y: 49+15=64 = exactly height
  ASSERT(rectFits(0, 49, 128, 15));
}

TEST(test_layout_sms_toast_fits) {
  // fillRect(20, 20, 88, 20) → X: 20+88=108, Y: 20+20=40
  ASSERT(rectFits(20, 20, 88, 20));
}

TEST(test_layout_menu_hint_fits) {
  // "M:Scroll S:Select" = 18 chars at x=1
  ASSERT(textFits(1, 18, 1));
}

TEST(test_layout_edit_hint_fits) {
  // "M:Val S:Nxt LS:Save" = 19 chars at x=1
  ASSERT(textFits(1, 19, 1));
}

TEST(test_layout_hr_display_fits) {
  // "HR:120" = 6 chars at x=76 in size 1 → 76 + 36 = 112
  ASSERT(textFits(76, 6, 1));
}

TEST(test_layout_saved_popup_fits) {
  // drawRect(10, 20, 108, 24) → 10+108=118, 20+24=44
  ASSERT(rectFits(10, 20, 108, 24));
}

// ============================================================
// PACKET LOSS TRACKING TESTS
// ============================================================
TEST(test_packet_loss_sequential) {
  resetNodes();
  NodeData *nd = getNode(1);
  // Receive seq 1, 2, 3 — no loss
  nd->pktSeq = 0; nd->pktsRcvd = 0; nd->pktsLost = 0;

  // Seq 1
  nd->pktsRcvd++; nd->pktSeq = 1;
  // Seq 2
  nd->pktsRcvd++; nd->pktSeq = 2;
  // Seq 3
  nd->pktsRcvd++; nd->pktSeq = 3;

  ASSERT_EQ(nd->pktsRcvd, (uint32_t)3);
  ASSERT_EQ(nd->pktsLost, (uint32_t)0);
}

TEST(test_packet_loss_gap) {
  resetNodes();
  NodeData *nd = getNode(1);
  nd->pktSeq = 0; nd->pktsRcvd = 0; nd->pktsLost = 0;

  // Seq 1
  nd->pktsRcvd++; nd->pktSeq = 1;
  // Seq 5 (missed 2, 3, 4)
  uint16_t incoming = 5;
  nd->pktsRcvd++;
  if (nd->pktSeq > 0 && incoming > nd->pktSeq + 1)
    nd->pktsLost += (incoming - nd->pktSeq - 1);
  nd->pktSeq = incoming;

  ASSERT_EQ(nd->pktsRcvd, (uint32_t)2);
  ASSERT_EQ(nd->pktsLost, (uint32_t)3);
}

// ============================================================
// MAIN
// ============================================================
int main() {
  printf("\n=== VetNode Unit Tests ===\n\n");

  printf("[TDMA Timing]\n");
  RUN(test_tdma_offset_node1);
  RUN(test_tdma_offset_node2);
  RUN(test_tdma_offset_node3);
  RUN(test_tdma_offset_node4);
  RUN(test_tdma_no_collision_2_nodes);
  RUN(test_tdma_4_nodes_fit_in_window);
  RUN(test_tdma_first_heartbeat_node1);
  RUN(test_tdma_first_heartbeat_node2);
  RUN(test_tdma_transmit_timing);

  printf("\n[Multi-Node Tracking]\n");
  RUN(test_getNode_creates_new);
  RUN(test_getNode_finds_existing);
  RUN(test_getNode_multiple_nodes);
  RUN(test_getNode_full_returns_null);
  RUN(test_countOnline_all_fresh);
  RUN(test_countOnline_one_stale);
  RUN(test_countOnline_all_stale);
  RUN(test_getActiveNode_index);
  RUN(test_getActiveNode_out_of_range);

  printf("\n[Packet Parsing]\n");
  RUN(test_parse_hb_valid);
  RUN(test_parse_hb_with_error);
  RUN(test_parse_hb_all_error);
  RUN(test_parse_hb_no_error_field);
  RUN(test_parse_hb_invalid_prefix);
  RUN(test_parse_hb_garbage);
  RUN(test_parse_data_valid);
  RUN(test_parse_data_high_hr);
  RUN(test_parse_data_invalid_prefix);
  RUN(test_parse_data_missing_fields);

  printf("\n[Alert Logic]\n");
  RUN(test_alert_normal_range);
  RUN(test_alert_temp_high);
  RUN(test_alert_temp_low);
  RUN(test_alert_hr_high);
  RUN(test_alert_hr_low);
  RUN(test_alert_combined_temp_high_hr_high);
  RUN(test_alert_combined_temp_low_hr_low);
  RUN(test_alert_temp_high_hr_low);
  RUN(test_alert_boundary_just_below_high);
  RUN(test_alert_boundary_exact_low);

  printf("\n[Config Bounds]\n");
  RUN(test_config_temp_high_increase_wraps);
  RUN(test_config_temp_high_decrease_wraps);
  RUN(test_config_temp_low_increase_wraps);
  RUN(test_config_temp_low_decrease_wraps);
  RUN(test_config_hr_high_increase_wraps);
  RUN(test_config_hr_high_decrease_wraps);
  RUN(test_config_hr_low_increase_wraps);
  RUN(test_config_hr_low_decrease_wraps);
  RUN(test_config_hr_high_normal_step);
  RUN(test_config_sms_interval_cycle_forward);
  RUN(test_config_sms_interval_cycle_backward);
  RUN(test_config_oled_timeout_cycle_forward);
  RUN(test_config_oled_timeout_cycle_backward);

  printf("\n[GSM State Machine]\n");
  RUN(test_gsm_boot_ok_goes_idle);
  RUN(test_gsm_boot_ok_with_pending_sms_still_goes_idle);
  RUN(test_gsm_text_mode_ok_with_pending_goes_to_tx);
  RUN(test_gsm_text_mode_ok_without_pending_goes_idle);
  RUN(test_gsm_error_clears_pending);
  RUN(test_gsm_error_from_boot);

  printf("\n[Signal Levels]\n");
  RUN(test_gsm_csq_levels);
  RUN(test_rssi_levels);

  printf("\n[Display Layout]\n");
  RUN(test_layout_top_bar_text_fits);
  RUN(test_layout_buzzer_icon_fits);
  RUN(test_layout_cow_id_size2_fits);
  RUN(test_layout_online_badge_fits);
  RUN(test_layout_status_banner_fits);
  RUN(test_layout_sms_toast_fits);
  RUN(test_layout_menu_hint_fits);
  RUN(test_layout_edit_hint_fits);
  RUN(test_layout_hr_display_fits);
  RUN(test_layout_saved_popup_fits);

  printf("\n[Packet Loss]\n");
  RUN(test_packet_loss_sequential);
  RUN(test_packet_loss_gap);

  printf("\n=============================\n");
  printf("TOTAL: %d passed, %d failed\n", passed, failed);
  printf("=============================\n\n");

  return failed > 0 ? 1 : 0;
}
