/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  VIPx3X4s — M12 A-CODED SENSOR DIAGNOSTIC MONITRING TOOL                 ║
 * ║  Platform: CrowPanel 7.0"  ESP32-S3                                      ║
 * ║  Theme: Cyber-Industrial / Hyper-Tech / ROG Aesthetic                    ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

/* ------------------------------------------------------------------
 * HARDWARE PIN DEFINITIONS & BAUD
 * ------------------------------------------------------------------ */
#define SENSOR_UART_RX  44
#define SENSOR_UART_TX  43
#define UART_BAUD       230400 // Matches Circuit Board
#define PCA9557_ADDR    0x18
#define PCA9557_OUTPUT  0x01
#define PCA9557_CONFIG  0x03
#define I2C_SDA         19
#define I2C_SCL         20
#define SD_SCK          12
#define SD_MOSI         11
#define SD_MISO         13
#define SD_CS           10

/* ------------------------------------------------------------------
 * SCREEN DIMENSIONS & LAYOUT ZONES
 * ------------------------------------------------------------------ */
#define SCR_W           800
#define SCR_H           480
#define HEADER_H        44
#define TAB_BAR_H       36
#define FOOTER_H        32
#define BODY_Y          (HEADER_H + TAB_BAR_H)
#define BODY_H          (SCR_H - HEADER_H - TAB_BAR_H - FOOTER_H)

/* ------------------------------------------------------------------
 * VIPx3X4s COLOR PALETTE — HYPER-TECH / ROG CYBERPUNK AESTHETIC
 * ------------------------------------------------------------------ */
// Base Layers
#define C_VOID          0x0000  // Pure Black Abyss
#define C_DARK_METAL    0x0821  // Extremely dark blue/gray for panels
#define C_STEEL         0x2124  // Gunmetal gray for borders
#define C_WHITE         0xFFFF  // Sharp white for primary text
#define C_MUTED_TEXT    0x5AEB  // Darkened cyan for inactive elements

// Aggressive Accents (High Dopamine)
#define C_CYAN          0x07FF  // Electric Cyan (Primary active state)
#define C_CRIMSON       0xE000  // Deep aggressive red
#define C_ROG_RED       0xF800  // Piercing Neon Red (Alarms, Fall Time)
#define C_VOLT_GREEN    0x07E0  // Acid/Volt Green (Good Health, Rise Time)
#define C_HAZARD        0xFD20  // Sharp Warning Orange
#define C_PURPLE        0xA01F  // Deep Tech Purple (Feedback/Power)
#define C_GLOW_CYAN     0x0410  // Soft background underglow

// UI Specific
#define C_PANEL_BG      0x0842
#define C_PANEL_BORDER  0x18C3
#define C_CYBER_GOLD    0xFEA0  // Amber/Gold for cycle counters and operator ID
#define C_GHOST_CYAN    0x0314  // Dim cyan for secondary voltage readouts
#define C_YELLOW        0xFFE0  // Pure Yellow (LEAK / VFloor state)

/* ------------------------------------------------------------------
 * TAB ROUTING SYSTEM
 * ------------------------------------------------------------------ */
#define TAB_HOME        0  
#define TAB_CALIBRATE   1  
#define TAB_LIVE        2  
#define TAB_DIAGNOSE    3  
#define TAB_LOGS        4  
#define TAB_CONFIG      5  
#define TAB_ARCHIVE     6  
#define NUM_TABS        7

const char* TAB_LABELS[NUM_TABS] = { 
  "HOME", "CONFIG", "GRAPH", "COMPARE", "LOG/ALERT", "SETTING", "STORAGE" 
};

const char* SENSOR_TYPE_NAMES[] = { "PNP-NO", "PNP-NC", "NPN-NO", "NPN-NC", "2-WIRE" };

/* ------------------------------------------------------------------
 * SYSTEM CONSTANTS & ALGORITHM WEIGHTS
 * ------------------------------------------------------------------ */
#define GRAPH_POINTS      1000   
#define GRAPH_V_MAX       26.0
#define GRAPH_WINDOW_MS   15000UL 

#define FAULT_NONE        0
#define FAULT_LEAK        1
#define FAULT_SPIKE       2
#define FAULT_RT_FAIL     3
#define FAULT_FT_FAIL     4
#define FAULT_ST_FAIL     5
#define FAULT_FREQ_FAIL   6
#define FAULT_VPEAK_FAIL  7
#define FAULT_VFLOOR_FAIL 8
#define FAULT_SWING_FAIL  9
#define FAULT_KIN_FAIL    10
#define FAULT_RATIO_FAIL  11
#define FAULT_SRT_FAIL    12
#define FAULT_SFT_FAIL    13
#define FAULT_SPIKE_CNT   14

static const float HEALTH_WEIGHTS[12] = { 
  0.12f, 0.12f, 0.10f, 0.08f, 0.10f, 0.10f, 
  0.08f, 0.06f, 0.06f, 0.06f, 0.06f, 0.06f 
};

/* ------------------------------------------------------------------
 * CORE DATA STRUCTURES (Updated for New Advanced Diagnostics)
 * ------------------------------------------------------------------ */
struct ChannelData { float V, I, P; };
struct FactorAlarm { float minVal, maxVal; uint8_t cntLimit, violCnt; };

#define SIGMA_BUF 32

struct SensorInfo {
  bool        active;
  bool        feedbackEnabled; 
  uint8_t     sType;           
  ChannelData ch[3]; // [0]=IN, [1]=OUT, [2]=FB
  int         stateID;
  float       RT, ST, FT;
  uint32_t    cycles;
  int         faultCode;

  // Advanced Diagnostic Metrics from Circuit
  float       inrush;           // Peak current spike on turn-on (mA)
  float       leakage;          // Max current draw while OFF (mA)
  float       ripple;           // Peak-to-peak voltage noise during STABLE (V)
  uint32_t    bounceCount;      // Contact stutters detected during RISING

  // NEW: Calculated Client-Side Parameters (from existing $S data)
  float       dutyCycle;        // ST / CT * 100% — output duty cycle
  float       satDrop_V;        // VIN - Vpeak — saturation voltage drop
  float       avgActiveCurrent; // EMA of output current during STABLE (mA)
  float       avgLeakage;       // EMA of idle leakage current (mA)

  // NEW: Statistical Drift Trackers (CUSUM / EWMA — Category 14)
  float       cusumRT;          // CUSUM drift accumulator for Rise Time
  float       cusumFT;          // CUSUM drift accumulator for Fall Time
  float       ewmaRT;           // EWMA smoothed Rise Time
  float       ewmaFT;           // EWMA smoothed Fall Time
  float       cusumTarget;      // Calibrated mean RT for CUSUM reference

  // NEW: Operational Event Counters (Category 16)
  uint32_t    ovVoltCount;      // Over-voltage events (VIN > 26V)
  uint32_t    uvVoltCount;      // Under-voltage events (VIN < 18V)
  float       powerOnHours;     // Accumulated session power-on hours
  unsigned long firstPacketMs; // ms timestamp of first $S packet (for powerOnHours)

  // NEW: Predicted Remaining Useful Life in hours
  float       rulHours;         // RUL converted from cycle estimate

  float       healthScore;  
  float       factorScores[12];
  int         goodCycles;
  int         totalFaults;
  bool        cycleNG;

  float       freq;
  float       sigmaRTbuf[SIGMA_BUF], sigmaFTbuf[SIGMA_BUF];
  uint8_t     sigmaIdx;
  float       sigmaRT, sigmaFT;
  float       rtMedBuf[9], ftMedBuf[9];
  uint8_t     medIdx;
  float       medRT, medFT;

  uint32_t    spikeCount;
  float       vFloor, vPeak;
  bool        calibReady;
  uint32_t    prevCycles;

  bool        alarmActive;
  char        alarmCause[64];
  char        alarmTime[24];
  FactorAlarm alarms[12];

  float         graphBuf[GRAPH_POINTS];
  unsigned long graphTimeBuf[GRAPH_POINTS];
  uint16_t      graphColorBuf[GRAPH_POINTS]; 
  int           graphIdx;
  bool          graphFull;
};
SensorInfo sData[4];

// Circuit Heartbeat Status
int circuitReadRate = 0;
bool circuitOnline = false;
unsigned long lastHeartbeat = 0;
bool bootCircuitDetected = false;  // Set during boot cinematic UART check

/* ------------------------------------------------------------------
 * FLIGHT RECORDER & ALARM LEDGER STRUCTS
 * ------------------------------------------------------------------ */
#define MAX_HISTORY 200
struct CycleRecord {
  uint8_t sensor; uint32_t cycleNum; unsigned long ts;
  float RT, ST, FT, freq, vFloor, vPeak, vHigh;
  bool ng; int faultCode;
};
CycleRecord cycleHistory[MAX_HISTORY];
int historyCount = 0, historyHead = 0;

#define MAX_ALARM_HISTORY 20
struct AlarmRecord { uint8_t sensor; char cause[48]; char time[24]; int faultCode; };
AlarmRecord alarmHistory[MAX_ALARM_HISTORY];
int alarmHistCount = 0, alarmHistHead = 0;

/* ------------------------------------------------------------------
 * STORAGE TAB STATE — Plant/Line/Option/System/Sensor Tree
 * ------------------------------------------------------------------ */
int  stgPlant    = 0;      // 0=PlantA 1=PlantB
int  stgLine     = 0;      // 0=Line1 1=Line2 2=Line3
int  stgOption   = 0;      // 0=Option7 1=Option70
int  stgSystem   = 0;      // 0..3 = System1..4
int  stgSensor   = -1;     // selected sensor (-1 = none)
int  stgView     = 0;      // 0=browse tree, 1=sensor detail

// Sensor registry — each sensor in the system gets a unique 8-char ID
struct SensorRecord {
  char uid[12];       // Unique ID e.g. "PA1A070S1S01"
  char name[24];
  int  plant, line, option, system;
};
#define MAX_STG_SENSORS 64
SensorRecord stgSensors[MAX_STG_SENSORS];
int stgSensorCount = 0;

void generateSensorUID(char* buf, int plant, int line, int option, int system, int sensorNum) {
  // Format: PA/PB + L1-3 + O7/70 + S1-4 + SXX
  snprintf(buf, 12, "P%c%dO%02dS%dN%02d",
    (plant == 0) ? 'A' : 'B',
    line + 1,
    (option == 0) ? 7 : 70,
    system + 1,
    sensorNum);
}

/* ------------------------------------------------------------------
 * GLOBAL DISPLAY STATE MANAGERS
 * ------------------------------------------------------------------ */
int  currentTab = TAB_HOME;
bool needFullRedraw = true;

int  graphMode = 0; bool graphPaused = false;
int  graphCmpTop = 0, graphCmpBot = 1;

int  calibSelectedSensor = 0;
bool calibActionPulse = false;

int  cmpSensorA = 0, cmpSensorB = 1;
bool  cmpPredValid[4]  = {false,false,false,false};
float cmpPredSlope[4]  = {0,0,0,0};
long  cmpPredCycles[4]  = {0,0,0,0};
bool  cmpLiveMode = true;             
char  cmpLoadedDate[12] = "";         
bool  cmpDataLoaded = false;          
float cmpLoadedBuf[4][200];           
int   cmpLoadedCount[4] = {0,0,0,0};

int logViewMode = 0; 
int logScrollOffset = 0;
int logFilterSensor = -1;

int  archiveSelectedSensor = 0;
int  archiveCategory = 0; 
const char* ARCHIVE_CATS[] = { "PNEUMATIC", "HYDRAULIC", "ELECTRICAL", "MECHANICAL" };
bool archiveSuccessPulse = false;
unsigned long archivePulseStart = 0;

// ── STORAGE TAB: Hierarchical Navigator State ──────────────────────────────
// Levels: 0=Plant  1=Line  2=Option  3=System  4=Sensor(entry)
int  archNavLevel  = 0;   // current navigation depth
int  archNavPlant  = -1;  // 0=PlantA  1=PlantB
int  archNavLine   = -1;  // 0=Line1  1=Line2  2=Line3
int  archNavOption = -1;  // 0=Option7  1=Option70  2=Option700
int  archNavSystem = -1;  // 0=System1 … 3=System4
char archSensorID[16]  = "";
int  archSensorIDLen   = 0;
bool archSaveSuccess   = false;
unsigned long archSaveFlash = 0;
#define NP_STORAGE 4      // numpad mode for Storage sensor-ID entry

bool buzzerMuted = false; unsigned long buzzerMuteEnd = 0;
bool sdReady = false; char todayLogFile[32] = "", todayEvtFile[32] = "";
int  dtYear = 2026, dtMonth = 3, dtDay = 25, dtHour = 12, dtMin = 0, dtSec = 0;
unsigned long dtBaseMillis = 0;
bool dtClockReset = false;  // set true after manual time update to re-latch clock origin
char operatorID[16] = ""; int operatorIDLen = 0;

#define UART_BUF_SIZE 1024
char uartLine[UART_BUF_SIZE]; int uartLineIdx = 0;
unsigned long lastScreenUpdate=0, lastTimeDraw=0;
uint32_t totalPackets = 0, totalLogs = 0;

#define NP_NONE 0
#define NP_LOGIN_ID 1
#define NP_DT_FIELD 2
#define NP_SETTINGS 3
int  npMode = NP_NONE, npLen = 0, npTargetField = 0, npTargetSensor = 0;
char npValue[16] = "";
bool loginDone = false;
bool fieldSelectorActive = false; 
int  fieldSelectorSensor = -1;

int  configSelectedSensor = 0, settingsGroup = 0;

/* ------------------------------------------------------------------
 * FACTOR MAPS (For UI Automation)
 * ------------------------------------------------------------------ */
const char* FACTOR_NAMES[12] = { "RT", "FT", "ST", "FREQ", "VPEAK", "VFLOOR", "SWING", "KIN", "RATIO", "SRT", "SFT", "SPIKE" };
const char* FACTOR_UNITS[12] = { "ms", "ms", "ms", "Hz", "V", "V", "V", "ms", "", "ms", "ms", "cnt" };
const uint8_t FACTOR_GROUP[12] = { 0,0,0, 1,1,1,1, 2,2,2,2, 3 };
const char* GROUP_NAMES[4] = { "TIMING", "SIGNAL", "QUALITY", "FAULTS" };
uint16_t GROUP_COLORS[4] = { C_VOLT_GREEN, C_CYAN, C_HAZARD, C_ROG_RED };

const char* numpadFieldNames[] = { "RT MIN", "RT MAX", "RT CNT", "ST MIN", "ST MAX", "ST CNT", "FT MIN", "FT MAX", "FT CNT" };

/* ------------------------------------------------------------------
 * HARDWARE CONFIGURATION: PCA9557 & LGFX
 * ------------------------------------------------------------------ */

void pca9557_write(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA9557_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void init_pca9557() {
  pinMode(38, OUTPUT); digitalWrite(38, LOW); delay(10);
  pca9557_write(PCA9557_CONFIG, 0x00); delay(10);
  pca9557_write(PCA9557_OUTPUT, 0x00); delay(20);
  pca9557_write(PCA9557_OUTPUT, B00000101); delay(100);
  pca9557_write(PCA9557_CONFIG, B00000010); delay(50);
  Serial.println("[PCA9557] Touch reset sequence complete.");
}

class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB     _bus_instance;
  lgfx::Panel_RGB   _panel_instance;
  lgfx::Light_PWM   _light_instance;
  lgfx::Touch_GT911 _touch_instance;

  LGFX(void) {
    {
      auto cfg = _panel_instance.config();
      cfg.memory_width = 800; cfg.memory_height = 480;
      cfg.panel_width  = 800; cfg.panel_height  = 480;
      cfg.offset_x = 0; cfg.offset_y = 0;
      _panel_instance.config(cfg);
    }
    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;
      cfg.pin_d0  = GPIO_NUM_15; cfg.pin_d1  = GPIO_NUM_7;  cfg.pin_d2  = GPIO_NUM_6;  cfg.pin_d3  = GPIO_NUM_5;
      cfg.pin_d4  = GPIO_NUM_4;  cfg.pin_d5  = GPIO_NUM_9;  cfg.pin_d6  = GPIO_NUM_46; cfg.pin_d7  = GPIO_NUM_3;
      cfg.pin_d8  = GPIO_NUM_8;  cfg.pin_d9  = GPIO_NUM_16; cfg.pin_d10 = GPIO_NUM_1;  cfg.pin_d11 = GPIO_NUM_14;
      cfg.pin_d12 = GPIO_NUM_21; cfg.pin_d13 = GPIO_NUM_47; cfg.pin_d14 = GPIO_NUM_48; cfg.pin_d15 = GPIO_NUM_45;
      cfg.pin_henable = GPIO_NUM_41; cfg.pin_vsync   = GPIO_NUM_40; cfg.pin_hsync   = GPIO_NUM_39; cfg.pin_pclk    = GPIO_NUM_0;
      cfg.freq_write  = 12000000; cfg.hsync_polarity = 0; cfg.hsync_front_porch = 40; cfg.hsync_pulse_width = 48; cfg.hsync_back_porch = 40;
      cfg.vsync_polarity = 0; cfg.vsync_front_porch = 1; cfg.vsync_pulse_width = 31; cfg.vsync_back_porch = 13;
      cfg.pclk_active_neg = 1; cfg.de_idle_high = 0; cfg.pclk_idle_high  = 1;
      _bus_instance.config(cfg); _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = GPIO_NUM_2; cfg.invert = false; cfg.freq = 44100; cfg.pwm_channel = 7;
      _light_instance.config(cfg); _panel_instance.setLight(&_light_instance);
    }
    {
      auto cfg = _touch_instance.config();
      cfg.x_min = 0; cfg.x_max = 799; cfg.y_min = 0; cfg.y_max = 479;
      cfg.pin_int = 38; cfg.pin_rst = -1; cfg.bus_shared = true; cfg.offset_rotation = 0;
      cfg.i2c_port = 0; cfg.i2c_addr = 0x14; cfg.pin_sda = I2C_SDA; cfg.pin_scl = I2C_SCL; cfg.freq = 300000;
      _touch_instance.config(cfg); _panel_instance.setTouch(&_touch_instance);
    }
    setPanel(&_panel_instance);
  }
};

LGFX display;
lgfx::LGFX_Sprite canvas(&display);
bool spriteReady = false;

/* ------------------------------------------------------------------
 * HARDWARE TRANSLATORS & WAVEFORM ENGINE
 * ------------------------------------------------------------------ */

const char* getStateName(int s) { 
  switch(s) { 
    case 0: return "IDLE"; 
    case 1: return "LEAK";   // Repurposed VFLOOR as Leak indicator per user request
    case 2: return "RISE"; 
    case 3: return "STBL"; 
    case 4: return "FALL"; 
    default: return "VOID"; 
  } 
}

uint16_t getStateColor(int s) { 
  switch(s) { 
    case 0: return C_WHITE;       // IDLE — deadband, 0V
    case 1: return C_YELLOW;      // LEAK — VFloor holding (0.4-3V constant)
    case 2: return C_VOLT_GREEN;  // RISE — voltage climbing (GREEN)
    case 3: return C_CYAN;        // STBL — stable high voltage (CYAN)
    case 4: return C_ROG_RED;     // FALL — voltage dropping (RED)
    default: return C_STEEL; 
  } 
}

const char* getFaultName(int c) {
  switch(c) {
    case 0: return "NOMINAL"; case 1: return "LEAKAGE"; case 2: return "SPIKE";
    case 3: return "RT_ERR"; case 4: return "FT_ERR"; case 5: return "ST_ERR";
    case 6: return "FREQ_ERR"; case 7: return "VPK_ERR"; case 8: return "VFL_ERR";
    case 9: return "SWG_ERR"; case 10: return "KIN_ERR"; case 11: return "RATIO_ERR";
    case 12: return "JITTER_R"; case 13: return "JITTER_F"; case 14: return "ARCING";
    default: return "UNKNOWN";
  }
}

// Waveform Color Mapping — 5 Stage Signal Colors
// Stage 0: IDLE (0V)        → WHITE
// Stage 1: LEAK (0.4-3V)    → YELLOW (VFloor constant, no rise)
// Stage 2: RISE (climbing)  → GREEN
// Stage 3: STABLE (high V)  → CYAN
// Stage 4: FALL (dropping)  → RED
uint16_t getWaveformColor(int idx, float v, int stateID) {
  // Absolute Zero / Deadband — below 0.4V
  if (v <= 0.4f) return C_WHITE;
  
  // Map strictly to physical circuit states
  if (stateID == 1) return C_YELLOW;      // LEAK — VFloor holding (YELLOW)
  if (stateID == 2) return C_VOLT_GREEN;  // RISE — voltage climbing (GREEN)
  if (stateID == 3) return C_CYAN;        // STABLE — locked high (CYAN)
  if (stateID == 4) return C_ROG_RED;     // FALL — voltage dropping (RED)
  
  // Voltage present but no state match — use voltage threshold
  if (v > 0.4f && v < 3.0f) return C_YELLOW;  // Low voltage zone = leak/VFloor
  
  return C_STEEL;
}

/* ------------------------------------------------------------------
 * THE ANALYTICAL ENGINE (Fast Math & Health Grading)
 * ------------------------------------------------------------------ */

float medianOf9(float* arr) {
  float a[9]; memcpy(a, arr, sizeof(a));
  for (int i = 0; i < 8; i++) for (int j = 0; j < 8 - i; j++) if (a[j] > a[j+1]) { float t = a[j]; a[j] = a[j+1]; a[j+1] = t; }
  return a[4];
}

void updateSensorSigma(int idx) {
  SensorInfo &s = sData[idx]; float sumRT = 0, sumFT = 0; int count = 0;
  for (int i = 0; i < SIGMA_BUF; i++) if (s.sigmaRTbuf[i] > 0.0f) { sumRT += s.sigmaRTbuf[i]; sumFT += s.sigmaFTbuf[i]; count++; }
  if (count < 2) return;
  float meanRT = sumRT / count, meanFT = sumFT / count; float varRT = 0, varFT = 0;
  for (int i = 0; i < SIGMA_BUF; i++) if (s.sigmaRTbuf[i] > 0.0f) { varRT += (s.sigmaRTbuf[i] - meanRT) * (s.sigmaRTbuf[i] - meanRT); varFT += (s.sigmaFTbuf[i] - meanFT) * (s.sigmaFTbuf[i] - meanFT); }
  s.sigmaRT = sqrtf(varRT / count); s.sigmaFT = sqrtf(varFT / count);
}

void computeHealthScore(int idx) {
  SensorInfo &s = sData[idx]; if (s.cycles < 5) { s.healthScore = 100.0f; return; }
  float rtRef = (s.medRT > 0.01f) ? s.medRT : s.RT; s.factorScores[0] = (rtRef > 0.01f && s.RT > 0.01f) ? max(0.0f, 100.0f - fabsf(s.RT - rtRef) / rtRef * 200.0f) : 100.0f;
  float ftRef = (s.medFT > 0.01f) ? s.medFT : s.FT; s.factorScores[1] = (ftRef > 0.01f && s.FT > 0.01f) ? max(0.0f, 100.0f - fabsf(s.FT - ftRef) / ftRef * 200.0f) : 100.0f;
  s.factorScores[2] = (s.ST > 0.5f) ? 100.0f : max(0.0f, s.ST / 0.5f * 100.0f);
  s.factorScores[3] = (s.freq > 0.1f) ? 100.0f : 80.0f;
  s.factorScores[4] = (s.vPeak > 10.0f) ? 100.0f : max(0.0f, s.vPeak / 24.0f * 100.0f);
  s.factorScores[5] = (s.vFloor < 5.0f && s.vFloor > 0.0f) ? 100.0f : max(0.0f, 100.0f - (s.vFloor - 5.0f) * 20.0f);
  float swing = s.vPeak - s.vFloor; s.factorScores[6] = (swing > 10.0f) ? 100.0f : max(0.0f, swing / 10.0f * 100.0f);
  float kin = s.RT + s.FT; s.factorScores[7] = (kin < 5.0f) ? 100.0f : max(0.0f, 100.0f - (kin - 5.0f) * 10.0f);
  float ratio = (s.medFT > 0.001f) ? s.medRT / s.medFT : 1.0f; s.factorScores[8] = max(0.0f, 100.0f - fabsf(ratio - 1.0f) * 50.0f);
  s.factorScores[9] = (s.sigmaRT < 0.1f) ? 100.0f : max(0.0f, 100.0f - s.sigmaRT * 200.0f);
  s.factorScores[10] = (s.sigmaFT < 0.1f) ? 100.0f : max(0.0f, 100.0f - s.sigmaFT * 200.0f);
  
  // Bounce count factor — contact arcing / chatter detection
  s.factorScores[11] = (s.bounceCount == 0) ? 100.0f : max(0.0f, 100.0f - (float)s.bounceCount * 15.0f);
  
  float score = 0.0f; for (int f = 0; f < 12; f++) { s.factorScores[f] = constrain(s.factorScores[f], 0.0f, 100.0f); score += s.factorScores[f] * HEALTH_WEIGHTS[f]; }
  
  // Aggressive penalty for high leakage (Category 3: Quiescent Current)
  if (s.leakage > 5.0f) score -= (s.leakage - 5.0f) * 2.0f;

  // NEW: Penalty for high voltage ripple (Category 4A: Signal quality)
  if (s.ripple > 1.5f) score -= (s.ripple - 1.5f) * 8.0f;

  // NEW: Penalty for high saturation drop (Category 4A: Sat Drop)
  if (s.satDrop_V > 3.0f) score -= (s.satDrop_V - 3.0f) * 5.0f;

  // NEW: CUSUM drift penalty (Category 14: Statistical Health)
  float cusumPenalty = fabsf(s.cusumRT) * 0.5f + fabsf(s.cusumFT) * 0.5f;
  if (cusumPenalty > 5.0f) score -= (cusumPenalty - 5.0f) * 1.5f;
  
  s.healthScore = constrain(score, 0.0f, 100.0f);
}

// NEW: Maintenance action recommendation (Category 18: Composite Health)
const char* getMaintenanceAction(float health) {
  if (health > 80.0f) return "MONITOR";
  if (health > 50.0f) return "INSPECT SOON";
  return "REPLACE";
}

uint16_t getMaintenanceColor(float health) {
  if (health > 80.0f) return C_VOLT_GREEN;
  if (health > 50.0f) return C_HAZARD;
  return C_ROG_RED;
}

// NEW: CUSUM / EWMA drift detection (Category 14: Statistical Health)
void updateCUSUM(int idx) {
  SensorInfo &s = sData[idx];
  if (s.RT <= 0.01f || s.medRT <= 0.01f) return;

  // EWMA — exponential weighted moving average
  const float ewmaAlpha = 0.1f;
  if (s.ewmaRT == 0) { s.ewmaRT = s.RT; s.ewmaFT = s.FT; }
  s.ewmaRT = s.ewmaRT + ewmaAlpha * (s.RT - s.ewmaRT);
  s.ewmaFT = s.ewmaFT + ewmaAlpha * (s.FT - s.ewmaFT);

  // CUSUM — detect slow upward drift in RT (degradation signal)
  if (s.cusumTarget == 0 && s.cycles > 20) s.cusumTarget = s.medRT; // lock reference on first 20 cycles
  if (s.cusumTarget > 0) {
    float k = s.cusumTarget * 0.1f;  // Allowance: 10% of reference
    s.cusumRT += (s.RT - s.cusumTarget - k);
    s.cusumRT = max(0.0f, s.cusumRT); // One-sided CUSUM (drift UP only)
    s.cusumFT += (s.FT - (s.medFT > 0.01f ? s.medFT : s.FT) - k);
    s.cusumFT = max(0.0f, s.cusumFT);
  }
}

/* ------------------------------------------------------------------
 * TIMEKEEPING & SD CARD ARCHIVAL
 * ------------------------------------------------------------------ */

void updateSoftwareClock() {
  unsigned long elapsed = (millis() - dtBaseMillis) / 1000;
  static int origHour = -1, origMin = -1, origSec = -1, origDay = -1, origMonth = -1, origYear = -1;
  if (origHour < 0 || dtClockReset) {
    origHour = dtHour; origMin = dtMin; origSec = dtSec;
    origDay = dtDay; origMonth = dtMonth; origYear = dtYear;
    dtClockReset = false;
  }
  unsigned long totalSec = (unsigned long)origHour * 3600 + origMin * 60 + origSec + elapsed;
  unsigned long days = totalSec / 86400; totalSec %= 86400;
  int h = totalSec / 3600, m = (totalSec % 3600) / 60, s = totalSec % 60;
  int d = origDay + (int)days, mo = origMonth, yr = origYear;
  static const int dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  while (true) {
    uint8_t daysInMo = dim[mo-1];
    if (mo == 2 && ((yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0))) daysInMo = 29;
    if (d <= daysInMo) break;
    d -= daysInMo; mo++; if (mo > 12) { mo = 1; yr++; }
  }
  dtYear = yr; dtMonth = mo; dtDay = d; dtHour = h; dtMin = m; dtSec = s;
}

void getTimeStr(char* buf, int bufSize) { updateSoftwareClock(); snprintf(buf, bufSize, "%02d/%02d/%04d %02d:%02d:%02d", dtDay, dtMonth, dtYear, dtHour, dtMin, dtSec); }

void initSDCard() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (SD.begin(SD_CS)) { 
    sdReady = true; 
    if (!SD.exists("/logs"))    SD.mkdir("/logs"); 
    if (!SD.exists("/cfg"))     SD.mkdir("/cfg"); 
    if (!SD.exists("/cal"))     SD.mkdir("/cal"); 
    if (!SD.exists("/archive")) SD.mkdir("/archive");
    // VIPx3x4s hierarchical storage tree
    const char* plants[] = { "PlantA", "PlantB" };
    const char* options[] = { "Option7", "Option70", "Option700" };
    for (int p = 0; p < 2; p++) {
      char plantPath[32]; snprintf(plantPath, sizeof(plantPath), "/VIPx3x4s/%s", plants[p]);
      if (!SD.exists("/VIPx3x4s")) SD.mkdir("/VIPx3x4s");
      if (!SD.exists(plantPath))   SD.mkdir(plantPath);
      for (int ln = 1; ln <= 3; ln++) {
        char linePath[48]; snprintf(linePath, sizeof(linePath), "%s/Line%d", plantPath, ln);
        if (!SD.exists(linePath)) SD.mkdir(linePath);
        for (int op = 0; op < 3; op++) {
          char optPath[64]; snprintf(optPath, sizeof(optPath), "%s/%s", linePath, options[op]);
          if (!SD.exists(optPath)) SD.mkdir(optPath);
          for (int sys = 1; sys <= 4; sys++) {
            char sysPath[80]; snprintf(sysPath, sizeof(sysPath), "%s/System%d", optPath, sys);
            if (!SD.exists(sysPath)) SD.mkdir(sysPath);
          }
        }
      }
    }
    Serial.println("[STORAGE] VOLUME MOUNTED SUCCESSFULLY"); 
  } else { sdReady = false; Serial.println("[STORAGE] MOUNT FAILED - DATA WILL NOT PERSIST"); }
}

void updateLogFilenames() {
  updateSoftwareClock();
  snprintf(todayLogFile, sizeof(todayLogFile), "/logs/%04d%02d%02d.csv", dtYear, dtMonth, dtDay);
  snprintf(todayEvtFile, sizeof(todayEvtFile), "/logs/%04d%02d%02d_evt.csv", dtYear, dtMonth, dtDay);
  if (!SD.exists(todayLogFile)) { File f = SD.open(todayLogFile, FILE_WRITE); if (f) { f.println("SLNO,Port,Timestamp,Vin_V,Iin_mA,Vout_V,Iout_mA,Vfb_V,Ifb_mA,RT_us,ST_us,FT_us,CT_us,Hz,Vpeak_V,Vfloor_V,Swing_V,Inrush_mA,Leakage_mA,Ripple_V,Bounce,TotalBounce,DutyCycle,SatDrop_V,OV_Cnt,UV_Cnt,Spike_Cnt,FaultCode,Operator"); f.close(); } }
  if (!SD.exists(todayEvtFile)) { File f = SD.open(todayEvtFile, FILE_WRITE); if (f) { f.println("Sensor,Date,Time,Event,Operator"); f.close(); } }
}

void writeLogToSD(const char* logLine) {
  if (!sdReady) return; const char* data = logLine + 5; int len = strlen(data);
  File f = SD.open(todayLogFile, FILE_APPEND);
  if (f) { for (int i = 0; i < len; i++) { if (data[i] == '*') break; f.write(data[i]); } f.printf(",%s\n", operatorID); f.close(); totalLogs++; }
}

void writeEventToSD(const char* evtLine) {
  if (!sdReady) return; const char* data = evtLine + 5; int len = strlen(data);
  File f = SD.open(todayEvtFile, FILE_APPEND);
  if (f) { for (int i = 0; i < len; i++) { if (data[i] == '*') break; f.write(data[i]); } f.printf(",%s\n", operatorID); f.close(); }
}

void saveSettingsToSD(int idx) {
  if (!sdReady) return; char path[32]; snprintf(path, sizeof(path), "/cfg/P%d_limits.txt", idx + 1);
  File f = SD.open(path, FILE_WRITE); if (!f) return;
  for (int a = 0; a < 12; a++) f.printf("%.3f,%.3f,%d\n", sData[idx].alarms[a].minVal, sData[idx].alarms[a].maxVal, sData[idx].alarms[a].cntLimit);
  f.close();
}

void loadSettingsFromSD(int idx) {
  if (!sdReady) return; char path[32]; snprintf(path, sizeof(path), "/cfg/P%d_limits.txt", idx + 1);
  if (!SD.exists(path)) return; File f = SD.open(path, FILE_READ); if (!f) return;
  for (int a = 0; a < 12; a++) {
    char line[64]; int len = f.readBytesUntil('\n', line, sizeof(line) - 1); if (len <= 0) break; line[len] = '\0';
    float mn, mx; int cnt; if (sscanf(line, "%f,%f,%d", &mn, &mx, &cnt) == 3) { sData[idx].alarms[a].minVal = mn; sData[idx].alarms[a].maxVal = mx; sData[idx].alarms[a].cntLimit = cnt; }
  }
  f.close();
}

void loadCompareDataFromSD() {
  if (!sdReady) return;
  char path[32]; snprintf(path, sizeof(path), "/logs/%s.csv", cmpLoadedDate);
  if (!SD.exists(path)) { cmpDataLoaded = false; return; }
  File f = SD.open(path, FILE_READ); if (!f) { cmpDataLoaded = false; return; }
  
  for (int i = 0; i < 4; i++) cmpLoadedCount[i] = 0;
  char lineBuf[256]; f.readBytesUntil('\n', lineBuf, sizeof(lineBuf)); 
  
  while (f.available() && !f.isDirectory()) {
    int len = f.readBytesUntil('\n', lineBuf, sizeof(lineBuf) - 1); if (len <= 0) break; lineBuf[len] = '\0';
    int sID = 0; float vout = 0; int fieldNum = 0; char* tok = strtok(lineBuf, ",");
    while (tok) {
      if (fieldNum == 1) sID = atoi(tok);
      if (fieldNum == 7) { vout = atof(tok); break; }
      fieldNum++; tok = strtok(NULL, ",");
    }
    if (sID >= 1 && sID <= 4) {
      int idx = sID - 1;
      if (cmpLoadedCount[idx] < 200) cmpLoadedBuf[idx][cmpLoadedCount[idx]++] = vout;
    }
  }
  f.close(); cmpDataLoaded = true;
}

void saveArchiveToSD(int idx, int catIdx) {
  if (!sdReady) return; updateSoftwareClock();
  const char* catName = ARCHIVE_CATS[catIdx]; char catDir[32]; 
  snprintf(catDir, sizeof(catDir), "/archive/%s", catName);
  if (!SD.exists(catDir)) SD.mkdir(catDir);

  char fileName[64]; snprintf(fileName, sizeof(fileName), "%s/P%d_%04d%02d%02d_%02d%02d%02d.txt", catDir, idx + 1, dtYear, dtMonth, dtDay, dtHour, dtMin, dtSec);
  File f = SD.open(fileName, FILE_WRITE);
  if (f) {
    SensorInfo &s = sData[idx];
    f.println("=========================================");
    f.printf(" VIPx3x4s - DIAGNOSTIC ARCHIVE SNAPSHOT\n");
    f.println("=========================================");
    f.printf(" DATE/TIME: %02d/%02d/%04d %02d:%02d:%02d\n", dtDay, dtMonth, dtYear, dtHour, dtMin, dtSec);
    f.printf(" OPERATOR:  %s\n", operatorIDLen > 0 ? operatorID : "UNKNOWN");
    f.printf(" CATEGORY:  %s\n", catName);
    f.printf(" SENSOR:    PORT %d\n", idx + 1);
    f.println("-----------------------------------------");
    f.printf(" HEALTH:    %.0f%%\n", s.healthScore);
    f.printf(" STATUS:    %s\n", s.cycleNG ? "FAULT DETECTED" : "NOMINAL");
    f.println("-----------------------------------------");
    f.printf(" [ TELEMETRY ]\n");
    f.printf(" INPUT:     %.2f V | %.1f mA | %.1f mW\n", s.ch[0].V, s.ch[0].I, s.ch[0].P);
    f.printf(" OUTPUT:    %.2f V | %.1f mA | %.1f mW\n", s.ch[1].V, s.ch[1].I, s.ch[1].P);
    f.printf(" FEEDBACK:  %.2f V | %.1f mA | %.1f mW\n", s.ch[2].V, s.ch[2].I, s.ch[2].P);
    f.println("-----------------------------------------");
    f.printf(" [ KINETICS & JITTER ]\n");
    f.printf(" RISE TIME: %.2f ms (Jitter: %.2f)\n", s.RT, s.sigmaRT);
    f.printf(" STBL TIME: %.2f ms (Ripple: %.3f V)\n", s.ST, s.ripple);
    f.printf(" FALL TIME: %.2f ms (Jitter: %.2f)\n", s.FT, s.sigmaFT);
    f.printf(" FREQUENCY: %.1f Hz\n", s.freq);
    f.printf(" [ ADVANCED ]\n");
    f.printf(" INRUSH:    %.1f mA\n", s.inrush);
    f.printf(" LEAKAGE:   %.2f mA\n", s.leakage);
    f.printf(" BOUNCES:   %lu cnt\n", s.bounceCount);
    f.println("=========================================");
    f.close();
    archiveSuccessPulse = true; archivePulseStart = millis();
  }
}

/* ------------------------------------------------------------------
 * THE DIAGNOSTIC BRAIN (UART INGESTION & ALARM LOGIC)
 * ------------------------------------------------------------------ */

void addAlarmToHistory(int idx, const char* cause, int fc) {
  char timeStr[24]; getTimeStr(timeStr, sizeof(timeStr));
  AlarmRecord &ar = alarmHistory[alarmHistHead];
  ar.sensor = idx; strncpy(ar.cause, cause, 47); ar.cause[47] = '\0';
  strncpy(ar.time, timeStr, 23); ar.time[23] = '\0'; ar.faultCode = fc;
  alarmHistHead = (alarmHistHead + 1) % MAX_ALARM_HISTORY;
  if (alarmHistCount < MAX_ALARM_HISTORY) alarmHistCount++;
}

// Fast Packet Parser (66Hz - 500Hz)
void parseFastPacket(const char* line) {
  int id; unsigned long ts; float v1,i1,v2,i2,v3,i3;
  if (sscanf(line, "$D,%d,%lu,%f,%f,%f,%f,%f,%f", &id, &ts, &v1, &i1, &v2, &i2, &v3, &i3) >= 8) {
    if (id >= 1 && id <= 4 && !graphPaused) {
      int idx = id - 1; SensorInfo &s = sData[idx];
      
      // NEW: Display handles power calculation natively to reduce circuit load
      s.ch[0] = { v1, i1, v1*i1 };
      s.ch[1] = { v2, i2, v2*i2 };
      s.ch[2] = { v3, i3, v3*i3 };

      s.graphBuf[s.graphIdx] = v2;
      s.graphTimeBuf[s.graphIdx] = millis();
      s.graphColorBuf[s.graphIdx] = getWaveformColor(idx, v2, s.stateID);
      
      s.graphIdx++;
      if (s.graphIdx >= GRAPH_POINTS) { s.graphIdx = 0; s.graphFull = true; }
    }
  }
}

// FIX: Use float for Hz — the circuit sends "%.1f" which loses data if parsed as int
void parseHeartbeat(const char* line) {
  float hz; int s1, s2, s3, s4;
  if (sscanf(line, "$H,%f,%d,%d,%d,%d", &hz, &s1, &s2, &s3, &s4) >= 5) {
    circuitReadRate = (int)hz; circuitOnline = true; lastHeartbeat = millis();
    sData[0].active = (s1 == 1); sData[1].active = (s2 == 1);
    sData[2].active = (s3 == 1); sData[3].active = (s4 == 1);
  }
}

// FIX: Corrected comma counting + added client-side computed parameters
void parseSensorPacket(const char* line) {
  if (line[0] != '$' || line[1] != 'S' || line[2] != ',') return;
  int id, stateID, faultCode, alarmActive;
  float vin, vout, vfb, iin, iout, ifb, rt, st, ft, ct, hz, vpk;
  unsigned long cycles;

  int parsed = sscanf(line, "$S,%d,%f,%f,%*f,%f,%f,%*f,%f,%f,%*f,%d,%*lu,%*lu,%*lu,%*lu,%f,%f,%f,%f,%f,%f,%lu,%*lu,%d,%d,",
    &id, &vin, &iin, &vout, &iout, &vfb, &ifb,
    &stateID, &rt, &st, &ft, &ct, &hz, &vpk, &cycles, &faultCode, &alarmActive);

  if (parsed >= 16 && id >= 1 && id <= 4) {
    int idx = id - 1; SensorInfo &s = sData[idx];
    s.stateID = stateID; s.RT = rt; s.ST = st; s.FT = ft; s.cycles = cycles; s.faultCode = faultCode;
    
    float totalMs = rt + st + ft;
    if (totalMs > 0.1f) s.freq = 1000.0f / totalMs;
    s.rtMedBuf[s.medIdx] = rt; s.ftMedBuf[s.medIdx] = ft; s.medIdx = (s.medIdx + 1) % 9;
    s.medRT = medianOf9(s.rtMedBuf); s.medFT = medianOf9(s.ftMedBuf);
    s.sigmaRTbuf[s.sigmaIdx] = rt; s.sigmaFTbuf[s.sigmaIdx] = ft; s.sigmaIdx = (s.sigmaIdx + 1) % SIGMA_BUF;
    updateSensorSigma(idx);
    
    if (faultCode == FAULT_SPIKE) s.spikeCount++;
    if (faultCode != FAULT_NONE) { s.totalFaults++; s.cycleNG = true; s.goodCycles = 0; }
    else { s.cycleNG = false; s.goodCycles++; }

    // FIX: Corrected comma positions ($S prefix comma is #1, so cause is after comma #26)
    bool hardwareAlarm = (alarmActive == 1);
    int commaCount = 0; int causeStart = -1, causeEnd = -1, timeStart = -1, timeEnd = -1, extStart = -1;
    for (int i = 0; line[i] != '\0'; i++) {
      if (line[i] == ',') {
        commaCount++;
        if (commaCount == 26) causeStart = i + 1;     // after alarmActive
        if (commaCount == 27) { causeEnd = i; timeStart = i + 1; }   // after cause
        if (commaCount == 28) { timeEnd = i; extStart = i + 1; }     // after timestamp
      }
    }
    
    if (hardwareAlarm && !s.alarmActive) {
        s.alarmActive = true;
        if (causeStart >= 0 && causeEnd >= 0) { int len = causeEnd - causeStart; if (len > 0 && len < 63) { strncpy(s.alarmCause, line + causeStart, len); s.alarmCause[len] = '\0'; } }
        if (timeStart >= 0) { int ti = 0; int endIdx = (timeEnd > 0) ? timeEnd : strlen(line); for (int i = timeStart; i < endIdx && line[i] != '*' && ti < 23; i++) { s.alarmTime[ti++] = line[i]; } s.alarmTime[ti] = '\0'; }
        addAlarmToHistory(idx, s.alarmCause, s.faultCode);
    }

    // Extract Advanced Diagnostics from extended tail of $S packet
    if (extStart > 0) {
      float vfl, vpk_cal, swg, inr, lk, rip; uint32_t bnc;
      if (sscanf(line + extStart, "%f,%f,%f,%f,%f,%f,%lu", &vfl, &vpk_cal, &swg, &inr, &lk, &rip, &bnc) >= 7) {
        s.vFloor = vfl; s.vPeak = vpk_cal; s.calibReady = (vpk_cal - vfl > 2.0f);
        s.inrush = inr; s.leakage = lk; s.ripple = rip; s.bounceCount = bnc;

        // NEW: Compute dutyCycle client-side (Category 4A: Duty Cycle)
        if (ct > 0.1f) s.dutyCycle = (st / ct) * 100.0f;

        // NEW: Compute saturation drop client-side (Category 4A: Vcc - Vpeak)
        if (s.ch[0].V > 1.0f) s.satDrop_V = s.ch[0].V - vpk_cal;

        // NEW: Track average active current via EMA (Category 3: Active Current)
        if (s.ch[1].I > 0) {
          if (s.avgActiveCurrent == 0) s.avgActiveCurrent = s.ch[1].I;
          else s.avgActiveCurrent += 0.05f * (s.ch[1].I - s.avgActiveCurrent);
        }

        // NEW: Track average leakage via EMA (Category 3: Quiescent Current)
        if (lk > 0) {
          if (s.avgLeakage == 0) s.avgLeakage = lk;
          else s.avgLeakage += 0.05f * (lk - s.avgLeakage);
        }

        // NEW: Over/Under voltage event counting (Category 1) — edge-triggered
        static bool prevOv[4] = {false,false,false,false};
        static bool prevUv[4] = {false,false,false,false};
        bool nowOv = (s.ch[0].V > 26.0f);
        bool nowUv = (s.ch[0].V > 1.0f && s.ch[0].V < 18.0f);
        if (nowOv  && !prevOv[idx]) s.ovVoltCount++;
        if (nowUv  && !prevUv[idx]) s.uvVoltCount++;
        prevOv[idx] = nowOv;
        prevUv[idx] = nowUv;
      }
    }

    // NEW: Update CUSUM/EWMA drift trackers (Category 14)
    updateCUSUM(idx);

    // NEW: Track power-on hours (Category 16) since first $S packet per sensor
    if (s.firstPacketMs == 0) s.firstPacketMs = millis();
    s.powerOnHours = (millis() - s.firstPacketMs) / 3600000.0f;

    computeHealthScore(idx);

    // Software Alarm Latch
    if (!s.alarmActive && s.calibReady && s.cycles > 5) {
        float factorVals[12] = { s.RT, s.FT, s.ST, s.freq, s.vPeak, s.vFloor, s.vPeak - s.vFloor, s.RT + s.FT, (s.medFT > 0.001f) ? (s.medRT / s.medFT) : 1.0f, s.sigmaRT, s.sigmaFT, (float)s.spikeCount };
        for (int f = 3; f < 12; f++) { 
            FactorAlarm &fa = s.alarms[f];
            if (fa.maxVal > 0.01f) { 
                if (factorVals[f] < fa.minVal || factorVals[f] > fa.maxVal) {
                    fa.violCnt++;
                    if (fa.violCnt >= fa.cntLimit || fa.cntLimit == 0) {
                        s.alarmActive = true; s.faultCode = 6 + (f - 3); 
                        snprintf(s.alarmCause, sizeof(s.alarmCause), "SYS ALERT: %s [%.2f%s]", FACTOR_NAMES[f], factorVals[f], FACTOR_UNITS[f]);
                        getTimeStr(s.alarmTime, sizeof(s.alarmTime));
                        addAlarmToHistory(idx, s.alarmCause, s.faultCode);
                        char evtBuf[128]; snprintf(evtBuf, sizeof(evtBuf), "$EVT,%d,*,*,%s*", idx+1, s.alarmCause); writeEventToSD(evtBuf);
                        break; 
                    }
                } else { fa.violCnt = 0; }
            }
        }
    }

    totalPackets++;
    if (cycles > 0 && cycles != s.prevCycles) {
      cycleHistory[historyHead] = { (uint8_t)idx, s.cycles, millis(), s.RT, s.ST, s.FT, s.freq, s.vFloor, s.vPeak, s.ch[1].V, s.cycleNG, s.faultCode };
      historyHead = (historyHead + 1) % MAX_HISTORY;
      if (historyCount < MAX_HISTORY) historyCount++;
      s.prevCycles = cycles;
    }
  }
}

void parseLimitPacket(const char* line) {
  if (line[0] != '$' || line[1] != 'L') return;
  int id, rtCnt, stCnt, ftCnt; unsigned long rtMin, rtMax, stMin, stMax, ftMin, ftMax;
  if (sscanf(line, "$L,%d,%lu,%lu,%d,%lu,%lu,%d,%lu,%lu,%d", &id, &rtMin, &rtMax, &rtCnt, &stMin, &stMax, &stCnt, &ftMin, &ftMax, &ftCnt) == 10 && id >= 1 && id <= 4) {
    int idx = id - 1;
    sData[idx].alarms[0] = { (float)rtMin, (float)rtMax, (uint8_t)rtCnt, 0 };
    sData[idx].alarms[1] = { (float)stMin, (float)stMax, (uint8_t)stCnt, 0 };
    sData[idx].alarms[2] = { (float)ftMin, (float)ftMax, (uint8_t)ftCnt, 0 };
  }
}

void parseAlarmBroadcast(const char* line) {
  // $ALRM,fc1,act1,fc2,act2,fc3,act3,fc4,act4*
  int fc[4], act[4];
  if (sscanf(line, "$ALRM,%d,%d,%d,%d,%d,%d,%d,%d",
             &fc[0], &act[0],
             &fc[1], &act[1],
             &fc[2], &act[2],
             &fc[3], &act[3]) != 8) {
    return;
  }

  for (int i = 0; i < 4; i++) {
    if (act[i] == 1) {
      if (!sData[i].alarmActive) {
        sData[i].alarmActive = true;
        sData[i].faultCode = fc[i];
        snprintf(sData[i].alarmCause, sizeof(sData[i].alarmCause), "%s", getFaultName(fc[i]));
        getTimeStr(sData[i].alarmTime, sizeof(sData[i].alarmTime));
        addAlarmToHistory(i, sData[i].alarmCause, sData[i].faultCode);
      } else {
        // Keep existing alarmCause/time; just update fault code.
        sData[i].faultCode = fc[i];
      }
    } else {
      sData[i].alarmActive = false;
      sData[i].faultCode = FAULT_NONE;
      sData[i].alarmCause[0] = '\0';
      sData[i].alarmTime[0] = '\0';
    }
  }
}

void readAllUART() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      if (uartLineIdx > 0) { 
        uartLine[uartLineIdx] = '\0'; 
        if (strncmp(uartLine, "$ALRM,", 6) == 0) parseAlarmBroadcast(uartLine);
        else if (uartLine[1] == 'D') parseFastPacket(uartLine);
        else if (uartLine[1] == 'S' && uartLine[2] == ',') parseSensorPacket(uartLine);
        else if (uartLine[1] == 'H') parseHeartbeat(uartLine);
        else if (uartLine[1] == 'L') parseLimitPacket(uartLine);
        else if (strncmp(uartLine, "$LOG,", 5) == 0) writeLogToSD(uartLine);
        else if (strncmp(uartLine, "$EVT,", 5) == 0) writeEventToSD(uartLine);
        uartLineIdx = 0; 
      }
    } else if (c != '\r') { if (uartLineIdx < UART_BUF_SIZE - 1) uartLine[uartLineIdx++] = c; else uartLineIdx = 0; }
  }
}

void sendConfig(int sID, int typeIdx, float mn, float mx, int cnt) {
  // Circuit expects: $CFG,sid,type,min_us,max_us,cnt*
  // type: 0=RT, 1=ST, 2=FT
  char c[64];
  snprintf(c, 64, "$CFG,%d,%d,%.0f,%.0f,%d*", sID, typeIdx, mn, mx, cnt);
  Serial1.println(c);
  Serial1.flush();
}
void sendReset(int sID) { char c[32]; snprintf(c, 32, "$RST,%d*", sID); Serial1.println(c); Serial1.flush(); }
void requestLimits() { Serial1.println("$GETL,0*"); Serial1.flush(); }
void sendTimeSync() { updateSoftwareClock(); char cmd[48]; snprintf(cmd, sizeof(cmd), "$TIME,%d,%d,%d,%d,%d,%d*", dtDay, dtMonth, dtYear, dtHour, dtMin, dtSec); Serial1.println(cmd); Serial1.flush(); }
void sendSensorType(int sID, int type) { char c[32]; snprintf(c, 32, "$STYPE,%d,%d*", sID+1, type); Serial1.println(c); Serial1.flush(); }
void sendCalReset(int sID) { char c[32]; snprintf(c, 32, "$RCAL,%d*", sID+1); Serial1.println(c); Serial1.flush(); }

/* ------------------------------------------------------------------
 * GLOBAL UI PRIMITIVES & HUD CHROME
 * ------------------------------------------------------------------ */

// Local state for the Home Screen Mini-Graph Toggles
bool homeShowGraph[4] = {true, true, true, true}; 

void drawTabBar() {
  int tabW = SCR_W / NUM_TABS; int y = HEADER_H;
  for (int t = 0; t < NUM_TABS; t++) {
    int x = t * tabW; bool active = (t == currentTab);
    
    if (active) {
      // Active tab: crimson fill with angular cut corners
      canvas.fillRect(x, y, tabW, TAB_BAR_H, C_CRIMSON);
      canvas.drawFastHLine(x, y, tabW, C_ROG_RED);
      // Angular bottom edge highlight
      canvas.drawFastHLine(x, y + TAB_BAR_H - 1, tabW, C_WHITE);
      canvas.drawFastHLine(x, y + TAB_BAR_H - 2, tabW, C_ROG_RED);
      // Corner chamfers
      canvas.drawLine(x, y, x + 6, y + 6, C_WHITE);
      canvas.drawLine(x + tabW - 1, y, x + tabW - 7, y + 6, C_WHITE);
    } else {
      canvas.fillRect(x, y, tabW, TAB_BAR_H, C_VOID);
      canvas.drawFastHLine(x, y, tabW, C_DARK_METAL);
      canvas.drawFastHLine(x, y + TAB_BAR_H - 1, tabW, C_STEEL);
      // Chamfered top corners on inactive tabs
      canvas.drawLine(x, y, x + 5, y + 5, C_STEEL);
      canvas.drawLine(x + tabW - 1, y, x + tabW - 6, y + 5, C_STEEL);
      // Separator slits
      canvas.drawLine(x + tabW - 1, y + 6, x + tabW - 1, y + TAB_BAR_H - 6, C_DARK_METAL);
    }
    
    uint16_t txCol = active ? C_WHITE : C_MUTED_TEXT;
    uint16_t bgCol = active ? C_CRIMSON : C_VOID;
    canvas.setFont(&fonts::FreeSansBold9pt7b); canvas.setTextColor(txCol, bgCol);
    canvas.setTextDatum(middle_center); canvas.drawString(TAB_LABELS[t], x + tabW / 2, y + TAB_BAR_H / 2);
  }
}

void drawHeader() {
  canvas.fillRect(0, 0, SCR_W, HEADER_H, C_VOID);
  // Bottom edge: dual-line with angular endpoints
  canvas.fillRect(0, HEADER_H - 3, SCR_W, 3, C_STEEL);
  canvas.drawFastHLine(0, HEADER_H - 1, SCR_W, C_CYAN);
  // L-bracket corner accents (ROG signature)
  canvas.drawFastHLine(3, 3, 14, C_CYAN);
  canvas.drawFastVLine(3, 3, 14, C_CYAN);
  canvas.drawFastHLine(SCR_W-17, 3, 14, C_ROG_RED);
  canvas.drawFastVLine(SCR_W-3, 3, 14, C_ROG_RED);

  // ROG-Style Vent Slits (left side accent)
  for (int sl = 0; sl < 6; sl++) { 
    canvas.drawLine(10 + sl * 8, 6, 22 + sl * 8, HEADER_H - 8, C_CYAN); 
    canvas.drawLine(11 + sl * 8, 6, 23 + sl * 8, HEADER_H - 8, C_GLOW_CYAN);
  }

  // Title with subtle chromatic split
  canvas.setFont(&fonts::FreeSansBold12pt7b); 
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(C_ROG_RED, C_VOID);
  canvas.drawString("VIPx3X4s", 66, HEADER_H / 2 + 1);
  canvas.setTextColor(C_CYAN, C_VOID);
  canvas.drawString("VIPx3X4s", 65, HEADER_H / 2 - 1);
  
  // ── SYSTEM STATUS INDICATOR (moved here from footer) ──
  {
    bool anyAlarm = false; int alarmSensor = -1;
    for (int i = 0; i < 4; i++) { if (sData[i].active && sData[i].alarmActive) { anyAlarm = true; alarmSensor = i; break; } }
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_left);
    if (anyAlarm) {
      uint16_t aC = ((millis() / 120) % 2 == 0) ? C_ROG_RED : C_HAZARD;
      canvas.fillCircle(191, HEADER_H / 2, 4, aC);
      canvas.drawCircle(191, HEADER_H / 2, 6, aC);
      canvas.setTextColor(aC, C_VOID);
      char alertBuf[32]; snprintf(alertBuf, sizeof(alertBuf), "FAULT P%d", alarmSensor + 1);
      canvas.drawString(alertBuf, 201, HEADER_H / 2);
    } else {
      uint16_t dotCol = ((millis() / 500) % 2 == 0) ? C_VOLT_GREEN : C_GLOW_CYAN;
      canvas.fillCircle(191, HEADER_H / 2, 4, dotCol);
      canvas.setTextColor(C_VOLT_GREEN, C_VOID);
      canvas.drawString("SYSTEM NOMINAL", 201, HEADER_H / 2);
    }
  }

  if (operatorIDLen > 0) {
    canvas.setTextColor(C_CYBER_GOLD, C_VOID);
    char idStr[24]; snprintf(idStr, sizeof(idStr), "OP: %s", operatorID); 
    canvas.drawString(idStr, 310, HEADER_H / 2);
  }

  char timeStr[24]; getTimeStr(timeStr, sizeof(timeStr));
  canvas.setFont(&fonts::FreeMonoBold9pt7b); canvas.setTextColor(C_WHITE, C_VOID);
  canvas.setTextDatum(middle_center); canvas.drawString(timeStr, SCR_W / 2 + 80, HEADER_H / 2);

  // SD Status Badge (angular)
  canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_left);
  if (sdReady) {
    canvas.fillRect(SCR_W - 128, 8, 68, 22, C_VOLT_GREEN); 
    canvas.drawLine(SCR_W - 128, 8, SCR_W - 122, 14, C_WHITE); // corner cut
    canvas.setTextColor(C_VOID, C_VOLT_GREEN); canvas.drawString("DATA_REC", SCR_W - 124, HEADER_H / 2);
  } else {
    canvas.fillRect(SCR_W - 128, 8, 68, 22, C_ROG_RED); 
    canvas.drawLine(SCR_W - 128, 8, SCR_W - 122, 14, C_WHITE);
    canvas.setTextColor(C_WHITE, C_ROG_RED); canvas.drawString("NO_MEDIA", SCR_W - 124, HEADER_H / 2);
  }

  // Active Data Pulse (Heartbeat indicator — aggressive strobe)
  bool act = (millis() - lastHeartbeat < 2000);
  int dotX = SCR_W - 20, dotY = HEADER_H / 2;
  if (act) {
    uint16_t ringCol = ((millis() / 80) % 2 == 0) ? C_CYAN : C_VOID;
    canvas.fillRect(dotX - 6, dotY - 6, 12, 12, ringCol); 
    canvas.fillRect(dotX - 3, dotY - 3, 6, 6, C_VOID);
    // Pulse ring
    canvas.drawRect(dotX - 8, dotY - 8, 16, 16, C_CYAN);
  } else {
    canvas.fillRect(dotX - 6, dotY - 6, 12, 12, C_CRIMSON); 
    canvas.drawLine(dotX - 6, dotY - 6, dotX + 6, dotY + 6, C_VOID);
    canvas.drawRect(dotX - 8, dotY - 8, 16, 16, C_ROG_RED);
  }

  drawTabBar();
}

void drawFooter() {
  int y = SCR_H - FOOTER_H;
  canvas.fillRect(0, y, SCR_W, FOOTER_H, C_VOID);
  // Top edge: dual line with angular endpoints
  canvas.drawFastHLine(0, y, SCR_W, C_STEEL);
  canvas.drawFastHLine(0, y + 1, SCR_W, C_CYAN);
  // L-bracket corner accents
  canvas.drawFastHLine(3, y+3, 12, C_CYAN);
  canvas.drawFastVLine(3, y+3, 12, C_CYAN);
  canvas.drawFastHLine(SCR_W-15, y+3, 12, C_ROG_RED);
  canvas.drawFastVLine(SCR_W-3, y+3, 12, C_ROG_RED);

  bool anyAlarm = false; int alarmSensor = -1;
  for (int i = 0; i < 4; i++) { if (sData[i].active && sData[i].alarmActive) { anyAlarm = true; alarmSensor = i; break; } }

  canvas.setFont(&fonts::FreeMonoBold9pt7b); canvas.setTextDatum(middle_left);
  if (anyAlarm) {
    uint16_t alarmCol = ((millis() / 120) % 2 == 0) ? C_ROG_RED : C_HAZARD;
    canvas.fillRect(0, y + 2, 260, FOOTER_H - 2, alarmCol);
    // Angular cut on right edge of alarm badge
    canvas.drawLine(260, y + 2, 248, y + FOOTER_H, C_VOID);
    canvas.setTextColor(C_VOID, alarmCol);
    char alertStr[96]; snprintf(alertStr, sizeof(alertStr), " FAULT: PORT %d", alarmSensor + 1);
    canvas.drawString(alertStr, 10, y + FOOTER_H / 2 + 1);
    canvas.setTextColor(alarmCol, C_VOID);
    canvas.drawString(sData[alarmSensor].alarmCause, 270, y + FOOTER_H / 2 + 1);
  } else {
    // Footer is clean — status is now shown in header
  }

  // LINK/RX info removed from footer (status moved to header)
}

/* ------------------------------------------------------------------
 * DYNAMIC WAVEFORM ENGINE (Hyper-Tech Grid)
 * ------------------------------------------------------------------ */

void drawGraph(lgfx::LGFXBase& gfx, int idx, int gx, int gy, int gw, int gh) {
  gfx.fillRect(gx, gy, gw, gh, C_VOID);
  gfx.drawRect(gx, gy, gw, gh, C_STEEL);
  // L-bracket corner accents on graph frame
  gfx.drawFastHLine(gx+3, gy+3, 12, C_CYAN);
  gfx.drawFastVLine(gx+3, gy+3, 12, C_CYAN);
  gfx.drawFastHLine(gx+gw-15, gy+3, 12, C_ROG_RED);
  gfx.drawFastVLine(gx+gw-3, gy+3, 12, C_ROG_RED);
  gfx.drawFastHLine(gx+3, gy+gh-3, 12, C_ROG_RED);
  gfx.drawFastVLine(gx+3, gy+gh-15, 12, C_ROG_RED);
  gfx.drawFastHLine(gx+gw-15, gy+gh-3, 12, C_CYAN);
  gfx.drawFastVLine(gx+gw-3, gy+gh-15, 12, C_CYAN);
  
  // Crosshatch Grid & Reticles (subtle dark grid)
  for (int i = 1; i < 4; i++) {
    int ly = gy + (gh * i) / 4;
    for (int dx = gx + 4; dx < gx + gw - 4; dx += 6) gfx.drawPixel(dx, ly, C_DARK_METAL);
    if (gw > 100) {
      gfx.setFont(&fonts::Font0); gfx.setTextColor(C_MUTED_TEXT, C_VOID); gfx.setTextDatum(middle_right);
      int vLabel = (int)(GRAPH_V_MAX * (4 - i) / 4); char vStr[8]; snprintf(vStr, 8, "%dV", vLabel); gfx.drawString(vStr, gx + 26, ly);
    }
  }

  int plotX = gx + (gw > 100 ? 30 : 4); int plotW = gw - (gw > 100 ? 34 : 8);
  for (int s = 1; s < 5; s++) {
    int lx = plotX + (s * plotW) / 5;
    for (int dy = gy + 4; dy < gy + gh - 4; dy += 8) gfx.drawFastVLine(lx, dy, 2, C_DARK_METAL);
  }

  SensorInfo &sd = sData[idx];
  int count = sd.graphFull ? GRAPH_POINTS : sd.graphIdx; if (count < 2) return;
  
  unsigned long now = millis(); unsigned long winStart = (now >= GRAPH_WINDOW_MS) ? now - GRAPH_WINDOW_MS : 0;
  int plotTop = gy + 4, plotBot = gy + gh - 4; int plotH = plotBot - plotTop;
  int prevPx = -1, prevPy = -1; 

  for (int i = 0; i < count; i++) {
    int si = (sd.graphFull ? (sd.graphIdx + i) : i) % GRAPH_POINTS;
    unsigned long t = sd.graphTimeBuf[si]; if (t < winStart) continue;
    int px = plotX + (int)(((float)(t - winStart) / GRAPH_WINDOW_MS) * plotW); px = constrain(px, plotX, plotX + plotW - 1);
    int py = plotBot - (int)((sd.graphBuf[si] / GRAPH_V_MAX) * plotH); py = constrain(py, plotTop, plotBot);

    // Color logic pulled from parseFastPacket buffers
    uint16_t lc = sd.alarmActive ? (((millis() / 150) % 2 == 0) ? C_ROG_RED : C_HAZARD) : sd.graphColorBuf[si];
    
    if (prevPx >= 0) { 
      gfx.drawLine(prevPx, prevPy, px, py, lc); 
      gfx.drawLine(prevPx, prevPy-1, px, py-1, lc); // Double thickness
    }
    prevPx = px; prevPy = py; 
  }
  
  // Live Head Cursor (Target Lock)
  if (prevPx >= 0) { 
    gfx.drawLine(prevPx - 5, prevPy, prevPx + 5, prevPy, C_WHITE); 
    gfx.drawLine(prevPx, prevPy - 5, prevPx, prevPy + 5, C_WHITE); 
  }
}

/* ------------------------------------------------------------------
 * TAB: DASHBOARD (Aggressive 2x2 Matrix)
 * ------------------------------------------------------------------ */

void drawSensorPanelHome(int idx, int px, int py, int pw, int ph) {
  SensorInfo &s = sData[idx];
  
  canvas.fillRect(px, py, pw, ph, C_VOID);
  
  // Outer Status Frame
  uint16_t frameCol = s.alarmActive ? C_ROG_RED : (s.active ? C_STEEL : C_DARK_METAL);
  if (s.stateID == 1 && !s.alarmActive) frameCol = C_YELLOW; // Yellow frame if leaking/stuck in Vfloor
  
  canvas.drawRect(px, py, pw, ph, frameCol);
  canvas.drawRect(px + 2, py + 2, pw - 4, ph - 4, C_VOID); 
  
  // Cyber Chamfered Corners
  canvas.drawLine(px, py + 15, px + 15, py, frameCol);
  canvas.drawLine(px + pw, py + 15, px + pw - 15, py, frameCol);
  canvas.drawLine(px, py + ph - 15, px + 15, py + ph, frameCol);
  canvas.drawLine(px + pw, py + ph - 15, px + pw - 15, py + ph, frameCol);

  int L = px + 10, R = px + pw - 10, W = pw - 20;

  // ==========================================
  // ROW 1: HEADER & VIEW TOGGLE
  // ==========================================
  int y1 = py + 8;
  canvas.setFont(&fonts::FreeSansBold12pt7b);
  canvas.setTextColor(C_CYAN, C_VOID);
  canvas.setTextDatum(top_left);
  char buf[80]; snprintf(buf, sizeof(buf), "PORT %d", idx + 1);
  canvas.drawString(buf, L, y1);

  // View Toggle Button (Graph vs Data)
  int tglW = 70; int tglX = px + pw/2 - tglW/2;
  canvas.drawRect(tglX, y1, tglW, 20, C_STEEL);
  canvas.setFont(&fonts::Font0); canvas.setTextColor(C_WHITE, C_VOID); canvas.setTextDatum(middle_center);
  canvas.drawString(homeShowGraph[idx] ? "WAVEFORM" : "TELEMETRY", tglX + tglW/2, y1 + 10);

  // Status Badge
  bool ng = s.cycleNG;
  int badgeW = 75; int badgeX = R - badgeW;
  uint16_t badgeBg = ng ? C_ROG_RED : C_VOLT_GREEN;
  if (s.stateID == 1 && !ng) badgeBg = C_YELLOW; // Leakage specific warning

  canvas.fillRect(badgeX, y1, badgeW, 20, badgeBg);
  canvas.setFont(&fonts::FreeMonoBold9pt7b); canvas.setTextColor(C_VOID, badgeBg); canvas.setTextDatum(middle_center);
  const char* bText = ng ? "FAULT" : (s.stateID == 1 ? "LEAK" : "NOMINAL");
  canvas.drawString(bText, badgeX + badgeW/2, y1 + 10);

  canvas.drawFastHLine(L, py + 32, W, C_DARK_METAL);

  if (!s.active) {
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(C_DARK_METAL, C_VOID); canvas.setTextDatum(middle_center);
    canvas.drawString("OFFLINE", px + pw/2, py + ph/2);
    return;
  }

  // ==========================================
  // ROW 2: VIP MATRIX (Input, Output, Feedback)
  // ==========================================
  int y2 = py + 38;
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextDatum(top_left);
  
  const char* chNames[] = { "[IN ]", "[OUT]", "[FB ]" };
  for (int c = 0; c < 3; c++) {
    int rowY = y2 + (c * 18);
    uint16_t rowCol = (c == 1) ? C_CYAN : C_WHITE;
    
    if (c == 2 && !s.feedbackEnabled) {
      canvas.setTextColor(C_DARK_METAL, C_VOID);
      snprintf(buf, sizeof(buf), "[FB ]   -- N/A --");
    } else {
      canvas.setTextColor(rowCol, C_VOID);
      snprintf(buf, sizeof(buf), "%s  %05.2fV   %04.0fmA   %04.0fmW", chNames[c], s.ch[c].V, s.ch[c].I, s.ch[c].P);
    }
    canvas.drawString(buf, L, rowY);
  }

  canvas.drawFastHLine(L, y2 + 58, W, C_DARK_METAL);

  // ==========================================
  // CONDITIONAL ROW 3: WAVEFORM OR TELEMETRY GRID
  // ==========================================
  int y3 = y2 + 64;
  int graphH = (py + ph - 8) - y3;

  if (s.alarmActive) {
    // Critical Alarm Override
    canvas.fillRect(L, y3, W, graphH, C_ROG_RED);
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(C_VOID, C_ROG_RED);
    canvas.setTextDatum(middle_center);
    canvas.drawString(s.alarmCause, px + pw/2, y3 + graphH/2);
  } 
  else if (homeShowGraph[idx]) {
    // MINI-GRAPH VIEW
    if (graphH > 15) drawGraph(canvas, idx, L, y3, W, graphH);
  } 
  else {
    // DEEP TELEMETRY GRID VIEW
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    
    // Column 1: Timing
    canvas.setTextColor(C_VOLT_GREEN, C_VOID); canvas.drawString("RT:", L, y3);
    canvas.setTextColor(C_WHITE, C_VOID);      snprintf(buf, 32, "%.1f", s.RT); canvas.drawString(buf, L+30, y3);
    
    canvas.setTextColor(C_CYAN, C_VOID);       canvas.drawString("ST:", L, y3 + 20);
    canvas.setTextColor(C_WHITE, C_VOID);      snprintf(buf, 32, "%.1f", s.ST); canvas.drawString(buf, L+30, y3 + 20);
    
    canvas.setTextColor(C_ROG_RED, C_VOID);    canvas.drawString("FT:", L, y3 + 40);
    canvas.setTextColor(C_WHITE, C_VOID);      snprintf(buf, 32, "%.1f", s.FT); canvas.drawString(buf, L+30, y3 + 40);

    // Column 2: Total Time & Volts
    int col2X = L + 110;
    float totalTime = s.RT + s.ST + s.FT;
    canvas.setTextColor(C_PURPLE, C_VOID);     canvas.drawString("TOT:", col2X, y3);
    canvas.setTextColor(C_WHITE, C_VOID);      snprintf(buf, 32, "%.1f", totalTime); canvas.drawString(buf, col2X+40, y3);

    canvas.setTextColor(C_GHOST_CYAN, C_VOID); canvas.drawString("VFL:", col2X, y3 + 20);
    canvas.setTextColor(C_WHITE, C_VOID);      snprintf(buf, 32, "%.2f", s.vFloor); canvas.drawString(buf, col2X+40, y3 + 20);

    canvas.setTextColor(C_GHOST_CYAN, C_VOID); canvas.drawString("VPK:", col2X, y3 + 40);
    canvas.setTextColor(C_WHITE, C_VOID);      snprintf(buf, 32, "%.2f", s.vPeak); canvas.drawString(buf, col2X+40, y3 + 40);

    // Column 3: Cycles, Frequency, Duty Cycle
    int col3X = col2X + 130;
    canvas.setTextColor(C_CYBER_GOLD, C_VOID); canvas.drawString("CYC:", col3X, y3);
    canvas.setTextColor(C_WHITE, C_VOID);      snprintf(buf, 32, "%lu", s.cycles); canvas.drawString(buf, col3X+40, y3);

    canvas.setTextColor(C_CYAN, C_VOID);       canvas.drawString("FRQ:", col3X, y3 + 20);
    canvas.setTextColor(C_WHITE, C_VOID);      snprintf(buf, 32, "%.1f", s.freq); canvas.drawString(buf, col3X+40, y3 + 20);

    // NEW: Duty Cycle + Saturation Drop on row 3 (Category 4A)
    canvas.setTextColor(C_PURPLE, C_VOID);     canvas.drawString("DUT:", col3X, y3 + 40);
    canvas.setTextColor(C_WHITE, C_VOID);      snprintf(buf, 32, "%.0f%%", s.dutyCycle); canvas.drawString(buf, col3X+40, y3 + 40);

    // NEW: Health bar with Maintenance Action text (Category 18)
    int barY = y3 + 65;
    uint16_t hCol = getMaintenanceColor(s.healthScore);
    canvas.fillRect(L, barY, W, 8, C_DARK_METAL);
    int fillW = (int)((s.healthScore / 100.0f) * W);
    if (fillW > 0) canvas.fillRect(L, barY, fillW, 8, hCol);

    // Maintenance recommendation label (Category 18: Recommended Action)
    canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_left);
    canvas.setTextColor(hCol, C_VOID);
    snprintf(buf, 32, "%.0f%% [%s]", s.healthScore, getMaintenanceAction(s.healthScore));
    canvas.drawString(buf, L, barY + 16);
  }
}

void drawTabHome() {
  int gap = 6; int h = (BODY_H - gap) / 2; int w = (SCR_W - gap) / 2;
  canvas.fillRect(0, BODY_Y, SCR_W, BODY_H, C_VOID);
  drawSensorPanelHome(0, 0,       BODY_Y,            w, h);
  drawSensorPanelHome(1, w + gap, BODY_Y,            w, h);
  drawSensorPanelHome(2, 0,       BODY_Y + h + gap, w, h);
  drawSensorPanelHome(3, w + gap, BODY_Y + h + gap, w, h);
}

/* ------------------------------------------------------------------
 * TAB: WAVEFORM (High-Speed Oscilloscope / Live Dashboard)
 * ------------------------------------------------------------------ */

void drawTabLive() {
  canvas.fillRect(0, BODY_Y, SCR_W, BODY_H, C_VOID);

  // Sharp Cyberpunk Mode Selectors
  const char* mLabels[] = { "ALL", "P1", "P2", "P3", "P4", "SPLIT" };
  uint16_t sensorColors[] = { C_CYAN, C_CYAN, C_VOLT_GREEN, C_HAZARD, C_PURPLE, C_ROG_RED };
  int btnW = 60, btnH = 26, btnY = BODY_Y + 6;

  for (int m = 0; m <= 5; m++) {
    int btnX = 8 + m * (btnW + 6);
    bool act = (m == graphMode);
    uint16_t bg = act ? sensorColors[m] : C_VOID;
    uint16_t tx = act ? C_VOID : C_MUTED_TEXT;
    
    canvas.fillRect(btnX, btnY, btnW, btnH, bg);
    canvas.drawRect(btnX, btnY, btnW, btnH, act ? sensorColors[m] : C_STEEL);
    
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(tx, bg);
    canvas.setTextDatum(middle_center);
    canvas.drawString(mLabels[m], btnX + btnW/2, btnY + btnH/2 - 1);
  }

  // Aggressive LIVE/HALTED block
  int pauseX = SCR_W - 110, pauseY = BODY_Y + 6;
  uint16_t pCol = graphPaused ? C_ROG_RED : C_VOLT_GREEN;
  uint16_t pBg  = graphPaused ? 0x2000 : C_VOID;
  
  canvas.fillRect(pauseX, pauseY, 100, 26, pBg);
  canvas.drawRect(pauseX, pauseY, 100, 26, pCol);
  
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextColor(pCol, pBg);
  canvas.setTextDatum(middle_center);
  canvas.drawString(graphPaused ? "HALTED" : "LIVE", pauseX + 50, pauseY + 12);
  
  if (!graphPaused) {
    uint16_t dotC = ((millis()/100)%2==0) ? C_VOLT_GREEN : C_VOID; // High speed strobe
    canvas.fillRect(pauseX + 88, pauseY + 9, 6, 8, dotC);
  }

  int graphY = BODY_Y + 38;
  canvas.drawFastHLine(0, graphY - 1, SCR_W, C_STEEL);
  int graphH = BODY_H - 42;

  // Single Sensor Focus (Tactical HUD Strip)
  if (graphMode >= 1 && graphMode <= 4) {
    int si = graphMode - 1;
    if (sData[si].active) {
      SensorInfo &sd = sData[si];
      char infoBuf[80];
      
      canvas.setFont(&fonts::Font0);
      canvas.setTextDatum(top_left);

      // Deep Metal Data Strip 
      canvas.fillRect(4, graphY + 4, SCR_W - 8, 18, C_DARK_METAL);
      canvas.drawRect(4, graphY + 4, SCR_W - 8, 18, C_STEEL);
      
      canvas.setTextColor(C_CYAN, C_DARK_METAL);
      snprintf(infoBuf, 80, ">> P%d_VOUT: %05.2fV", si+1, sd.ch[1].V);
      canvas.drawString(infoBuf, 10, graphY + 8);

      canvas.setTextColor(C_HAZARD, C_DARK_METAL);
      snprintf(infoBuf, 80, "CURR: %05.1fmA", sd.ch[1].I);
      canvas.drawString(infoBuf, 140, graphY + 8);

      canvas.setTextColor(C_VOLT_GREEN, C_DARK_METAL);
      snprintf(infoBuf, 80, "RT: %05.1f", sd.RT);
      canvas.drawString(infoBuf, 250, graphY + 8);

      canvas.setTextColor(C_CYAN, C_DARK_METAL);
      snprintf(infoBuf, 80, "ST: %05.1f", sd.ST);
      canvas.drawString(infoBuf, 320, graphY + 8);

      canvas.setTextColor(C_ROG_RED, C_DARK_METAL);
      snprintf(infoBuf, 80, "FT: %05.1f", sd.FT);
      canvas.drawString(infoBuf, 390, graphY + 8);

      canvas.setTextColor(C_WHITE, C_DARK_METAL);
      snprintf(infoBuf, 80, "FRQ: %05.1fHz", sd.freq);
      canvas.drawString(infoBuf, 470, graphY + 8);

      canvas.setTextColor(C_CYBER_GOLD, C_DARK_METAL);
      snprintf(infoBuf, 80, "JITTER: sRT:%.2f sFT:%.2f", sd.sigmaRT, sd.sigmaFT);
      canvas.drawString(infoBuf, 570, graphY + 8);

      // NEW: Second HUD strip for advanced diagnostics (Category 3, 4A)
      canvas.fillRect(4, graphY + 26, SCR_W - 8, 18, 0x0421);
      canvas.drawRect(4, graphY + 26, SCR_W - 8, 18, C_STEEL);
      canvas.setTextColor(C_HAZARD, 0x0421);
      snprintf(infoBuf, 80, "INRUSH: %.1fmA", sd.inrush);
      canvas.drawString(infoBuf, 10, graphY + 30);
      canvas.setTextColor(C_ROG_RED, 0x0421);
      snprintf(infoBuf, 80, "LEAK: %.2fmA", sd.leakage);
      canvas.drawString(infoBuf, 135, graphY + 30);
      canvas.setTextColor(C_PURPLE, 0x0421);
      snprintf(infoBuf, 80, "RIPPLE: %.3fV", sd.ripple);
      canvas.drawString(infoBuf, 255, graphY + 30);
      canvas.setTextColor(C_HAZARD, 0x0421);
      snprintf(infoBuf, 80, "BOUNCE: %lu", sd.bounceCount);
      canvas.drawString(infoBuf, 380, graphY + 30);
      canvas.setTextColor(C_CYAN, 0x0421);
      snprintf(infoBuf, 80, "DUT: %.0f%%", sd.dutyCycle);
      canvas.drawString(infoBuf, 480, graphY + 30);
      canvas.setTextColor(C_GHOST_CYAN, 0x0421);
      snprintf(infoBuf, 80, "SAT: %.3fV  CUSUM-RT: %.2f", sd.satDrop_V, sd.cusumRT);
      canvas.drawString(infoBuf, 555, graphY + 30);

      drawGraph(canvas, si, 4, graphY + 48, SCR_W - 8, graphH - 48);
    } else {
      canvas.setFont(&fonts::FreeMonoBold9pt7b);
      canvas.setTextColor(C_ROG_RED, C_VOID);
      canvas.setTextDatum(middle_center);
      canvas.drawString(">> NO SIGNAL OVERRIDE <<", SCR_W/2, graphY + graphH/2);
    }
  } else if (graphMode == 5) {
    // SPLIT mode
    int halfH = (graphH - 6) / 2;
    drawGraph(canvas, graphCmpTop, 0, graphY, SCR_W, halfH);
    drawGraph(canvas, graphCmpBot, 0, graphY + halfH + 6, SCR_W, halfH);
  } else {
    // ALL 4 MODE
    for (int i = 0; i < 4; i++) {
      if (sData[i].active) drawGraph(canvas, i, 4, graphY, SCR_W - 8, graphH);
    }
  }
}

/* ------------------------------------------------------------------
 * TAB: MATRIX (12-Factor Compare & Linear Regression RUL)
 * ------------------------------------------------------------------ */

void drawTabDiagnose() {
  canvas.fillRect(0, BODY_Y, SCR_W, BODY_H, C_VOID);

  int btnY = BODY_Y + 6;

  // 1. LIVE vs FROM SD Toggle (Left Side)
  canvas.fillRect(8, btnY, 80, 26, cmpLiveMode ? C_CYAN : C_VOID);
  canvas.drawRect(8, btnY, 80, 26, cmpLiveMode ? C_CYAN : C_STEEL);
  canvas.setFont(&fonts::Font0); canvas.setTextDatum(middle_center);
  canvas.setTextColor(cmpLiveMode ? C_VOID : C_MUTED_TEXT, cmpLiveMode ? C_CYAN : C_VOID);
  canvas.drawString("LIVE FEED", 48, btnY + 13);

  canvas.fillRect(96, btnY, 80, 26, !cmpLiveMode ? C_PURPLE : C_VOID);
  canvas.drawRect(96, btnY, 80, 26, !cmpLiveMode ? C_PURPLE : C_STEEL);
  canvas.setTextColor(!cmpLiveMode ? C_VOID : C_MUTED_TEXT, !cmpLiveMode ? C_PURPLE : C_VOID);
  canvas.drawString("ARCHIVE", 136, btnY + 13);

  // 2. Sensor Selectors A and B (Middle)
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextColor(C_MUTED_TEXT, C_VOID);
  canvas.setTextDatum(middle_left);
  canvas.drawString("> TARGET A:", 185, btnY + 14);
  canvas.drawString("> TARGET B:", 355, btnY + 14);

  for (int i = 0; i < 4; i++) {
    int axX = 280 + i * 16;
    bool actA = (i == cmpSensorA);
    canvas.fillRect(axX, btnY + 4, 14, 20, actA ? C_CYAN : C_VOID);
    canvas.drawRect(axX, btnY + 4, 14, 20, actA ? C_CYAN : C_STEEL);
    canvas.setFont(&fonts::Font0); canvas.setTextColor(actA ? C_VOID : C_MUTED_TEXT, actA ? C_CYAN : C_VOID);
    canvas.setTextDatum(middle_center);
    char la[4]; snprintf(la, 4, "%d", i+1);
    canvas.drawString(la, axX + 7, btnY + 14);

    int bxX = 450 + i * 16;
    bool actB = (i == cmpSensorB);
    canvas.fillRect(bxX, btnY + 4, 14, 20, actB ? C_PURPLE : C_VOID);
    canvas.drawRect(bxX, btnY + 4, 14, 20, actB ? C_PURPLE : C_STEEL);
    canvas.setTextColor(actB ? C_VOID : C_MUTED_TEXT, actB ? C_PURPLE : C_VOID);
    canvas.drawString(la, bxX + 7, btnY + 14);
  }

  // 3. SD Date Picker (Right Side)
  if (!cmpLiveMode) {
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(C_MUTED_TEXT, C_VOID);
    canvas.setTextDatum(middle_left);
    canvas.drawString("> DATE:", 530, btnY + 14);

    canvas.fillRect(590, btnY + 2, 110, 24, C_DARK_METAL);
    canvas.drawRect(590, btnY + 2, 110, 24, C_STEEL);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(cmpDataLoaded ? C_VOLT_GREEN : C_MUTED_TEXT, C_DARK_METAL);
    canvas.setTextDatum(middle_center);
    canvas.drawString(strlen(cmpLoadedDate) ? cmpLoadedDate : "TAP TO PICK", 645, btnY + 14);

    canvas.fillRect(710, btnY + 2, 80, 24, C_CYAN);
    canvas.drawRect(710, btnY + 2, 80, 24, C_WHITE);
    canvas.setTextColor(C_VOID, C_CYAN);
    canvas.drawString("LOAD SD", 750, btnY + 14);
  }

  int tableY = BODY_Y + 38;
  int rowH = 20;
  int colFactor = 6, colA = 200, colB = 350, colDelta = 500, colStatus = 640;

  // Terminal Header
  canvas.fillRect(0, tableY, SCR_W, rowH, C_STEEL);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(C_VOID, C_STEEL);
  canvas.drawString("DIAGNOSTIC METRIC", colFactor, tableY + rowH/2);
  char aLbl[12], bLbl[12]; snprintf(aLbl, 12, "TARGET A:P%d", cmpSensorA+1); snprintf(bLbl, 12, "TARGET B:P%d", cmpSensorB+1);
  canvas.drawString(aLbl, colA, tableY + rowH/2);
  canvas.drawString(bLbl, colB, tableY + rowH/2);
  canvas.drawString("DELTA", colDelta, tableY + rowH/2);
  canvas.drawString("SYSTEM STATUS", colStatus, tableY + rowH/2);

  SensorInfo &sa = sData[cmpSensorA];
  SensorInfo &sb = sData[cmpSensorB];

  float valsA[12] = { sa.RT, sa.FT, sa.ST, sa.freq, sa.vPeak, sa.vFloor, sa.vPeak-sa.vFloor, sa.RT+sa.FT, (sa.medFT>0.001f)?sa.medRT/sa.medFT:1.0f, sa.sigmaRT, sa.sigmaFT, (float)sa.spikeCount };
  float valsB[12] = { sb.RT, sb.FT, sb.ST, sb.freq, sb.vPeak, sb.vFloor, sb.vPeak-sb.vFloor, sb.RT+sb.FT, (sb.medFT>0.001f)?sb.medRT/sb.medFT:1.0f, sb.sigmaRT, sb.sigmaFT, (float)sb.spikeCount };

  for (int f = 0; f < 12; f++) {
    int ry = tableY + rowH + f * rowH;
    uint16_t rowBg = (f % 2 == 0) ? C_DARK_METAL : C_VOID;
    canvas.fillRect(0, ry, SCR_W, rowH, rowBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(GROUP_COLORS[FACTOR_GROUP[f]], rowBg);
    canvas.drawString(FACTOR_NAMES[f], colFactor, ry + rowH/2);

    char vA[16], vB[16], vD[16];
    snprintf(vA, 16, "%06.2f %s", valsA[f], FACTOR_UNITS[f]);
    snprintf(vB, 16, "%06.2f %s", valsB[f], FACTOR_UNITS[f]);
    float delta = valsB[f] - valsA[f];
    snprintf(vD, 16, "%+06.2f", delta);

    canvas.setTextColor(C_CYAN, rowBg);   canvas.drawString(vA, colA, ry + rowH/2);
    canvas.setTextColor(C_PURPLE, rowBg); canvas.drawString(vB, colB, ry + rowH/2);

    // Aggressive Threat Highlighting
    uint16_t dCol = (fabsf(delta) < 0.1f) ? C_VOLT_GREEN : (fabsf(delta) < 1.0f) ? C_HAZARD : C_ROG_RED;
    canvas.setTextColor(dCol, rowBg); canvas.drawString(vD, colDelta, ry + rowH/2);

    const char* status = (fabsf(delta) < 0.1f) ? "[ SYNC OK ]" : (fabsf(delta) < 1.0f) ? "[ WARNING ]" : "[ DIVERGENCE ]";
    canvas.setTextColor(dCol, rowBg); canvas.drawString(status, colStatus, ry + rowH/2);
  }

  // RUL Prediction Sub-Console
  int predY = tableY + rowH + 12 * rowH + 6;
  runLifePrediction(cmpSensorA);
  runLifePrediction(cmpSensorB);

  canvas.fillRect(0, predY, SCR_W, 46, C_VOID);
  canvas.drawFastHLine(0, predY, SCR_W, C_STEEL);
  canvas.drawFastHLine(0, predY+2, SCR_W, C_STEEL);
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextDatum(middle_left);

  for (int si = 0; si < 2; si++) {
    int idx = (si == 0) ? cmpSensorA : cmpSensorB;
    uint16_t col = (si == 0) ? C_CYAN : C_PURPLE;
    int tx = 10 + si * 400;
    
    if (cmpPredValid[idx]) {
      char pStr[80];
      if (cmpPredSlope[idx] > 0.001f) {
        snprintf(pStr, sizeof(pStr), "P%d: REGRESSION +%.3fms/c | RUL: %ld CYC", idx+1, cmpPredSlope[idx], max(0L, cmpPredCycles[idx]));
      } else {
        snprintf(pStr, sizeof(pStr), "P%d: REGRESSION STABLE | RUL: OPTIMAL", idx+1);
      }
      canvas.setTextColor(C_VOID, col);
      canvas.fillRect(tx, predY+14, 380, 20, col); // Solid block highlight
      canvas.drawString(pStr, tx+4, predY + 24);
    } else {
      char pStr[48]; snprintf(pStr, sizeof(pStr), "P%d: AWAITING LINEAR REGRESSION...", idx+1);
      canvas.setTextColor(C_MUTED_TEXT, C_VOID);
      canvas.drawString(pStr, tx, predY + 24);
    }
  }
}

/* ------------------------------------------------------------------
 * TAB: LEDGER (Merged Cycle Logs & Critical Alerts)
 * ------------------------------------------------------------------ */

void drawTabLogs() {
  canvas.fillRect(0, BODY_Y, SCR_W, BODY_H, C_VOID);

  // 1. VIEW MODE TOGGLE
  int togY = BODY_Y + 8;
  bool isCycles = (logViewMode == 0);
  
  // Cycles Button
  canvas.fillRect(8, togY, 180, 32, isCycles ? C_CYAN : C_VOID);
  canvas.drawRect(8, togY, 180, 32, isCycles ? C_CYAN : C_STEEL);
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextColor(isCycles ? C_VOID : C_MUTED_TEXT, isCycles ? C_CYAN : C_VOID);
  canvas.setTextDatum(middle_center);
  canvas.drawString("CYCLE LEDGER", 98, togY + 15);

  // Alerts Button
  canvas.fillRect(196, togY, 180, 32, !isCycles ? C_ROG_RED : C_VOID);
  canvas.drawRect(196, togY, 180, 32, !isCycles ? C_ROG_RED : C_STEEL);
  canvas.setTextColor(!isCycles ? C_VOID : C_MUTED_TEXT, !isCycles ? C_ROG_RED : C_VOID);
  canvas.drawString("CRITICAL ALERTS", 286, togY + 15);

  canvas.drawFastHLine(0, togY + 40, SCR_W, C_STEEL);

  // =========================================================
  // VIEW MODE 0: CYCLE LEDGER (History Matrix)
  // =========================================================
  if (isCycles) {
    uint16_t sColors[] = { C_CYAN, C_CYAN, C_VOLT_GREEN, C_HAZARD, C_PURPLE };
    int btnW = 56, btnH = 24, btnY = togY + 48;

    for (int f = -1; f < 4; f++) {
      int btnX = 8 + (f + 1) * (btnW + 4);
      bool act = (f == logFilterSensor);
      uint16_t bg = act ? sColors[f+1] : C_VOID;
      uint16_t tx = act ? C_VOID : C_MUTED_TEXT;
      
      canvas.fillRect(btnX, btnY, btnW, btnH, bg);
      canvas.drawRect(btnX, btnY, btnW, btnH, act ? sColors[f+1] : C_STEEL);
      canvas.setTextColor(tx, bg);
      char lbl[8]; if (f == -1) snprintf(lbl, 8, "ALL"); else snprintf(lbl, 8, "P%d", f+1);
      canvas.drawString(lbl, btnX + btnW/2, btnY + btnH/2 - 1);
    }

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(C_CYAN, C_VOID);
    canvas.setTextDatum(middle_right);
    char cntStr[32]; snprintf(cntStr, 32, "[ MEMORY: %d / %d ]", historyCount, MAX_HISTORY);
    canvas.drawString(cntStr, SCR_W - 32, btnY + 12);

    int tableY = btnY + 32;
    int rowH = 24;
    int col[] = { 4, 46, 90, 170, 240, 310, 380, 460, 540, 620 };
    const char* hdr[] = { "ID","PRT","TIME","RT","ST","FT","FRQ","V_MIN","V_MAX","STATUS" };

    canvas.fillRect(0, tableY, SCR_W - 30, rowH, C_STEEL);
    canvas.drawFastHLine(0, tableY, SCR_W - 30, C_CYAN);
    canvas.drawFastHLine(0, tableY + rowH, SCR_W - 30, C_CYAN);
    
    canvas.setTextDatum(middle_left);
    for (int c = 0; c < 10; c++) {
      canvas.setTextColor(C_VOID, C_STEEL); 
      canvas.drawString(hdr[c], col[c] + 4, tableY + rowH/2);
    }

    int visibleRows = (BODY_H - (tableY - BODY_Y) - rowH - 4) / rowH;
    int dataY = tableY + rowH + 4;

    if (historyCount == 0) {
      canvas.setFont(&fonts::FreeMonoBold9pt7b);
      canvas.setTextColor(C_MUTED_TEXT, C_VOID);
      canvas.setTextDatum(middle_center);
      canvas.drawString(">> NO DATA IN LEDGER <<", SCR_W/2 - 15, dataY + 60);
      return;
    }

    static int filtIdx[MAX_HISTORY];
    int filtCount = 0;
    for (int i = 0; i < historyCount; i++) {
      int ri = (historyHead - historyCount + i + MAX_HISTORY) % MAX_HISTORY;
      if (logFilterSensor == -1 || cycleHistory[ri].sensor == logFilterSensor) filtIdx[filtCount++] = ri;
    }

    int maxScroll = max(0, filtCount - visibleRows);
    logScrollOffset = constrain(logScrollOffset, 0, maxScroll);

    for (int row = 0; row < visibleRows && row + logScrollOffset < filtCount; row++) {
      int ri = filtIdx[filtCount - 1 - (row + logScrollOffset)];
      CycleRecord &r = cycleHistory[ri];
      int ry = dataY + row * rowH;
      
      uint16_t rowBg = r.ng ? 0x3000 : ((row % 2 == 0) ? C_VOID : C_DARK_METAL);
      canvas.fillRect(0, ry, SCR_W - 30, rowH - 2, rowBg);
      
      if (r.ng) {
        canvas.drawRect(0, ry, SCR_W - 30, rowH - 2, C_ROG_RED);
        canvas.drawLine(0, ry, 6, ry+6, C_ROG_RED); 
      }

      canvas.setFont(&fonts::Font0);
      canvas.setTextDatum(middle_left);
      char tmp[24];

      snprintf(tmp, 24, "%lu", r.cycleNum); canvas.setTextColor(C_MUTED_TEXT, rowBg); canvas.drawString(tmp, col[0]+4, ry+rowH/2 - 1);
      snprintf(tmp, 24, "P%d", r.sensor+1); uint16_t sCol[] = { C_CYAN, C_VOLT_GREEN, C_HAZARD, C_PURPLE }; canvas.setTextColor(sCol[r.sensor], rowBg); canvas.drawString(tmp, col[1]+4, ry+rowH/2 - 1);
      snprintf(tmp, 24, "%lu", r.ts/1000);  canvas.setTextColor(C_MUTED_TEXT, rowBg); canvas.drawString(tmp, col[2]+4, ry+rowH/2 - 1);
      snprintf(tmp, 24, "%.1f", r.RT);      canvas.setTextColor(C_VOLT_GREEN, rowBg); canvas.drawString(tmp, col[3]+4, ry+rowH/2 - 1);
      snprintf(tmp, 24, "%.1f", r.ST);      canvas.setTextColor(C_CYAN, rowBg);       canvas.drawString(tmp, col[4]+4, ry+rowH/2 - 1);
      snprintf(tmp, 24, "%.1f", r.FT);      canvas.setTextColor(C_ROG_RED, rowBg);    canvas.drawString(tmp, col[5]+4, ry+rowH/2 - 1);
      snprintf(tmp, 24, "%.1f", r.freq);    canvas.setTextColor(C_WHITE, rowBg);      canvas.drawString(tmp, col[6]+4, ry+rowH/2 - 1);
      snprintf(tmp, 24, "%.1f", r.vFloor);  canvas.setTextColor(C_GHOST_CYAN, rowBg); canvas.drawString(tmp, col[7]+4, ry+rowH/2 - 1);
      snprintf(tmp, 24, "%.1f", r.vPeak);   canvas.setTextColor(C_GHOST_CYAN, rowBg); canvas.drawString(tmp, col[8]+4, ry+rowH/2 - 1);

      const char* stTxt = r.ng ? "[FAULT]" : "[ OK ]";
      uint16_t stTx = r.ng ? C_ROG_RED : C_VOLT_GREEN;
      canvas.setTextColor(stTx, rowBg); canvas.drawString(stTxt, col[9]+4, ry+rowH/2 - 1);
    }

    int scrollX = SCR_W - 24;
    uint16_t upCol = logScrollOffset > 0 ? C_CYAN : C_STEEL;
    canvas.fillRect(scrollX, dataY, 20, 30, C_VOID);
    canvas.drawRect(scrollX, dataY, 20, 30, upCol);
    canvas.fillTriangle(scrollX+10, dataY+8, scrollX+4, dataY+22, scrollX+16, dataY+22, upCol);

    uint16_t dnCol = logScrollOffset < maxScroll ? C_CYAN : C_STEEL;
    canvas.fillRect(scrollX, dataY + 36, 20, 30, C_VOID);
    canvas.drawRect(scrollX, dataY + 36, 20, 30, dnCol);
    canvas.fillTriangle(scrollX+4, dataY+36+8, scrollX+16, dataY+36+8, scrollX+10, dataY+36+22, dnCol);
  } 
  
  // =========================================================
  // VIEW MODE 1: CRITICAL ALERTS
  // =========================================================
  else {
    bool anyAlarm = false;
    for (int i = 0; i < 4; i++) if (sData[i].alarmActive) { anyAlarm = true; break; }

    bool muteActive = buzzerMuted && (millis() < buzzerMuteEnd);
    int silX = SCR_W - 160, silY = togY, silW = 150, silH = 32;
    uint16_t silBg = muteActive ? 0x2000 : C_VOID;
    uint16_t silBdr = muteActive ? C_ROG_RED : C_HAZARD;
    
    canvas.fillRect(silX, silY, silW, silH, silBg);
    canvas.drawRect(silX, silY, silW, silH, silBdr);
    canvas.drawLine(silX, silY, silX+8, silY+8, silBdr); 
    
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(silBdr, silBg);
    canvas.setTextDatum(middle_center);
    if (muteActive) {
      unsigned long remSec = (buzzerMuteEnd - millis()) / 1000;
      char mStr[24]; snprintf(mStr, sizeof(mStr), "MUTED: %03lus", remSec);
      canvas.drawString(mStr, silX + silW/2, silY + silH/2 - 1);
    } else {
      canvas.drawString("MUTE ALARM", silX + silW/2, silY + silH/2 - 1);
    }

    if (!anyAlarm) {
      int cy = BODY_Y + 100;
      int boxX = 140, boxW = SCR_W - 280;
      canvas.fillRect(boxX, cy, boxW, 80, C_VOID);
      canvas.drawRect(boxX, cy, boxW, 80, C_VOLT_GREEN);
      
      canvas.setFont(&fonts::FreeSansBold12pt7b);
      canvas.setTextColor(C_VOLT_GREEN, C_VOID);
      canvas.setTextDatum(middle_center);
      canvas.drawString("ALL CLEAR", SCR_W/2, cy + 28);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(C_CYAN, C_VOID);
      canvas.drawString("No active system threats detected.", SCR_W/2, cy + 56);

      if (alarmHistCount > 0) {
        int hy = cy + 94;
        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(C_STEEL, C_VOID);
        canvas.setTextDatum(top_left);
        canvas.drawString("Recent Threat History", 10, hy);
        hy += 14;
        int shown = 0;
        for (int i = alarmHistCount - 1; i >= 0 && shown < 8; i--) {
          int ri = (alarmHistHead - alarmHistCount + i + MAX_ALARM_HISTORY) % MAX_ALARM_HISTORY;
          AlarmRecord &ar = alarmHistory[ri];
          char line[96];
          snprintf(line, sizeof(line), ">> P%d | %s | %s", ar.sensor + 1, ar.time, ar.cause);
          canvas.setTextColor(C_MUTED_TEXT, C_VOID);
          canvas.drawString(line, 10, hy);
          hy += 12; shown++;
        }
      }
      return;
    }

    int hdrY = togY + 48;
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(C_ROG_RED, C_VOID);
    canvas.setTextDatum(middle_left);
    canvas.drawString("ACTIVE THREATS", 10, hdrY + 12);

    int rowY = hdrY + 40;
    int rowH = 60;
    for (int i = 0; i < 4; i++) {
      if (!sData[i].alarmActive) continue;
      SensorInfo &s = sData[i];
      
      uint16_t blinkBg = ((millis()/300)%2==0) ? 0x4000 : 0x1000;
      canvas.fillRect(8, rowY, SCR_W - 140, rowH - 4, blinkBg);
      canvas.drawRect(8, rowY, SCR_W - 140, rowH - 4, C_ROG_RED);
      canvas.drawFastVLine(12, rowY+4, rowH-12, C_VOID);

      canvas.fillRect(18, rowY + 10, 44, 36, C_ROG_RED);
      canvas.setFont(&fonts::FreeSansBold12pt7b);
      canvas.setTextColor(C_VOID, C_ROG_RED);
      canvas.setTextDatum(middle_center);
      char sb[4]; snprintf(sb, 4, "P%d", i+1);
      canvas.drawString(sb, 40, rowY + 28);

      canvas.setFont(&fonts::FreeMonoBold9pt7b);
      canvas.setTextColor(C_HAZARD, blinkBg);
      canvas.setTextDatum(bottom_left);
      canvas.drawString(s.alarmCause, 75, rowY + 28);

      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(C_WHITE, blinkBg);
      canvas.setTextDatum(top_left);
      char tStr[64]; snprintf(tStr, 64, "T:%s | HLTH:%03.0f%% | R:%.1f S:%.1f F:%.1f", s.alarmTime, s.healthScore, s.RT, s.ST, s.FT);
      canvas.drawString(tStr, 75, rowY + 34);

      // Aggressive ACK button
      int ackX = SCR_W - 120, ackY = rowY + 4, ackW = 100, ackH = 48;
      canvas.fillRect(ackX, ackY, ackW, ackH, C_VOID);
      canvas.drawRect(ackX, ackY, ackW, ackH, C_ROG_RED);
      for(int hx=0; hx<ackW; hx+=6) canvas.drawLine(ackX+hx, ackY+ackH, ackX+hx+6, ackY, 0x4000); 
      
      canvas.setFont(&fonts::FreeSansBold9pt7b);
      canvas.setTextColor(C_WHITE, C_VOID); 
      canvas.setTextDatum(middle_center);
      canvas.drawString("CLEAR", ackX + ackW/2, ackY + ackH/2);

      rowY += rowH + 4;
      if (rowY + rowH > BODY_Y + BODY_H - FOOTER_H - 10) break;
    }
  }
}

/* ------------------------------------------------------------------
 * TAB: ALIGNMENT (Formerly Calibrate)
 * ------------------------------------------------------------------ */

void drawTabCalibrate() {
  canvas.fillRect(0, BODY_Y, SCR_W, BODY_H, C_VOID);
  SensorInfo &sd = sData[calibSelectedSensor];

  int btnW = 80, btnH = 30, btnY = BODY_Y + 8;
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextColor(C_MUTED_TEXT, C_VOID);
  canvas.setTextDatum(middle_left);
  canvas.drawString("> TARGET:", 10, btnY + btnH/2);

  uint16_t cSColors[] = { C_CYAN, C_CYAN, C_VOLT_GREEN, C_HAZARD };
  for (int i = 0; i < 4; i++) {
    int btnX = 120 + i * (btnW + 6);
    bool act = (i == calibSelectedSensor);
    canvas.fillRect(btnX, btnY, btnW, btnH, act ? cSColors[i] : C_VOID);
    canvas.drawRect(btnX, btnY, btnW, btnH, act ? cSColors[i] : C_STEEL);
    canvas.setTextColor(act ? C_VOID : C_MUTED_TEXT, act ? cSColors[i] : C_VOID);
    canvas.setTextDatum(middle_center);
    char lbl[8]; snprintf(lbl, sizeof(lbl), "P%d", i+1);
    canvas.drawString(lbl, btnX + btnW/2, btnY + btnH/2 - 1);
  }

  // Type Selector
  int tY = btnY + btnH + 10;
  canvas.setTextColor(C_MUTED_TEXT, C_VOID); canvas.setTextDatum(middle_left);
  canvas.drawString("> LOGIC:", 10, tY + btnH/2);
  for(int t=0; t<5; t++) {
    int btnX = 90 + t * 100; bool act = (sd.sType == t);
    canvas.fillRect(btnX, tY, 90, 26, act ? C_PURPLE : C_VOID); 
    canvas.drawRect(btnX, tY, 90, 26, act ? C_PURPLE : C_STEEL);
    canvas.setFont(&fonts::Font0); 
    canvas.setTextColor(act ? C_VOID : C_MUTED_TEXT, act ? C_PURPLE : C_VOID); 
    canvas.setTextDatum(middle_center);
    canvas.drawString(SENSOR_TYPE_NAMES[t], btnX + 45, tY + 13);
  }

  // Tactical Signal Diagram (ROG Styling)
  int diagY = tY + 40; int diagH = 140; int diagW = SCR_W - 20; int diagX = 10;
  canvas.fillRect(diagX, diagY, diagW, diagH, C_DARK_METAL); 
  canvas.drawRect(diagX, diagY, diagW, diagH, C_CYAN);
  
  // Crosshatch Grid
  for(int x=diagX; x<diagX+diagW; x+=20) canvas.drawFastVLine(x, diagY+1, diagH-2, 0x18C3);
  for(int y=diagY; y<diagY+diagH; y+=20) canvas.drawFastHLine(diagX+1, y, diagW-2, 0x18C3);

  int yFloor = diagY + diagH - 30; int yPeak = diagY + 40;
  int x0 = diagX + 20, x1 = x0 + 60, x2 = x1 + 60, x3 = x2 + 80, x4 = x3 + 280, x5 = x4 + 80, x6 = x5 + 60, x7 = x6 + 60;
  int yLeak = yFloor - 15; // Leak sits slightly above floor

  // 5-Stage Signal Wireframe using correct state colors
  canvas.drawLine(x0, yFloor, x1, yFloor, C_WHITE);        // IDLE  (state 0) — WHITE
  canvas.drawLine(x1, yFloor, x2, yLeak,  C_YELLOW);       // LEAK  (state 1) — YELLOW
  canvas.drawLine(x2, yLeak,  x3, yLeak,  C_YELLOW);       // LEAK hold       — YELLOW
  canvas.drawLine(x3, yLeak,  x3+5, yPeak, C_VOLT_GREEN);  // RISE  (state 2) — GREEN
  canvas.drawLine(x3+5,yPeak, x4, yPeak,  C_CYAN);         // STABLE(state 3) — CYAN
  canvas.drawLine(x4, yPeak,  x5, yFloor, C_ROG_RED);      // FALL  (state 4) — RED
  canvas.drawLine(x5, yFloor, x7, yFloor, C_WHITE);        // Return IDLE     — WHITE

  // Stage label tags
  canvas.setFont(&fonts::FreeMonoBold9pt7b); canvas.setTextDatum(middle_center);
  char tBuf[32];

  canvas.setTextColor(C_WHITE, C_VOID);   canvas.drawString("IDLE", x0 + 30, yFloor + 14);
  canvas.setTextColor(C_YELLOW, C_VOID);  canvas.drawString("LEAK", x2 - 10, yLeak - 13);
  canvas.setTextColor(C_VOLT_GREEN, C_VOID); snprintf(tBuf,32,"RT:%.1fms", sd.RT); canvas.drawString(tBuf, x3 - 20, yFloor - 18);
  canvas.setTextColor(C_CYAN, C_VOID);    snprintf(tBuf,32,"ST:%.1fms", sd.ST); canvas.drawString(tBuf, x3 + 140, yPeak + 14);
  canvas.setTextColor(C_ROG_RED, C_VOID); snprintf(tBuf,32,"FT:%.1fms", sd.FT); canvas.drawString(tBuf, x4 + 50, yFloor - 18);

  canvas.setTextColor(C_YELLOW, C_VOID);
  snprintf(tBuf,32,"VF:%.2fV", sd.vFloor); canvas.drawString(tBuf, x2, yLeak + 14);
  canvas.setTextColor(C_CYAN, C_VOID);
  snprintf(tBuf,32,"VP:%.2fV", sd.vPeak); canvas.drawString(tBuf, x3 + 50, yPeak - 13);

  // Status Banner
  int stY = diagY + diagH + 10;
  canvas.setTextColor(sd.calibReady ? C_VOLT_GREEN : C_HAZARD, C_VOID); canvas.setTextDatum(middle_left);
  canvas.drawString(sd.calibReady ? "> ALIGNMENT COMPLETE. CIRCUIT LOCKED." : "> AWAITING RAW TELEMETRY FROM CIRCUIT...", 10, stY + 10);

  // Massive Commit Button
  int actY = BODY_Y + BODY_H - 50;
  canvas.fillRect(10, actY, SCR_W-20, 40, calibActionPulse ? C_WHITE : C_CYAN);
  canvas.drawRect(10, actY, SCR_W-20, 40, C_WHITE);
  canvas.setTextColor(C_VOID, calibActionPulse ? C_WHITE : C_CYAN);
  canvas.setTextDatum(middle_center);
  canvas.drawString("SAVE BASELINE TO CIRCUIT", SCR_W/2, actY + 20);
  if(calibActionPulse) calibActionPulse = false;
}

/* ------------------------------------------------------------------
 * TAB: SYSTEM CONFIG (Aggressive Settings Matrix)
 * ------------------------------------------------------------------ */

void drawTabConfig() {
  canvas.fillRect(0, BODY_Y, SCR_W, BODY_H, C_VOID);

  int selBtnW = 80, selBtnH = 26, selY = BODY_Y + 8;
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextColor(C_MUTED_TEXT, C_VOID);
  canvas.setTextDatum(middle_left);
  canvas.drawString("> TARGET:", 10, selY + selBtnH/2);

  uint16_t sColors[] = { C_CYAN, C_CYAN, C_VOLT_GREEN, C_HAZARD };
  for (int i = 0; i < 4; i++) {
    int btnX = 120 + i * (selBtnW + 8);
    bool act = (i == configSelectedSensor);
    canvas.fillRect(btnX, selY, selBtnW, selBtnH, act ? sColors[i] : C_VOID);
    canvas.drawRect(btnX, selY, selBtnW, selBtnH, act ? sColors[i] : C_STEEL);
    // Chamfered corners
    uint16_t ch = act ? sColors[i] : C_STEEL;
    canvas.drawLine(btnX,          selY,          btnX+5,          selY+5,          ch);
    canvas.drawLine(btnX+selBtnW,  selY,          btnX+selBtnW-5,  selY+5,          ch);
    canvas.drawLine(btnX,          selY+selBtnH,  btnX+5,          selY+selBtnH-5,  ch);
    canvas.drawLine(btnX+selBtnW,  selY+selBtnH,  btnX+selBtnW-5,  selY+selBtnH-5,  ch);
    canvas.setTextColor(act ? C_VOID : C_MUTED_TEXT, act ? sColors[i] : C_VOID);
    canvas.setTextDatum(middle_center);
    char lbl[4]; snprintf(lbl, 4, "P%d", i+1);
    canvas.drawString(lbl, btnX + selBtnW/2, selY + selBtnH/2 - 1);
  }

  // Feedback Toggle
  SensorInfo &s = sData[configSelectedSensor];
  int fbX = SCR_W - 200;
  canvas.fillRect(fbX, selY, 180, selBtnH, s.feedbackEnabled ? C_VOLT_GREEN : C_ROG_RED);
  canvas.drawRect(fbX, selY, 180, selBtnH, C_WHITE);
  canvas.setTextColor(C_VOID, s.feedbackEnabled ? C_VOLT_GREEN : C_ROG_RED);
  canvas.drawString(s.feedbackEnabled ? "FEEDBACK: ON" : "FEEDBACK: OFF", fbX + 90, selY + 12);

  int grpY = BODY_Y + 42;
  int grpW = (SCR_W - 20) / 4;
  for (int g = 0; g < 4; g++) {
    int gx = 4 + g * (grpW + 4);
    bool act = (g == settingsGroup);
    uint16_t bg = act ? GROUP_COLORS[g] : C_VOID;
    canvas.fillRect(gx, grpY, grpW, 26, bg);
    canvas.drawRect(gx, grpY, grpW, 26, act ? GROUP_COLORS[g] : C_STEEL);
    // Chamfered corners
    uint16_t gc = act ? GROUP_COLORS[g] : C_STEEL;
    canvas.drawLine(gx,       grpY,    gx+5,    grpY+5,   gc);
    canvas.drawLine(gx+grpW,  grpY,    gx+grpW-5,grpY+5,  gc);
    canvas.drawLine(gx,       grpY+26, gx+5,    grpY+21,  gc);
    canvas.drawLine(gx+grpW,  grpY+26, gx+grpW-5,grpY+21, gc);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(act ? C_VOID : C_MUTED_TEXT, act ? GROUP_COLORS[g] : C_VOID);
    canvas.setTextDatum(middle_center);
    canvas.drawString(GROUP_NAMES[g], gx + grpW/2, grpY + 13);
  }

  int gridY = grpY + 36;
  int fieldW = (SCR_W - 24) / 3 - 4;
  int fieldH = 60;
  int fieldGap = 8;
  int factorIdx = 0;

  for (int f = 0; f < 12; f++) {
    if (FACTOR_GROUP[f] != settingsGroup) continue;
    int row = factorIdx / 3; int col = factorIdx % 3;
    int fx = 8 + col * (fieldW + fieldGap);
    int fy = gridY + row * (fieldH * 3 + 16);

    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(GROUP_COLORS[settingsGroup], C_VOID);
    canvas.setTextDatum(top_left);
    canvas.drawString(FACTOR_NAMES[f], fx, fy);
    canvas.drawFastHLine(fx, fy+14, fieldW, C_STEEL);
    fy += 20;

    const char* subLabels[] = { "[MIN]", "[MAX]", "[LIMIT]" };
    float subVals[] = { s.alarms[f].minVal, s.alarms[f].maxVal, (float)s.alarms[f].cntLimit };

    for (int sub = 0; sub < 3; sub++) {
      int sy = fy + sub * (fieldH + 6);
      canvas.fillRect(fx, sy, fieldW, fieldH, C_VOID);
      canvas.drawRect(fx, sy, fieldW, fieldH, GROUP_COLORS[settingsGroup]);
      // Sharp chamfered corners — all 4, 8px
      uint16_t cc = GROUP_COLORS[settingsGroup];
      canvas.drawLine(fx,         sy,         fx+8,         sy+8,         cc);
      canvas.drawLine(fx+fieldW,  sy,         fx+fieldW-8,  sy+8,         cc);
      canvas.drawLine(fx,         sy+fieldH,  fx+8,         sy+fieldH-8,  cc);
      canvas.drawLine(fx+fieldW,  sy+fieldH,  fx+fieldW-8,  sy+fieldH-8,  cc);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(C_CYAN, C_VOID);
      canvas.setTextDatum(top_center);
      canvas.drawString(subLabels[sub], fx + fieldW/2, sy + 4);
      canvas.setFont(&fonts::FreeSansBold12pt7b);
      canvas.setTextColor(C_WHITE, C_VOID);
      canvas.setTextDatum(middle_center);
      char valStr[16];
      if (sub == 2) snprintf(valStr, 16, "%02.0f", subVals[sub]);
      else snprintf(valStr, 16, "%.1f %s", subVals[sub], FACTOR_UNITS[f]);
      canvas.drawString(valStr, fx + fieldW/2, sy + fieldH/2 + 4);
    }
    factorIdx++;
  }

  int actY = BODY_Y + BODY_H - 42;
  int abW = (SCR_W - 32) / 3;

  // RESET LIMITS
  canvas.fillRect(8, actY, abW, 36, C_VOID);
  canvas.drawRect(8, actY, abW, 36, C_ROG_RED);
  canvas.drawLine(8,      actY,    8+7,    actY+7,    C_ROG_RED);
  canvas.drawLine(8+abW,  actY,    8+abW-7,actY+7,    C_ROG_RED);
  canvas.drawLine(8,      actY+36, 8+7,    actY+29,   C_ROG_RED);
  canvas.drawLine(8+abW,  actY+36, 8+abW-7,actY+29,   C_ROG_RED);
  canvas.setFont(&fonts::FreeMonoBold9pt7b); canvas.setTextColor(C_ROG_RED, C_VOID); canvas.setTextDatum(middle_center);
  canvas.drawString("RESET LIMITS", 8 + abW/2, actY + 17);

  // CLEAR DATA
  canvas.fillRect(16+abW, actY, abW, 36, C_VOID);
  canvas.drawRect(16+abW, actY, abW, 36, C_HAZARD);
  canvas.drawLine(16+abW,      actY,    16+abW+7,    actY+7,    C_HAZARD);
  canvas.drawLine(16+abW*2,    actY,    16+abW*2-7,  actY+7,    C_HAZARD);
  canvas.drawLine(16+abW,      actY+36, 16+abW+7,    actY+29,   C_HAZARD);
  canvas.drawLine(16+abW*2,    actY+36, 16+abW*2-7,  actY+29,   C_HAZARD);
  canvas.setTextColor(C_HAZARD, C_VOID);
  canvas.drawString("CLEAR DATA", 16 + abW + abW/2, actY + 17);

  // LOAD FROM FLASH
  canvas.fillRect(24+abW*2, actY, abW, 36, C_VOLT_GREEN);
  canvas.drawRect(24+abW*2, actY, abW, 36, C_WHITE);
  canvas.drawLine(24+abW*2,      actY,    24+abW*2+7,    actY+7,    C_WHITE);
  canvas.drawLine(24+abW*3,      actY,    24+abW*3-7,    actY+7,    C_WHITE);
  canvas.drawLine(24+abW*2,      actY+36, 24+abW*2+7,    actY+29,   C_WHITE);
  canvas.drawLine(24+abW*3,      actY+36, 24+abW*3-7,    actY+29,   C_WHITE);
  canvas.setTextColor(C_VOID, C_VOLT_GREEN);
  canvas.drawString("LOAD FROM FLASH", 24 + abW*2 + abW/2, actY + 17);
}

/* ------------------------------------------------------------------
 * TAB: STORAGE — Hierarchical SD Navigator
 *   VIPx3x4s > PlantA/B > Line1-3 > Option7/70/700 > System1-4 > SensorID
 * ------------------------------------------------------------------ */

// Helper: draw a grid of selector buttons, returns nothing
static void archDrawGrid(const char** labels, int count, int selected,
                         uint16_t activeCol, int startY, int btnH,
                         int marginX, int gap) {
  int totalW = SCR_W - marginX * 2;
  int cols = (count <= 4) ? count : (count <= 6 ? 3 : 4);
  int rows = (count + cols - 1) / cols;
  int btnW = (totalW - gap * (cols - 1)) / cols;
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextDatum(middle_center);
  for (int i = 0; i < count; i++) {
    int col = i % cols, row = i / cols;
    int bx = marginX + col * (btnW + gap);
    int by = startY + row * (btnH + gap);
    bool act = (i == selected);
    canvas.fillRect(bx, by, btnW, btnH, act ? activeCol : C_DARK_METAL);
    canvas.drawRect(bx, by, btnW, btnH, act ? activeCol : C_STEEL);
    canvas.setTextColor(act ? C_VOID : C_MUTED_TEXT, act ? activeCol : C_DARK_METAL);
    canvas.drawString(labels[i], bx + btnW / 2, by + btnH / 2);
  }
}

void drawTabArchive() {
  canvas.fillRect(0, BODY_Y, SCR_W, BODY_H, C_VOID);
  const int MX = 14, GAP = 6, BUTH = 48;
  const int CRUMB_Y = BODY_Y + 8;
  const int GRID_Y  = CRUMB_Y + 34;

  // ── Breadcrumb Path Bar ────────────────────────────────────────────
  canvas.fillRect(MX, CRUMB_Y, SCR_W - MX * 2, 26, C_DARK_METAL);
  canvas.drawRect(MX, CRUMB_Y, SCR_W - MX * 2, 26, C_STEEL);
  char crumb[128] = "/VIPx3x4s";
  const char* plants[]  = { "PlantA", "PlantB" };
  const char* lines[]   = { "Line1",  "Line2",  "Line3" };
  const char* options[] = { "Option7", "Option70", "Option700" };
  const char* systems[] = { "System1", "System2", "System3", "System4" };
  if (archNavPlant  >= 0) { strcat(crumb, "/"); strcat(crumb, plants[archNavPlant]); }
  if (archNavLine   >= 0) { strcat(crumb, "/"); strcat(crumb, lines[archNavLine]);   }
  if (archNavOption >= 0) { strcat(crumb, "/"); strcat(crumb, options[archNavOption]); }
  if (archNavSystem >= 0) { strcat(crumb, "/"); strcat(crumb, systems[archNavSystem]); }
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(C_CYAN, C_DARK_METAL);
  canvas.setTextDatum(middle_left);
  canvas.drawString(crumb, MX + 8, CRUMB_Y + 13);

  // ── Back Button (shown when not at root) ──────────────────────────
  if (archNavLevel > 0) {
    int bkW = 80, bkH = 26;
    int bkX = SCR_W - MX - bkW;
    canvas.fillRect(bkX, CRUMB_Y, bkW, bkH, C_STEEL);
    canvas.drawRect(bkX, CRUMB_Y, bkW, bkH, C_MUTED_TEXT);
    canvas.setTextColor(C_WHITE, C_STEEL);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("< BACK", bkX + bkW / 2, CRUMB_Y + 13);
  }

  // ── Level Title + Grid ────────────────────────────────────────────
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextDatum(middle_left);
  char title[48];

  if (archNavLevel == 0) {
    snprintf(title, sizeof(title), "SELECT PLANT:");
    canvas.setTextColor(C_MUTED_TEXT, C_VOID);
    canvas.drawString(title, MX, GRID_Y - 10);
    archDrawGrid(plants, 2, archNavPlant, C_CYAN, GRID_Y + 4, BUTH, MX, GAP);

  } else if (archNavLevel == 1) {
    snprintf(title, sizeof(title), "SELECT LINE [%s]:", plants[archNavPlant]);
    canvas.setTextColor(C_MUTED_TEXT, C_VOID);
    canvas.drawString(title, MX, GRID_Y - 10);
    archDrawGrid(lines, 3, archNavLine, C_VOLT_GREEN, GRID_Y + 4, BUTH, MX, GAP);

  } else if (archNavLevel == 2) {
    snprintf(title, sizeof(title), "SELECT OPTION:");
    canvas.setTextColor(C_MUTED_TEXT, C_VOID);
    canvas.drawString(title, MX, GRID_Y - 10);
    archDrawGrid(options, 3, archNavOption, C_PURPLE, GRID_Y + 4, BUTH, MX, GAP);

  } else if (archNavLevel == 3) {
    snprintf(title, sizeof(title), "SELECT SYSTEM:");
    canvas.setTextColor(C_MUTED_TEXT, C_VOID);
    canvas.drawString(title, MX, GRID_Y - 10);
    archDrawGrid(systems, 4, archNavSystem, C_HAZARD, GRID_Y + 4, BUTH, MX, GAP);

  } else if (archNavLevel == 4) {
    // ── Level 4: Sensor ID entry + data preview + SAVE ─────────────
    snprintf(title, sizeof(title), "ENTER SENSOR ID:");
    canvas.setTextColor(C_MUTED_TEXT, C_VOID);
    canvas.drawString(title, MX, GRID_Y - 10);

    // Sensor ID input box
    int idBx = MX, idBy = GRID_Y + 4, idBw = SCR_W - MX * 2, idBh = 52;
    bool npActive = (npMode == NP_STORAGE);
    canvas.fillRect(idBx, idBy, idBw, idBh, C_DARK_METAL);
    canvas.drawRect(idBx, idBy, idBw, idBh, npActive ? C_CYAN : C_STEEL);
    if (npActive) canvas.drawRect(idBx + 2, idBy + 2, idBw - 4, idBh - 4, C_CYAN);
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(C_WHITE, C_DARK_METAL);
    canvas.setTextDatum(middle_left);
    char idDisp[20];
    const char* showID = npActive ? npValue : archSensorID;
    snprintf(idDisp, sizeof(idDisp), "%s%s", showID, npActive ? "_" : "");
    canvas.drawString(showID[0] ? idDisp : (npActive ? "_" : "TAP TO ENTER ID"),
                      idBx + 14, idBy + idBh / 2);

    // Live data preview for selected port (Port 1 by default, or archiveSelectedSensor)
    int prevY = idBy + idBh + 10;
    int prevH = 108;
    SensorInfo &s = sData[archiveSelectedSensor];
    canvas.fillRect(MX, prevY, SCR_W - MX * 2, prevH, C_DARK_METAL);
    canvas.drawRect(MX, prevY, SCR_W - MX * 2, prevH, C_STEEL);
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(C_CYAN, C_DARK_METAL);
    canvas.setTextDatum(top_left);
    char pbuf[80];
    snprintf(pbuf, sizeof(pbuf), "LIVE DATA — PORT P%d  |  PATH: %s", archiveSelectedSensor + 1, crumb);
    canvas.drawString(pbuf, MX + 8, prevY + 8);
    canvas.drawFastHLine(MX, prevY + 26, SCR_W - MX * 2, C_STEEL);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(C_VOLT_GREEN, C_DARK_METAL);
    snprintf(pbuf, sizeof(pbuf), "HEALTH: %.0f%%   STATUS: %s", s.healthScore, s.cycleNG ? "FAULT" : "NOMINAL");
    canvas.drawString(pbuf, MX + 8, prevY + 34);
    canvas.setTextColor(C_CYAN, C_DARK_METAL);
    snprintf(pbuf, sizeof(pbuf), "RT:%.2fms  ST:%.2fms  FT:%.2fms  FREQ:%.1fHz", s.RT, s.ST, s.FT, s.freq);
    canvas.drawString(pbuf, MX + 8, prevY + 52);
    canvas.setTextColor(C_WHITE, C_DARK_METAL);
    snprintf(pbuf, sizeof(pbuf), "Vin:%.2fV  Vout:%.2fV  Inrush:%.1fmA  Leak:%.2fmA", s.ch[0].V, s.ch[1].V, s.inrush, s.leakage);
    canvas.drawString(pbuf, MX + 8, prevY + 70);

    // Port selector row (tiny)
    const char* portLbls[] = { "P1", "P2", "P3", "P4" };
    int pBtnW = 50, pBtnH = 26, pGap = 6;
    int pTotalW = 4 * pBtnW + 3 * pGap, pStartX = (SCR_W - pTotalW) / 2;
    int pBtnY = prevY + prevH + 8;
    canvas.setFont(&fonts::Font0);
    for (int i = 0; i < 4; i++) {
      int px = pStartX + i * (pBtnW + pGap);
      bool act = (i == archiveSelectedSensor);
      canvas.fillRect(px, pBtnY, pBtnW, pBtnH, act ? C_CYAN : C_DARK_METAL);
      canvas.drawRect(px, pBtnY, pBtnW, pBtnH, act ? C_CYAN : C_STEEL);
      canvas.setTextColor(act ? C_VOID : C_MUTED_TEXT, act ? C_CYAN : C_DARK_METAL);
      canvas.setTextDatum(middle_center);
      canvas.drawString(portLbls[i], px + pBtnW / 2, pBtnY + pBtnH / 2);
    }

    // SAVE button
    int svY = pBtnY + pBtnH + 10;
    int svW = SCR_W - MX * 2, svH = 46;
    bool saved = archSaveSuccess && (millis() - archSaveFlash < 1500);
    canvas.fillRect(MX, svY, svW, svH, saved ? C_VOLT_GREEN : C_VOID);
    canvas.drawRect(MX, svY, svW, svH, saved ? C_WHITE : C_CYAN);
    canvas.drawRect(MX + 2, svY + 2, svW - 4, svH - 4, saved ? C_WHITE : C_CYAN);
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(saved ? C_VOID : C_CYAN, saved ? C_VOLT_GREEN : C_VOID);
    canvas.setTextDatum(middle_center);
    canvas.drawString(saved ? "SAVED TO SD" : "SAVE TO SD", SCR_W / 2, svY + svH / 2);
    if (archSaveSuccess && !saved) archSaveSuccess = false;
  }

  // ── SD Status Bar ─────────────────────────────────────────────────
  int stBarY = BODY_Y + BODY_H - 22;
  canvas.drawFastHLine(0, stBarY, SCR_W, C_STEEL);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(sdReady ? C_VOLT_GREEN : C_ROG_RED, C_VOID);
  canvas.drawString(sdReady ? "SD: MOUNTED  |  /VIPx3x4s  READY" : "SD: NOT FOUND — INSERT CARD", 10, stBarY + 11);
}

void drawBody() {
  switch (currentTab) {
    case TAB_HOME:      drawTabHome();      break;
    case TAB_CALIBRATE: drawTabCalibrate(); break;
    case TAB_LIVE:      drawTabLive();      break;
    case TAB_DIAGNOSE:  drawTabDiagnose();  break;
    case TAB_LOGS:      drawTabLogs();      break;
    case TAB_CONFIG:    drawTabConfig();    break;
    case TAB_ARCHIVE:   drawTabArchive();   break;
  }
}

/* ------------------------------------------------------------------
 * LINEAR REGRESSION ENGINE (RUL Prediction)
 * ------------------------------------------------------------------ */

void runLifePrediction(int idx) {
  if (idx < 0 || idx > 3) return;
  if (historyCount < 10) { cmpPredValid[idx] = false; return; }
  
  int count = 0;
  float sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
  
  for (int i = 0; i < historyCount; i++) {
    int ri = (historyHead - historyCount + i + MAX_HISTORY) % MAX_HISTORY;
    if (cycleHistory[ri].sensor != idx) continue;
    float x = (float)count;
    float y = cycleHistory[ri].RT;
    sumX += x; sumY += y; sumXX += x * x; sumXY += x * y;
    count++;
  }
  
  if (count < 5) { cmpPredValid[idx] = false; return; }
  
  float denom = (count * sumXX - sumX * sumX);
  if (fabsf(denom) < 0.001f) { cmpPredValid[idx] = false; return; }
  
  float slope  = (count * sumXY - sumX * sumY) / denom;
  float intercept = (sumY - slope * sumX) / count;
  cmpPredSlope[idx]  = slope;
  cmpPredValid[idx]  = true;
  
  // Estimate cycles to RT hitting user max limit (alarms[0].maxVal)
  float limitRT = sData[idx].alarms[0].maxVal;
  if (limitRT > 0.1f && slope > 0.0001f) {
    long cyclesToLimit = (long)((limitRT - intercept) / slope);
    cmpPredCycles[idx] = max(0L, cyclesToLimit - (long)count);
  } else {
    cmpPredCycles[idx] = 9999999L;
  }
}

/* ------------------------------------------------------------------
 * FIELD SELECTOR MODAL (Tap panel to configure display fields)
 * ------------------------------------------------------------------ */

void drawFieldSelector(int sensorIdx) {
  fieldSelectorActive  = true;
  fieldSelectorSensor  = sensorIdx;
  needFullRedraw       = true;
}

void drawFieldSelectorOverlay() {
  int mW = 400, mH = 260;
  int mX = (SCR_W - mW) / 2, mY = (SCR_H - mH) / 2;
  
  // ROG-style border: outer crimson glow
  canvas.fillRect(mX - 6, mY - 6, mW + 12, mH + 12, C_CRIMSON);
  canvas.fillRect(mX - 4, mY - 4, mW + 8, mH + 8, C_VOID);
  canvas.drawRect(mX - 4, mY - 4, mW + 8, mH + 8, C_CYAN);
  canvas.fillRect(mX, mY, mW, mH, C_VOID);
  canvas.drawRect(mX, mY, mW, mH, C_STEEL);
  // L-bracket corner accents
  canvas.drawFastHLine(mX+3, mY+3, 10, C_CYAN);
  canvas.drawFastVLine(mX+3, mY+3, 10, C_CYAN);
  canvas.drawFastHLine(mX+mW-13, mY+3, 10, C_ROG_RED);
  canvas.drawFastVLine(mX+mW-3, mY+3, 10, C_ROG_RED);
  
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextColor(C_CYAN, C_VOID);
  canvas.setTextDatum(top_center);
  char titBuf[32]; snprintf(titBuf, sizeof(titBuf), "PORT %d — DISPLAY CONFIG", fieldSelectorSensor + 1);
  canvas.drawString(titBuf, mX + mW/2, mY + 10);
  canvas.drawFastHLine(mX + 10, mY + 28, mW - 20, C_STEEL);
  
  // Toggle: Graph vs Telemetry
  int togY = mY + 40;
  bool showG = homeShowGraph[fieldSelectorSensor];
  
  canvas.fillRect(mX + 20, togY, 160, 36, showG ? C_CYAN : C_VOID);
  canvas.drawRect(mX + 20, togY, 160, 36, showG ? C_WHITE : C_STEEL);
  canvas.setTextColor(showG ? C_VOID : C_MUTED_TEXT, showG ? C_CYAN : C_VOID);
  canvas.setTextDatum(middle_center);
  canvas.drawString("WAVEFORM", mX + 100, togY + 18);
  
  canvas.fillRect(mX + 200, togY, 160, 36, !showG ? C_PURPLE : C_VOID);
  canvas.drawRect(mX + 200, togY, 160, 36, !showG ? C_WHITE : C_STEEL);
  canvas.setTextColor(!showG ? C_VOID : C_MUTED_TEXT, !showG ? C_PURPLE : C_VOID);
  canvas.drawString("TELEMETRY", mX + 280, togY + 18);

  // Alarm ACK for this sensor
  int ackY = togY + 60;
  bool hasAlarm = sData[fieldSelectorSensor].alarmActive;
  canvas.fillRect(mX + 20, ackY, mW - 40, 36, hasAlarm ? C_ROG_RED : C_DARK_METAL);
  canvas.drawRect(mX + 20, ackY, mW - 40, 36, hasAlarm ? C_WHITE : C_STEEL);
  canvas.setTextColor(hasAlarm ? C_WHITE : C_MUTED_TEXT, hasAlarm ? C_ROG_RED : C_DARK_METAL);
  canvas.setTextDatum(middle_center);
  canvas.drawString(hasAlarm ? "ACKNOWLEDGE & CLEAR ALARM" : "NO ACTIVE ALARM", mX + mW/2, ackY + 18);

  // Close button
  int closeY = mY + mH - 50;
  canvas.fillRect(mX + 20, closeY, mW - 40, 36, C_VOID);
  canvas.drawRect(mX + 20, closeY, mW - 40, 36, C_STEEL);
  canvas.setTextColor(C_MUTED_TEXT, C_VOID);
  canvas.drawString("[ CLOSE ]", mX + mW/2, closeY + 18);
}

/* ------------------------------------------------------------------
 * UNIVERSAL NUMPAD (Overlay for Config settings entry)
 * ------------------------------------------------------------------ */

void applyNumpadValue() {
  if (npLen <= 0) { npMode = NP_NONE; needFullRedraw = true; return; }

  if (npMode == NP_STORAGE) {
    // Commit sensor ID from numpad into archSensorID
    strncpy(archSensorID, npValue, 15); archSensorID[15] = '\0';
    archSensorIDLen = strlen(archSensorID);
    npMode = NP_NONE; npLen = 0; npValue[0] = '\0'; needFullRedraw = true;
    return;
  }

  float val = atof(npValue);
  
  if (npMode == NP_SETTINGS) {
    // npTargetField groups are in order: RT(0..2), ST(3..5), FT(6..8)
    int typeIdx = npTargetField / 3; // 0=RT, 1=ST, 2=FT (circuit expects this)
    int sub = npTargetField % 3;     // 0=min, 1=max, 2=cnt

    // Display alarm array order: alarms[0]=RT, alarms[1]=FT, alarms[2]=ST
    int alarmIdx = 0;
    if (typeIdx == 0) alarmIdx = 0;        // RT
    else if (typeIdx == 1) alarmIdx = 2;   // ST
    else alarmIdx = 1;                     // FT

    FactorAlarm &fa = sData[npTargetSensor].alarms[alarmIdx];
    if (sub == 0) fa.minVal = val;
    else if (sub == 1) fa.maxVal = val;
    else fa.cntLimit = (uint8_t)constrain((int)val, 0, 255);
    saveSettingsToSD(npTargetSensor);
    sendConfig(npTargetSensor + 1, typeIdx, fa.minVal, fa.maxVal, (int)fa.cntLimit);
  }
  
  npMode = NP_NONE; npLen = 0; npValue[0] = '\0'; needFullRedraw = true;
}

void drawUniversalNumpad() {
  int npW = 300, npH = 360;
  int npX = (SCR_W - npW) / 2, npY = (SCR_H - npH) / 2;
  
  // ROG-style border: outer crimson glow + inner frame
  canvas.fillRect(npX - 6, npY - 6, npW + 12, npH + 12, C_CRIMSON);
  canvas.fillRect(npX - 4, npY - 4, npW + 8, npH + 8, C_VOID);
  canvas.drawRect(npX - 4, npY - 4, npW + 8, npH + 8, C_CYAN);
  canvas.fillRect(npX, npY, npW, npH, C_VOID);
  canvas.drawRect(npX, npY, npW, npH, C_STEEL);
  // L-bracket corner accents
  canvas.drawFastHLine(npX+3, npY+3, 10, C_CYAN);
  canvas.drawFastVLine(npX+3, npY+3, 10, C_CYAN);
  canvas.drawFastHLine(npX+npW-13, npY+3, 10, C_ROG_RED);
  canvas.drawFastVLine(npX+npW-3, npY+3, 10, C_ROG_RED);
  canvas.drawFastHLine(npX+3, npY+npH-3, 10, C_ROG_RED);
  canvas.drawFastVLine(npX+3, npY+npH-13, 10, C_ROG_RED);
  canvas.drawFastHLine(npX+npW-13, npY+npH-3, 10, C_CYAN);
  canvas.drawFastVLine(npX+npW-3, npY+npH-13, 10, C_CYAN);
  
  // Display title
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(C_CYAN, C_VOID);
  canvas.setTextDatum(top_center);
  const char* npTitle = (npMode == NP_SETTINGS) ? numpadFieldNames[npTargetField % 9]
                      : (npMode == NP_STORAGE)  ? "ENTER SENSOR ID"
                      : "ENTER VALUE";
  canvas.drawString(npTitle, npX + npW/2, npY + 6);
  
  // Value display box (angular)
  canvas.fillRect(npX + 10, npY + 20, npW - 20, 36, C_PANEL_BG);
  canvas.drawRect(npX + 10, npY + 20, npW - 20, 36, C_CYAN);
  canvas.setFont(&fonts::FreeSansBold12pt7b);
  canvas.setTextColor(C_WHITE, C_PANEL_BG);
  canvas.setTextDatum(middle_center);
  canvas.drawString(npLen > 0 ? npValue : "0", npX + npW/2, npY + 38);
  
  // Numpad keys with angular styling
  const char* keys[] = { "7","8","9", "4","5","6", "1","2","3", ".","0","<" };
  int kW = (npW - 20) / 3, kH = 50, kGap = 4;
  
  for (int i = 0; i < 12; i++) {
    int col = i % 3, row = i / 3;
    int kx = npX + 10 + col * (kW + kGap);
    int ky = npY + 70 + row * (kH + kGap);
    
    bool isDel = (i == 11);
    canvas.fillRect(kx, ky, kW, kH, isDel ? 0x4000 : C_PANEL_BG);
    canvas.drawRect(kx, ky, kW, kH, isDel ? C_ROG_RED : C_PANEL_BORDER);
    // Chamfered corners — all 4, 6px cut
    uint16_t chamCol = isDel ? C_ROG_RED : C_STEEL;
    canvas.drawLine(kx,      ky,      kx+6,    ky+6,    chamCol);
    canvas.drawLine(kx+kW,   ky,      kx+kW-6, ky+6,    chamCol);
    canvas.drawLine(kx,      ky+kH,   kx+6,    ky+kH-6, chamCol);
    canvas.drawLine(kx+kW,   ky+kH,   kx+kW-6, ky+kH-6, chamCol);
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(isDel ? C_ROG_RED : C_WHITE, isDel ? 0x4000 : C_PANEL_BG);
    canvas.setTextDatum(middle_center);
    canvas.drawString(keys[i], kx + kW/2, ky + kH/2);
  }
  
  // ENTER button (green, chamfered all 4 corners)
  int btY = npY + npH - 54;
  int halfW = (npW - 24) / 2;
  canvas.fillRect(npX + 10, btY, halfW, 44, C_VOLT_GREEN);
  canvas.drawRect(npX + 10, btY, halfW, 44, C_WHITE);
  canvas.drawLine(npX+10,        btY,      npX+16,        btY+6,    C_WHITE);
  canvas.drawLine(npX+10+halfW,  btY,      npX+10+halfW-6,btY+6,    C_WHITE);
  canvas.drawLine(npX+10,        btY+44,   npX+16,        btY+38,   C_WHITE);
  canvas.drawLine(npX+10+halfW,  btY+44,   npX+10+halfW-6,btY+38,   C_WHITE);
  canvas.setTextColor(C_VOID, C_VOLT_GREEN);
  canvas.setTextDatum(middle_center);
  canvas.drawString("ENTER", npX + 10 + halfW/2, btY + 22);
  
  // CANCEL button (red outline, chamfered all 4 corners)
  int cancelX = npX + 14 + halfW;
  canvas.fillRect(cancelX, btY, halfW, 44, C_VOID);
  canvas.drawRect(cancelX, btY, halfW, 44, C_ROG_RED);
  canvas.drawLine(cancelX,        btY,      cancelX+6,        btY+6,    C_ROG_RED);
  canvas.drawLine(cancelX+halfW,  btY,      cancelX+halfW-6,  btY+6,    C_ROG_RED);
  canvas.drawLine(cancelX,        btY+44,   cancelX+6,        btY+38,   C_ROG_RED);
  canvas.drawLine(cancelX+halfW,  btY+44,   cancelX+halfW-6,  btY+38,   C_ROG_RED);
  canvas.setTextColor(C_ROG_RED, C_VOID);
  canvas.drawString("CANCEL", cancelX + halfW/2, btY + 22);
}

/* ------------------------------------------------------------------
 * PER-TAB TOUCH HANDLERS
 * ------------------------------------------------------------------ */

void handleHomeTouch(int tx, int ty) {
  ty -= BODY_Y; if (ty < 0 || ty >= BODY_H) return;
  int gap = 6, h = (BODY_H - gap) / 2, w = (SCR_W - gap) / 2;
  int row = (ty < h) ? 0 : 1;
  int col = (tx < w) ? 0 : 1;
  int sIdx = row * 2 + col;
  
  if (sIdx >= 0 && sIdx < 4 && sData[sIdx].active) {
    int px = col * (w + gap);
    int py = row * (h + gap);
    
    // Check toggle button (WAVEFORM / TELEMETRY)
    int tglW = 70; int tglX = px + w/2 - tglW/2; int tglY = py + 8;
    if (tx >= tglX && tx <= tglX + tglW && ty >= tglY && ty <= tglY + 20) {
      homeShowGraph[sIdx] = !homeShowGraph[sIdx];
      needFullRedraw = true;
      return;
    }
    
    // Anywhere else on panel → open field selector modal
    drawFieldSelector(sIdx);
  }
}

void handleCalibrateTouch(int tx, int ty) {
  int btnW = 80, btnH = 30, btnY = BODY_Y + 8;
  // Sensor selector
  for (int i = 0; i < 4; i++) {
    int btnX = 120 + i * (btnW + 6);
    if (tx >= btnX && tx <= btnX + btnW && ty >= btnY && ty <= btnY + btnH) {
      calibSelectedSensor = i; needFullRedraw = true; return;
    }
  }
  // Type selector
  int tY = btnY + btnH + 10;
  for (int t = 0; t < 5; t++) {
    int btnX = 90 + t * 100;
    if (tx >= btnX && tx <= btnX + 90 && ty >= tY && ty <= tY + 26) {
      sData[calibSelectedSensor].sType = t;
      sendSensorType(calibSelectedSensor, t);
      needFullRedraw = true; return;
    }
  }
  // Commit / Reset Cal buttons
  int actY = BODY_Y + BODY_H - 50;
  if (ty >= actY && ty <= actY + 40) {
    calibActionPulse = true;
    // typeIdx: 0=RT, 1=ST, 2=FT (circuit expects)
    // Display alarm array order: alarms[0]=RT, alarms[1]=FT, alarms[2]=ST
    const int alarmIdxByType[3] = { 0, 2, 1 }; // {RT, ST, FT} -> {0,2,1}
    for (int typeIdx = 0; typeIdx < 3; typeIdx++) {
      int alarmIdx = alarmIdxByType[typeIdx];
      FactorAlarm &fa = sData[calibSelectedSensor].alarms[alarmIdx];
      sendConfig(calibSelectedSensor + 1, typeIdx, fa.minVal, fa.maxVal, (int)fa.cntLimit);
    }
    needFullRedraw = true;
  }
}

void handleLiveTouch(int tx, int ty) {
  int btnW = 60, btnH = 26, btnY = BODY_Y + 6;
  for (int m = 0; m <= 5; m++) {
    int btnX = 8 + m * (btnW + 6);
    if (tx >= btnX && tx <= btnX + btnW && ty >= btnY && ty <= btnY + btnH) {
      graphMode = m; needFullRedraw = true; return;
    }
  }
  // Pause/Resume
  int pauseX = SCR_W - 110;
  if (tx >= pauseX && tx <= pauseX + 100 && ty >= btnY && ty <= btnY + 26) {
    graphPaused = !graphPaused; needFullRedraw = true;
  }
}

void handleDiagnoseTouch(int tx, int ty) {
  int btnY = BODY_Y + 6;
  // Live / SD toggle
  if (tx >= 8 && tx <= 88 && ty >= btnY && ty <= btnY + 26) { cmpLiveMode = true; needFullRedraw = true; return; }
  if (tx >= 96 && tx <= 176 && ty >= btnY && ty <= btnY + 26) { cmpLiveMode = false; needFullRedraw = true; return; }
  // Sensor A selector
  for (int i = 0; i < 4; i++) {
    int ax = 280 + i * 16;
    if (tx >= ax && tx <= ax + 14 && ty >= btnY + 4 && ty <= btnY + 24) { cmpSensorA = i; needFullRedraw = true; return; }
    int bx = 450 + i * 16;
    if (tx >= bx && tx <= bx + 14 && ty >= btnY + 4 && ty <= btnY + 24) { cmpSensorB = i; needFullRedraw = true; return; }
  }
  // Load SD button
  if (!cmpLiveMode && tx >= 710 && tx <= 790 && ty >= btnY + 2 && ty <= btnY + 26) {
    loadCompareDataFromSD(); needFullRedraw = true;
  }
}

void handleLogsTouch(int tx, int ty) {
  int togY = BODY_Y + 8;
  // View mode toggle
  if (tx >= 8 && tx <= 188 && ty >= togY && ty <= togY + 32) { logViewMode = 0; logScrollOffset = 0; needFullRedraw = true; return; }
  if (tx >= 196 && tx <= 376 && ty >= togY && ty <= togY + 32) { logViewMode = 1; needFullRedraw = true; return; }
  
  if (logViewMode == 0) {
    // Filter buttons
    int btnW = 56, btnH = 24, btnY = togY + 48;
    for (int f = -1; f < 4; f++) {
      int btnX = 8 + (f + 1) * (btnW + 4);
      if (tx >= btnX && tx <= btnX + btnW && ty >= btnY && ty <= btnY + btnH) { logFilterSensor = f; logScrollOffset = 0; needFullRedraw = true; return; }
    }
    // Scroll
    int tableY = btnY + 32;
    int rowH = 24;
    int visibleRows = (BODY_H - (tableY - BODY_Y) - rowH - 4) / rowH;
    int scrollX = SCR_W - 24, dataY = tableY + rowH + 4;
    if (tx >= scrollX && tx <= scrollX + 20) {
      if (ty >= dataY && ty <= dataY + 30) { logScrollOffset = max(0, logScrollOffset - 1); needFullRedraw = true; }
      if (ty >= dataY + 36 && ty <= dataY + 66) { logScrollOffset++; needFullRedraw = true; }
    }
  } else {
    // Mute alarm button
    int silX = SCR_W - 160, silY = togY, silW = 150, silH = 32;
    if (tx >= silX && tx <= silX + silW && ty >= silY && ty <= silY + silH) {
      buzzerMuted = true; buzzerMuteEnd = millis() + 60000UL; needFullRedraw = true; return;
    }
    // ACK alarm clear buttons
    int hdrY = togY + 48;
    int rowY = hdrY + 40, rowH = 60;
    for (int i = 0; i < 4; i++) {
      if (!sData[i].alarmActive) continue;
      int ackX = SCR_W - 120, ackY = rowY + 4, ackW = 100, ackH = 48;
      if (tx >= ackX && tx <= ackX + ackW && ty >= ackY && ty <= ackY + ackH) {
        sData[i].alarmActive = false; sData[i].faultCode = FAULT_NONE;
        sData[i].alarmCause[0] = '\0';
        sendReset(i + 1);  // Tell the physical circuit board to disarm
        needFullRedraw = true;
      }
      rowY += rowH + 4;
    }
  }
}

void handleConfigTouch(int tx, int ty) {
  int selBtnW = 80, selBtnH = 26, selY = BODY_Y + 8;
  // Sensor selector
  for (int i = 0; i < 4; i++) {
    int btnX = 120 + i * (selBtnW + 8);
    if (tx >= btnX && tx <= btnX + selBtnW && ty >= selY && ty <= selY + selBtnH) { configSelectedSensor = i; needFullRedraw = true; return; }
  }
  // Feedback toggle
  int fbX = SCR_W - 200;
  if (tx >= fbX && tx <= fbX + 180 && ty >= selY && ty <= selY + selBtnH) {
    sData[configSelectedSensor].feedbackEnabled = !sData[configSelectedSensor].feedbackEnabled;
    int sid = configSelectedSensor + 1;
    int state = sData[configSelectedSensor].feedbackEnabled ? 1 : 0;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "$FB,%d,%d*", sid, state);
    Serial1.println(cmd);
    Serial1.flush();
    needFullRedraw = true;
    return;
  }
  // Group selector
  int grpY = BODY_Y + 42, grpW = (SCR_W - 20) / 4;
  for (int g = 0; g < 4; g++) {
    int gx = 4 + g * (grpW + 4);
    if (tx >= gx && tx <= gx + grpW && ty >= grpY && ty <= grpY + 26) { settingsGroup = g; needFullRedraw = true; return; }
  }
  // Action buttons (bottom)
  int actY = BODY_Y + BODY_H - 42;
  int abW = (SCR_W - 32) / 3;
  if (ty >= actY && ty <= actY + 36) {
    if (tx >= 8 && tx <= 8 + abW) {
      // Reset limits for active sensor & group
      for (int f = 0; f < 12; f++) if (FACTOR_GROUP[f] == settingsGroup) { sData[configSelectedSensor].alarms[f] = {0,0,0,0}; }
      saveSettingsToSD(configSelectedSensor); needFullRedraw = true;
    } else if (tx >= 16 + abW && tx <= 16 + abW * 2) {
      sendReset(configSelectedSensor + 1); sendCalReset(configSelectedSensor); needFullRedraw = true;
    } else if (tx >= 24 + abW * 2) {
      loadSettingsFromSD(configSelectedSensor); needFullRedraw = true;
    }
    return;
  }
  // Field tap → open numpad
  int gridY = grpY + 36;
  int fieldW = (SCR_W - 24) / 3 - 4;
  int fieldH = 60, fieldGap = 8;
  int factorIdx = 0;
  for (int f = 0; f < 12; f++) {
    if (FACTOR_GROUP[f] != settingsGroup) continue;
    int row = factorIdx / 3, col = factorIdx % 3;
    int fx = 8 + col * (fieldW + fieldGap);
    int fy = gridY + row * (fieldH * 3 + 16) + 20;
    for (int sub = 0; sub < 3; sub++) {
      int sy = fy + sub * (fieldH + 6);
      if (tx >= fx && tx <= fx + fieldW && ty >= sy && ty <= sy + fieldH) {
        npMode = NP_SETTINGS; npTargetSensor = configSelectedSensor;
        npTargetField = f * 3 + sub; npLen = 0; npValue[0] = '\0';
        needFullRedraw = true; return;
      }
    }
    factorIdx++;
  }
}

// Helper: hit-test a button grid, returns index or -1
static int archGridHit(int tx, int ty, int count, int startY, int btnH,
                       int marginX, int gap) {
  int totalW = SCR_W - marginX * 2;
  int cols = (count <= 4) ? count : (count <= 6 ? 3 : 4);
  int btnW = (totalW - gap * (cols - 1)) / cols;
  for (int i = 0; i < count; i++) {
    int col = i % cols, row = i / cols;
    int bx = marginX + col * (btnW + gap);
    int by = startY + row * (btnH + gap);
    if (tx >= bx && tx <= bx + btnW && ty >= by && ty <= by + btnH) return i;
  }
  return -1;
}

void saveToHierarchySD(int portIdx) {
  if (!sdReady) return;
  if (archNavSystem < 0 || archSensorIDLen == 0) return;
  const char* plants[]  = { "PlantA", "PlantB" };
  const char* lines[]   = { "Line1",  "Line2",  "Line3" };
  const char* options[] = { "Option7", "Option70", "Option700" };
  const char* systems[] = { "System1", "System2", "System3", "System4" };
  // Build full path
  char sysPath[100];
  snprintf(sysPath, sizeof(sysPath), "/VIPx3x4s/%s/%s/%s/%s",
           plants[archNavPlant], lines[archNavLine],
           options[archNavOption], systems[archNavSystem]);
  // Ensure directories exist (SD.mkdir is idempotent if already present)
  SD.mkdir(sysPath);

  updateSoftwareClock();
  char filePath[140];
  snprintf(filePath, sizeof(filePath), "%s/%s_%04d%02d%02d_%02d%02d%02d.txt",
           sysPath, archSensorID, dtYear, dtMonth, dtDay, dtHour, dtMin, dtSec);

  File f = SD.open(filePath, FILE_WRITE);
  if (!f) return;

  SensorInfo &s = sData[portIdx];
  f.println("===============================================");
  f.printf(" VIPx3X4s — SENSOR DATA RECORD\n");
  f.println("===============================================");
  f.printf(" DATE/TIME : %02d/%02d/%04d  %02d:%02d:%02d\n",
           dtDay, dtMonth, dtYear, dtHour, dtMin, dtSec);
  f.printf(" OPERATOR  : %s\n", operatorIDLen > 0 ? operatorID : "UNKNOWN");
  f.printf(" SENSOR ID : %s\n", archSensorID);
  f.printf(" PORT      : P%d\n", portIdx + 1);
  f.println("-----------------------------------------------");
  f.printf(" PATH      : %s/%s/%s/%s/%s\n",
           plants[archNavPlant], lines[archNavLine],
           options[archNavOption], systems[archNavSystem]);
  f.println("-----------------------------------------------");
  f.printf(" HEALTH    : %.0f%%\n", s.healthScore);
  f.printf(" STATUS    : %s\n", s.cycleNG ? "FAULT DETECTED" : "NOMINAL");
  f.println("-----------------------------------------------");
  f.printf(" [ TELEMETRY ]\n");
  f.printf(" INPUT     : %.3fV | %.2fmA | %.2fmW\n", s.ch[0].V, s.ch[0].I, s.ch[0].P);
  f.printf(" OUTPUT    : %.3fV | %.2fmA | %.2fmW\n", s.ch[1].V, s.ch[1].I, s.ch[1].P);
  f.printf(" FEEDBACK  : %.3fV | %.2fmA | %.2fmW\n", s.ch[2].V, s.ch[2].I, s.ch[2].P);
  f.println("-----------------------------------------------");
  f.printf(" [ KINETICS ]\n");
  f.printf(" RISE TIME : %.3fms  (σ=%.3f)\n", s.RT, s.sigmaRT);
  f.printf(" STBL TIME : %.3fms  (Ripple=%.3fV)\n", s.ST, s.ripple);
  f.printf(" FALL TIME : %.3fms  (σ=%.3f)\n", s.FT, s.sigmaFT);
  f.printf(" FREQUENCY : %.2fHz\n", s.freq);
  f.printf(" CYCLES    : %lu\n", s.cycles);
  f.println("-----------------------------------------------");
  f.printf(" [ ADVANCED ]\n");
  f.printf(" INRUSH    : %.2fmA\n", s.inrush);
  f.printf(" LEAKAGE   : %.3fmA\n", s.leakage);
  f.printf(" BOUNCES   : %lu\n", s.bounceCount);
  f.printf(" FAULT CODE: %d\n", s.faultCode);
  f.println("===============================================");
  f.close();
  archSaveSuccess = true;
  archSaveFlash   = millis();
}

void handleArchiveTouch(int tx, int ty) {
  const int MX = 14, GAP = 6, BUTH = 48;
  const int CRUMB_Y = BODY_Y + 8;
  const int GRID_Y  = CRUMB_Y + 34;

  // ── BACK button ───────────────────────────────────────────────────
  if (archNavLevel > 0) {
    int bkW = 80, bkH = 26, bkX = SCR_W - MX - bkW;
    if (tx >= bkX && tx <= bkX + bkW && ty >= CRUMB_Y && ty <= CRUMB_Y + bkH) {
      archNavLevel--;
      if (archNavLevel == 0) { archNavPlant  = -1; }
      else if (archNavLevel == 1) { archNavLine   = -1; }
      else if (archNavLevel == 2) { archNavOption = -1; }
      else if (archNavLevel == 3) { archNavSystem = -1; archSensorID[0] = '\0'; archSensorIDLen = 0; npMode = NP_NONE; }
      needFullRedraw = true; return;
    }
  }

  if (archNavLevel == 0) {
    int hit = archGridHit(tx, ty, 2, GRID_Y + 4, BUTH, MX, GAP);
    if (hit >= 0) { archNavPlant = hit; archNavLevel = 1; needFullRedraw = true; }

  } else if (archNavLevel == 1) {
    int hit = archGridHit(tx, ty, 3, GRID_Y + 4, BUTH, MX, GAP);
    if (hit >= 0) { archNavLine = hit; archNavLevel = 2; needFullRedraw = true; }

  } else if (archNavLevel == 2) {
    int hit = archGridHit(tx, ty, 3, GRID_Y + 4, BUTH, MX, GAP);
    if (hit >= 0) { archNavOption = hit; archNavLevel = 3; needFullRedraw = true; }

  } else if (archNavLevel == 3) {
    int hit = archGridHit(tx, ty, 4, GRID_Y + 4, BUTH, MX, GAP);
    if (hit >= 0) { archNavSystem = hit; archNavLevel = 4; needFullRedraw = true; }

  } else if (archNavLevel == 4) {
    // Sensor ID input box tap → open numpad
    int idBx = MX, idBy = GRID_Y + 4, idBw = SCR_W - MX * 2, idBh = 52;
    if (tx >= idBx && tx <= idBx + idBw && ty >= idBy && ty <= idBy + idBh) {
      npMode = NP_STORAGE; npLen = 0; npValue[0] = '\0';
      needFullRedraw = true; return;
    }
    // Port selector row
    int prevH = 108;
    int pBtnW = 50, pBtnH = 26, pGap = 6;
    int pTotalW = 4 * pBtnW + 3 * pGap, pStartX = (SCR_W - pTotalW) / 2;
    int pBtnY = idBy + idBh + 10 + prevH + 8;
    for (int i = 0; i < 4; i++) {
      int px = pStartX + i * (pBtnW + pGap);
      if (tx >= px && tx <= px + pBtnW && ty >= pBtnY && ty <= pBtnY + pBtnH) {
        archiveSelectedSensor = i; needFullRedraw = true; return;
      }
    }
    // SAVE button
    int svY = pBtnY + pBtnH + 10;
    int svW = SCR_W - MX * 2, svH = 46;
    if (tx >= MX && tx <= MX + svW && ty >= svY && ty <= svY + svH) {
      saveToHierarchySD(archiveSelectedSensor); needFullRedraw = true;
    }
  }
}

/* ------------------------------------------------------------------
 * NUMPAD & FIELD-SELECTOR TOUCH DISPATCHERS
 * ------------------------------------------------------------------ */

bool handleNumpadTouch(int tx, int ty) {
  if (npMode == NP_NONE) return false;
  int npW = 300, npH = 360;
  int npX = (SCR_W - npW) / 2, npY = (SCR_H - npH) / 2;
  if (tx < npX || tx > npX + npW || ty < npY || ty > npY + npH) return false;
  
  // Key grid
  const char* keys[] = { "7","8","9","4","5","6","1","2","3",".","0","<" };
  int kW = (npW - 20) / 3, kH = 50, kGap = 4;
  for (int i = 0; i < 12; i++) {
    int col = i % 3, row = i / 3;
    int kx = npX + 10 + col * (kW + kGap);
    int ky = npY + 70 + row * (kH + kGap);
    if (tx >= kx && tx <= kx + kW && ty >= ky && ty <= ky + kH) {
      if (i == 11) { // Backspace
        if (npLen > 0) { npLen--; npValue[npLen] = '\0'; }
      } else {
        if (npLen < 12) { npValue[npLen++] = keys[i][0]; npValue[npLen] = '\0'; }
      }
      needFullRedraw = true; return true;
    }
  }
  // Enter / Cancel
  int btY = npY + npH - 54;
  int halfW = (npW - 24) / 2;
  if (tx >= npX + 10 && tx <= npX + 10 + halfW && ty >= btY && ty <= btY + 44) {
    applyNumpadValue(); return true;
  }
  int cancelX = npX + 14 + halfW;
  if (tx >= cancelX && ty >= btY && ty <= btY + 44) {
    npMode = NP_NONE; npLen = 0; npValue[0] = '\0'; needFullRedraw = true; return true;
  }
  return true;
}

bool handleFieldSelectorTouch(int tx, int ty) {
  if (!fieldSelectorActive) return false;
  int mW = 400, mH = 260;
  int mX = (SCR_W - mW) / 2, mY = (SCR_H - mH) / 2;
  if (tx < mX || tx > mX + mW || ty < mY || ty > mY + mH) {
    fieldSelectorActive = false; needFullRedraw = true; return true;
  }
  int togY = mY + 40;
  // WAVEFORM
  if (tx >= mX + 20 && tx <= mX + 180 && ty >= togY && ty <= togY + 36) {
    homeShowGraph[fieldSelectorSensor] = true; needFullRedraw = true; return true;
  }
  // TELEMETRY
  if (tx >= mX + 200 && tx <= mX + 360 && ty >= togY && ty <= togY + 36) {
    homeShowGraph[fieldSelectorSensor] = false; needFullRedraw = true; return true;
  }
  // ACK alarm
  int ackY = togY + 60;
  if (tx >= mX + 20 && tx <= mX + mW - 20 && ty >= ackY && ty <= ackY + 36) {
    if (sData[fieldSelectorSensor].alarmActive) {
      sData[fieldSelectorSensor].alarmActive = false;
      sData[fieldSelectorSensor].faultCode = FAULT_NONE;
      sData[fieldSelectorSensor].alarmCause[0] = '\0';
      sendReset(fieldSelectorSensor + 1);  // Tell the physical circuit board to disarm
    }
    needFullRedraw = true; return true;
  }
  // Close
  int closeY = mY + mH - 50;
  if (ty >= closeY && ty <= closeY + 36) {
    fieldSelectorActive = false; needFullRedraw = true; return true;
  }
  return true;
}

/* ------------------------------------------------------------------
 * MASTER TOUCH DISPATCHER
 * ------------------------------------------------------------------ */

void handleTouch() {
  int tx, ty; 
  if (!display.getTouch(&tx, &ty)) return;
  
  // Modals consume all touch first
  if (npMode != NP_NONE) { handleNumpadTouch(tx, ty); return; }
  if (fieldSelectorActive) { handleFieldSelectorTouch(tx, ty); return; }
  
  // Tab bar (sits in HEADER zone, rows 44–80)
  if (ty >= HEADER_H && ty < BODY_Y) {
    int tabW = SCR_W / NUM_TABS;
    int t = tx / tabW;
    if (t >= 0 && t < NUM_TABS && t != currentTab) {
      currentTab = t; needFullRedraw = true; logScrollOffset = 0;
    }
    return;
  }
  
  // Body touch routing
  switch (currentTab) {
    case TAB_HOME:      handleHomeTouch(tx, ty);      break;
    case TAB_CALIBRATE: handleCalibrateTouch(tx, ty); break;
    case TAB_LIVE:      handleLiveTouch(tx, ty);      break;
    case TAB_DIAGNOSE:  handleDiagnoseTouch(tx, ty);  break;
    case TAB_LOGS:      handleLogsTouch(tx, ty);      break;
    case TAB_CONFIG:    handleConfigTouch(tx, ty);    break;
    case TAB_ARCHIVE:   handleArchiveTouch(tx, ty);   break;
  }
}

void runLoginAndDateTimeSetup() {
  int fieldValues[] = { dtDay, dtMonth, dtYear, dtHour, dtMin, dtSec };
  int fieldMax[] = { 31, 12, 2099, 23, 59, 59 };
  const char* fieldLabels[] = {"DD","MM","YYYY","HR","MIN","SEC"};
  int activeField = -1;
  bool numpadOpen = false;
  char npBuf[16] = "";
  int npBufLen = 0;
  
  auto drawScreen = [&]() {
    canvas.fillScreen(C_VOID);

    // ── OUTER FRAME — double border ──
    canvas.drawRect(4, 4, SCR_W-8, SCR_H-8, C_DARK_METAL);
    canvas.drawRect(6, 6, SCR_W-12, SCR_H-12, C_CYAN);

    // ── CORNER ACCENTS — clean angular L-brackets ──
    canvas.drawFastHLine(12, 12, 40, C_CYAN);
    canvas.drawFastVLine(12, 12, 40, C_CYAN);
    canvas.drawFastHLine(SCR_W-52, 12, 40, C_ROG_RED);
    canvas.drawFastVLine(SCR_W-12, 12, 40, C_ROG_RED);
    canvas.drawFastHLine(12, SCR_H-12, 40, C_ROG_RED);
    canvas.drawFastVLine(12, SCR_H-52, 40, C_ROG_RED);
    canvas.drawFastHLine(SCR_W-52, SCR_H-12, 40, C_CYAN);
    canvas.drawFastVLine(SCR_W-12, SCR_H-52, 40, C_CYAN);

    // ── TITLE BLOCK — centered ──
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextDatum(top_center);
    canvas.setTextColor(C_ROG_RED, C_VOID);
    canvas.drawString("[ VIPx3X4s ]", SCR_W/2-2, 18);
    canvas.setTextColor(C_CYAN, C_VOID);
    canvas.drawString("[ VIPx3X4s ]", SCR_W/2, 16);
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(C_CYAN, C_VOID);
    canvas.setTextDatum(top_center);
    canvas.drawString("{ M12 A-CODED SENSOR DIAGNOSE & MONITOR TOOL }", SCR_W/2, 78);
    canvas.drawFastHLine(SCR_W/2-180, 98, 360, C_STEEL);

    // ── OPERATOR ID FIELD — centered ──
    int idW = 440, idH = 44;
    int idX = (SCR_W - idW) / 2, idY = 126;
    bool idActive = (activeField == 6);
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(C_CYAN, C_VOID);
    canvas.setTextDatum(middle_center);
    canvas.drawString("> OPERATOR ID:", SCR_W/2, idY-13);
    canvas.fillRect(idX, idY, idW, idH, C_PANEL_BG);
    canvas.drawRect(idX, idY, idW, idH, idActive ? C_CYAN : C_PANEL_BORDER);
    if (idActive) canvas.drawRect(idX-2, idY-2, idW+4, idH+4, C_CYAN);
    uint16_t idCh = idActive ? C_CYAN : C_STEEL;
    canvas.drawLine(idX,      idY,      idX+9,     idY+9,    idCh);
    canvas.drawLine(idX+idW,  idY,      idX+idW-9, idY+9,    idCh);
    canvas.drawLine(idX,      idY+idH,  idX+9,     idY+idH-9,idCh);
    canvas.drawLine(idX+idW,  idY+idH,  idX+idW-9, idY+idH-9,idCh);
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(operatorIDLen > 0 ? C_WHITE : C_MUTED_TEXT, C_PANEL_BG);
    canvas.setTextDatum(middle_center);
    canvas.drawString(operatorIDLen > 0 ? operatorID : "[ TAP TO ENTER ]", idX+idW/2, idY+idH/2);

    // ── DATE / TIME FIELDS ──
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(C_MUTED_TEXT, C_VOID);
    canvas.setTextDatum(middle_center);
    canvas.drawString("> DATE / TIME:", SCR_W/2, 196);
    int fW = 85, fH = 48, fGap = 10;
    int totalW = 6*fW + 5*fGap;
    int fx = (SCR_W - totalW) / 2;
    int fy = 216;
    for (int i = 0; i < 6; i++) {
      int x = fx + i*(fW+fGap);
      bool isActive = (activeField == i);
      canvas.fillRect(x, fy, fW, fH, C_PANEL_BG);
      canvas.drawRect(x, fy, fW, fH, isActive ? C_CYAN : C_PANEL_BORDER);
      if (isActive) canvas.drawRect(x-2, fy-2, fW+4, fH+4, C_CYAN);
      uint16_t fCh = isActive ? C_CYAN : C_STEEL;
      canvas.drawLine(x,    fy,    x+7,   fy+7,   fCh);
      canvas.drawLine(x+fW, fy,    x+fW-7,fy+7,   fCh);
      canvas.drawLine(x,    fy+fH, x+7,   fy+fH-7,fCh);
      canvas.drawLine(x+fW, fy+fH, x+fW-7,fy+fH-7,fCh);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(C_CYAN, C_VOID);
      canvas.setTextDatum(bottom_center);
      canvas.drawString(fieldLabels[i], x+fW/2, fy-4);
      canvas.setFont(&fonts::FreeSansBold12pt7b);
      canvas.setTextColor(isActive ? C_CYAN : C_WHITE, C_PANEL_BG);
      canvas.setTextDatum(middle_center);
      char vStr[8];
      if (i==2) snprintf(vStr,sizeof(vStr),"%04d",fieldValues[i]);
      else       snprintf(vStr,sizeof(vStr),"%02d", fieldValues[i]);
      canvas.drawString(vStr, x+fW/2, fy+fH/2);
      if (i==2 || i==4) { canvas.setTextColor(C_STEEL,C_VOID); canvas.drawString(":", x+fW+fGap/2, fy+fH/2); }
    }

    // ── STATUS BADGES ──
    int stY = fy + fH + 22;
    int bdW = 230, bdH = 28, bdGap = 10;
    int bdX0 = (SCR_W - (bdW*2 + bdGap)) / 2;
    auto drawBadge = [&](int bx, uint16_t bg, const char* txt) {
      canvas.fillRect(bx, stY, bdW, bdH, bg);
      canvas.drawRect(bx, stY, bdW, bdH, C_WHITE);
      canvas.drawLine(bx,      stY,      bx+7,    stY+7,      C_WHITE);
      canvas.drawLine(bx+bdW,  stY,      bx+bdW-7,stY+7,      C_WHITE);
      canvas.drawLine(bx,      stY+bdH,  bx+7,    stY+bdH-7,  C_WHITE);
      canvas.drawLine(bx+bdW,  stY+bdH,  bx+bdW-7,stY+bdH-7,  C_WHITE);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(C_VOID, bg);
      canvas.setTextDatum(middle_center);
      canvas.drawString(txt, bx+bdW/2, stY+bdH/2);
    };
    drawBadge(bdX0,           sdReady ? C_VOLT_GREEN : C_ROG_RED,
              sdReady ? "SD STORAGE .... PASS" : "SD STORAGE .... FAIL");
    { bool c = bootCircuitDetected;
      canvas.setTextColor(c ? C_VOID : C_WHITE, c ? C_VOLT_GREEN : C_ROG_RED);
      drawBadge(bdX0+bdW+bdGap, c ? C_VOLT_GREEN : C_ROG_RED,
                c ? "CIRCUIT .... ONLINE" : "CIRCUIT .... OFFLINE"); }

    // ── START MONITORING BUTTON — Green Glow ──
    int startY = stY + bdH + 24;
    int btnW = 380, btnH = 58;
    int btnX = (SCR_W - btnW) / 2;
    // Outer glow border
    canvas.drawRect(btnX-3, startY-3, btnW+6, btnH+6, C_VOLT_GREEN);
    canvas.drawRect(btnX-2, startY-2, btnW+4, btnH+4, 0x0320);
    // Main button fill — dark green base
    canvas.fillRect(btnX, startY, btnW, btnH, 0x0320);
    canvas.drawRect(btnX, startY, btnW, btnH, C_VOLT_GREEN);
    canvas.drawRect(btnX+2, startY+2, btnW-4, btnH-4, 0x03E0);
    // Corner accents
    canvas.drawLine(btnX,       startY,       btnX+14,      startY+14,      C_VOLT_GREEN);
    canvas.drawLine(btnX+btnW,  startY,       btnX+btnW-14, startY+14,      C_VOLT_GREEN);
    canvas.drawLine(btnX,       startY+btnH,  btnX+14,      startY+btnH-14, C_VOLT_GREEN);
    canvas.drawLine(btnX+btnW,  startY+btnH,  btnX+btnW-14, startY+btnH-14, C_VOLT_GREEN);
    // Top highlight stripe
    canvas.drawFastHLine(btnX+16, startY+1, btnW-32, C_VOLT_GREEN);
    // Button text
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(C_WHITE, 0x0320);
    canvas.setTextDatum(middle_center);
    canvas.drawString("START MONITORING >>", SCR_W/2, startY+btnH/2);
    // Chevron arrows removed — now embedded in button text above

    // ── FOOTER BRANDING ──
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(C_DARK_METAL, C_VOID);
    canvas.setTextDatum(bottom_center);
    canvas.drawString("VIPx3X4s  ●  M12 SENSOR DIAGNOSTIC & MONITOR", SCR_W/2, SCR_H-10);
  };
  
  drawScreen();
  canvas.pushSprite(0, 0);

  while (true) {
    int tx, ty;
    if (display.getTouch(&tx, &ty)) {
      if (numpadOpen) {
        int npW = 300, npH = 360, npX = (SCR_W - npW) / 2, npY = (SCR_H - npH) / 2;
        const char* keys[] = { "7","8","9","4","5","6","1","2","3","","0","DEL" };
        int kW = (npW - 20) / 3, kH = 50, kGap = 4;

        for (int i = 0; i < 12; i++) {
          if (i == 9) continue;
          int col = i % 3, row = i / 3;
          int kx = npX + 10 + col * (kW + kGap), ky = npY + 70 + row * (kH + kGap);
          if (tx >= kx && tx <= kx + kW && ty >= ky && ty <= ky + kH) {
            if (i == 11) { // DEL / backspace
              if (npBufLen > 0) { npBuf[--npBufLen] = '\0'; }
            } else {
              if (npBufLen < 15) { npBuf[npBufLen++] = keys[i][0]; npBuf[npBufLen] = '\0'; }
            }
            // Sync local buf → global npValue so drawUniversalNumpad shows it
            strncpy(npValue, npBuf, 15); npValue[15] = '\0'; npLen = npBufLen;
            drawScreen(); drawUniversalNumpad(); canvas.pushSprite(0, 0);
            delay(180); break;
          }
        }

        int btY = npY + npH - 54, halfW = (npW - 24) / 2;
        if (tx >= npX + 10 && tx <= npX + 10 + halfW && ty >= btY && ty <= btY + 44) {
          if (activeField == 6) { strncpy(operatorID, npBuf, 15); operatorIDLen = strlen(operatorID); }
          else if (activeField >= 0 && activeField < 6) { fieldValues[activeField] = constrain(atoi(npBuf), 0, fieldMax[activeField]); }
          numpadOpen = false; activeField = -1;
          npMode = NP_NONE; npLen = 0; npValue[0] = '\0';
          drawScreen(); canvas.pushSprite(0, 0); delay(200);
        } else if (tx >= npX + 14 + halfW && tx <= npX + 14 + halfW + halfW && ty >= btY && ty <= btY + 44) {
          numpadOpen = false; activeField = -1;
          npMode = NP_NONE; npLen = 0; npValue[0] = '\0';
          drawScreen(); canvas.pushSprite(0, 0); delay(200);
        }
      } else {
        // Operator ID box — matches new centered layout
        int idW = 440, idH = 44;
        int idX = (SCR_W - idW) / 2, idY = 126;
        if (tx >= idX && tx <= idX + idW && ty >= idY && ty <= idY + idH) {
          activeField = 6; numpadOpen = true; npBufLen = 0; npBuf[0] = '\0';
          npMode = NP_LOGIN_ID; strcpy(npValue, ""); npLen = 0;
          drawScreen(); drawUniversalNumpad(); canvas.pushSprite(0, 0);
          delay(200); continue;
        }
        // Date/time fields — matches new layout: fy=203
        int fW = 85, fH = 48, fGap = 10, totalW = 6 * fW + 5 * fGap, fx = (SCR_W - totalW) / 2, fy = 216;
        for (int i = 0; i < 6; i++) {
          int x = fx + i * (fW + fGap);
          if (tx >= x && tx <= x + fW && ty >= fy && ty <= fy + fH) {
            activeField = i; numpadOpen = true; npBufLen = 0; npBuf[0] = '\0';
            npMode = NP_DT_FIELD; strcpy(npValue, ""); npLen = 0;
            drawScreen(); drawUniversalNumpad(); canvas.pushSprite(0, 0);
            delay(200); break;
          }
        }
        // BOOT button — matches new layout
        int stY2 = fy + fH + 22;
        int startY = stY2 + 28 + 24;
        int btnW2 = 380, btnH2 = 58;
        int btnX2 = (SCR_W - btnW2) / 2;
        if (tx >= btnX2 && tx <= btnX2 + btnW2 && ty >= startY && ty <= startY + btnH2) {
          dtDay = fieldValues[0]; dtMonth = fieldValues[1]; dtYear = fieldValues[2];
          dtHour = fieldValues[3]; dtMin = fieldValues[4]; dtSec = fieldValues[5];
          dtBaseMillis = millis();
          dtClockReset = true;  // re-latch origin statics in updateSoftwareClock()
          canvas.fillRect(btnX2, startY, btnW2, btnH2, C_VOLT_GREEN);
          canvas.setTextColor(C_VOID, C_VOLT_GREEN);
          canvas.setTextDatum(middle_center);
          canvas.drawString("SYSTEM ARMED", SCR_W/2, startY + btnH2/2);
          canvas.pushSprite(0, 0);
          delay(500); break;
        }
      }
    }
  }
  loginDone = true;
}

/* ------------------------------------------------------------------
 * NEW: HIGH-DOPAMINE ROG/CYBERPUNK BOOT CINEMATIC
 * ------------------------------------------------------------------ */

void drawBootCinematic() {
  if (!spriteReady) { display.fillScreen(C_VOID); return; }
  int cx = SCR_W / 2, cy = SCR_H / 2 - 50;

  // ══════════════════════════════════════════════════════════════════
  // PHASE 1: VOID IGNITION — Dual scan lines sweep + corner locks
  // ══════════════════════════════════════════════════════════════════
  for (int i = 0; i <= SCR_W; i += 24) {
    canvas.fillScreen(C_VOID);
    canvas.drawFastHLine(0, cy - 2, i, C_CYAN);
    canvas.drawFastHLine(SCR_W - i, cy + 2, i, C_ROG_RED);
    canvas.drawFastHLine(0, cy - 1, i, C_GLOW_CYAN);
    canvas.drawFastHLine(SCR_W - i, cy + 1, i, C_CRIMSON);
    int cLen = min(i / 6, 50);
    canvas.drawFastHLine(0, 0, cLen, C_CYAN); canvas.drawFastVLine(0, 0, cLen, C_CYAN);
    canvas.drawFastHLine(SCR_W - cLen, 0, cLen, C_ROG_RED); canvas.drawFastVLine(SCR_W - 1, 0, cLen, C_ROG_RED);
    canvas.drawFastHLine(0, SCR_H - 1, cLen, C_ROG_RED); canvas.drawFastVLine(0, SCR_H - cLen, cLen, C_ROG_RED);
    canvas.drawFastHLine(SCR_W - cLen, SCR_H - 1, cLen, C_CYAN); canvas.drawFastVLine(SCR_W - 1, SCR_H - cLen, cLen, C_CYAN);
    canvas.pushSprite(0, 0);
    delay(5);
  }
  canvas.fillRect(0, cy - 3, SCR_W, 7, C_WHITE);
  canvas.pushSprite(0, 0); delay(40);

  // ══════════════════════════════════════════════════════════════════
  // PHASE 2: BAND WIPE — Expand + ROG corner brackets
  // ══════════════════════════════════════════════════════════════════
  for (int band = 0; band <= 240; band += 22) {
    canvas.fillScreen(C_VOID);
    canvas.drawFastHLine(0, cy - band, SCR_W, C_CYAN);
    canvas.drawFastHLine(0, cy + band, SCR_W, C_ROG_RED);
    for (int k = 0; k < 3; k++) {
      canvas.drawFastHLine(0, k, 50, C_CYAN); canvas.drawFastVLine(k, 0, 50, C_CYAN);
      canvas.drawFastHLine(SCR_W - 50, k, 50, C_ROG_RED); canvas.drawFastVLine(SCR_W - 1 - k, 0, 50, C_ROG_RED);
      canvas.drawFastHLine(0, SCR_H - 1 - k, 50, C_ROG_RED); canvas.drawFastVLine(k, SCR_H - 50, 50, C_ROG_RED);
      canvas.drawFastHLine(SCR_W - 50, SCR_H - 1 - k, 50, C_CYAN); canvas.drawFastVLine(SCR_W - 1 - k, SCR_H - 50, 50, C_CYAN);
    }
    canvas.pushSprite(0, 0);
    delay(8);
  }

  // ══════════════════════════════════════════════════════════════════
  // PHASE 3: GLITCH RESOLVE — [ VIPx3X4s ] + subtitle materialises
  // ══════════════════════════════════════════════════════════════════
  const char* glitchFrames[] = { "[V##x3X@s]", "[V!Px3X4!]", "[VIPx#X4$]", "[ VIPx3X4s ]" };
  uint16_t glitchCols[] = { C_STEEL, C_MUTED_TEXT, C_CYAN, C_WHITE };
  for (int g = 0; g < 4; g++) {
    canvas.fillScreen(C_VOID);
    canvas.drawRect(6, 6, SCR_W - 12, SCR_H - 12, C_DARK_METAL);
    canvas.drawRect(8, 8, SCR_W - 16, SCR_H - 16, (g < 3) ? C_STEEL : C_CYAN);
    canvas.drawFastHLine(12, 12, 20, C_CYAN);
    canvas.drawFastVLine(12, 12, 20, C_CYAN);
    canvas.drawFastHLine(SCR_W-32, 12, 20, C_ROG_RED);
    canvas.drawFastVLine(SCR_W-12, 12, 20, C_ROG_RED);
    canvas.drawFastHLine(12, SCR_H-12, 20, C_ROG_RED);
    canvas.drawFastVLine(12, SCR_H-32, 20, C_ROG_RED);
    canvas.drawFastHLine(SCR_W-32, SCR_H-12, 20, C_CYAN);
    canvas.drawFastVLine(SCR_W-12, SCR_H-32, 20, C_CYAN);
    if (g == 3) {
      canvas.setFont(&fonts::FreeSansBold24pt7b);
      canvas.setTextDatum(middle_center);
      canvas.setTextColor(C_ROG_RED, C_VOID);
      canvas.drawString("[ VIPx3X4s ]", cx - 3, cy - 70);
      canvas.setTextColor(0x001F, C_VOID);
      canvas.drawString("[ VIPx3X4s ]", cx + 3, cy - 70);
    }
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(glitchCols[g], C_VOID);
    canvas.drawString(glitchFrames[g], cx, cy - 70);
    canvas.pushSprite(0, 0);
    delay(g < 3 ? 40 : 70);
  }

  // Subtitle line 1: M12 A-CODED SENSOR DIAGNOSE TOOL
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextColor(C_CYAN, C_VOID);
  canvas.setTextDatum(middle_center);
  canvas.drawString("{ M12 A-CODED SENSOR DIAGNOSE & MONITOR TOOL }", cx, cy - 20);
  canvas.pushSprite(0, 0); delay(120);

  // Subtitle line 2
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(C_MUTED_TEXT, C_VOID);
  canvas.drawString("", cx, cy - 4);
  // Clean centered rule
  canvas.drawFastHLine(cx - 220, cy + 6, 440, C_STEEL);
  canvas.pushSprite(0, 0);
  delay(150);

  // ══════════════════════════════════════════════════════════════════
  // PHASE 4: REAL HARDWARE VERIFICATION — Actual live checks
  // ══════════════════════════════════════════════════════════════════
  struct HWCheck { const char* name; const char* pin; bool passed; };
  HWCheck checks[6];

  // Check 1: Display/Sprite
  checks[0] = { "DISPLAY BUFFER", "[PSRAM]", spriteReady };
  // Check 2: SD Card
  checks[1] = { "SD STORAGE",     "[SPI1]", sdReady };
  // Check 3: UART TX/RX lines initialised (always true after Serial1.begin)
  checks[2] = { "UART BUS",       "[TX/RX]", true };

  // Check 4: CIRCUIT BOARD — Try to get a heartbeat from circuit over UART
  while (Serial1.available()) Serial1.read(); // flush
  Serial1.println("$PING*");
  Serial1.flush();
  bool gotHeartbeat = false;
  unsigned long waitStart = millis();
  while (millis() - waitStart < 800) {
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        if (uartLineIdx > 0) {
          uartLine[uartLineIdx] = '\0';
          if (uartLine[0] == '$' && (uartLine[1] == 'H' || strncmp(uartLine, "$PONG", 5) == 0)) {
            if (uartLine[1] == 'H') parseHeartbeat(uartLine);
            gotHeartbeat = true;
          }
          uartLineIdx = 0;
        }
      } else if (c != '\r') {
        if (uartLineIdx < UART_BUF_SIZE - 1) uartLine[uartLineIdx++] = c;
        else uartLineIdx = 0;
      }
    }
    if (gotHeartbeat) break;
    delay(5);
  }
  checks[3] = { "CIRCUIT BOARD",  "[UART]", gotHeartbeat };
  bootCircuitDetected = gotHeartbeat;

  // Check 5: SENSOR PORTS — any port active?
  checks[4] = { "SENSOR PORTS",   "[P1-P4]", (gotHeartbeat ? true : false) };

  // Check 6: I2C Bus (PCA9557 Touch Controller)
  Wire.beginTransmission(PCA9557_ADDR);
  bool i2cOK = (Wire.endTransmission() == 0);
  checks[5] = { "I2C TOUCH CTRL", "[I2C]", i2cOK };

  int numChecks = 6;
  int listX = cx - 190;
  int listY = cy + 30;
  int rowH = 18;
  int passCount = 0;

  // Section header — centered
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextColor(C_CYAN, C_VOID);
  canvas.setTextDatum(middle_center);
  canvas.drawString("HARDWARE VERIFICATION", cx, listY - 20);
  canvas.drawFastHLine(cx - 190, listY - 6, 380, C_STEEL);
  canvas.pushSprite(0, 0);
  delay(80);

  // Animate each check
  for (int i = 0; i < numChecks; i++) {
    int ry = listY + i * rowH;
    // Scanning state: red dot + "..."
    canvas.fillCircle(listX - 14, ry + 6, 5, C_ROG_RED);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(C_MUTED_TEXT, C_VOID);
    canvas.setTextDatum(top_left);
    char rowStr[48]; snprintf(rowStr, sizeof(rowStr), "%-18s %s", checks[i].name, checks[i].pin);
    canvas.drawString(rowStr, listX, ry);
    canvas.setTextColor(C_ROG_RED, C_VOID);
    canvas.drawString("...", listX + 290, ry);
    canvas.pushSprite(0, 0);
    delay(80 + (int)(random(60)));

    // Result: green/red dot + YES/NO
    canvas.fillRect(listX + 290, ry, 60, 12, C_VOID); // Clear "..."
    if (checks[i].passed) {
      canvas.fillCircle(listX - 14, ry + 6, 5, C_VOLT_GREEN);
      canvas.drawCircle(listX - 14, ry + 6, 7, C_VOLT_GREEN);
      canvas.setTextColor(C_VOLT_GREEN, C_VOID);
      canvas.drawString("PASS", listX + 290, ry);
      passCount++;
    } else {
      canvas.fillCircle(listX - 14, ry + 6, 5, C_ROG_RED);
      canvas.drawCircle(listX - 14, ry + 6, 7, C_ROG_RED);
      canvas.setTextColor(C_ROG_RED, C_VOID);
      canvas.drawString("FAIL", listX + 290, ry);
    }
    canvas.pushSprite(0, 0);
    delay(40);
  }

  // Summary line
  int sumY = listY + numChecks * rowH + 12;
  canvas.drawFastHLine(cx - 190, sumY, 380, C_STEEL);
  canvas.setFont(&fonts::FreeMonoBold9pt7b);
  canvas.setTextDatum(middle_center);
  char sumStr[48];
  bool allPass = (passCount == numChecks);
  if (allPass) {
    canvas.setTextColor(C_VOLT_GREEN, C_VOID);
    snprintf(sumStr, sizeof(sumStr), "ALL SYSTEMS ONLINE  [ %d / %d ]", passCount, numChecks);
    canvas.drawString(sumStr, cx, sumY + 16);
  } else {
    canvas.setTextColor(C_HAZARD, C_VOID);
    snprintf(sumStr, sizeof(sumStr), "WARNING: %d / %d PASSED", passCount, numChecks);
    canvas.drawString(sumStr, cx, sumY + 16);
  }

  // Progress bar
  int barX = cx - 200, barY = sumY + 44, barW = 400, barH = 6;
  canvas.fillRect(barX, barY, barW, barH, C_DARK_METAL);
  canvas.drawRect(barX - 1, barY - 1, barW + 2, barH + 2, C_STEEL);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(C_MUTED_TEXT, C_VOID);
  canvas.setTextDatum(top_left);
  canvas.drawString("SYSTEM BOOT", barX, barY - 14);
  canvas.pushSprite(0, 0);
  for (int p = 0; p <= 100; p += 4) {
    int fillW = (barW * p) / 100;
    canvas.fillRect(barX, barY, fillW, barH, C_CRIMSON);
    if (fillW > 4) canvas.fillRect(barX + fillW - 4, barY - 1, 6, barH + 2, C_ROG_RED);
    canvas.fillRect(barX + barW + 8, barY - 2, 40, 12, C_VOID);
    canvas.setFont(&fonts::Font0); canvas.setTextColor(C_ROG_RED, C_VOID); canvas.setTextDatum(top_left);
    char pStr[8]; snprintf(pStr, 8, "%d%%", p);
    canvas.drawString(pStr, barX + barW + 8, barY - 2);
    canvas.pushSprite(0, 0);
    delay(6);
  }
  canvas.pushSprite(0, 0);

  // ══════════════════════════════════════════════════════════════════
  // PHASE 5: If any check failed — show CONTINUE / RE-CHECK buttons
  // ══════════════════════════════════════════════════════════════════
  if (!allPass) {
    int btnY = barY + 20;
    int btnW = 180, btnH = 40;
    // CONTINUE button
    canvas.fillRect(cx - btnW - 10, btnY, btnW, btnH, C_HAZARD);
    canvas.drawRect(cx - btnW - 10, btnY, btnW, btnH, C_WHITE);
    canvas.drawLine(cx - btnW - 10, btnY, cx - btnW + 2, btnY + 12, C_WHITE);
    canvas.setFont(&fonts::FreeMonoBold9pt7b);
    canvas.setTextColor(C_VOID, C_HAZARD);
    canvas.setTextDatum(middle_center);
    canvas.drawString("CONTINUE >>", cx - btnW - 10 + btnW / 2, btnY + btnH / 2);
    // RE-CHECK button
    canvas.fillRect(cx + 10, btnY, btnW, btnH, C_VOID);
    canvas.drawRect(cx + 10, btnY, btnW, btnH, C_CYAN);
    canvas.drawLine(cx + 10 + btnW, btnY + btnH, cx + btnW - 2, btnY + btnH - 12, C_CYAN);
    canvas.setTextColor(C_CYAN, C_VOID);
    canvas.drawString("RE-CHECK", cx + 10 + btnW / 2, btnY + btnH / 2);
    canvas.pushSprite(0, 0);

    // Wait for touch
    while (true) {
      int tx, ty;
      if (display.getTouch(&tx, &ty)) {
        // CONTINUE
        if (tx >= cx - btnW - 10 && tx <= cx - 10 && ty >= btnY && ty <= btnY + btnH) {
          canvas.fillRect(cx - btnW - 10, btnY, btnW, btnH, C_WHITE);
          canvas.setTextColor(C_VOID, C_WHITE);
          canvas.drawString("PROCEEDING...", cx - btnW - 10 + btnW / 2, btnY + btnH / 2);
          canvas.pushSprite(0, 0); delay(400);
          break;
        }
        // RE-CHECK — restart entire boot cinematic
        if (tx >= cx + 10 && tx <= cx + 10 + btnW && ty >= btnY && ty <= btnY + btnH) {
          canvas.fillScreen(C_VOID); canvas.pushSprite(0, 0); delay(200);
          drawBootCinematic(); // Recursive re-check
          return;
        }
        delay(150);
      }
    }
  } else {
    delay(300);
  }

  // ══════════════════════════════════════════════════════════════════
  // PHASE 6: FLASH BANG TRANSITION
  // ══════════════════════════════════════════════════════════════════
  for (int b = 0; b < 3; b++) {
    canvas.fillScreen(b % 2 == 0 ? C_WHITE : C_VOID);
    canvas.pushSprite(0, 0);
    delay(b == 0 ? 35 : 20);
  }
  canvas.fillScreen(C_VOID);
  canvas.pushSprite(0, 0);
}

/* ------------------------------------------------------------------
 * TOUCH DISPATCHERS & MAIN LOOP
 * ------------------------------------------------------------------ */

// (Touch Dispatcher logic remains identical, calling the specific drawTab logic)

void setup() {
  Serial.begin(115200); delay(300); Serial.println("VIPx3X4s — ARMED");
  
  Serial1.setRxBufferSize(4096); 
  Serial1.begin(UART_BAUD, SERIAL_8N1, SENSOR_UART_RX, SENSOR_UART_TX);
  
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(300000); init_pca9557();
  
  display.init(); display.setRotation(0); display.setBrightness(255); display.fillScreen(C_VOID);
  // Sprite allocation can fail depending on PSRAM settings/heap.
  // Try PSRAM@16bpp first, then fall back to a lower bpp without PSRAM.
  spriteReady = false;
  canvas.setColorDepth(16);
  canvas.setPsram(true);
  if (canvas.createSprite(SCR_W, SCR_H)) {
    spriteReady = true;
  } else {
    canvas.setPsram(false);
    canvas.setColorDepth(8);
    spriteReady = canvas.createSprite(SCR_W, SCR_H);
  }
  
  initSDCard();

  for (int i = 0; i < 4; i++) {
    memset(&sData[i], 0, sizeof(SensorInfo)); 
    sData[i].healthScore = 100.0f;
    for (int f = 0; f < 12; f++) sData[i].factorScores[f] = 100.0f;
    sData[i].feedbackEnabled = true; // Default ON
  }

  if (sdReady) for (int i = 0; i < 4; i++) loadSettingsFromSD(i);

  drawBootCinematic(); 
  runLoginAndDateTimeSetup();
  needFullRedraw = true; 
  renderFrame(); 
  
  delay(200); sendTimeSync(); delay(100); requestLimits();
  if (sdReady) updateLogFilenames();
}

void renderFrame() {
  if (!spriteReady) { drawHeader(); drawBody(); drawFooter(); return; }
  drawHeader(); drawBody(); drawFooter();
  
  if (fieldSelectorActive) drawFieldSelectorOverlay();
  if (npMode != NP_NONE) drawUniversalNumpad();
  
  canvas.pushSprite(0, 0);
}
void loop() {
  unsigned long now = millis();
  readAllUART(); 
  handleTouch();
  
  if (buzzerMuted && now >= buzzerMuteEnd) buzzerMuted = false;
  if (circuitOnline && now - lastHeartbeat > 3000) circuitOnline = false;
  
  unsigned long renderInterval = (currentTab == TAB_LIVE || currentTab == TAB_HOME) ? 80 : 250; 
  if (needFullRedraw || (now - lastScreenUpdate >= renderInterval)) { 
    lastScreenUpdate = now; needFullRedraw = false; renderFrame(); 
  }
  
  if (now - lastTimeDraw >= 1000) {
    lastTimeDraw = now; needFullRedraw = true; static int lastLogDay = 0; updateSoftwareClock();
    if (dtDay != lastLogDay && sdReady) { lastLogDay = dtDay; updateLogFilenames(); }
  }
}