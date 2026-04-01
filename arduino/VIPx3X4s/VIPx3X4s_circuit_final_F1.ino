/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  VIPx3X4s — CIRCUIT BOARD FIRMWARE                                       ║
 * ║  ESP32-S3  ×  4× INA3221  |  12-Channel  Real-Time  V / I / P            ║
 * ║  High-Speed Slew-Rate Timing & Advanced Diagnostic Engine                ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_task_wdt.h"

// ═════════════════════════════════════════════════════════════════════════
// §1  HARDWARE PINS & ADDRESSES
// ═════════════════════════════════════════════════════════════════════════

#define NEO_PIN         48
#define NEO_BRIGHT      80          // 0–255 global brightness scale

#define I2C0_SDA        41
#define I2C0_SCL        17
#define I2C1_SDA        21
#define I2C1_SCL        13
#define I2C_FREQ        400000UL    // 400 kHz

#define PWR_PIN_A       10          // Power rail for INA #3 board
#define PWR_PIN_B       11          // Power rail for INA #4 board

#define UART_TX         1
#define UART_RX         2
#define UART_BAUD       230400

// INA3221 I2C addresses
#define INA1_ADDR       0x42        // Bus 0
#define INA2_ADDR       0x43        // Bus 0
#define INA3_ADDR       0x40        // Bus 1
#define INA4_ADDR       0x41        // Bus 1

// INA3221 register map
#define REG_CONFIG      0x00
#define REG_CH1_SHT     0x01
#define REG_CH1_BUS     0x02
#define REG_CH2_SHT     0x03
#define REG_CH2_BUS     0x04
#define REG_CH3_SHT     0x05
#define REG_CH3_BUS     0x06

// NEW: Dual-Mode TURBO configurations for High-Speed Interleaving
#define INA_TURBO_WITH_FB 0x7007    // CH1=1, CH2=1, CH3=1 (840µs internal cycle)
#define INA_TURBO_NO_FB   0x6007    // CH1=1, CH2=1, CH3=0 (560µs internal cycle - FASTER)

// INA3221 physics
#define BUS_LSB_V       0.008f      // 8 mV per bus voltage LSB
#define SHT_LSB_V       0.000040f   // 40 µV per shunt LSB
#define SHUNT_OHMS      0.100f      // 100 mΩ shunt resistor

// ═════════════════════════════════════════════════════════════════════════
// §2  SIGNAL ZONES, TIMING & CALIBRATION CONSTANTS
// ═════════════════════════════════════════════════════════════════════════

// Stage IDs (0–4)
#define STAGE_NOSIG     0   // V ≤ 50 mV — dead-band
#define STAGE_VFLOOR    1   // 50 mV < V ≤ 3.0 V — idle low
#define STAGE_RISING    2   // climbing Vfloor → Vpeak
#define STAGE_STABLE    3   // holding at Vpeak  (sensor fully ON)
#define STAGE_FALLING   4   // dropping Vpeak → Vfloor

#define NO_SIG_V        0.050f      // 50 mV dead-band threshold
#define VFLOOR_MAX_V    3.0f        // Vfloor zone hard ceiling (hardware spec)
#define VPEAK_MIN_V     3.0f        // Vpeak must be ABOVE this to run timing
#define VPEAK_MAX_V     24.0f       // Vpeak maximum (M12 sensor spec)

// Schmitt trigger fractions of calibrated swing (Slew-Rate Window)
#define THRESH_UP       0.75f       // 75% rising  — RISING→STABLE entry gate
#define THRESH_DN       0.25f       // 25% falling — VFLOOR→RISING & STABLE→FALLING

// Calibration
#define CAL_WARMUP      200         // samples before timing starts  (~168 ms at 1190 Hz)
#define CAL_EMA         0.01f       // EMA smoothing alpha — slow, noise-resistant
#define CAL_MIN_SWING   1.0f        // minimum V swing to run timing

// Fault codes
#define FAULT_NONE      0
#define FAULT_LEAK      1
#define FAULT_SPIKE     2
#define FAULT_RT        3
#define FAULT_FT        4
#define FAULT_ST        5

// Packet send intervals
#define FAST_MS         15
#define FULL_MS         500
#define BEAT_MS         1000
#define DIAG_MS         3000
#define HEALTH_MS       30000

// UART RX line buffer size
#define RX_BUF          256

// ═════════════════════════════════════════════════════════════════════════
// §3  DATA STRUCTURES
// ═════════════════════════════════════════════════════════════════════════

struct ChannelReading {
  float voltage_V;
  float current_mA;
  float power_mW;
  bool  valid;
};

struct SensorLimits {
  uint32_t rt_min_us, rt_max_us;  uint8_t rt_cnt;
  uint32_t st_min_us, st_max_us;  uint8_t st_cnt;
  uint32_t ft_min_us, ft_max_us;  uint8_t ft_cnt;
  uint8_t  rt_fail, st_fail, ft_fail;
};

struct SensorAlarm {
  bool  active;
  int   faultCode;
  char  cause[64];
  char  time[24];
};

/*
 * TimingState — Complete per-sensor state machine + diagnostic metrics.
 */
struct TimingState {
  uint8_t  stage;           // current STAGE_* 0–4

  // Stage entry timestamps (micros())
  uint32_t t_rise_us;       // when RISING  began
  uint32_t t_stable_us;     // when STABLE  began
  uint32_t t_fall_us;       // when FALLING began
  uint32_t t_stage_us;      // entry time of current stage (for live elapsed)

  // Last completed cycle timing
  uint32_t RT_us;           // rise  time
  uint32_t ST_us;           // stable time
  uint32_t FT_us;           // fall  time
  uint32_t CT_us;           // cycle total = RT + ST + FT

  // Min / max tracking across all completed cycles
  uint32_t RT_min_us, RT_max_us;
  uint32_t ST_min_us, ST_max_us;
  uint32_t FT_min_us, FT_max_us;

  // Advanced Diagnostic Metrics (Category 3, 4A, 14 — sensor_diagnostics.md)
  float    maxPotential_V;      // peak voltage seen during STABLE
  float    minPotential_V;      // lowest voltage seen during STABLE (ripple calc)
  float    maxLeakage_mA;       // absolute peak idle current (Category 3)
  float    inrush_mA;           // peak inrush spike on turn-on (Category 3)
  uint32_t bounceCount;         // per-cycle contact stutters during RISING (Category 4A)
  uint32_t totalBounceCount;    // lifetime bounce accumulator (Category 16)
  uint32_t spikeCount;          // total inrush spike events detected (Category 16)

  uint32_t cycleCount;          // total completed cycles since boot / last RCAL

  // NEW: Background EMA trackers (Category 3: Current, Category 4A: Signal)
  float    avgVoltage_V;        // EWMA voltage during STABLE
  float    avgActiveCurrent_mA; // EWMA output current during STABLE
  float    avgLeakage_mA;       // EWMA idle leakage current

  // NEW: Derived per-cycle metrics (Category 4A: Duty Cycle, Sat Drop)
  float    dutyCycle;           // ST/CT * 100% — output duty cycle
  float    satDrop_V;           // VIN - Vpeak — saturation voltage drop

  // NEW: Power supply event counters (Category 1: OV/UV)
  uint32_t ovVoltCount;         // events where VIN > 26V
  uint32_t uvVoltCount;         // events where VIN < 18V (and > 1V)
};

// ═════════════════════════════════════════════════════════════════════════
// §4  GLOBAL STATE
// ═════════════════════════════════════════════════════════════════════════

// Live V/I/P readings — Core 1 writes, Core 0 reads (under dataMux)
// NOTE: 'volatile' removed from structs to fix GCC assignment compiler errors
ChannelReading  chipCH1[4];
ChannelReading  chipCH2[4];
ChannelReading  chipCH3[4];
volatile float  chipVcc[4]       = {0,0,0,0};
volatile bool   chipOnline[4]    = {false,false,false,false};
volatile bool   chipWasOnline[4] = {false,false,false,false};

// NEW: UI toggled Feedback states (determines if CH3 is read to save I2C bandwidth)
bool fbEnabled[4] = {false, false, false, false}; 

// Auto-calibration — Core 1 only (no mutex needed)
float    calVfloor[4] = {99.0f, 99.0f, 99.0f, 99.0f};
float    calVpeak[4]  = { 0.0f,  0.0f,  0.0f,  0.0f};
uint32_t calCount[4]  = {0,0,0,0};

// Timing state — Core 1 writes, Core 0 reads for $S packet
TimingState sTiming[4];

// Sensor configuration & alarms
uint8_t      sensorType[4] = {0,0,0,0};  // default: PNP-NO (normal)
SensorLimits sLimits[4];
SensorAlarm  sAlarm[4];

// Performance counters
volatile uint32_t totalReads  = 0;
volatile float    liveReadHz  = 0.0f;

// $LOG packet serial number and per-sensor last-logged cycle tracker
static uint32_t logSlno              = 0;
static uint32_t lastLoggedCycle[4]   = {0, 0, 0, 0};

// Software clock (synced by $TIME command)
int           clkDay=1, clkMon=1, clkYr=2026;
int           clkH=0,   clkM=0,   clkS=0;
unsigned long clkBase   = 0;
bool          clkSync   = false;
bool          clkReset  = false;

// NeoPixel + display-connection tracking
unsigned long lastNeo     = 0;
unsigned long lastContact = 0;
unsigned long lastBlink   = 0;
uint8_t       blinkSt     = 0;
bool          dispConn    = false;

// Packet send timers
unsigned long lastFast   = 0;
unsigned long lastFull   = 0;
unsigned long lastBeat   = 0;
unsigned long lastDiag   = 0;
unsigned long lastHealth = 0;

// UART RX line buffer
char rxBuf[RX_BUF];
int  rxIdx = 0;

// USB serial command accumulator
String usbCmd = "";

// FreeRTOS synchronisation primitives
SemaphoreHandle_t mtx0;    // guards I2C Bus 0  (Wire)
SemaphoreHandle_t mtx1;    // guards I2C Bus 1  (Wire1)
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;  // guards chipCH* arrays

// ═════════════════════════════════════════════════════════════════════════
// §5  SOFTWARE CLOCK
// ═════════════════════════════════════════════════════════════════════════

static void clkTick() {
  if (!clkSync) return;
  static int oh=-1, om=-1, os=-1, od=-1, omo=-1, oy=-1;
  if (oh < 0 || clkReset) {
    oh=clkH; om=clkM; os=clkS;
    od=clkDay; omo=clkMon; oy=clkYr;
    clkReset = false;
  }
  unsigned long el   = (millis() - clkBase) / 1000UL;
  unsigned long tot  = (unsigned long)oh*3600 + om*60 + os + el;
  unsigned long days = tot / 86400; tot %= 86400;
  clkH = tot/3600; clkM = (tot%3600)/60; clkS = tot%60;
  
  static const uint8_t dm[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  int d=od+(int)days, mo=omo, yr=oy;
  
  while (true) {
    uint8_t daysInMo = dm[mo-1];
    // Leap year patch
    if (mo == 2 && ((yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0))) {
      daysInMo = 29;
    }
    if (d <= daysInMo) break;
    d -= daysInMo; 
    if (++mo > 12) { mo = 1; yr++; }
  }
  clkDay=d; clkMon=mo; clkYr=yr;
}

static void clkStr(char *buf, int sz) {
  clkTick();
  snprintf(buf, sz, "%02d/%02d/%04d %02d:%02d:%02d",
           clkDay, clkMon, clkYr, clkH, clkM, clkS);
}

// ═════════════════════════════════════════════════════════════════════════
// §6  INA3221 — HARDWARE LAYER (High-Speed Interleaved)
// ═════════════════════════════════════════════════════════════════════════

// Write a 16-bit register
static void inaWrite(TwoWire &bus, uint8_t addr, uint8_t reg, uint16_t val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write((uint8_t)(val >> 8));
  bus.write((uint8_t)(val & 0xFF));
  bus.endTransmission(true);
}

// Read a 16-bit register — returns 0xFFFF on NACK (device absent)
static uint16_t inaRead(TwoWire &bus, uint8_t addr, uint8_t reg) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(true) != 0) return 0xFFFF;
  if (bus.requestFrom(addr, (uint8_t)2) != 2) return 0xFFFF;
  return ((uint16_t)bus.read() << 8) | (uint8_t)bus.read();
}

// Configure INA3221 to TURBO mode dynamically based on Feedback state
static void inaTurbo(TwoWire &bus, uint8_t addr, bool useFb) {
  inaWrite(bus, addr, REG_CONFIG, useFb ? INA_TURBO_WITH_FB : INA_TURBO_NO_FB);
  delayMicroseconds(900);   // Wait for first conversion cycle
}

// Convert raw INA3221 register pair → physical ChannelReading
static ChannelReading rawToReading(uint16_t shtRaw, uint16_t busRaw) {
  ChannelReading r = {0.0f, 0.0f, 0.0f, false};
  if (shtRaw == 0xFFFF || busRaw == 0xFFFF) return r;
  float V    = (float)((int16_t)busRaw >> 3) * BUS_LSB_V;
  float Vsh  = (float)((int16_t)shtRaw >> 3) * SHT_LSB_V;
  float I_mA = (Vsh / SHUNT_OHMS) * 1000.0f;
  float P_mW = V * I_mA;
  if (V < NO_SIG_V) { V=0; I_mA=0; P_mW=0; }   // dead-band → true zero
  r = {V, I_mA, P_mW, true};
  return r;
}

// ── HIGH-SPEED INTERLEAVED READER ─────────────────────────────────────────
// Reads CH2 every single time. Reads CH1 and CH3 only once every 10 loops.
// portIdx (0–3): each port gets its OWN independent counter — fixes the
// shared-static bug where all four ports shared one cycleCount variable,
// causing CH1/CH3 to fire at the wrong port's loop boundary.
static bool chipRead(TwoWire &bus, uint8_t addr, bool useFb,
                     ChannelReading &ch1, ChannelReading &ch2, ChannelReading &ch3,
                     bool &ch1_updated, bool &ch3_updated, uint8_t portIdx) {

  static uint8_t cc[4] = {0, 0, 0, 0};   // independent per-port counters
  cc[portIdx]++;

  ch1_updated = false;
  ch3_updated = false;

  // PRIORITY 1: ALWAYS read CH2 (Timing Output) as fast as possible
  uint16_t s2 = inaRead(bus, addr, REG_CH2_SHT);
  uint16_t b2 = inaRead(bus, addr, REG_CH2_BUS);

  if (s2 == 0xFFFF && b2 == 0xFFFF) return false;  // chip completely absent
  ch2 = rawToReading(s2, b2);

  // PRIORITY 2: Read CH1 (Power Supply) only every 10th loop per port
  if (cc[portIdx] % 10 == 0) {
    uint16_t s1 = inaRead(bus, addr, REG_CH1_SHT);
    uint16_t b1 = inaRead(bus, addr, REG_CH1_BUS);
    ch1 = rawToReading(s1, b1);
    ch1_updated = true;
  }

  // PRIORITY 3: Read CH3 (Feedback) only if ON, and only every 10th loop per port
  if (useFb && (cc[portIdx] % 10 == 0)) {
    uint16_t s3 = inaRead(bus, addr, REG_CH3_SHT);
    uint16_t b3 = inaRead(bus, addr, REG_CH3_BUS);
    ch3 = rawToReading(s3, b3);
    ch3_updated = true;
  }

  return true;
}

// ═════════════════════════════════════════════════════════════════════════
// §7  AUTO-CALIBRATION — rolling Vfloor / Vpeak tracker
// ═════════════════════════════════════════════════════════════════════════

static void calUpdate(uint8_t i, float v) {
  calCount[i]++;

  if (calCount[i] <= 10) {
    if (v < calVfloor[i]) calVfloor[i] = v;
    if (v > calVpeak[i])  calVpeak[i]  = v;
    return;
  }

  // Vfloor tracking (hard ceiling 3.0V)
  if (v <= VFLOOR_MAX_V) {
    if      (v < calVfloor[i])            calVfloor[i] = v;                      
    else if (v < calVfloor[i] + 1.0f)     calVfloor[i] += CAL_EMA*(v-calVfloor[i]); 
  }
  calVfloor[i] = constrain(calVfloor[i], 0.0f, VFLOOR_MAX_V);

  // Vpeak tracking (working range)
  if (v > VPEAK_MIN_V && v <= VPEAK_MAX_V) {
    if      (v > calVpeak[i])             calVpeak[i] = v;                        
    else if (v > calVpeak[i] - 1.0f)      calVpeak[i] += CAL_EMA*(v-calVpeak[i]); 
  }
  calVpeak[i] = constrain(calVpeak[i], VPEAK_MIN_V, VPEAK_MAX_V);

  if (calVpeak[i] <= calVfloor[i]) calVpeak[i] = calVfloor[i] + 0.1f;
}

static void calReset(uint8_t i) {
  calVfloor[i] = 99.0f;
  calVpeak[i]  = 0.0f;
  calCount[i]  = 0;
}

// ═════════════════════════════════════════════════════════════════════════
// §8  SIGNAL STATE MACHINE — Schmitt-trigger FSM + Adv. Diagnostics
// ═════════════════════════════════════════════════════════════════════════

static inline void trackMinMax(uint32_t v, uint32_t &mn, uint32_t &mx) {
  if (mn == UINT32_MAX || v < mn) mn = v;
  if (v > mx) mx = v;
}

// NEW: Takes current_mA to calculate Inrush and Leakage
static void processSignal(uint8_t i, float v, float c_mA) {
  TimingState &t   = sTiming[i];
  uint32_t    now  = micros();

  if (t.stage == STAGE_NOSIG || t.stage == STAGE_VFLOOR) {
    t.stage = (v <= NO_SIG_V) ? STAGE_NOSIG : STAGE_VFLOOR;
  }

  if (calCount[i] < CAL_WARMUP) return;

  float vF    = calVfloor[i];
  float vP    = calVpeak[i];
  float swing = vP - vF;
  if (swing < CAL_MIN_SWING) return;

  float tUp = vF + swing * THRESH_UP;  
  float tDn = vF + swing * THRESH_DN;  

  bool inverted = (sensorType[i] == 1 || sensorType[i] == 2);

  if (!inverted) {
    switch (t.stage) {
      case STAGE_NOSIG:
      case STAGE_VFLOOR:
        // DIAGNOSTIC: Track absolute maximum leakage current while OFF
        if (c_mA > t.maxLeakage_mA) t.maxLeakage_mA = c_mA;
        // EWMA leakage current (Category 3: Quiescent Current)
        t.avgLeakage_mA = (t.avgLeakage_mA == 0.0f) ? c_mA
                        : t.avgLeakage_mA + 0.05f * (c_mA - t.avgLeakage_mA);

        t.stage = (v <= NO_SIG_V) ? STAGE_NOSIG : STAGE_VFLOOR;
        if (v >= tDn) {
          t.stage      = STAGE_RISING;
          t.t_rise_us  = now;
          t.t_stage_us = now;
          t.inrush_mA  = c_mA; // Reset inrush for new cycle
        }
        break;

      case STAGE_RISING:
        // DIAGNOSTIC: Track peak current spike during turn-on
        if (c_mA > t.inrush_mA) t.inrush_mA = c_mA;

        if (v >= tUp) {                        
          t.RT_us          = now - t.t_rise_us;
          t.stage          = STAGE_STABLE;
          t.t_stable_us    = now;
          t.t_stage_us     = now;
          t.maxPotential_V = v;                
          t.minPotential_V = v; // Setup for ripple tracking
          trackMinMax(t.RT_us, t.RT_min_us, t.RT_max_us);
        } else if (v < tDn) {                  
          t.stage      = (v <= NO_SIG_V) ? STAGE_NOSIG : STAGE_VFLOOR;
          t.t_stage_us = now;
          t.bounceCount++; // DIAGNOSTIC: Contact bounce detected!
        }
        break;

      case STAGE_STABLE:
        // DIAGNOSTIC: Continuously track peak and floor to calculate Ripple later
        if (v > t.maxPotential_V) t.maxPotential_V = v;
        if (v < t.minPotential_V) t.minPotential_V = v;
        // EWMA voltage and active current during STABLE (Category 3 & 4A)
        t.avgVoltage_V        = (t.avgVoltage_V == 0.0f)        ? v    : t.avgVoltage_V        + 0.05f * (v    - t.avgVoltage_V);
        t.avgActiveCurrent_mA = (t.avgActiveCurrent_mA == 0.0f) ? c_mA : t.avgActiveCurrent_mA + 0.05f * (c_mA - t.avgActiveCurrent_mA);

        if (v < tDn) {
          t.ST_us      = now - t.t_stable_us;
          t.stage      = STAGE_FALLING;
          t.t_fall_us  = now;
          t.t_stage_us = now;
          trackMinMax(t.ST_us, t.ST_min_us, t.ST_max_us);
        }
        break;

      case STAGE_FALLING:
        if (v < tDn) {
          t.FT_us = now - t.t_fall_us;
          t.CT_us = t.RT_us + t.ST_us + t.FT_us;
          trackMinMax(t.FT_us, t.FT_min_us, t.FT_max_us);
          t.cycleCount++;

          // Category 4A: Duty Cycle and Saturation Drop
          if (t.CT_us > 0) t.dutyCycle = ((float)t.ST_us / (float)t.CT_us) * 100.0f;
          t.satDrop_V = chipVcc[i] - t.maxPotential_V;

          // Accumulate lifetime bounces before resetting per-cycle count
          t.totalBounceCount += t.bounceCount;

          // ── ALARM ENGINE (only latch if no active alarm — guard prevents overwrite) ──
          if (!sAlarm[i].active) {
            char ts[24]; clkStr(ts, sizeof(ts));

            // Limit checks from $CFG (RT / ST / FT out-of-band)
            SensorLimits &l = sLimits[i];
            if (l.rt_cnt > 0 && (t.RT_us < l.rt_min_us || t.RT_us > l.rt_max_us)) {
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_RT;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:RT_OOB [%luus]", t.RT_us);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:RT_OOB [%luus]*\n", i+1, ts, t.RT_us);
            } else if (l.st_cnt > 0 && (t.ST_us < l.st_min_us || t.ST_us > l.st_max_us)) {
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_ST;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:ST_OOB [%luus]", t.ST_us);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:ST_OOB [%luus]*\n", i+1, ts, t.ST_us);
            } else if (l.ft_cnt > 0 && (t.FT_us < l.ft_min_us || t.FT_us > l.ft_max_us)) {
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_FT;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:FT_OOB [%luus]", t.FT_us);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:FT_OOB [%luus]*\n", i+1, ts, t.FT_us);
            }
            // FAULT_LEAK: idle leakage current too high (Category 3: Quiescent)
            else if (t.maxLeakage_mA > 15.0f && t.cycleCount > 5) {
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_LEAK;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:LEAKAGE [%.1fmA]", t.maxLeakage_mA);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:LEAKAGE [%.1fmA]*\n", i+1, ts, t.maxLeakage_mA);
            }
            // FAULT_SPIKE: inrush > 5× avg active current AND > 50 mA (Category 3: Inrush)
            else if (t.inrush_mA > 50.0f && t.avgActiveCurrent_mA > 0.1f
                     && t.inrush_mA > 5.0f * t.avgActiveCurrent_mA) {
              t.spikeCount++;
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_SPIKE;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:INRUSH_SPIKE [%.0fmA]", t.inrush_mA);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:INRUSH_SPIKE [%.0fmA]*\n", i+1, ts, t.inrush_mA);
            }
          }

          t.stage       = (v <= NO_SIG_V) ? STAGE_NOSIG : STAGE_VFLOOR;
          t.t_stage_us  = now;
          t.bounceCount = 0;  // reset per-cycle bounce counter
        } else if (v >= tUp) {
          t.stage      = STAGE_STABLE;
          t.t_stage_us = now;
        }
        break;
    }
  } else {
    // Inverted logic (PNP-NC / NPN-NO) follows same diagnostic principles
    switch (t.stage) {
      case STAGE_NOSIG:
      case STAGE_VFLOOR:
        if (c_mA > t.maxLeakage_mA) t.maxLeakage_mA = c_mA;
        t.avgLeakage_mA = (t.avgLeakage_mA == 0.0f) ? c_mA
                        : t.avgLeakage_mA + 0.05f * (c_mA - t.avgLeakage_mA);

        t.stage = (v <= NO_SIG_V) ? STAGE_NOSIG : STAGE_VFLOOR;
        if (v < tUp) {
          t.stage      = STAGE_RISING;
          t.t_rise_us  = now;
          t.t_stage_us = now;
          t.inrush_mA  = c_mA;
        }
        break;

      case STAGE_RISING:
        if (c_mA > t.inrush_mA) t.inrush_mA = c_mA;

        if (v <= tDn) {                        
          t.RT_us          = now - t.t_rise_us;
          t.stage          = STAGE_STABLE;
          t.t_stable_us    = now;
          t.t_stage_us     = now;
          t.maxPotential_V = v;                
          t.minPotential_V = v;
          trackMinMax(t.RT_us, t.RT_min_us, t.RT_max_us);
        } else if (v >= tUp) {                 
          t.stage      = STAGE_NOSIG;
          t.t_stage_us = now;
          t.bounceCount++;
        }
        break;

      case STAGE_STABLE:
        if (v < t.maxPotential_V) t.maxPotential_V = v;
        if (v > t.minPotential_V) t.minPotential_V = v;
        t.avgVoltage_V        = (t.avgVoltage_V == 0.0f)        ? v    : t.avgVoltage_V        + 0.05f * (v    - t.avgVoltage_V);
        t.avgActiveCurrent_mA = (t.avgActiveCurrent_mA == 0.0f) ? c_mA : t.avgActiveCurrent_mA + 0.05f * (c_mA - t.avgActiveCurrent_mA);

        if (v > tUp) {
          t.ST_us      = now - t.t_stable_us;
          t.stage      = STAGE_FALLING;
          t.t_fall_us  = now;
          t.t_stage_us = now;
          trackMinMax(t.ST_us, t.ST_min_us, t.ST_max_us);
        }
        break;

      case STAGE_FALLING:
        if (v >= tUp) {
          t.FT_us = now - t.t_fall_us;
          t.CT_us = t.RT_us + t.ST_us + t.FT_us;
          trackMinMax(t.FT_us, t.FT_min_us, t.FT_max_us);
          t.cycleCount++;

          if (t.CT_us > 0) t.dutyCycle = ((float)t.ST_us / (float)t.CT_us) * 100.0f;
          t.satDrop_V = chipVcc[i] - t.maxPotential_V;

          t.totalBounceCount += t.bounceCount;

          if (!sAlarm[i].active) {
            char ts[24]; clkStr(ts, sizeof(ts));
            SensorLimits &l = sLimits[i];
            if (l.rt_cnt > 0 && (t.RT_us < l.rt_min_us || t.RT_us > l.rt_max_us)) {
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_RT;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:RT_OOB [%luus]", t.RT_us);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:RT_OOB [%luus]*\n", i+1, ts, t.RT_us);
            } else if (l.st_cnt > 0 && (t.ST_us < l.st_min_us || t.ST_us > l.st_max_us)) {
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_ST;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:ST_OOB [%luus]", t.ST_us);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:ST_OOB [%luus]*\n", i+1, ts, t.ST_us);
            } else if (l.ft_cnt > 0 && (t.FT_us < l.ft_min_us || t.FT_us > l.ft_max_us)) {
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_FT;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:FT_OOB [%luus]", t.FT_us);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:FT_OOB [%luus]*\n", i+1, ts, t.FT_us);
            } else if (t.maxLeakage_mA > 15.0f && t.cycleCount > 5) {
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_LEAK;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:LEAKAGE [%.1fmA]", t.maxLeakage_mA);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:LEAKAGE [%.1fmA]*\n", i+1, ts, t.maxLeakage_mA);
            } else if (t.inrush_mA > 50.0f && t.avgActiveCurrent_mA > 0.1f
                       && t.inrush_mA > 5.0f * t.avgActiveCurrent_mA) {
              t.spikeCount++;
              sAlarm[i].active = true; sAlarm[i].faultCode = FAULT_SPIKE;
              snprintf(sAlarm[i].cause, sizeof(sAlarm[i].cause), "HW:INRUSH_SPIKE [%.0fmA]", t.inrush_mA);
              strncpy(sAlarm[i].time, ts, sizeof(sAlarm[i].time));
              Serial1.printf("$EVT,%d,%s,HW:INRUSH_SPIKE [%.0fmA]*\n", i+1, ts, t.inrush_mA);
            }
          }

          t.stage       = STAGE_NOSIG;
          t.t_stage_us  = now;
          t.bounceCount = 0;
        } else if (v <= tDn) {
          t.stage      = STAGE_STABLE;
          t.t_stage_us = now;
        }
        break;
    }
  }
}

static void timingReset(uint8_t i) {
  TimingState &t = sTiming[i];
  memset(&t, 0, sizeof(TimingState));
  t.stage      = STAGE_NOSIG;
  t.RT_min_us  = UINT32_MAX;
  t.ST_min_us  = UINT32_MAX;
  t.FT_min_us  = UINT32_MAX;
}

// ═════════════════════════════════════════════════════════════════════════
// §9  NEOPIXEL HELPERS
// ═════════════════════════════════════════════════════════════════════════

static void neoSet(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t sc = NEO_BRIGHT;
  neopixelWrite(NEO_PIN,
    (uint8_t)((uint16_t)g * sc / 255),
    (uint8_t)((uint16_t)r * sc / 255),
    (uint8_t)((uint16_t)b * sc / 255));
}
static void neoOff() { neopixelWrite(NEO_PIN, 0, 0, 0); }

// NOTE: setup()-only — uses delay() intentionally for boot cinematic sequencing.
// Must never be called after FreeRTOS tasks have started (would starve Core 0).
static void neoRainbow() {
  configASSERT(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);
  const uint8_t cols[7][3] = {
    {255,0,0},{255,127,0},{255,255,0},
    {0,255,0},{0,0,255},{75,0,130},{148,0,211}
  };
  for (int c=0; c<7; c++) { neoSet(cols[c][0],cols[c][1],cols[c][2]); delay(120); }
  neoOff(); delay(300);
}

static void neoUpdate() {
  unsigned long now = millis();
  if (now - lastNeo < 200) return;
  lastNeo = now;

  bool anyOn = chipOnline[0]||chipOnline[1]||chipOnline[2]||chipOnline[3];
  bool allOn = chipOnline[0]&&chipOnline[1]&&chipOnline[2]&&chipOnline[3];

  // dispConn is now derived from lastContact timeout (set by readDisplayUART)
  dispConn = (lastContact > 0 && (now - lastContact) < 5000);

  if (dispConn) {
    if (allOn) {
      neoSet(0, 180, 0);                       // solid green  = all OK + display
    } else if (anyOn) {
      blinkSt = !blinkSt;
      blinkSt ? neoSet(180,80,0) : neoOff();   // orange blink = partial
    } else {
      neoSet(180, 0, 0);                       // solid red    = no chips
    }
  } else {
    if (now - lastContact < 5000) {
      blinkSt = !blinkSt;
      blinkSt ? neoSet(0,150,0) : neoOff();    // slow green blink = no display
    } else {
      allOn ? neoSet(0,60,0) : neoSet(80,0,0); // dim idle
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════
// §10  I2C SCAN
// ═════════════════════════════════════════════════════════════════════════

static void runScan() {
  Serial.println("[SCAN] I2C Bus 0  (SDA=41, SCL=17):");
  uint8_t b0[] = {INA1_ADDR, INA2_ADDR};
  for (auto addr : b0) {
    Wire.beginTransmission(addr);
    bool ok = (Wire.endTransmission() == 0);
    Serial.printf("       0x%02X  INA#%d  %s\n",
      addr, (addr==INA1_ADDR)?1:2, ok?"FOUND":"NOT FOUND");
    neoSet(ok?0:180, ok?180:0, 0); delay(200); neoOff(); delay(100);
  }
  Serial.println("[SCAN] I2C Bus 1  (SDA=21, SCL=13):");
  uint8_t b1[] = {INA3_ADDR, INA4_ADDR};
  for (auto addr : b1) {
    Wire1.beginTransmission(addr);
    bool ok = (Wire1.endTransmission() == 0);
    Serial.printf("       0x%02X  INA#%d  %s\n",
      addr, (addr==INA3_ADDR)?3:4, ok?"FOUND":"NOT FOUND");
    neoSet(ok?0:180, ok?180:0, 0); delay(200); neoOff(); delay(100);
  }
}

// ═════════════════════════════════════════════════════════════════════════
// §11  UART TX — PACKETS TO CROWPANEL DISPLAY
// ═════════════════════════════════════════════════════════════════════════

// $D — fast raw V/I packet (every FAST_MS = 15 ms)
static void sendFast(int i) {
  ChannelReading c1,c2,c3; bool on;
  portENTER_CRITICAL(&dataMux);
  c1=chipCH1[i]; c2=chipCH2[i]; c3=chipCH3[i]; on=chipOnline[i];
  portEXIT_CRITICAL(&dataMux);
  if (!on) return;
  Serial1.printf("$D,%d,%lu,%.3f,%.2f,%.3f,%.2f,%.3f,%.2f*\n",
    i+1, millis(),
    c1.voltage_V, c1.current_mA,
    c2.voltage_V, c2.current_mA,
    c3.voltage_V, c3.current_mA);
}

static void sendAllFast() { for(int i=0;i<4;i++) sendFast(i); }

// ─── $S — full data + timing packet (every FULL_MS = 500 ms) ────────────
static void sendFull(int i) {
  ChannelReading c1,c2,c3; bool on;
  portENTER_CRITICAL(&dataMux);
  c1=chipCH1[i]; c2=chipCH2[i]; c3=chipCH3[i]; on=chipOnline[i];
  portEXIT_CRITICAL(&dataMux);
  if (!on) return;

  TimingState &t  = sTiming[i];
  SensorAlarm &al = sAlarm[i];

  uint32_t live_us = micros() - t.t_stage_us;
  float rt_ms = t.RT_us / 1000.0f;
  float st_ms = t.ST_us / 1000.0f;
  float ft_ms = t.FT_us / 1000.0f;
  float ct_ms = t.CT_us / 1000.0f;
  float hz    = (t.CT_us > 0) ? (1000000.0f / (float)t.CT_us) : 0.0f;

  char ts[24]; clkStr(ts, sizeof(ts));
  float swing = calVpeak[i] - calVfloor[i];
  
  // Calculate STABLE stage noise ripple
  float ripple = t.maxPotential_V - t.minPotential_V;

  // NEW FORMAT: Added Inrush, Leakage, Ripple, and Bounce to the end
  Serial1.printf(
    "$S,%d,"
    "%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,"
    "%d,"
    "%lu,%lu,%lu,%lu,"
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
    "%lu,%lu,"
    "%d,%d,%s,%s,"
    "%.3f,%.3f,%.3f,"
    "%.2f,%.2f,%.3f,%lu*\n",
    i+1,
    c1.voltage_V, c1.current_mA, c1.power_mW,
    c2.voltage_V, c2.current_mA, c2.power_mW,
    c3.voltage_V, c3.current_mA, c3.power_mW,
    (int)t.stage,
    t.RT_us, t.ST_us, t.FT_us, t.CT_us,
    rt_ms, st_ms, ft_ms, ct_ms, hz, t.maxPotential_V,
    t.cycleCount, live_us,
    al.faultCode, (int)al.active, al.cause, ts,
    calVfloor[i], calVpeak[i], swing,
    t.inrush_mA, t.maxLeakage_mA, ripple, t.bounceCount);

  // ── $LOG — CSV archive record emitted once per completed cycle ──────────
  // Only fires when cycleCount advances beyond what we last logged.
  // Format: $LOG,slno,portID,timestamp,Vin_V,Iin_mA,Vout_V,Iout_V,Vfb_V,
  //              RT_us,ST_us,FT_us,CT_us,Hz,Vpeak_V,Vfloor_V,swing_V,
  //              inrush_mA,leakage_mA,ripple_V,bounceCount,totalBounce,
  //              dutyCycle,satDrop_V,ovCnt,uvCnt,spikeCnt,faultCode*
  if (t.cycleCount > 0 && t.cycleCount != lastLoggedCycle[i]) {
    lastLoggedCycle[i] = t.cycleCount;
    Serial1.printf(
      "$LOG,%lu,%d,%s,"
      "%.3f,%.2f,%.3f,%.2f,%.3f,%.2f,"
      "%lu,%lu,%lu,%lu,%.3f,"
      "%.3f,%.3f,%.3f,"
      "%.2f,%.2f,%.3f,%lu,%lu,"
      "%.2f,%.3f,%lu,%lu,%lu,%d*\n",
      ++logSlno, i+1, ts,
      c1.voltage_V, c1.current_mA,
      c2.voltage_V, c2.current_mA,
      c3.voltage_V, c3.current_mA,
      t.RT_us, t.ST_us, t.FT_us, t.CT_us, hz,
      calVpeak[i], calVfloor[i], swing,
      t.inrush_mA, t.maxLeakage_mA, ripple, t.bounceCount, t.totalBounceCount,
      t.dutyCycle, t.satDrop_V,
      t.ovVoltCount, t.uvVoltCount, t.spikeCount,
      al.faultCode);
  }
}

static void sendAllFull() { for(int i=0;i<4;i++) sendFull(i); }

// $H — heartbeat (every BEAT_MS = 1000 ms)
static void sendHeartbeat() {
  Serial1.printf("$H,%.1f,%d,%d,%d,%d*\n",
    liveReadHz,
    (int)chipOnline[0], (int)chipOnline[1],
    (int)chipOnline[2], (int)chipOnline[3]);

  // $ALRM — compact 4-port alarm status broadcast piggybacked on heartbeat.
  // Lets the display resync all alarm states with a single packet on reconnect.
  // Format: $ALRM,fc1,act1,fc2,act2,fc3,act3,fc4,act4*
  Serial1.printf("$ALRM,%d,%d,%d,%d,%d,%d,%d,%d*\n",
    sAlarm[0].faultCode, (int)sAlarm[0].active,
    sAlarm[1].faultCode, (int)sAlarm[1].active,
    sAlarm[2].faultCode, (int)sAlarm[2].active,
    sAlarm[3].faultCode, (int)sAlarm[3].active);
}

// $L — limit packet for one sensor
static void sendLimits(int i) {
  SensorLimits &l = sLimits[i];
  Serial1.printf("$L,%d,%lu,%lu,%d,%lu,%lu,%d,%lu,%lu,%d*\n",
    i+1,
    l.rt_min_us, l.rt_max_us, l.rt_cnt,
    l.st_min_us, l.st_max_us, l.st_cnt,
    l.ft_min_us, l.ft_max_us, l.ft_cnt);
}

// ═════════════════════════════════════════════════════════════════════════
// §12  UART RX — COMMANDS FROM CROWPANEL DISPLAY
// ═════════════════════════════════════════════════════════════════════════

static void handleCmd(const char *cmd) {
  int  sid, type; unsigned long mn, mx; uint8_t cnt;
  int  d, mo, yr, h, mi, s;
  char ts[24]; clkStr(ts, sizeof(ts));

  // $CFG,sid,type,min_us,max_us,cnt* — set alarm limits
  if (sscanf(cmd,"$CFG,%d,%d,%lu,%lu,%hhu",&sid,&type,&mn,&mx,&cnt)==5
      && sid>=1 && sid<=4) {
    int k=sid-1;
    if      (type==0) { sLimits[k].rt_min_us=mn; sLimits[k].rt_max_us=mx; sLimits[k].rt_cnt=cnt; }
    else if (type==1) { sLimits[k].st_min_us=mn; sLimits[k].st_max_us=mx; sLimits[k].st_cnt=cnt; }
    else if (type==2) { sLimits[k].ft_min_us=mn; sLimits[k].ft_max_us=mx; sLimits[k].ft_cnt=cnt; }
    Serial1.printf("$ACK,CFG,%d,%d,%lu,%lu,%u,%s*\n",sid,type,mn,mx,cnt,ts);
    return;
  }

  // $RST,sid* — clear alarm
  if (sscanf(cmd,"$RST,%d",&sid)==1 && sid>=1 && sid<=4) {
    memset(&sAlarm[sid-1], 0, sizeof(SensorAlarm));
    sAlarm[sid-1].faultCode = FAULT_NONE;
    Serial1.printf("$ACK,RST,%d,%s*\n",sid,ts);
    return;
  }

  // $TIME,d,mo,yr,h,mi,s* — sync clock
  if (sscanf(cmd,"$TIME,%d,%d,%d,%d,%d,%d",&d,&mo,&yr,&h,&mi,&s)==6) {
    clkDay=d; clkMon=mo; clkYr=yr; clkH=h; clkM=mi; clkS=s;
    clkBase=millis(); clkSync=true; clkReset=true;
    Serial1.printf("$ACK,TIME,%02d/%02d/%04d %02d:%02d:%02d*\n",d,mo,yr,h,mi,s);
    return;
  }

  // $GETL,0* — send all limit packets
  if (strncmp(cmd,"$GETL,",6)==0) {
    for (int i=0;i<4;i++) if(chipOnline[i]) sendLimits(i);
    Serial1.printf("$ACK,GETL,%s*\n",ts);
    return;
  }

  // $STYPE,sid,type* — change sensor type (resets cal + timing)
  if (sscanf(cmd,"$STYPE,%d,%d",&sid,&type)==2 && sid>=1 && sid<=4) {
    sensorType[sid-1] = (uint8_t)constrain(type,0,4);
    calReset(sid-1);
    timingReset(sid-1);
    lastLoggedCycle[sid-1] = 0;   // prevent stale $LOG repeat after type change
    Serial1.printf("$ACK,STYPE,%d,%d,%s*\n",sid,type,ts);
    return;
  }

  // $RCAL,sid* — reset rolling calibration + timing
  if (sscanf(cmd,"$RCAL,%d",&sid)==1 && sid>=1 && sid<=4) {
    calReset(sid-1);
    timingReset(sid-1);
    lastLoggedCycle[sid-1] = 0;   // prevent stale $LOG repeat after recal
    Serial1.printf("$ACK,RCAL,%d,%s*\n",sid,ts);
    return;
  }

  // NEW: $FB,sid,state* — Toggle Feedback channel (1 = ON, 0 = OFF)
  if (sscanf(cmd,"$FB,%d,%d",&sid,&type)==2 && sid>=1 && sid<=4) {
    fbEnabled[sid-1] = (type == 1);
    chipWasOnline[sid-1] = false; // Forces the chip to reboot into the new hardware mode!
    Serial1.printf("$ACK,FB,%d,%d,%s*\n",sid,type,ts);
    return;
  }

  // $STAT,0* — force immediate $S dump for ALL ports (must be checked before $STAT,sid)
  if (strncmp(cmd,"$STAT,0",7)==0) {
    for (int i=0;i<4;i++) sendFull(i);
    Serial1.printf("$ACK,STAT,0,%s*\n",ts);
    return;
  }

  // $STAT,sid* — force immediate $S dump for one port (display uses on reconnect)
  if (sscanf(cmd,"$STAT,%d",&sid)==1 && sid>=1 && sid<=4) {
    sendFull(sid-1);
    Serial1.printf("$ACK,STAT,%d,%s*\n",sid,ts);
    return;
  }

  // NEW: $PING* — liveness handshake (display checks circuit is alive)
  if (strncmp(cmd,"$PING",5)==0) {
    Serial1.printf("$PONG,%lu,%.1f,%d,%d,%d,%d*\n",
      millis(), liveReadHz,
      (int)chipOnline[0],(int)chipOnline[1],
      (int)chipOnline[2],(int)chipOnline[3]);
    return;
  }

  // NEW: $ALRST,0* — clear ALL alarms at once (bulk reset)
  if (strncmp(cmd,"$ALRST,0",8)==0) {
    for (int i=0;i<4;i++) {
      memset(&sAlarm[i], 0, sizeof(SensorAlarm));
      sAlarm[i].faultCode = FAULT_NONE;
    }
    Serial1.printf("$ACK,ALRST,0,%s*\n",ts);
    return;
  }
}

// Character-by-character UART1 RX parser  (called from loop, Core 0)
static void readDisplayUART() {
  bool gotBytes = false;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '*') {
      rxBuf[rxIdx] = '\0';
      handleCmd(rxBuf);
      rxIdx = 0;
    } else if (c == '$') {
      rxIdx = 0;
      rxBuf[rxIdx++] = c;
    } else if (rxIdx < RX_BUF - 1) {
      rxBuf[rxIdx++] = c;
    }
    gotBytes = true;
  }
  // Update lastContact here so neoUpdate() timeout works independently
  // of dispConn — dispConn is now derived from the 5s timeout in neoUpdate()
  if (gotBytes) lastContact = millis();
}

// ═════════════════════════════════════════════════════════════════════════
// §13  USB SERIAL DEBUG COMMANDS
// ═════════════════════════════════════════════════════════════════════════

static const char *stageName(uint8_t s) {
  static const char *n[] = {"NO_SIGNAL","VFLOOR","RISING","STABLE","FALLING"};
  return (s < 5) ? n[s] : "?";
}

static void printTimingFull(int i) {
  TimingState &t = sTiming[i];
  bool rdy  = (calCount[i] >= CAL_WARMUP);
  float vF  = calVfloor[i];
  float vP  = calVpeak[i];
  float sw  = vP - vF;
  float rip = t.maxPotential_V - t.minPotential_V;

  Serial.printf("  ┌─ Port P%d  [%s]  count=%lu\n",
    i+1, rdy?"CALIBRATED":"WARMING UP", calCount[i]);
  Serial.printf("  │  Vfloor_cal = %.3f V   Vpeak_cal = %.3f V   Swing = %.3f V\n",
    vF, vP, sw);

  if (rdy && sw >= CAL_MIN_SWING) {
    Serial.printf("  │  Schmitt  threshUp = %.3f V  (%.0f%%)   threshDn = %.3f V  (%.0f%%)\n",
      vF+sw*THRESH_UP, THRESH_UP*100, vF+sw*THRESH_DN, THRESH_DN*100);
  }

  Serial.printf("  │  Stage    : %-10s   Max Potential : %.3f V   Cycles : %lu\n",
    stageName(t.stage), t.maxPotential_V, t.cycleCount);

  // NEW: Print Advanced Diagnostics
  Serial.printf("  │  ── ADVANCED DIAGNOSTICS ────────────────────────────\n");
  Serial.printf("  │  Inrush Spike  : %8.1f mA     Leakage (max): %8.2f mA\n", t.inrush_mA, t.maxLeakage_mA);
  Serial.printf("  │  EWMA Voltage  : %8.3f V      EWMA Active I: %8.2f mA\n", t.avgVoltage_V, t.avgActiveCurrent_mA);
  Serial.printf("  │  EWMA Leakage  : %8.3f mA     Duty Cycle   : %8.2f %%\n",  t.avgLeakage_mA, t.dutyCycle);
  Serial.printf("  │  Sat Drop      : %8.3f V      Noise Ripple : %8.3f V\n",   t.satDrop_V, rip);
  Serial.printf("  │  OV Events     : %8lu        UV Events    : %8lu\n",       t.ovVoltCount, t.uvVoltCount);
  Serial.printf("  │  Bounce/cycle  : %8lu        Total Bounce : %8lu\n",       t.bounceCount, t.totalBounceCount);
  Serial.printf("  │  Spike Events  : %8lu\n",                                   t.spikeCount);

  Serial.printf("  │  ── LAST COMPLETED CYCLE ────────────────────────────\n");
  Serial.printf("  │  Rise  Time  : %8lu µs   =  %10.3f ms\n", t.RT_us, t.RT_us/1000.0f);
  Serial.printf("  │  Stable Time : %8lu µs   =  %10.3f ms\n", t.ST_us, t.ST_us/1000.0f);
  Serial.printf("  │  Fall  Time  : %8lu µs   =  %10.3f ms\n", t.FT_us, t.FT_us/1000.0f);
  Serial.printf("  │  Cycle Time  : %8lu µs   =  %10.3f ms   (%.3f Hz)\n",
    t.CT_us, t.CT_us/1000.0f,
    (t.CT_us>0)?(1000000.0f/t.CT_us):0.0f);

  Serial.printf("  │  ── MIN / MAX  (all completed cycles) ────────────────\n");
  if (t.RT_min_us == UINT32_MAX)
    Serial.printf("  │  Rise  min/max : (no data yet)\n");
  else
    Serial.printf("  │  Rise  min/max : %lu µs  /  %lu µs\n", t.RT_min_us, t.RT_max_us);
  if (t.ST_min_us == UINT32_MAX)
    Serial.printf("  │  Stable min/max: (no data yet)\n");
  else
    Serial.printf("  │  Stable min/max: %lu µs  /  %lu µs\n", t.ST_min_us, t.ST_max_us);
  if (t.FT_min_us == UINT32_MAX)
    Serial.printf("  │  Fall  min/max : (no data yet)\n");
  else
    Serial.printf("  │  Fall  min/max : %lu µs  /  %lu µs\n", t.FT_min_us, t.FT_max_us);

  Serial.printf("  └  LIVE elapsed in current stage : %lu µs\n", micros()-t.t_stage_us);
}

static void handleUSB(const String &cmd) {
  if (cmd == "SCAN") {
    runScan();

  } else if (cmd == "RATE") {
    Serial.println("── PERFORMANCE ──────────────────────────────────────────────");
    Serial.printf("   Read rate : %.1f Hz\n", liveReadHz);
    Serial.printf("   Total     : %lu reads\n", totalReads);
    Serial.printf("   UART1     : %d baud\n", UART_BAUD);
    for (int i=0;i<4;i++)
      Serial.printf("   P%d : %-8s  IN=%.3fV  OUT=%.3fV  FB=%.3fV  [FB %s]\n",
        i+1, chipOnline[i]?"LIVE":"OFFLINE",
        (float)chipCH1[i].voltage_V,
        (float)chipCH2[i].voltage_V,
        (float)chipCH3[i].voltage_V,
        fbEnabled[i]?"ON ":"OFF");
    Serial.println("─────────────────────────────────────────────────────────────");

  } else if (cmd == "LIVE") {
    for (int i=0;i<4;i++) {
      if (!chipOnline[i]) { Serial.printf("P%d: OFFLINE\n",i+1); continue; }
      Serial.printf("$D,%d,%lu,%.3f,%.2f,%.3f,%.2f,%.3f,%.2f*\n",
        i+1, millis(),
        (float)chipCH1[i].voltage_V,(float)chipCH1[i].current_mA,
        (float)chipCH2[i].voltage_V,(float)chipCH2[i].current_mA,
        (float)chipCH3[i].voltage_V,(float)chipCH3[i].current_mA);
    }

  } else if (cmd == "CAL") {
    Serial.println("── CALIBRATION STATUS ───────────────────────────────────────");
    for (int i=0;i<4;i++) {
      bool rdy = (calCount[i] >= CAL_WARMUP);
      float sw = calVpeak[i]-calVfloor[i];
      Serial.printf("   P%d  [%s]  count=%-5lu  Vfl=%.3fV  Vpk=%.3fV  Swing=%.3fV\n",
        i+1, rdy?"READY ":"WARMUP",
        calCount[i], calVfloor[i], calVpeak[i], sw);
    }
    Serial.println("─────────────────────────────────────────────────────────────");

  } else if (cmd == "TIMING") {
    Serial.println("── TIMING ENGINE  (µs precision) ────────────────────────────");
    for (int i=0;i<4;i++) printTimingFull(i);
    Serial.println("─────────────────────────────────────────────────────────────");

  } else if (cmd.length() > 0) {
    Serial.println("Commands:  SCAN   RATE   LIVE   CAL   TIMING");
    Serial.println("           SCAN   — I2C bus scan");
    Serial.println("           RATE   — read speed + per-port live V/I");
    Serial.println("           LIVE   — one-shot $D packet dump to USB");
    Serial.println("           CAL    — calibration Vfloor/Vpeak status");
    Serial.println("           TIMING — full diagnostic engine state");
  }
}

// ═════════════════════════════════════════════════════════════════════════
// §14  CORE 1 — BURST READER TASK
//
//  Pinned to Core 1, priority 2 (above loop at priority 1).
//  Reads CH2 continuously, interleaving CH1/CH3 to maximize loop speed.
// ═════════════════════════════════════════════════════════════════════════

static void readerTask(void *pv) {
  (void)pv;
  esp_task_wdt_add(NULL);

  uint32_t      rCount = 0;
  unsigned long rStart = millis();

  for (;;) {
    esp_task_wdt_reset();
    ChannelReading c1, c2, c3;
    bool up1, up3; // Flags to track if background channels were read this loop

    // ─── I2C Bus 0 : INA #1 (index 0) and INA #2 (index 1) ────────────
    if (xSemaphoreTake(mtx0, 0) == pdTRUE) {

      // ── INA #1 (Port 1 — index 0)
      if (chipRead(Wire, INA1_ADDR, fbEnabled[0], c1, c2, c3, up1, up3, 0)) {
        if (!chipWasOnline[0]) {
          inaTurbo(Wire, INA1_ADDR, fbEnabled[0]);
          chipWasOnline[0] = true;
        }
        portENTER_CRITICAL(&dataMux);
        chipCH2[0] = c2;
        if (up1) chipCH1[0] = c1;
        if (up3) chipCH3[0] = c3; else if (!fbEnabled[0]) chipCH3[0] = {0,0,0,false};
        chipOnline[0] = true;
        if (up1 && c1.valid && c1.voltage_V >= 1.0f) {
          chipVcc[0] = c1.voltage_V;
          // OV/UV event counting (Category 1: Supply Voltage)
          if (c1.voltage_V > 26.0f)                              sTiming[0].ovVoltCount++;
          if (c1.voltage_V > 1.0f && c1.voltage_V < 18.0f)      sTiming[0].uvVoltCount++;
        }
        portEXIT_CRITICAL(&dataMux);
        if (c2.valid) { calUpdate(0, c2.voltage_V); processSignal(0, c2.voltage_V, c2.current_mA); }
      } else { chipOnline[0] = false; chipWasOnline[0] = false; }

      // ── INA #2 
      if (chipRead(Wire, INA2_ADDR, fbEnabled[1], c1, c2, c3, up1, up3, 1)) {
        if (!chipWasOnline[1]) { 
          inaTurbo(Wire, INA2_ADDR, fbEnabled[1]); 
          chipWasOnline[1] = true; 
        }
        portENTER_CRITICAL(&dataMux);
        chipCH2[1] = c2; 
        if (up1) chipCH1[1] = c1; 
        if (up3) chipCH3[1] = c3; else if (!fbEnabled[1]) chipCH3[1] = {0,0,0,false};
        chipOnline[1] = true;
        if (up1 && c1.valid && c1.voltage_V >= 1.0f) {
          chipVcc[1] = c1.voltage_V;
          // OV/UV event counting (Category 1: Supply Voltage)
          if (c1.voltage_V > 26.0f)                          sTiming[1].ovVoltCount++;
          if (c1.voltage_V > 1.0f && c1.voltage_V < 18.0f)  sTiming[1].uvVoltCount++;
        }
        portEXIT_CRITICAL(&dataMux);
        
        if (c2.valid) { 
          calUpdate(1, c2.voltage_V); 
          processSignal(1, c2.voltage_V, c2.current_mA); 
        }
      } else { 
        chipOnline[1] = false; chipWasOnline[1] = false; 
      }

      xSemaphoreGive(mtx0);
    }

    // ─── I2C Bus 1 : INA #3 (index 2) and INA #4 (index 3) ────────────
    if (xSemaphoreTake(mtx1, 0) == pdTRUE) {

      // ── INA #3 
      if (chipRead(Wire1, INA3_ADDR, fbEnabled[2], c1, c2, c3, up1, up3, 2)) {
        if (!chipWasOnline[2]) { 
          inaTurbo(Wire1, INA3_ADDR, fbEnabled[2]); 
          chipWasOnline[2] = true; 
        }
        portENTER_CRITICAL(&dataMux);
        chipCH2[2] = c2; 
        if (up1) chipCH1[2] = c1; 
        if (up3) chipCH3[2] = c3; else if (!fbEnabled[2]) chipCH3[2] = {0,0,0,false};
        chipOnline[2] = true;
        if (up1 && c1.valid && c1.voltage_V >= 1.0f) {
          chipVcc[2] = c1.voltage_V;
          // OV/UV event counting (Category 1: Supply Voltage)
          if (c1.voltage_V > 26.0f)                          sTiming[2].ovVoltCount++;
          if (c1.voltage_V > 1.0f && c1.voltage_V < 18.0f)  sTiming[2].uvVoltCount++;
        }
        portEXIT_CRITICAL(&dataMux);
        
        if (c2.valid) { 
          calUpdate(2, c2.voltage_V); 
          processSignal(2, c2.voltage_V, c2.current_mA); 
        }
      } else { 
        chipOnline[2] = false; chipWasOnline[2] = false; 
      }

      // ── INA #4 
      if (chipRead(Wire1, INA4_ADDR, fbEnabled[3], c1, c2, c3, up1, up3, 3)) {
        if (!chipWasOnline[3]) { 
          inaTurbo(Wire1, INA4_ADDR, fbEnabled[3]); 
          chipWasOnline[3] = true; 
        }
        portENTER_CRITICAL(&dataMux);
        chipCH2[3] = c2; 
        if (up1) chipCH1[3] = c1; 
        if (up3) chipCH3[3] = c3; else if (!fbEnabled[3]) chipCH3[3] = {0,0,0,false};
        chipOnline[3] = true;
        if (up1 && c1.valid && c1.voltage_V >= 1.0f) {
          chipVcc[3] = c1.voltage_V;
          // OV/UV event counting (Category 1: Supply Voltage)
          if (c1.voltage_V > 26.0f)                          sTiming[3].ovVoltCount++;
          if (c1.voltage_V > 1.0f && c1.voltage_V < 18.0f)  sTiming[3].uvVoltCount++;
        }
        portEXIT_CRITICAL(&dataMux);
        
        if (c2.valid) { 
          calUpdate(3, c2.voltage_V); 
          processSignal(3, c2.voltage_V, c2.current_mA); 
        }
      } else { 
        chipOnline[3] = false; chipWasOnline[3] = false; 
      }

      xSemaphoreGive(mtx1);
    }

    // ── Read-rate counter ──
    rCount++;
    totalReads++;
    unsigned long now = millis();
    if (now - rStart >= 1000) {
      liveReadHz = rCount * 1000.0f / (float)(now - rStart);
      rCount = 0;
      rStart = now;
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════
// §15  SETUP
// ═════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  { unsigned long t=millis(); while (!Serial && millis()-t<2000) delay(10); }

  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║  VIPx3X4s  Firmware v4.2  —  HIGH-SPEED INTERLEAVED ENGINE   ║");
  Serial.println("║  ESP32-S3  |  4× INA3221  |  12-Channel Real-Time V/I/P      ║");
  Serial.println("║  Diagnostics: Inrush, Leakage, Ripple, Bounce, OV/UV, $LOG   ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝");

  mtx0 = xSemaphoreCreateMutex();
  mtx1 = xSemaphoreCreateMutex();

  // Arm the hardware watchdog — readerTask must call esp_task_wdt_reset()
  // within this window or the ESP32-S3 will hard-reset.
  // Compatible with both ESP-IDF 4.x and 5.x Arduino cores
  esp_task_wdt_init(8, true);  // 8-second timeout, panic on expire

  for (int i=0; i<4; i++) {
    memset(&sLimits[i], 0, sizeof(SensorLimits));
    memset(&sAlarm[i],  0, sizeof(SensorAlarm));
    sAlarm[i].faultCode = FAULT_NONE;
    timingReset(i);                               
    chipOnline[i]    = false;
    chipWasOnline[i] = false;
    fbEnabled[i]     = false; // Default: FB Off for maximum speed
    chipCH1[i] = {0,0,0,false};
    chipCH2[i] = {0,0,0,false};
    chipCH3[i] = {0,0,0,false};
  }

  Serial1.setRxBufferSize(2048);
  Serial1.setTxBufferSize(1024);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  Serial.printf("[UART] %d baud   TX=GPIO%d   RX=GPIO%d\n", UART_BAUD, UART_TX, UART_RX);

  neoRainbow();

  pinMode(PWR_PIN_A, OUTPUT); digitalWrite(PWR_PIN_A, HIGH);
  pinMode(PWR_PIN_B, OUTPUT); digitalWrite(PWR_PIN_B, HIGH);
  delay(100);

  Wire.begin(I2C0_SDA, I2C0_SCL);  Wire.setClock(I2C_FREQ);
  Wire1.begin(I2C1_SDA, I2C1_SCL); Wire1.setClock(I2C_FREQ);
  delay(100);

  Serial.println("[CFG] Booting INA3221s into Ultra-Fast Interleaved Mode...");
  inaTurbo(Wire,  INA1_ADDR, fbEnabled[0]);
  inaTurbo(Wire,  INA2_ADDR, fbEnabled[1]);
  inaTurbo(Wire1, INA3_ADDR, fbEnabled[2]);
  inaTurbo(Wire1, INA4_ADDR, fbEnabled[3]);

  runScan();

  xTaskCreatePinnedToCore(readerTask, "Reader", 8192, NULL, 2, NULL, 1);

  unsigned long t0 = millis();
  lastFast = lastFull = lastBeat = lastDiag = lastHealth = t0;

  Serial.println("══════════════════════════════════════════════════════════════");
  Serial.println("  SYSTEM ARMED AND READY");
  Serial.println("══════════════════════════════════════════════════════════════");
}

// ═════════════════════════════════════════════════════════════════════════
// §16  LOOP  —  Core 0 only
// ═════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  neoUpdate();
  readDisplayUART();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      usbCmd.trim(); usbCmd.toUpperCase();
      if (usbCmd.length() > 0) handleUSB(usbCmd);
      usbCmd = "";
    } else if (usbCmd.length() < 64) { 
      usbCmd += c; // Prevent memory leak from infinite strings
    }
  }

  if (now - lastFast   >= FAST_MS)  { lastFast  = now; sendAllFast();   }
  if (now - lastFull   >= FULL_MS)  { lastFull  = now; sendAllFull();   }
  if (now - lastBeat   >= BEAT_MS)  { lastBeat  = now; sendHeartbeat(); }

  if (now - lastDiag >= DIAG_MS) {
    lastDiag = now;
    Serial.printf("[DIAG %lums] %.0f Hz | %lu total reads | UART %d baud\n",
      now, liveReadHz, totalReads, UART_BAUD);
    for (int i=0;i<4;i++) {
      if (!chipOnline[i]) continue;
      TimingState &t = sTiming[i];
      float hz = (t.CT_us>0) ? (1000000.0f/t.CT_us) : 0.0f;
      Serial.printf(
        "  P%d %-9s  OUT=%.3fV  RT=%luµs  ST=%luµs  FT=%luµs"
        "  CT=%luµs  %.2fHz  Vpk=%.3fV  #%lu\n",
        i+1, stageName(t.stage),
        (float)chipCH2[i].voltage_V,
        t.RT_us, t.ST_us, t.FT_us, t.CT_us,
        hz, t.maxPotential_V, t.cycleCount);
    }
  }
  if (now - lastHealth >= HEALTH_MS) {
    lastHealth = now;
    for (int i=0;i<4;i++)
      if (!chipOnline[i]) Serial.printf("[HEALTH] INA #%d is OFFLINE\n", i+1);
  }
}