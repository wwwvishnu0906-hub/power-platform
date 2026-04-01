# Workspace History Log

This file tracks every file import, edit, and change made to this workspace.
Each entry records the date, source, files affected, and a summary of the operation.
All timestamps are in ISO 8601 format (YYYY-MM-DD). Entries are appended chronologically.

---

## 2026-04-01 — Import: VIPx3X4s Arduino Project

**Source path:** `C:\Users\wwwvi\OneDrive\Desktop\VIPx3X4s\`
**Destination:** `arduino/VIPx3X4s/`
**Operation:** Full folder copy (all files preserved, no modifications)

### Files copied

| File | Size (approx) | Role |
|---|---|---|
| `VIPx3X4s_circuit_final_F1.ino` | ~800 lines | ESP32-S3 circuit board firmware |
| `VIPx3X4s_display_final__F1s.ino` | ~2000+ lines | ESP32-S3 CrowPanel 7.0" HMI firmware |

### File summaries

#### `VIPx3X4s_circuit_final_F1.ino`
Firmware for a custom ESP32-S3 circuit board driving **4× INA3221 triple-channel power monitors** (12 total measurement channels) across two I2C buses. Implements a **5-stage Schmitt-trigger FSM** per sensor port (NOSIG → VFLOOR → RISING → STABLE → FALLING) with auto-calibration of Vfloor/Vpeak via rolling EMA. Computes real-time Rise Time, Stable Time, Fall Time, Cycle Time, and Frequency in microsecond resolution. Emits structured UART packets to the display at up to ~1190 Hz: `$D` (fast raw V/I/P every 15ms), `$S` (full timing + diagnostics every 500ms), `$H` (heartbeat 1Hz), `$LOG` (CSV archive per cycle), `$ALRM`, `$EVT`. Receives display commands: `$CFG`, `$RST`, `$TIME`, `$STYPE`, `$RCAL`, `$FB`, `$STAT`, `$PING`, `$ALRST`. Runs on FreeRTOS dual-core (Core 1 = I2C reads, Core 0 = UART/logic). Includes advanced diagnostics: inrush spike detection, leakage current tracking, contact bounce counting, duty cycle, saturation drop, voltage ripple, OV/UV event counters, NeoPixel health LED.

#### `VIPx3X4s_display_final__F1s.ino`
Full diagnostic HMI dashboard for the **CrowPanel 7.0" 800×480 RGB ESP32-S3** display. Configured via LovyanGFX with hardware RGB bus, GT911 capacitive touch (I2C), and PCA9557 I/O expander for backlight. Parses UART packets from the circuit board and renders a **7-tab UI**: HOME (live sensor overview), CONFIG (alarm limits), GRAPH (waveform oscilloscope, 1000-point 15s window), COMPARE (historical vs. live), LOG/ALERT (cycle flight recorder + alarm ledger), SETTING (operator ID, date/time), STORAGE (SD archive browser with hierarchical PlantA/B → Line → Option → System tree). Implements a **12-factor weighted health scoring engine** (timing, signal, quality, fault categories) with CUSUM/EWMA statistical drift detection (Category 14) and predicted Remaining Useful Life (RUL). Archives data to SD card as daily CSV logs, event logs, and per-sensor diagnostic snapshots. Supports operator login via numpad, software clock independent from circuit clock (synced via `$TIME` command).

### Bugs identified

1. **`dispConn` never times out correctly** (`circuit`, `readDisplayUART()` line ~984): `dispConn = true` is set inside the UART read loop on every byte, but `lastContact` (used for 5s timeout logic in `neoUpdate()`) is only updated inside `neoUpdate()` when `dispConn` is already true — creating a circular dependency. Once any byte arrives, `dispConn` stays `true` permanently regardless of timeout.

2. **ST/FT alarm index swap** (`display`, `parseLimitPacket()` line ~867): The circuit sends RT=type0, ST=type1, FT=type2, but the display maps `alarms[2] = ST` and `alarms[1] = FT`, swapping Stable Time and Fall Time alarm thresholds.

3. **OV/UV counters are level-triggered, not edge-triggered** (`display`, `parseSensorPacket()` lines ~817-818): `ovVoltCount` and `uvVoltCount` increment on every `$S` packet (every 500ms) while the condition holds, rather than counting transitions. A 10-second over-voltage event generates 20 counts instead of 1.

4. **Display software clock has no reset mechanism** (`display`, `updateSoftwareClock()` line ~548): The static origin variables (origHour, origMin, etc.) initialize once and never update if `dtHour` changes (e.g., after user sets the time). The circuit's `clkTick()` has a `clkReset` flag for this but the display equivalent does not.

5. **`neoRainbow()` uses blocking `delay()`** (`circuit`, line ~685): Calls `delay(120)` and `delay(300)` per color cycle. Only safe during boot cinematic, but the function is not guarded or annotated as setup-only — calling it at runtime would block the main loop and starve UART/sensors.

6. **Inverted sensor STAGE_NOSIG hardcoded** (`circuit`, `processSignal()` inverted branch line ~559): In the inverted logic path (PNP-NC/NPN-NO), the stage is set to `STAGE_NOSIG` unconditionally rather than checking the `NO_SIG_V` threshold like the normal path does — inverted sensors never enter `STAGE_VFLOOR`.

7. **`$STAT,0` handler reachable only by accident** (`circuit`, `handleCmd()` lines ~937-948): `$STAT,sid` sscanf succeeds with `sid=0` but the bounds check `sid>=1` rejects it, falling through to the `strncmp("$STAT,0")` handler. Correct by coincidence but fragile.
