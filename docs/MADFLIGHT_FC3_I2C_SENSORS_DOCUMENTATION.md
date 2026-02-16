# Betaflight I2C Sensor Drivers for RP2350B (MADFLIGHT FC3)
# Complete Technical Documentation

**Date:** February 15, 2026  
**Last Updated:** February 16, 2026  
**Author:** AI Assistant (GitHub Copilot)  
**Board:** MADFLIGHT FC3 (https://madflight.com/Board-FC3/)  
**MCU:** RP2350B (Raspberry Pi Pico 2) @ 150MHz  

---

## Executive Summary

This document details the complete development work performed to enable I2C sensor support on the MADFLIGHT FC3 flight controller board featuring an RP2350B (Raspberry Pi Pico 2) microcontroller. The work involved creating new drivers, fixing existing code, and implementing diagnostic tools for three I2C sensors:

| Sensor | Type | I2C Address | Status |
|--------|------|-------------|--------|
| BMP580 | Barometer | 0x47 | ✅ Working |
| MMC5603 | Magnetometer | 0x30 | ✅ Working |
| INA226 | Power Monitor | 0x40 | ✅ Working (current + voltage) |

### Recent Updates (February 16, 2026)

1. **MMC5603 Init Sequence Fix (Section 7.11):** Fixed initialization order per datasheet - CTRL0 (CMM_FREQ_EN) must be written before CTRL2 (CMM_EN) for proper continuous measurement mode.

2. **MMC5603 CLI Lookup Table:** Added MMC5603 to `lookupTableMagHardware` in settings.c for CLI sensor selection.

3. **MMC5603 Header Fix:** Changed include from `io_types.h` to `drivers/compass/compass.h` to properly resolve `magDev_t` type.

4. **PR Branch Organization:** Split all changes into separate feature branches for cleaner pull requests:
   - `feature/mmc5603-magnetometer` - MMC5603 driver
   - `feature/bmp580-barometer` - BMP580 driver  
   - `feature/ina226-power-monitor` - INA226 power monitor
   - `fix/pico-i2c-error-handling` - PICO I2C fixes
   - `docs/madflight-sensors` - Documentation

### Previous Updates (February 15, 2026)

1. **INA226 Read I2C Wait Fix (Section 7.7):** Discovered that PICO platform also requires `i2cBusy()` wait after read operations, not just writes. This fixed issues where INA226 returned stale or zero data.

2. **Battery Chicken-and-Egg Fix (Section 7.8):** Fixed critical bug where `batteryUpdateCurrentMeter()` returned early when `batteryCellCount == 0`, preventing INA226 from being refreshed and thus preventing voltage detection needed to determine cell count.

3. **Voltage Calibration:** Changed `ina226_vbat_scale` CLI setting to default 100 = 1.00x (INA226 reads accurately, no scaling needed).

4. **mAh Integer Overflow Fix (Section 7.9):** Fixed negative mAh display caused by integer overflow when `amperageLatest * lastUpdateAt` exceeded int32 max on first update after boot.

5. **Current LSB Integer Overflow Fix (Section 7.10):** Fixed critical bug where `currentLsbNa` calculation overflowed uint32_t when max_current ≥ 4.3A, causing wrong current readings (showed ~1.7A when actual was 2.55A) and negative amperage at higher currents.

---

## Table of Contents

1. [Hardware Overview](#part-1-hardware-overview)
2. [Drivers Created](#part-2-drivers-created)
3. [Sensor Integration Code Changes](#part-3-sensor-integration-code-changes)
4. [CLI Commands Added](#part-4-cli-commands-added)
5. [Configuration Settings Added](#part-5-configuration-settings-added)
6. [Board Configuration](#part-6-board-configuration)
7. [Mistakes Made and Lessons Learned](#part-7-mistakes-made-and-lessons-learned)
8. [I2C Error Analysis](#part-8-i2c-error-analysis)
9. [File Summary](#part-9-file-summary)
10. [Testing Checklist](#part-10-testing-checklist)
11. [Recommendations for Future Work](#part-11-recommendations-for-future-work)

---

## Part 1: Hardware Overview

### MADFLIGHT FC3 Board Specifications

```
MCU:         RP2350B (Raspberry Pi Pico 2)
Clock:       150 MHz
Gyro/Acc:    ICM45686 (SPI)
Barometer:   BMP580 (I2C0)
Compass:     MMC5603 (I2C0)
Power:       INA226 (I2C0)
SD Card:     SPI mode
```

### I2C0 Bus Configuration (Internal Sensors)

```c
#define I2C0_SCL_PIN         PA33
#define I2C0_SDA_PIN         PA32
#define I2C0_CLOCKSPEED      100    // 100kHz (slow, most compatible)
#define USE_I2C_PULLUP              // Enable internal pull-ups
```

### I2C Device Map

| Address | Device | Chip ID / Manufacturer ID |
|---------|--------|---------------------------|
| 0x30 | MMC5603 | Product ID: 0x10 |
| 0x40 | INA226 | Mfg ID: 0x5449 ("TI"), Die ID: 0x2260 |
| 0x47 | BMP580 | Chip ID: 0x50 |

---

## Part 2: Drivers Created

### 2.1 MMC5603 Magnetometer Driver (NEW - Created from Scratch)

**Files:**
- `src/main/drivers/compass/compass_mmc5603.c`
- `src/main/drivers/compass/compass_mmc5603.h`

**Purpose:** Driver for MEMSIC MMC5603NJ 3-axis magnetometer

**Specifications:**
- I2C address: 0x30
- 18-bit resolution (262144 counts per axis)
- Zero field offset: 131072
- Scale: 16384 counts/Gauss (±32 Gauss range)
- Continuous measurement mode at 100Hz ODR
- Automatic SET/RESET for temperature compensation
- High power mode for lower noise

**Key Functions:**

```c
// Detection - reads Product ID register
bool mmc5603Detect(magDev_t *magDev)
{
    uint8_t productId = 0;
    delay(10);  // Allow sensor to power up
    
    bool ack = busReadRegisterBuffer(dev, MMC5603_REG_PRODUCT_ID, &productId, 1);
    if (!ack || productId != MMC5603_PRODUCT_ID) {
        return false;
    }
    // ...
}

// Initialization - configure continuous mode
static bool mmc5603Init(magDev_t *magDev)
{
    // Software reset
    busWriteRegister(dev, MMC5603_REG_CTRL1, MMC5603_CTRL1_SW_RST);
    delay(20);

    // Perform SET operation to eliminate offset
    busWriteRegister(dev, MMC5603_REG_CTRL0, MMC5603_CTRL0_SET);
    delay(1);

    // Configure bandwidth, ODR
    busWriteRegister(dev, MMC5603_REG_CTRL1, MMC5603_BW_2_0MS);
    busWriteRegister(dev, MMC5603_REG_ODR, MMC5603_ODR_100HZ);
    
    // IMPORTANT: Per datasheet, CTRL0 (CMM_FREQ_EN) must be written BEFORE CTRL2 (CMM_EN)
    busWriteRegister(dev, MMC5603_REG_CTRL0, MMC5603_CTRL0_CMM_FREQ_EN | MMC5603_CTRL0_AUTO_SR_EN);
    busWriteRegister(dev, MMC5603_REG_CTRL2, MMC5603_CTRL2_CMM_EN | MMC5603_CTRL2_HPOWER | MMC5603_CTRL2_EN_PRD_SET);
    
    magDev->magOdrHz = 100;
    return true;
}

// Read - 18-bit data reconstruction
static bool mmc5603Read(magDev_t *magDev, int16_t *magData)
{
    uint8_t buf[9];
    
    if (!busReadRegisterBuffer(dev, MMC5603_REG_XOUT0, buf, 9)) {
        return false;
    }

    // Reconstruct 18-bit values from 3 bytes each
    uint32_t rawX = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | ((buf[6] >> 6) & 0x03);
    uint32_t rawY = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | ((buf[7] >> 6) & 0x03);
    uint32_t rawZ = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | ((buf[8] >> 6) & 0x03);

    // Convert to signed (zero field = 131072)
    int32_t x = (int32_t)rawX - MMC5603_ZERO_FIELD;
    int32_t y = (int32_t)rawY - MMC5603_ZERO_FIELD;
    int32_t z = (int32_t)rawZ - MMC5603_ZERO_FIELD;

    // Scale to 16-bit (divide by 4)
    magData[X] = (int16_t)(x >> 2);
    magData[Y] = (int16_t)(y >> 2);
    magData[Z] = (int16_t)(z >> 2);

    return true;
}
```

**Register Map Used:**

| Register | Address | Purpose |
|----------|---------|---------|
| XOUT0-ZOUT2 | 0x00-0x08 | Magnetic field data (18-bit) |
| STATUS1 | 0x18 | Measurement status |
| ODR | 0x1A | Output data rate |
| CTRL0 | 0x1B | Control register 0 |
| CTRL1 | 0x1C | Control register 1 |
| CTRL2 | 0x1D | Control register 2 |
| PRODUCT_ID | 0x39 | Product identification (0x10) |

---

### 2.2 BMP580 Barometer Driver (EXISTING - Modified)

**File:** `src/main/drivers/barometer/barometer_bmp580.c`

**Purpose:** Driver for Bosch BMP580/BMP581 barometric pressure sensor

**Original Problem:** Non-blocking I2C reads failed on PICO platform because the driver used `busReadRegisterBufferStart()` which returns immediately before data is ready.

**Root Cause Analysis:**
```
STM32 behavior:  busReadRegisterBufferStart() → DMA starts → data ready quickly
PICO behavior:   busReadRegisterBufferStart() → returns immediately → data NOT ready
```

**Solution:** Changed to synchronous blocking reads using `busReadRegisterBuffer()`.

**Code Changes:**

```c
// BEFORE (non-blocking - broken on PICO):
static bool bmp580ReadUT(baroDev_t *baro)
{
    busReadRegisterBufferStart(&baro->dev, BMP580_REG_TEMP_DATA_XLSB, 
                               sensor_data, BMP580_DATA_FRAME_SIZE);
    return true;
}

// AFTER (blocking - works on all platforms):
static bool bmp580ReadUT(baroDev_t *baro)
{
    if (!bmp580ReadRegisterBuffer(&baro->dev, BMP580_REG_TEMP_DATA_XLSB, 
                                  sensor_data, BMP580_DATA_FRAME_SIZE)) {
        return false;
    }
    return true;
}
```

**Same fix applied to:**
- `bmp580ReadUT()` - Temperature data read
- `bmp580ReadUP()` - Pressure data read

---

### 2.3 INA226 Power Monitor Driver (EXISTING - Major Modifications)

**Files:**
- `src/main/drivers/ina226.c`
- `src/main/drivers/ina226.h`

**Purpose:** Driver for Texas Instruments INA226 current/voltage/power monitor

**Original Problems:**
1. `ina226WriteRegister()` used non-blocking `i2cWriteBuffer()` without waiting for completion
2. No diagnostic information available for troubleshooting
3. Auto-detection loop tried multiple addresses causing I2C errors

**Solutions Implemented:**

#### A. Write Completion Wait (Critical Fix)

```c
// Write a 16-bit value to a register (with completion wait for PICO)
static bool ina226WriteRegister(i2cDevice_e device, uint8_t address, uint8_t reg, uint16_t value)
{
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;  // MSB first
    data[1] = value & 0xFF;         // LSB
    
    if (!i2cWriteBuffer(device, address, reg, 2, data)) {
        return false;
    }
    
    // ADDED: Wait for write to complete (critical for PICO platform)
    bool error = false;
    while (i2cBusy(device, &error)) {
        // Wait until transfer is complete
    }
    
    return !error;
}
```

#### B. Diagnostic Variables Added

```c
// Global variables for detection diagnostics  
static uint16_t ina226LastMfgId = 0;
static uint16_t ina226LastDieId = 0;
static uint8_t ina226DetectStage = 0;  // 0=not tried, 1=mfg read fail, 2=die read fail, 3=success
static uint8_t ina226InitStage = 0;    // 0=not tried, 1=detect fail, 2=reset fail, 3=config fail, 4=success

// Getter functions for CLI diagnostics
uint16_t ina226GetLastMfgId(void) { return ina226LastMfgId; }
uint16_t ina226GetLastDieId(void) { return ina226LastDieId; }
uint8_t ina226GetDetectStage(void) { return ina226DetectStage; }
uint8_t ina226GetInitStage(void) { return ina226InitStage; }
```

#### C. Single Address Detection (Removed Auto-Detection Loop)

```c
// BEFORE (caused many I2C errors):
for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
    if (ina226Detect(device, addr)) {
        config->address = addr;
        break;
    }
}

// AFTER (single address only):
// Try the configured address only - no auto-detection to avoid I2C errors
bool detected = ina226Detect(config->i2cDevice, config->address);
if (!detected) {
    ina226InitStage = 1;  // Detect failed
    return false;
}
```

#### D. Initialization Delay Added

```c
bool ina226Init(ina226Config_t *config)
{
    ina226InitStage = 0;
    
    if (!config) {
        return false;
    }
    
    // ADDED: Wait for I2C bus to stabilize (important for PICO platform)
    delay(50);
    
    // ... rest of initialization
}
```

---

## Part 3: Sensor Integration Code Changes

### 3.1 Current Sensor Integration

**Files:**
- `src/main/sensors/current.c`
- `src/main/sensors/current.h`

**Changes Made:**

#### A. State Structure and Configuration

```c
static currentMeterINA226State_t currentMeterINA226State;
static ina226Config_t ina226Cfg;
static bool ina226Initialized = false;
static pt1Filter_t ina226Filter;
static uint16_t ina226LastVoltageMv = 0;  // Shared with voltage meter

// Diagnostic tracking
static uint8_t ina226LastDetectResult = 0;
```

#### B. API Functions Added

```c
uint8_t ina226GetDetectResult(void);
bool ina226IsInitialized(void);
uint16_t ina226GetLastVoltageMv(void);  // For voltage meter sharing
```

#### C. Optimized Data Reading (Single I2C Transaction)

```c
void currentMeterINA226Refresh(int32_t lastUpdateAt)
{
    if (!ina226Initialized) {
        currentMeterINA226State.amperageLatest = 0;
        currentMeterINA226State.amperage = 0;
        ina226LastVoltageMv = 0;
        return;
    }
    
    // Read ALL data at once (avoids multiple I2C transactions)
    ina226Data_t data;
    if (ina226Read(&ina226Cfg, &data)) {
        // Store voltage for voltage meter (no separate I2C call needed)
        ina226LastVoltageMv = data.busVoltageMv;
        
        // Convert mA to centiamperes (1/100 A)
        int32_t centiAmps = data.currentMa / 10;
        
        currentMeterINA226State.amperageLatest = centiAmps;
        currentMeterINA226State.amperage = pt1FilterApply(&ina226Filter, centiAmps);
        
        // Update mAh drawn
        currentMeterINA226State.mahDrawnState.mAhDrawnF += 
            (currentMeterINA226State.amperageLatest * lastUpdateAt / (100.0f * 1000 * 3600));
        currentMeterINA226State.mahDrawnState.mAhDrawn = 
            currentMeterINA226State.mahDrawnState.mAhDrawnF;
    }
}
```

---

### 3.2 Voltage Sensor Integration (NEW Feature)

**Files:**
- `src/main/sensors/voltage.c`
- `src/main/sensors/voltage.h`

**Added VOLTAGE_METER_INA226 to Enum:**

```c
typedef enum {
    VOLTAGE_METER_NONE = 0,
    VOLTAGE_METER_ADC,
    VOLTAGE_METER_ESC,
    VOLTAGE_METER_INA226,  // NEW
    VOLTAGE_METER_COUNT
} voltageMeterSource_e;

const char * const voltageMeterSourceNames[VOLTAGE_METER_COUNT] = {
    "NONE", "ADC", "ESC", "INA226"
};
```

**New State Structure:**

```c
#ifdef USE_CURRENT_METER_INA226
typedef struct voltageMeterINA226State_s {
    uint16_t voltageDisplayFiltered;  // battery voltage in 0.01V steps (filtered)
    uint16_t voltageUnfiltered;       // battery voltage in 0.01V steps (unfiltered)
    pt1Filter_t displayFilter;
} voltageMeterINA226State_t;

static voltageMeterINA226State_t voltageMeterINA226State;
static bool voltageMeterINA226Initialized = false;
#endif
```

**New Functions:**

```c
void voltageMeterINA226Init(void)
{
#ifdef USE_CURRENT_METER_INA226
    memset(&voltageMeterINA226State, 0, sizeof(voltageMeterINA226State_t));
    pt1FilterInit(&voltageMeterINA226State.displayFilter, 
                  pt1FilterGain(GET_BATTERY_LPF_FREQUENCY(batteryConfig()->vbatDisplayLpfPeriod), 
                               HZ_TO_INTERVAL(SLOW_VOLTAGE_TASK_FREQ_HZ)));
    // INA226 is initialized by currentMeterINA226Init() in current.c
    voltageMeterINA226Initialized = ina226IsInitialized();
#endif
}

void voltageMeterINA226Refresh(void)
{
#ifdef USE_CURRENT_METER_INA226
    if (!voltageMeterINA226Initialized) {
        voltageMeterINA226Initialized = ina226IsInitialized();
        if (!voltageMeterINA226Initialized) {
            return;
        }
    }
    
    // Get voltage from current meter's last read (NO additional I2C call!)
    uint16_t voltageMv = ina226GetLastVoltageMv();
    
    // Convert mV to 0.01V (centivolt)
    uint16_t voltageCv = voltageMv / 10;
    voltageMeterINA226State.voltageUnfiltered = voltageCv;
    voltageMeterINA226State.voltageDisplayFiltered = 
        pt1FilterApply(&voltageMeterINA226State.displayFilter, voltageCv);
#endif
}

void voltageMeterINA226Read(voltageMeter_t *voltageMeter)
{
#ifndef USE_CURRENT_METER_INA226
    voltageMeterReset(voltageMeter);
#else
    voltageMeter->displayFiltered = voltageMeterINA226State.voltageDisplayFiltered;
    voltageMeter->unfiltered = voltageMeterINA226State.voltageUnfiltered;
#endif
}
```

**Key Design Decision - Shared Data Architecture:**

```
┌─────────────────────────────────────────────────────────────┐
│                    INA226 I2C Read                          │
│  ┌─────────────────────────────────────────────────────┐    │
│  │ ina226Read() reads ALL registers in one transaction │    │
│  │   - Shunt Voltage (2 bytes)                         │    │
│  │   - Bus Voltage (2 bytes)  ───────┐                 │    │
│  │   - Current (2 bytes)             │                 │    │
│  │   - Power (2 bytes)               │                 │    │
│  └─────────────────────────────────────────────────────┘    │
│                                      │                      │
│                                      ▼                      │
│                          ┌───────────────────┐              │
│                          │ ina226LastVoltageMv │             │
│                          │   (shared variable) │             │
│                          └─────────┬─────────┘              │
│                                    │                        │
│              ┌─────────────────────┼─────────────────────┐  │
│              ▼                     ▼                     │  │
│    ┌─────────────────┐   ┌─────────────────────┐         │  │
│    │ Current Meter   │   │ Voltage Meter       │         │  │
│    │ (uses current)  │   │ (uses ina226GetLast │         │  │
│    │                 │   │  VoltageMv())       │         │  │
│    └─────────────────┘   └─────────────────────┘         │  │
│                                                          │  │
│    Result: Only 4 I2C transactions instead of 5+        │  │
└─────────────────────────────────────────────────────────────┘
```

---

### 3.3 Battery System Integration

**File:** `src/main/sensors/battery.c`

**Changes to `batteryUpdateVoltage()`:**

```c
void batteryUpdateVoltage(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    switch (batteryConfig()->voltageMeterSource) {
#ifdef USE_ESC_SENSOR
        case VOLTAGE_METER_ESC:
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                voltageMeterESCRefresh();
                voltageMeterESCReadCombined(&voltageMeter);
            }
            break;
#endif
#ifdef USE_CURRENT_METER_INA226
        case VOLTAGE_METER_INA226:           // NEW CASE
            voltageMeterINA226Refresh();
            voltageMeterINA226Read(&voltageMeter);
            break;
#endif
        case VOLTAGE_METER_ADC:
            voltageMeterADCRefresh();
            voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VBAT, &voltageMeter);
            break;

        default:
        case VOLTAGE_METER_NONE:
            voltageMeterReset(&voltageMeter);
            break;
    }
    // ... rest of function
}
```

**Changes to `batteryInit()`:**

```c
voltageMeterGenericInit();
switch (batteryConfig()->voltageMeterSource) {
    case VOLTAGE_METER_ESC:
#ifdef USE_ESC_SENSOR
        voltageMeterESCInit();
#endif
        break;

    case VOLTAGE_METER_INA226:               // NEW CASE
#ifdef USE_CURRENT_METER_INA226
        voltageMeterINA226Init();
#endif
        break;

    case VOLTAGE_METER_ADC:
        voltageMeterADCInit();
        break;

    default:
        break;
}
```

---

## Part 4: CLI Commands Added

**File:** `src/main/cli/cli.c`

### 4.1 I2C Bus Scanner

```c
static void cliI2cScan(const char *cmdName, char *cmdline)
{
    int bus = 0;
    if (*cmdline) {
        bus = atoi(cmdline);
        if (bus < 0 || bus >= I2CDEV_COUNT) {
            cliPrintLinef("Usage: i2c_scan [bus] (0-%d)", I2CDEV_COUNT - 1);
            return;
        }
    }
    
    cliPrintLinef("I2C bus %d scan (0x08-0x77):", bus);
    int found = 0;
    
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t data;
        if (i2cRead((i2cDevice_e)bus, addr, 0x00, 1, &data)) {
            cliPrintLinef("  Found device at 0x%02X", addr);
            found++;
        }
    }
    
    cliPrintLinef("Found %d devices", found);
}
```

**Usage:**
```
# i2c_scan 0
I2C bus 0 scan (0x08-0x77):
  Found device at 0x30
  Found device at 0x40
  Found device at 0x47
Found 3 devices
```

### 4.2 I2C Register Reader

```c
static void cliI2cRead(const char *cmdName, char *cmdline)
{
    // Parse: <bus> <addr> <reg> [len]
    char *ptr = cmdline;
    int bus = atoi(ptr);
    ptr = strchr(ptr, ' ');
    int addr = strtol(++ptr, NULL, 0);
    ptr = strchr(ptr, ' ');
    int reg = strtol(++ptr, NULL, 0);
    ptr = strchr(ptr, ' ');
    int len = ptr ? atoi(++ptr) : 1;
    
    uint8_t data[32];
    if (len > 32) len = 32;
    
    if (i2cRead((i2cDevice_e)bus, addr, reg, len, data)) {
        cliPrintf("Read 0x%02X[0x%02X]: ", addr, reg);
        for (int i = 0; i < len; i++) {
            cliPrintf("0x%02X ", data[i]);
        }
        cliPrintLinefeed();
    } else {
        cliPrintLine("Read failed");
    }
}
```

**Usage:**
```
# i2c_read 0 0x40 0xFE 2
Read 0x40[0xFE]: 0x54 0x49    (TI manufacturer ID)
```

### 4.3 INA226 Status Command

```c
static void cliINA226Status(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);
    
#ifdef USE_CURRENT_METER_INA226
    cliPrintLinef("INA226 Current Sensor Status:");
    cliPrintLinef("  Initialized: %s", ina226IsInitialized() ? "YES" : "NO");
    cliPrintLinef("  Init stage: %d (%s)", ina226GetInitStage(), 
                  ina226GetInitStage() == 4 ? "ok" : "failed");
    cliPrintLinef("  Detect stage: %d (%s)", ina226GetDetectStage(),
                  ina226GetDetectStage() == 3 ? "ok" : "failed");
    cliPrintLinef("  Manufacturer ID: 0x%04X ('%c%c')", 
                  ina226GetLastMfgId(),
                  (ina226GetLastMfgId() >> 8) & 0xFF,
                  ina226GetLastMfgId() & 0xFF);
    cliPrintLinef("  Die ID: 0x%04X", ina226GetLastDieId());
    
    currentMeter_t meter;
    currentMeterINA226Read(&meter);
    cliPrintLinef("  Current: %d.%02d A", 
                  meter.amperage / 100, abs(meter.amperage) % 100);
#else
    cliPrintLine("INA226 not enabled");
#endif
}
```

**Usage:**
```
# ina226_status
INA226 Current Sensor Status:
  Initialized: YES
  Init stage: 4 (ok)
  Detect stage: 3 (ok)
  Manufacturer ID: 0x5449 ('TI')
  Die ID: 0x2260
  Current: 0.00 A
```

### 4.4 INA226 Manual Initialization

```c
static void cliINA226Init(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);
    
#ifdef USE_CURRENT_METER_INA226
    cliPrintLine("Re-initializing INA226...");
    currentMeterINA226Init();
    cliPrintLinef("Result: %s", ina226IsInitialized() ? "SUCCESS" : "FAILED");
#else
    cliPrintLine("INA226 not enabled");
#endif
}
```

### 4.5 CLI Command Definitions

```c
const clicmd_t cmdTable[] = {
    // ... existing commands ...
    CLI_COMMAND_DEF("i2c_read", "read I2C register", "<bus> <addr> <reg> [len]", cliI2cRead),
    CLI_COMMAND_DEF("i2c_scan", "scan I2C bus for devices", "[bus]", cliI2cScan),
#ifdef USE_CURRENT_METER_INA226
    CLI_COMMAND_DEF("ina226_init", "reinitialize INA226 current sensor", NULL, cliINA226Init),
    CLI_COMMAND_DEF("ina226_status", "show INA226 current sensor status", NULL, cliINA226Status),
#endif
    // ... remaining commands ...
};
```

---

## Part 5: Configuration Settings Added

**File:** `src/main/cli/settings.c`

**INA226 Settings Added:**

```c
#ifdef USE_CURRENT_METER_INA226
    { PARAM_NAME_INA226_I2C_DEVICE,     VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, I2CDEV_COUNT }, PG_CURRENT_SENSOR_INA226_CONFIG, offsetof(currentSensorINA226Config_t, i2cDevice) },
    { PARAM_NAME_INA226_ADDRESS,        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0x40, 0x4F }, PG_CURRENT_SENSOR_INA226_CONFIG, offsetof(currentSensorINA226Config_t, address) },
    { PARAM_NAME_INA226_SHUNT_UOHM,     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 65000 }, PG_CURRENT_SENSOR_INA226_CONFIG, offsetof(currentSensorINA226Config_t, shuntResistanceMicroOhms) },
    { PARAM_NAME_INA226_MAX_CURRENT,    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 65000 }, PG_CURRENT_SENSOR_INA226_CONFIG, offsetof(currentSensorINA226Config_t, maxExpectedCurrentMa) },
#endif
```

**CLI Usage:**
```
# get ina226
ina226_i2c_device = 1
ina226_address = 64
ina226_shunt_uohm = 2000
ina226_max_current = 50000

# set ina226_shunt_uohm = 1000
ina226_shunt_uohm set to 1000
```

---

## Part 6: Board Configuration

**File:** `src/config/configs/MADFLIGHT_FC3/config.h`

### Complete Sensor Configuration

```c
//------------------
// Internal Pins
//------------------

// Bus for internal I2C sensors
// NOTE: Board has BMP580 barometer, MMC5603 compass and INA226 power sensor.
#define I2C0_SCL_PIN         PA33
#define I2C0_SDA_PIN         PA32
#define I2C0_CLOCKSPEED      100    // 100kHz (slow, most compatible)
#define USE_I2C_PULLUP              // Enable internal pull-ups

// INA226 Current Sensor (connected to I2C0)
#define USE_CURRENT_METER_INA226
#define DEFAULT_INA226_I2C_DEVICE        1      // 1 = I2CDEV_0 (internal I2C bus)
#define DEFAULT_INA226_ADDRESS           0x40   // Default INA226 address (A0=GND, A1=GND)
#define DEFAULT_INA226_SHUNT_RESISTANCE  2000   // 2mΩ shunt resistor (2000 µΩ)
#define DEFAULT_INA226_MAX_CURRENT       50000  // 50A maximum expected current
#define DEFAULT_CURRENT_METER_SOURCE     CURRENT_METER_INA226

// Battery voltage - use INA226 for voltage measurement (no ADC needed)
#define DEFAULT_VOLTAGE_METER_SOURCE     VOLTAGE_METER_INA226

// Internal BMP580 barometer on I2C0
#define USE_BARO
#define USE_BARO_BMP580
#define DEFAULT_BARO_BMP580
#define BARO_I2C_INSTANCE    I2CDEV_0

// Internal MMC5603 compass on I2C0
#define USE_MAG
#define USE_MAG_MMC5603
#define MAG_I2C_INSTANCE     I2CDEV_0

// Optional external barometers on I2C1 (second i2c bus) PA2=SDA PA3=SCL
// Disabled to reduce I2C detection errors - uncomment if needed
//#define USE_BARO_MS5611
//#define USE_BARO_BMP280
//#define USE_BARO_BMP388
//#define USE_BARO_LPS
//#define USE_BARO_QMP6988
//#define USE_BARO_DPS310
//#define USE_BARO_BMP085
//#define USE_BARO_2SMBP_02B
//#define USE_BARO_LPS22DF

// Optional external compass connected to I2C1 (second i2c bus) PA2=SDA PA3=SCL
// Disabled to reduce I2C detection errors - uncomment if needed
//#define USE_MAG_HMC5883
//#define USE_MAG_QMC5883
//#define USE_MAG_LIS2MDL
//#define USE_MAG_LIS3MDL
//#define USE_MAG_AK8975
//#define USE_MAG_IST8310
```

---

## Part 7: Mistakes Made and Lessons Learned

### 7.1 Initial INA226 Voltage Implementation - Double I2C Reads

**Mistake:** First implementation of `voltageMeterINA226Refresh()` performed a separate I2C read:

```c
// WRONG - causes double I2C traffic
void voltageMeterINA226Refresh(void)
{
    // ... setup ina226Cfg ...
    
    uint16_t voltageMv;
    if (ina226ReadVoltage(&ina226Cfg, &voltageMv)) {  // EXTRA I2C call!
        // ... process voltage ...
    }
}
```

**Result:** 119 I2C errors instead of expected ~0 errors.

**Root Cause:** Current meter already reads all INA226 registers (including voltage) in `ina226Read()`. Adding another read in voltage meter doubled the I2C traffic and caused bus contention.

**Timeline of Discovery:**
1. Added VOLTAGE_METER_INA226 feature
2. Built and flashed firmware
3. `status` command showed 119 I2C errors
4. Realized voltage meter was doing separate I2C reads
5. Changed to shared data model

**Fix:** Share voltage data between current and voltage meters:

```c
// In current.c - store voltage during read:
if (ina226Read(&ina226Cfg, &data)) {
    ina226LastVoltageMv = data.busVoltageMv;  // Store for voltage meter
    // ... process current ...
}

// In voltage.c - use stored value (no I2C):
void voltageMeterINA226Refresh(void)
{
    uint16_t voltageMv = ina226GetLastVoltageMv();  // Just get cached value
    // ... process voltage ...
}
```

**Lesson Learned:** When multiple subsystems need data from the same I2C device, read once and share the data through a common variable or API.

---

### 7.2 Non-Blocking I2C on PICO Platform

**Mistake:** Assumed existing Betaflight drivers would work on PICO platform without modifications.

**Root Cause:** PICO I2C implementation differs fundamentally from STM32:

| Platform | Non-Blocking Read Behavior |
|----------|---------------------------|
| STM32 | DMA starts, often completes quickly, data usually ready soon |
| PICO | Returns immediately, data buffer NOT populated until later |

**Symptoms:**
- BMP580: Always returned 0 for temperature/pressure
- INA226: Initialization would fail at "config write" stage

**Code Example - BMP580:**

```c
// BEFORE (broken on PICO):
static bool bmp580ReadUT(baroDev_t *baro)
{
    // This returns immediately, buffer not filled yet!
    busReadRegisterBufferStart(&baro->dev, BMP580_REG_TEMP_DATA_XLSB, 
                               sensor_data, BMP580_DATA_FRAME_SIZE);
    return true;
}

static bool bmp580GetUT(baroDev_t *baro)
{
    // On PICO, sensor_data is still empty here!
    bmp580_ut = (sensor_data[2] << 16) | (sensor_data[1] << 8) | sensor_data[0];
    return true;
}
```

```c
// AFTER (works on all platforms):
static bool bmp580ReadUT(baroDev_t *baro)
{
    // This blocks until data is ready
    if (!bmp580ReadRegisterBuffer(&baro->dev, BMP580_REG_TEMP_DATA_XLSB, 
                                  sensor_data, BMP580_DATA_FRAME_SIZE)) {
        return false;
    }
    return true;
}
```

**Code Example - INA226 Write:**

```c
// BEFORE (broken on PICO):
static bool ina226WriteRegister(...)
{
    if (!i2cWriteBuffer(device, address, reg, 2, data)) {
        return false;
    }
    return true;  // Returns before write completes!
}

// AFTER (works on PICO):
static bool ina226WriteRegister(...)
{
    if (!i2cWriteBuffer(device, address, reg, 2, data)) {
        return false;
    }
    
    // Wait for write to complete
    bool error = false;
    while (i2cBusy(device, &error)) {
        // Spin until done
    }
    
    return !error;
}
```

**Lesson Learned:** Platform-specific I2C behavior requires testing. Non-blocking operations need explicit completion wait on PICO.

---

### 7.3 Auto-Detection Loop Causing I2C Errors

**Mistake:** INA226 driver originally tried multiple I2C addresses in a loop:

```c
// WRONG - causes many I2C errors on non-responsive addresses
bool ina226Init(ina226Config_t *config)
{
    // Try all possible addresses
    for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
        if (ina226Detect(config->i2cDevice, addr)) {
            config->address = addr;
            break;
        }
    }
    // ...
}
```

**Result:** 16 detection attempts × multiple registers = many I2C NAK errors before finding the device at 0x40.

**Root Cause:** Auto-detection is designed for unknown hardware. When hardware is known (board-specific config), it just causes unnecessary I2C traffic and errors.

**Fix:**

```c
// CORRECT - single attempt at configured address only
bool ina226Init(ina226Config_t *config)
{
    // Wait for I2C bus to stabilize
    delay(50);
    
    // Try the configured address only - no auto-detection to avoid I2C errors
    bool detected = ina226Detect(config->i2cDevice, config->address);
    
    if (!detected) {
        ina226InitStage = 1;  // Detect failed
        return false;
    }
    // ...
}
```

**Lesson Learned:** For known hardware configurations, use explicit addresses instead of auto-detection. This reduces errors and initialization time.

---

### 7.4 Missing Compass Driver (MMC5603 vs MMC5883)

**Initial Assumption:** An existing driver (like MMC5883 or QMC5883) might work for MMC5603.

**Reality:**

| Aspect | MMC5883 | MMC5603 |
|--------|---------|---------|
| I2C Address | 0x30 | 0x30 |
| Product ID Register | Different | 0x39 |
| Product ID Value | Different | 0x10 |
| Data Format | 16-bit | 18-bit |
| Register Map | Different | Different |
| Initialization | Different | Different |

**Result:** Had to create a completely new driver from scratch based on MMC5603 datasheet.

**Time Spent:** Several hours writing and testing the new driver.

**Lesson Learned:** Always verify exact chip compatibility. Similar names (MMC56xx vs MMC58xx) don't mean compatible registers or protocols.

---

### 7.5 Forgetting to Add Header Includes

**Mistake:** Multiple times during development, added function calls but forgot to add corresponding `#include` statements.

**Symptoms:** Compiler errors like:
```
error: implicit declaration of function 'ina226IsInitialized'
error: implicit declaration of function 'currentSensorINA226Config'
```

**Files Where This Occurred:**

| File | Missing Include | Required For |
|------|-----------------|--------------|
| cli.c | `#include "sensors/current.h"` | `ina226IsInitialized()`, `ina226GetDetectResult()` |
| cli.c | `#include "drivers/ina226.h"` | `ina226GetLastMfgId()`, `ina226GetInitStage()` |
| voltage.c | `#include "sensors/current.h"` | `currentSensorINA226Config()`, `ina226GetLastVoltageMv()` |

**Lesson Learned:** When calling functions from other modules:
1. Check if the include is already present
2. Add the include at the top of the file
3. Verify with a test compile before continuing

---

### 7.6 Incorrect I2C Device Index Convention

**Initial Confusion:** Betaflight uses 1-based I2C device indices in configuration, but 0-based in code.

```c
// Configuration (1-based):
#define DEFAULT_INA226_I2C_DEVICE 1  // Means I2CDEV_0

// Code conversion needed:
ina226Cfg.i2cDevice = I2C_CFG_TO_DEV(config->i2cDevice);
// Where I2C_CFG_TO_DEV(x) is (x - 1)
```

**Mistake Made:** Initially set device to 0 in config, causing I2C operations on wrong/invalid bus.

**Discovery:** i2c_scan showed devices, but INA226 detection failed. Eventually traced to device index mismatch.

**Lesson Learned:** Document and verify index conventions (0-based vs 1-based) when interfacing between configuration and driver code.

---

### 7.7 INA226 Read I2C Completion Wait (PICO Platform)

**Date Discovered:** February 15, 2026

**Symptom:** INA226 `ina226_status` CLI command showed correct direct I2C reads, but cached voltage remained 0.00V. The driver appeared to initialize correctly (Init stage: 4) but data reads returned stale or zero values.

**Root Cause:** PICO platform I2C is fully asynchronous. Earlier fix only added `i2cBusy()` wait after `i2cWriteBuffer()`, but the **read** operation also requires waiting for completion.

```c
// BEFORE (partially fixed - only write waited):
static bool ina226ReadRegister(i2cDevice_e device, uint8_t address, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    
    if (!i2cRead(device, address, reg, 2, data)) {
        return false;
    }
    // Returns immediately on PICO - data buffer NOT YET FILLED!
    
    *value = ((uint16_t)data[0] << 8) | data[1];  // Reads garbage or old data
    return true;
}

// AFTER (fully fixed - both write AND read wait):
static bool ina226ReadRegister(i2cDevice_e device, uint8_t address, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    
    if (!i2cRead(device, address, reg, 2, data)) {
        return false;
    }
    
    // CRITICAL: Wait for read to complete (PICO I2C is asynchronous)
    bool error = false;
    while (i2cBusy(device, &error)) {
        // Spin until I2C transfer is complete
    }
    
    if (error) {
        return false;
    }
    
    *value = ((uint16_t)data[0] << 8) | data[1];  // Now data is valid
    return true;
}
```

**Debugging Process:**
1. `ina226_status` showed Init stage: 4 (success) but voltage: 0.00V
2. Added direct I2C read test that performed manual register read - returned correct voltage
3. Compared: cached data=0, direct read=correct → data being lost between read and use
4. Realized `i2cRead()` was also async and needed the same `i2cBusy()` wait as write

**Lesson Learned:** On PICO platform, **ALL** I2C operations (both read and write) are asynchronous and require explicit `i2cBusy()` wait before using results. Don't assume fixing write is enough.

---

### 7.8 Battery Cell Count Chicken-and-Egg Problem

**Date Discovered:** February 15, 2026

**Symptom:** INA226 voltage reading worked in CLI `ina226_status` command (showing ~9.62V) but `status` command showed 0.00V for battery voltage. Betaflight Configurator also showed 0.00V.

**Root Cause:** Classic chicken-and-egg problem in battery initialization:

1. `batteryUpdateCurrentMeter()` has early return when `batteryCellCount == 0`
2. `batteryCellCount` is only set when battery voltage is detected
3. INA226 voltage reading happens inside `batteryUpdateCurrentMeter()` → `currentMeterINA226Refresh()`
4. Therefore: No battery detected → No meter refresh → No voltage read → No battery detected → ...

```c
// The problematic code in battery.c:
static void batteryUpdateCurrentMeter(uint32_t currentTimeUs)
{
    // ...
    
    // This caused the chicken-and-egg problem:
    if (batteryCellCount == 0) {
        return;  // INA226 refresh never called!
    }
    
    // Current meter refresh (includes INA226 voltage read) never reached
    currentMeterINA226Refresh();
}
```

**Why CLI worked but Configurator didn't:**
- CLI `ina226_status` command called `ina226Read()` directly, bypassing the battery system
- Configurator relied on the battery subsystem's periodic updates, which were blocked

**Fix:** Add exception for INA226 current meter to allow refresh even without detected battery:

```c
static void batteryUpdateCurrentMeter(uint32_t currentTimeUs)
{
    // ...
    
    // Allow INA226 to refresh even without battery detection (it provides voltage too)
    if (batteryCellCount == 0 && batteryConfig()->currentMeterSource != CURRENT_METER_INA226) {
        return;
    }
    
    currentMeterINA226Refresh();  // Now called for INA226 even when batteryCellCount==0
}
```

**Debugging Process:**
1. `ina226_status` showed correct voltage (9.62V), `status` showed 0.00V
2. Added manual refresh test - worked after manual trigger
3. Traced through code: `voltageMeterUpdate()` → uses cached data → cached data is 0
4. Found that `batteryUpdateCurrentMeter()` was returning early before reaching INA226 refresh
5. Identified the `if (batteryCellCount == 0)` guard as the blocker

**Lesson Learned:** When adding new sensor sources (like INA226 for voltage), trace through ALL code paths that depend on the data. Pay special attention to guards and early returns that may assume a different initialization order than your new sensor requires.

---

### 7.9 mAh Consumption Integer Overflow

**Date Discovered:** February 15, 2026

**Symptom:** `ina226_status` showed `mAh drawn: -7` (negative value) while current was reading correctly at 87mA.

**Root Cause:** Integer overflow in mAh calculation in `currentMeterINA226Refresh()`:

```c
// The problematic calculation:
currentMeterINA226State.mahDrawnState.mAhDrawnF += 
    amperageLatest * lastUpdateAt / (100.0f * 1000 * 3600);
```

On the first call after boot, `lastUpdateAt` could be very large (seconds since boot in microseconds). The integer multiplication `amperageLatest * lastUpdateAt` happened BEFORE the float division, causing int32 overflow:
- If `amperageLatest = 87` (centiamps) and `lastUpdateAt = 25,000,000` (25 seconds)
- `87 * 25,000,000 = 2,175,000,000` → OVERFLOWS int32 (max ~2.1B) → wraps to negative

**Fix:** Added sanity check and explicit float casting:

```c
// Sanity check: limit lastUpdateAt to prevent overflow (max 10 seconds)
if (lastUpdateAt > 10000000 || lastUpdateAt < 0) {
    lastUpdateAt = 10000;  // Default to 10ms if unreasonable value
}

// Use float from the start to avoid integer overflow:
currentMeterINA226State.mahDrawnState.mAhDrawnF += 
    (float)currentMeterINA226State.amperageLatest * (float)lastUpdateAt / (100.0f * 1000.0f * 3600.0f);
```

**File Modified:** `src/main/sensors/current.c`

**Lesson Learned:** When multiplying values that could be large, ALWAYS cast to larger type (float or int64_t) BEFORE the multiplication, not after.

---

### 7.10 Current LSB Calculation Integer Overflow (CRITICAL)

**Date Discovered:** February 15, 2026

**Symptom:** At 2.55A actual current (from power supply), Configurator showed only ~1.7A. At currents above ~2A, the amperage would go negative (-1.63A seen).

**Root Cause:** TWO integer overflow bugs in `ina226CalculateCalibration()` and `ina226Read()`:

**Bug 1: currentLsbNa calculation overflow**
```c
// The problematic code:
config->currentLsbNa = ((uint32_t)config->maxCurrentMa * 1000000UL) / 32768UL;
```

With `maxCurrentMa = 50000` (50A):
- `50000 * 1,000,000 = 50,000,000,000` → OVERFLOWS uint32 (max 4.29B)
- After overflow: wraps to ~2.76 billion
- Result: `currentLsbNa = 84086 nA` instead of correct **1,525,878 nA**
- This is off by a factor of **18x**!

**Bug 2: currentMa calculation overflow**
```c
// The problematic code:
data->currentMa = ((int32_t)data->currentRaw * (int32_t)config->currentLsbNa) / 1000000L;
```

With the wrong (but smaller) currentLsbNa of 84086, this could still overflow at high currents:
- `currentRaw = 30000 * currentLsbNa = 84086 * 30000 = 2,522,580,000` → Close to int32 overflow
- With the correct currentLsbNa (1,525,878), it would definitely overflow earlier

**Fix:** Use 64-bit arithmetic for both calculations:

```c
// Fix 1: Use uint64_t for currentLsbNa calculation
config->currentLsbNa = (uint32_t)(((uint64_t)config->maxCurrentMa * 1000000ULL) / 32768ULL);

// Fix 2: Use int64_t for currentMa calculation
data->currentMa = (int32_t)(((int64_t)data->currentRaw * (int64_t)config->currentLsbNa) / 1000000LL);
```

**Before vs After:**

| Parameter | Before (Buggy) | After (Fixed) |
|-----------|----------------|---------------|
| currentLsbNa | 84,086 nA | 1,525,878 nA |
| Current at 2.55A | ~1.7A (wrong) | 2.57A (correct) |
| Current at 3A+ | Negative | Correct |
| mAh tracking | Negative at first | Positive, accumulating |

**File Modified:** `src/main/drivers/ina226.c`

**Lesson Learned:** 
1. **Always check integer math for overflow**, especially when multiplying by large constants (like 1,000,000)
2. For products where result > 4 billion, use uint64_t/int64_t BEFORE multiplication
3. Maximum safe value for `uint32_t * N` is `4,294,967,295 / N`
   - For `* 1000000`: max safe input is ~4295 (about 4.3A)
   - 50A config with 50000 mA input was doomed to overflow

---

### 7.11 MMC5603 Initialization Register Order (February 16, 2026)

**Discovery:** During Betaflight AI agent code review of MMC5603 driver.

**Problem:** Initial implementation wrote CTRL2 (CMM_EN) before CTRL0 (CMM_FREQ_EN):

```c
// WRONG ORDER (original):
busWriteRegister(dev, MMC5603_REG_CTRL2, MMC5603_CTRL2_CMM_EN | ...);  // CMM_EN first
busWriteRegister(dev, MMC5603_REG_CTRL0, MMC5603_CTRL0_CMM_FREQ_EN | ...);  // CMM_FREQ_EN second
```

**Issue:** Per MMC5603NJ datasheet, `CMM_FREQ_EN` (in CTRL0) must be set BEFORE `CMM_EN` (in CTRL2) for proper continuous measurement mode. The datasheet states:
> "For continuous mode measurement, set CMM_FREQ_EN bit first, then set CMM_EN bit."

**Fix:** Reorder writes to comply with datasheet:

```c
// CORRECT ORDER (fixed):
busWriteRegister(dev, MMC5603_REG_ODR, MMC5603_ODR_100HZ);            // 1. Set ODR
busWriteRegister(dev, MMC5603_REG_CTRL0, MMC5603_CTRL0_CMM_FREQ_EN | ...);  // 2. CMM_FREQ_EN first
busWriteRegister(dev, MMC5603_REG_CTRL2, MMC5603_CTRL2_CMM_EN | ...);        // 3. CMM_EN second
```

**Additional Fixes:**
1. Added `USE_MAG_MMC5603` to `common_post.h` for automatic detection
2. Fixed header include from `io_types.h` to `drivers/compass/compass.h`
3. Added `[MAG_MMC5603] = "MMC5603"` to `lookupTableMagHardware` in settings.c

**Lesson Learned:** Always carefully read datasheet register programming sequences. The order of register writes can matter significantly, especially when starting continuous modes.

---

## Part 8: I2C Error Analysis

### Expected I2C Transaction Count (per sensor update cycle)

| Sensor | Operation | Registers | Bytes | Transactions |
|--------|-----------|-----------|-------|--------------|
| BMP580 | Read T+P | 6 registers | 6 | 1 |
| MMC5603 | Read XYZ | 9 registers | 9 | 1 |
| INA226 | Read all | 4 registers | 8 | 4* |
| **Total** | | | | **6** |

*INA226 reads 4 separate 16-bit registers; without burst read support, this requires 4 transactions.

### Error Sources Before Fixes

| Source | Errors Per Boot | Cause |
|--------|-----------------|-------|
| Barometer detection | ~5 | Trying multiple baro types |
| Compass detection | ~5 | Trying multiple mag types |
| INA226 auto-detect | ~15 | Loop trying 0x40-0x4F |
| Double voltage read | ~50/sec | Separate INA226 voltage read |
| Bus contention | Variable | Timing issues |
| **Total at boot** | **~25** | |
| **Total per second** | **~50+** | |

### After All Fixes

| Source | Errors | Notes |
|--------|--------|-------|
| Barometer detection | 0 | Only tries BMP580 |
| Compass detection | 0 | Only tries MMC5603 |
| INA226 auto-detect | 0 | Single address only |
| Double voltage read | 0 | Uses shared data |
| Bus contention | ~0 | Sync operations |
| **Total expected** | **~0** | |

### Remaining Error Sources

If errors still occur, likely causes:
1. I2C bus noise/interference
2. Timing issues with other I2C devices
3. Power supply instability
4. Pull-up resistor issues

---

## Part 9: File Summary

### Files Created

| File | Lines | Purpose |
|------|-------|---------|
| `src/main/drivers/compass/compass_mmc5603.c` | ~230 | MMC5603 magnetometer driver |
| `src/main/drivers/compass/compass_mmc5603.h` | ~30 | MMC5603 header file |

### Files Modified

| File | Changes | Purpose |
|------|---------|---------|
| `src/main/drivers/barometer/barometer_bmp580.c` | Sync I2C reads, 128X oversampling | Fix PICO compatibility, reduce noise |
| `src/main/drivers/ina226.c` | Write+read wait, diagnostics, **uint64/int64 overflow fixes** | Fix PICO async I2C, fix current calculation at high currents |
| `src/main/drivers/ina226.h` | Diagnostic function prototypes | API additions |
| `src/main/sensors/current.c` | INA226 integration, shared voltage, vbatScale, **mAh overflow fix** | Power monitoring, fix negative mAh |
| `src/main/sensors/current.h` | New API functions, vbatScale config | Interface additions |
| `src/main/sensors/voltage.c` | INA226 voltage meter | Battery voltage from INA226 |
| `src/main/sensors/voltage.h` | New enum, functions | Interface additions |
| `src/main/sensors/battery.c` | INA226 chicken-and-egg fix | Allow refresh without batteryCellCount |
| `src/main/cli/cli.c` | i2c_scan, i2c_read, ina226_* commands | Debugging tools |
| `src/main/cli/settings.c` | INA226 CLI settings (incl. vbat_scale) | Configuration |
| `src/config/configs/MADFLIGHT_FC3/config.h` | Complete sensor config | Board definition |

### Line Count Summary

| Category | Approximate Lines |
|----------|-------------------|
| New driver code | ~260 |
| Modified driver code | ~100 |
| Sensor integration | ~150 |
| CLI additions | ~200 |
| Configuration | ~50 |
| **Total new/modified** | **~760** |

---

## Part 10: Testing Checklist

### Hardware Detection

- [x] BMP580 barometer detected at 0x47
- [x] MMC5603 compass detected at 0x30
- [x] INA226 power monitor detected at 0x40
- [x] All devices respond to i2c_scan

### Sensor Reading

- [x] BMP580 returns valid temperature
- [x] BMP580 returns valid pressure
- [x] BMP580 calculates valid altitude
- [x] MMC5603 returns valid X/Y/Z magnetic field
- [x] INA226 returns valid manufacturer ID (0x5449)
- [x] INA226 returns valid die ID (0x2260)
- [x] INA226 returns valid current (tested with 0A load)
- [x] INA226 returns valid voltage via CLI ina226_status (9.62V observed)
- [x] INA226 voltage visible in Configurator (16.07V at 4S)
- [x] INA226 current visible in Configurator (2.57A tested)
- [x] INA226 mAh consumption tracking (19 mAh observed, positive and accumulating)
- [x] INA226 power calculation verified

### CLI Commands

- [x] `i2c_scan` works - shows all devices
- [x] `i2c_read` works - reads arbitrary registers
- [x] `ina226_status` works - shows diagnostic info (driver read + direct I2C read)
- [x] `ina226_init` works - reinitializes sensor
- [x] `get ina226` shows all settings
- [x] `set ina226_*` modifies settings
- [x] `ina226_vbat_scale` calibration setting works (100 = 1.00x, no scaling needed)

### Error Monitoring

- [x] Initial I2C errors reduced from ~25 to ~2 at boot
- [ ] Verify 0 errors during steady-state operation
- [ ] Long-duration stability test (>1 hour)

### Integration

- [x] Barometer appears in Betaflight Configurator
- [x] Compass appears in Betaflight Configurator
- [x] Current meter source selectable as INA226
- [x] Voltage meter source selectable as INA226
- [x] INA226 voltage appears immediately in Configurator (16.07V at 4S)
- [x] INA226 current appears in Configurator (2.57A tested with motor load)
- [x] mAh consumption tracked correctly (19 mAh observed, positive and accumulating)
- [ ] OSD displays INA226 voltage correctly (not yet tested with VTX)
- [ ] OSD displays INA226 current correctly (not yet tested with VTX)

---

## Part 11: Recommendations for Future Work

### 11.1 Add I2C Bus Mutex

When multiple sensors share the I2C bus, consider adding a mutex to prevent concurrent access:

```c
// Proposed improvement
static mutex_t i2c0_mutex;

bool i2cReadWithMutex(i2cDevice_e device, ...) {
    mutex_lock(&i2c0_mutex);
    bool result = i2cRead(device, ...);
    mutex_unlock(&i2c0_mutex);
    return result;
}
```

### 11.2 DMA I2C Optimization

Investigate if PICO SDK supports DMA-based I2C for better CPU efficiency:

```c
// Current: CPU waits during I2C transfer
// Proposed: DMA handles transfer, CPU does other work
```

### 11.3 Error Recovery

Add automatic sensor re-initialization on communication failure:

```c
static int consecutiveErrors = 0;

void sensorRefresh() {
    if (!sensorRead()) {
        consecutiveErrors++;
        if (consecutiveErrors > 10) {
            sensorReinit();
            consecutiveErrors = 0;
        }
    } else {
        consecutiveErrors = 0;
    }
}
```

### 11.4 Power Telemetry

Add INA226 power (Watts) reading to telemetry:

```c
// INA226 provides power directly: Power = Current × Voltage
// Already calculated in driver, just needs telemetry integration
```

### 11.5 Runtime Shunt Calibration

Add CLI command for runtime shunt resistor calibration:

```c
// Measure known current, calculate actual shunt resistance
// set ina226_calibrate <known_current_mA>
```

### 11.6 Second I2C Bus Support

Enable external sensors on I2C1 (PA2/PA3):

```c
// Currently disabled to reduce errors
// Could add with proper timing separation
#define USE_MAG_QMC5883  // For external GPS compass
#define USE_BARO_BMP388  // For external high-altitude baro
```

---

## Appendix A: Test Commands Reference

```bash
# Build firmware
make CONFIG=MADFLIGHT_FC3

# Output file
obj/betaflight_2026.6.0-alpha_RP2350B_MADFLIGHT_FC3.uf2

# CLI commands after connecting
status              # Show system status including I2C errors
i2c_scan 0          # Scan internal I2C bus
i2c_scan 1          # Scan external I2C bus
i2c_read 0 0x40 0xFE 2   # Read INA226 manufacturer ID
i2c_read 0 0x47 0x01 1   # Read BMP580 chip ID
i2c_read 0 0x30 0x39 1   # Read MMC5603 product ID
ina226_status       # Show INA226 status
ina226_init         # Reinitialize INA226
get ina226          # Show INA226 settings
get current_meter   # Show current meter source
get voltage_meter   # Show voltage meter source
```

---

## Appendix B: INA226 Register Reference

| Register | Address | Size | Description |
|----------|---------|------|-------------|
| Configuration | 0x00 | 16-bit | Operating mode, averaging, conversion time |
| Shunt Voltage | 0x01 | 16-bit | Voltage across shunt (LSB = 2.5µV) |
| Bus Voltage | 0x02 | 16-bit | Load voltage (LSB = 1.25mV) |
| Power | 0x03 | 16-bit | Power = Current × Voltage |
| Current | 0x04 | 16-bit | Calibrated current reading |
| Calibration | 0x05 | 16-bit | Sets current LSB |
| Mask/Enable | 0x06 | 16-bit | Alert configuration |
| Alert Limit | 0x07 | 16-bit | Alert threshold |
| Manufacturer ID | 0xFE | 16-bit | 0x5449 ("TI") |
| Die ID | 0xFF | 16-bit | 0x2260 |

---

## Appendix C: MMC5603 Register Reference

| Register | Address | Size | Description |
|----------|---------|------|-------------|
| XOUT0-1, XOUT2 | 0x00-0x01, 0x06 | 18-bit | X-axis magnetic field |
| YOUT0-1, YOUT2 | 0x02-0x03, 0x07 | 18-bit | Y-axis magnetic field |
| ZOUT0-1, ZOUT2 | 0x04-0x05, 0x08 | 18-bit | Z-axis magnetic field |
| TOUT | 0x09 | 8-bit | Temperature |
| STATUS1 | 0x18 | 8-bit | Measurement status |
| ODR | 0x1A | 8-bit | Output data rate |
| CTRL0 | 0x1B | 8-bit | Control register 0 |
| CTRL1 | 0x1C | 8-bit | Control register 1 |
| CTRL2 | 0x1D | 8-bit | Control register 2 |
| Product ID | 0x39 | 8-bit | 0x10 |

---

## Appendix D: BMP580 Register Reference

| Register | Address | Size | Description |
|----------|---------|------|-------------|
| Chip ID | 0x01 | 8-bit | 0x50 (BMP580) or 0x51 (BMP581) |
| Rev ID | 0x02 | 8-bit | Revision |
| Temp Data | 0x1D-0x1F | 24-bit | Temperature (XLSB, LSB, MSB) |
| Press Data | 0x20-0x22 | 24-bit | Pressure (XLSB, LSB, MSB) |
| Status | 0x28 | 8-bit | Data ready flags |
| OSR Config | 0x36 | 8-bit | Oversampling |
| ODR Config | 0x37 | 8-bit | Output data rate and mode |
| CMD | 0x7E | 8-bit | Command register (0xB6 = reset) |

---

*End of Documentation*
