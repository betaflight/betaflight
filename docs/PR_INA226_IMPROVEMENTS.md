# INA226 Power Monitor Driver for Betaflight

## Overview

This PR adds complete INA226 I2C power monitor support to Betaflight, enabling accurate current and voltage monitoring on boards without traditional ADC sensing.

## Features

- **Current Measurement:** High-precision current sensing via shunt resistor
- **Voltage Measurement:** Bus voltage monitoring (1.25mV resolution)
- **mAh Tracking:** Battery capacity consumption tracking
- **64-bit Math:** Overflow-safe calculations for high currents (up to 50A+)
- **CLI Commands:** Debugging and status commands
- **Configurable:** I2C address, bus, shunt resistance, max current

## Hardware Support

| Parameter | Value |
|-----------|-------|
| Chip | Texas Instruments INA226 |
| Interface | I2C (up to 2.94MHz, using 400kHz) |
| Default Address | 0x40 |
| Voltage Range | 0-36V |
| Current Range | ±81.92mV shunt voltage |
| Resolution | 16-bit ADC |

## Code Changes Summary

### New Files

| File | Description |
|------|-------------|
| `src/main/drivers/ina226.c` | INA226 driver with I2C communication |
| `src/main/drivers/ina226.h` | Driver header with config/data structures |

### Modified Files

| File | Change |
|------|--------|
| `src/main/sensors/current.c` | Add CURRENT_METER_INA226 implementation |
| `src/main/sensors/current.h` | Add INA226 current meter type and prototypes |
| `src/main/sensors/current_ids.h` | Add CURRENT_METER_INA226 enum |
| `src/main/sensors/voltage.c` | Add VOLTAGE_METER_INA226 implementation |
| `src/main/sensors/voltage.h` | Add INA226 voltage meter type |
| `src/main/sensors/battery.c` | Integrate INA226 into battery monitoring |
| `src/main/cli/cli.c` | Add `ina226_status` debug command |
| `src/main/cli/settings.c` | Add INA226 configuration parameters |
| `src/main/pg/pg_ids.h` | Add PG_INA226_CONFIG parameter group |
| `mk/source.mk` | Add ina226.c to build |

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                   INA226 Driver                      │
│  ┌─────────────────────────────────────────────┐    │
│  │     ina226Read() - single I2C transaction   │    │
│  │         ↓                  ↓                │    │
│  │    currentRaw         busVoltageMv          │    │
│  └─────────────────────────────────────────────┘    │
│                 ↓                  ↓                 │
│      ┌──────────┴──────────────────┴──────────┐     │
│      ↓                                        ↓     │
│  Current Meter                         Voltage Meter│
│  currentMeterINA226Refresh()     voltageMeterINA226│
│      ↓                                        ↓     │
│  currentMa, mAh                     busVoltageMv    │
└─────────────────────────────────────────────────────┘
```

## Key Implementation Details

### 64-Bit Overflow Protection

```c
// currentLsbNa calculation (prevents overflow when maxCurrentMa >= 4295)
config->currentLsbNa = (uint32_t)(((uint64_t)config->maxCurrentMa * 1000000ULL) / 32768ULL);

// currentMa calculation (prevents overflow at high currents)
data->currentMa = (int32_t)(((int64_t)data->currentRaw * (int64_t)config->currentLsbNa) / 1000000LL);
```

### Battery Integration Fix

The driver includes a fix for the battery initialization order - INA226 refresh is allowed even when `batteryCellCount == 0`, since INA226 provides voltage readings needed to detect cell count.

## CLI Configuration

```bash
# View current settings
get ina226

# Configure INA226
set ina226_i2c_address = 64      # I2C address (0x40 = 64 decimal)
set ina226_i2c_bus = 0           # I2C bus number
set ina226_shunt_mohms = 2       # Shunt resistance in milliohms
set ina226_max_current = 50000   # Maximum expected current in mA

# Enable INA226 as current and voltage source
set current_meter = INA226
set voltage_meter_source = INA226

save
```

### Debug Command

```bash
# Show INA226 status and readings
ina226_status

# Output example:
# INA226: DETECTED at bus 0
# Address: 0x40
# Shunt: 2 mOhm
# Max Current: 50000 mA
# Current LSB: 1525878 nA
# Voltage: 16070 mV
# Current: 2570 mA
```

## Testing

**Test Hardware:**
- Board: MADFLIGHT FC3 (https://madflight.com/Board-FC3/)
- MCU: RP2350B (Raspberry Pi Pico 2) @ 150MHz
- INA226: Address 0x40, 2mΩ shunt
- Battery: 4S LiPo (16.27V)
- Load: Up to 2.55A (motor test)

**Verified Results:**
| Test | Result |
|------|--------|
| INA226 detection | ✅ Pass |
| Voltage reading | 16.07V (matches multimeter) |
| Current reading | 2.57A (matches PSU 2.55A) |
| Power calculation | ~41W (correct) |
| mAh tracking | 19 mAh after test |
| Battery cell detect | 4S detected |
| Configurator voltage | ✅ Visible |
| Configurator current | ✅ Visible |

## Commit Message

```
feat(sensor): add INA226 power monitor support

Add complete INA226 I2C power monitor integration:

Driver (ina226.c/h):
- I2C communication with configurable address (default 0x40)
- Automatic LSB calibration for shunt resistance and max current
- Bus voltage measurement with 1.25mV resolution
- 64-bit arithmetic to prevent integer overflow at high currents
- CLI commands for testing and debugging

Sensor integration:
- CURRENT_METER_INA226 type in current.c/h
- VOLTAGE_METER_INA226 type in voltage.c/h
- mAh tracking in battery.c
- CLI settings for shunt resistance and max current

Configuration parameters:
- ina226_shunt_mohms: Shunt resistance in milliohms
- ina226_max_current: Maximum expected current in mA
- ina226_i2c_address: I2C address (7-bit)
- ina226_i2c_bus: I2C bus selection

Tested on RP2350B with 2mΩ shunt, accurate to within 1% at 2.5A.
```

## PR Checklist

- [x] Code follows Betaflight coding style guidelines
- [x] Single feature: INA226 power monitor support
- [x] Build passes for PICO targets
- [x] Tested on real hardware
- [x] All functionality verified working
