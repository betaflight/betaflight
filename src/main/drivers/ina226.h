/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/bus_i2c.h"

// INA226 I2C addresses (7-bit)
// Address is determined by A0 and A1 pins:
// A0=GND, A1=GND: 0x40
// A0=VS,  A1=GND: 0x41
// A0=SDA, A1=GND: 0x42
// A0=SCL, A1=GND: 0x43
// ...and so on (16 possible addresses: 0x40-0x4F)
#define INA226_I2C_ADDR_DEFAULT     0x40

// INA226 Register addresses
#define INA226_REG_CONFIG           0x00
#define INA226_REG_SHUNT_VOLTAGE    0x01
#define INA226_REG_BUS_VOLTAGE      0x02
#define INA226_REG_POWER            0x03
#define INA226_REG_CURRENT          0x04
#define INA226_REG_CALIBRATION      0x05
#define INA226_REG_MASK_ENABLE      0x06
#define INA226_REG_ALERT_LIMIT      0x07
#define INA226_REG_MANUFACTURER_ID  0xFE
#define INA226_REG_DIE_ID           0xFF

// INA226 expected ID values
#define INA226_MANUFACTURER_ID      0x5449  // "TI" in ASCII
#define INA226_DIE_ID               0x2260

// Configuration register bits
#define INA226_CONFIG_RESET         (1 << 15)

// Operating modes (bits 0-2)
#define INA226_MODE_POWER_DOWN      0x00
#define INA226_MODE_SHUNT_TRIG      0x01
#define INA226_MODE_BUS_TRIG        0x02
#define INA226_MODE_SHUNT_BUS_TRIG  0x03
#define INA226_MODE_SHUNT_CONT      0x05
#define INA226_MODE_BUS_CONT        0x06
#define INA226_MODE_SHUNT_BUS_CONT  0x07

// Conversion times (bits 3-5 for shunt, bits 6-8 for bus)
#define INA226_CT_140US             0x00
#define INA226_CT_204US             0x01
#define INA226_CT_332US             0x02
#define INA226_CT_588US             0x03
#define INA226_CT_1100US            0x04
#define INA226_CT_2116US            0x05
#define INA226_CT_4156US            0x06
#define INA226_CT_8244US            0x07

// Averaging modes (bits 9-11)
#define INA226_AVG_1                0x00
#define INA226_AVG_4                0x01
#define INA226_AVG_16               0x02
#define INA226_AVG_64               0x03
#define INA226_AVG_128              0x04
#define INA226_AVG_256              0x05
#define INA226_AVG_512              0x06
#define INA226_AVG_1024             0x07

// LSB values
#define INA226_SHUNT_VOLTAGE_LSB_UV 2.5f    // 2.5 µV per LSB
#define INA226_BUS_VOLTAGE_LSB_MV   1.25f   // 1.25 mV per LSB

// Calibration formula internal constant
#define INA226_CALIBRATION_CONSTANT 0.00512f

// Data structure for INA226 readings
typedef struct {
    int16_t shuntVoltageRaw;    // Raw shunt voltage register value
    uint16_t busVoltageRaw;     // Raw bus voltage register value
    int16_t currentRaw;         // Raw current register value
    uint16_t powerRaw;          // Raw power register value
    int32_t currentMa;          // Current in mA
    uint16_t busVoltageMv;      // Bus voltage in mV
    uint32_t powerMw;           // Power in mW
    bool dataValid;             // True if last read was successful
} ina226Data_t;

typedef struct {
    i2cDevice_e i2cDevice;
    uint8_t address;
    uint16_t shuntResistorMicroOhms;  // Shunt resistor value in µΩ (e.g., 1000 = 1mΩ)
    uint16_t maxCurrentMa;            // Maximum expected current in mA
    uint16_t calibrationValue;        // Calculated calibration register value
    uint32_t currentLsbNa;            // Current LSB in nA
} ina226Config_t;

// Function prototypes
bool ina226Detect(i2cDevice_e device, uint8_t address);
bool ina226Init(ina226Config_t *config);
bool ina226Configure(const ina226Config_t *config);
bool ina226Read(const ina226Config_t *config, ina226Data_t *data);
bool ina226ReadCurrent(const ina226Config_t *config, int32_t *currentMa);
bool ina226ReadVoltage(const ina226Config_t *config, uint16_t *voltageMv);
bool ina226ReadPower(const ina226Config_t *config, uint32_t *powerMw);
bool ina226Reset(const ina226Config_t *config);
// Diagnostic functions
uint16_t ina226GetLastMfgId(void);
uint16_t ina226GetLastDieId(void);
uint8_t ina226GetDetectStage(void);
uint8_t ina226GetInitStage(void);