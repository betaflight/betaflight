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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_CURRENT_METER_INA226

#include "common/maths.h"
#include "drivers/bus_i2c.h"
#include "drivers/ina226.h"
#include "drivers/time.h"

/**
 * @brief Write a 16-bit value to an INA226 register.
 *
 * Handles byte order (MSB first) and waits for the I2C transaction to complete.
 *
 * @param device  I2C device to use.
 * @param address I2C address of the INA226.
 * @param reg     Register address to write.
 * @param value   16-bit value to write.
 * @return true if write succeeded, false otherwise.
 */
static bool ina226WriteRegister(i2cDevice_e device, uint8_t address, uint8_t reg, uint16_t value)
{
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;  // MSB first
    data[1] = value & 0xFF;         // LSB
    
    if (!i2cWriteBuffer(device, address, reg, 2, data)) {
        return false;
    }
    
    // Wait for write to complete (important for PICO platform)
    bool error = false;
    while (i2cBusy(device, &error)) {
        // Wait until transfer is complete
    }
    
    return !error;
}

/**
 * @brief Convert raw bus voltage register value to millivolts.
 *
 * INA226 bus voltage LSB = 1.25mV (125/100).
 *
 * @param rawValue Raw register value from INA226_REG_BUS_VOLTAGE.
 * @return Bus voltage in millivolts.
 */
static inline uint16_t ina226RawToVoltageMv(uint16_t rawValue)
{
    return (uint16_t)((uint32_t)rawValue * 125 / 100);
}

/**
 * @brief Read a 16-bit value from an INA226 register.
 *
 * Handles byte order (MSB first) and waits for the I2C transaction to complete.
 *
 * @param device  I2C device to use.
 * @param address I2C address of the INA226.
 * @param reg     Register address to read.
 * @param value   Pointer to store the 16-bit value.
 * @return true if read succeeded, false otherwise.
 */
static bool ina226ReadRegister(i2cDevice_e device, uint8_t address, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    if (!i2cRead(device, address, reg, 2, data)) {
        return false;
    }
    
    // Wait for read to complete (important for PICO platform)
    bool error = false;
    while (i2cBusy(device, &error)) {
        // Wait until transfer is complete
    }
    
    if (error) {
        return false;
    }
    
    *value = ((uint16_t)data[0] << 8) | data[1];  // MSB first
    return true;
}

// Global variables for detection diagnostics  
static uint16_t ina226LastMfgId = 0;
static uint16_t ina226LastDieId = 0;
static uint8_t ina226DetectStage = 0;  // 0=not tried, 1=mfg read fail, 2=die read fail, 3=success
static uint8_t ina226InitStage = 0;    // 0=not tried, 1=detect fail, 2=reset fail, 3=config fail, 4=success

uint16_t ina226GetLastMfgId(void) { return ina226LastMfgId; }
uint16_t ina226GetLastDieId(void) { return ina226LastDieId; }
uint8_t ina226GetDetectStage(void) { return ina226DetectStage; }
uint8_t ina226GetInitStage(void) { return ina226InitStage; }

/**
 * @brief Detect INA226 at the specified I2C address.
 *
 * Reads manufacturer and die ID registers to verify device presence.
 * Detection results are stored for diagnostic access via getter functions.
 *
 * @param device  I2C device to use.
 * @param address I2C address to probe.
 * @return true if INA226 detected, false otherwise.
 */
bool ina226Detect(i2cDevice_e device, uint8_t address)
{
    ina226DetectStage = 0;
    ina226LastMfgId = 0;
    ina226LastDieId = 0;
    
    // Try to read manufacturer ID
    if (!ina226ReadRegister(device, address, INA226_REG_MANUFACTURER_ID, &ina226LastMfgId)) {
        ina226DetectStage = 1;  // Manufacturer ID read failed
        return false;
    }
    
    // Read die ID  
    if (!ina226ReadRegister(device, address, INA226_REG_DIE_ID, &ina226LastDieId)) {
        ina226DetectStage = 2;  // Die ID read failed
        return false;
    }
    
    // Validate manufacturer ID (Texas Instruments = 0x5449)
    if (ina226LastMfgId != INA226_MANUFACTURER_ID) {
        ina226DetectStage = 4;  // Manufacturer ID mismatch
        return false;
    }
    
    ina226DetectStage = 3;  // Success
    return true;
}

/**
 * @brief Calculate calibration register value for INA226.
 *
 * Computes Current_LSB and calibration value based on shunt resistor and
 * maximum expected current. Uses uint64_t arithmetic to prevent overflow.
 *
 * @param config Pointer to INA226 configuration (modified with calculated values).
 */
static void ina226CalculateCalibration(ina226Config_t *config)
{
    // Calculate Current_LSB = MaxCurrent / 2^15
    // Current_LSB in A = maxCurrentMa / 1000 / 32768
    // To preserve precision, we calculate in nA:
    // currentLsbNa = (maxCurrentMa * 1000000) / 32768
    // IMPORTANT: Use uint64_t to avoid overflow when maxCurrentMa >= 4295 mA (4.3A)
    // Example: 50000 * 1000000 = 50,000,000,000 which exceeds uint32_t max (4,294,967,295)
    config->currentLsbNa = (uint32_t)(((uint64_t)config->maxCurrentMa * 1000000ULL) / 32768ULL);
    
    // Calibration = 0.00512 / (Current_LSB * RSHUNT)
    // Where RSHUNT is in ohms
    // Cal = 0.00512 / ((currentLsbNa * 1e-9) * (shuntResistorMicroOhms * 1e-6))
    // Cal = 0.00512 / (currentLsbNa * shuntResistorMicroOhms * 1e-15)
    // Cal = 0.00512 * 1e15 / (currentLsbNa * shuntResistorMicroOhms)
    // Cal = 5.12e12 / (currentLsbNa * shuntResistorMicroOhms)
    
    uint64_t numerator = 5120000000000ULL;  // 5.12e12
    uint64_t denominator = (uint64_t)config->currentLsbNa * (uint64_t)config->shuntResistorMicroOhms;
    
    if (denominator > 0) {
        uint64_t calResult = numerator / denominator;
        // Clamp to UINT16_MAX to prevent silent truncation
        if (calResult > UINT16_MAX) {
            config->calibrationValue = UINT16_MAX;
        } else {
            config->calibrationValue = (uint16_t)calResult;
        }
    } else {
        config->calibrationValue = 0;
    }
}

/**
 * @brief Initialize the INA226 power monitor.
 *
 * Detects the device, performs a software reset, calculates calibration,
 * and configures the operating mode.
 *
 * @param config Pointer to INA226 configuration structure.
 * @return true if initialization succeeded, false otherwise.
 */
bool ina226Init(ina226Config_t *config)
{
    ina226InitStage = 0;
    
    if (!config) {
        return false;
    }
    
    // Wait for I2C bus to stabilize (important for PICO platform)
    delay(50);
    
    // Try the configured address only - no auto-detection to avoid I2C errors
    bool detected = ina226Detect(config->i2cDevice, config->address);
    
    if (!detected) {
        ina226InitStage = 1;  // Detect failed
        return false;
    }
    
    // Reset the device
    if (!ina226Reset(config)) {
        ina226InitStage = 2;  // Reset failed
        return false;
    }
    
    delay(5);  // Wait for reset to complete
    
    // Calculate calibration value
    ina226CalculateCalibration(config);
    
    // Configure and write calibration
    if (!ina226Configure(config)) {
        ina226InitStage = 3;  // Configure failed
        return false;
    }
    
    ina226InitStage = 4;  // Success
    return true;
}

/**
 * @brief Reset the INA226 to default settings.
 *
 * @param config Pointer to INA226 configuration.
 * @return true if reset succeeded, false otherwise.
 */
bool ina226Reset(const ina226Config_t *config)
{
    return ina226WriteRegister(config->i2cDevice, config->address, 
                               INA226_REG_CONFIG, INA226_CONFIG_RESET);
}

/**
 * @brief Configure INA226 operating mode and write calibration.
 *
 * Sets up continuous measurement with 16-sample averaging and 1.1ms
 * conversion time for both shunt and bus voltage.
 *
 * @param config Pointer to INA226 configuration.
 * @return true if configuration succeeded, false otherwise.
 */
bool ina226Configure(const ina226Config_t *config)
{
    // Configuration:
    // - Average: 16 samples (improves noise performance)
    // - Bus voltage conversion time: 1.1ms
    // - Shunt voltage conversion time: 1.1ms
    // - Mode: Continuous shunt and bus voltage
    // This gives ~35ms total conversion time (16 * 2 * 1.1ms)
    
    uint16_t configReg = (INA226_AVG_16 << 9) |       // Average 16 samples
                         (INA226_CT_1100US << 6) |     // Bus voltage 1.1ms
                         (INA226_CT_1100US << 3) |     // Shunt voltage 1.1ms
                         INA226_MODE_SHUNT_BUS_CONT;   // Continuous mode
    
    if (!ina226WriteRegister(config->i2cDevice, config->address, 
                             INA226_REG_CONFIG, configReg)) {
        return false;
    }
    
    // Write calibration register
    if (config->calibrationValue > 0) {
        if (!ina226WriteRegister(config->i2cDevice, config->address,
                                 INA226_REG_CALIBRATION, config->calibrationValue)) {
            return false;
        }
    }
    
    return true;
}

/**
 * @brief Read all measurement data from INA226.
 *
 * Reads shunt voltage, bus voltage, current, and power registers.
 * Converts raw values to engineering units (mV, mA, mW).
 *
 * NOTE: This performs 4 sequential I2C register reads with blocking busy-waits.
 * The INA226 supports sequential register reads with auto-increment, but the
 * current betaflight I2C API doesn't support multi-byte burst reads cleanly.
 * At ~50Hz battery task rate, the latency is acceptable. Consider burst reads
 * if lower latency is needed in the future.
 *
 * @param config Pointer to INA226 configuration.
 * @param data   Pointer to data structure to fill with readings.
 * @return true if read succeeded, false otherwise.
 */
bool ina226Read(const ina226Config_t *config, ina226Data_t *data)
{
    if (!config || !data) {
        return false;
    }
    
    uint16_t regValue;
    
    // Read shunt voltage register
    if (!ina226ReadRegister(config->i2cDevice, config->address, 
                            INA226_REG_SHUNT_VOLTAGE, &regValue)) {
        data->dataValid = false;
        return false;
    }
    data->shuntVoltageRaw = (int16_t)regValue;
    
    // Read bus voltage register
    if (!ina226ReadRegister(config->i2cDevice, config->address,
                            INA226_REG_BUS_VOLTAGE, &regValue)) {
        data->dataValid = false;
        return false;
    }
    data->busVoltageRaw = regValue;
    // Bus voltage: LSB = 1.25mV
    data->busVoltageMv = ina226RawToVoltageMv(data->busVoltageRaw);
    
    // Read current register (requires calibration to be set)
    if (config->calibrationValue > 0) {
        if (!ina226ReadRegister(config->i2cDevice, config->address,
                                INA226_REG_CURRENT, &regValue)) {
            data->dataValid = false;
            return false;
        }
        data->currentRaw = (int16_t)regValue;
        // Convert to mA: current_mA = currentRaw * currentLsbNa / 1000000
        // Use int64_t to prevent overflow when currentRaw * currentLsbNa > 2^31
        // Example: currentRaw=30000 * currentLsbNa=1525878 = 45,776,340,000 (exceeds int32_t)
        data->currentMa = (int32_t)(((int64_t)data->currentRaw * (int64_t)config->currentLsbNa) / 1000000LL);
        
        // Read power register
        if (!ina226ReadRegister(config->i2cDevice, config->address,
                                INA226_REG_POWER, &regValue)) {
            data->dataValid = false;
            return false;
        }
        data->powerRaw = regValue;
        // Power LSB = 25 * Current_LSB
        // power_mW = powerRaw * 25 * currentLsbNa / 1000000
        // Use 64-bit math to avoid overflow (powerRaw * 25 * currentLsbNa can exceed uint32_t)
        data->powerMw = (uint32_t)(((uint64_t)data->powerRaw * 25ULL * (uint64_t)config->currentLsbNa) / 1000000ULL);
    } else {
        // No calibration - calculate current from shunt voltage and resistance
        // shuntVoltage_uV = shuntVoltageRaw * 2.5
        // current_mA = shuntVoltage_uV / shuntResistorMicroOhms * 1000
        // current_mA = shuntVoltageRaw * 2.5 * 1000 / shuntResistorMicroOhms
        // current_mA = shuntVoltageRaw * 2500 / shuntResistorMicroOhms
        if (config->shuntResistorMicroOhms > 0) {
            data->currentMa = ((int32_t)data->shuntVoltageRaw * 2500L) / 
                              (int32_t)config->shuntResistorMicroOhms;
        } else {
            data->currentMa = 0;
        }
        data->currentRaw = 0;
        data->powerRaw = 0;
        // Use uint64_t for consistency with calibrated path to prevent overflow with high current
        // Use ABS() macro from common/maths.h for type-safe int32_t absolute value
        data->powerMw = (uint32_t)(((uint64_t)ABS(data->currentMa) * (uint64_t)data->busVoltageMv) / 1000ULL);
    }
    
    data->dataValid = true;
    return true;
}

/**
 * @brief Read current measurement from INA226.
 *
 * @param config    Pointer to INA226 configuration.
 * @param currentMa Pointer to store current in milliamps.
 * @return true if read succeeded, false otherwise.
 */
bool ina226ReadCurrent(const ina226Config_t *config, int32_t *currentMa)
{
    ina226Data_t data;
    
    if (!ina226Read(config, &data)) {
        return false;
    }
    
    *currentMa = data.currentMa;
    return true;
}

/**
 * @brief Read bus voltage from INA226.
 *
 * @param config    Pointer to INA226 configuration.
 * @param voltageMv Pointer to store voltage in millivolts.
 * @return true if read succeeded, false otherwise.
 */
bool ina226ReadVoltage(const ina226Config_t *config, uint16_t *voltageMv)
{
    uint16_t regValue;
    
    if (!ina226ReadRegister(config->i2cDevice, config->address,
                            INA226_REG_BUS_VOLTAGE, &regValue)) {
        return false;
    }
    
    // Bus voltage: LSB = 1.25mV
    *voltageMv = ina226RawToVoltageMv(regValue);
    return true;
}

/**
 * @brief Read power measurement from INA226.
 *
 * @param config  Pointer to INA226 configuration.
 * @param powerMw Pointer to store power in milliwatts.
 * @return true if read succeeded, false otherwise.
 */
bool ina226ReadPower(const ina226Config_t *config, uint32_t *powerMw)
{
    ina226Data_t data;
    
    if (!ina226Read(config, &data)) {
        return false;
    }
    
    *powerMw = data.powerMw;
    return true;
}

#endif // USE_CURRENT_METER_INA226
