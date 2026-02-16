/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "drivers/adc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#ifdef USE_CURRENT_METER_INA226
#include "drivers/bus_i2c.h"
#include "drivers/ina226.h"
#endif

#include "current.h"

const char * const currentMeterSourceNames[CURRENT_METER_COUNT] = {
    "NONE", "ADC", "VIRTUAL", "ESC", "MSP", "INA226"
};

const uint8_t currentMeterIds[] = {
    CURRENT_METER_ID_BATTERY_1,
#ifdef USE_VIRTUAL_CURRENT_METER
    CURRENT_METER_ID_VIRTUAL_1,
#endif
#ifdef USE_ESC_SENSOR
    CURRENT_METER_ID_ESC_COMBINED_1,
    CURRENT_METER_ID_ESC_MOTOR_1,
    CURRENT_METER_ID_ESC_MOTOR_2,
    CURRENT_METER_ID_ESC_MOTOR_3,
    CURRENT_METER_ID_ESC_MOTOR_4,
    CURRENT_METER_ID_ESC_MOTOR_5,
    CURRENT_METER_ID_ESC_MOTOR_6,
    CURRENT_METER_ID_ESC_MOTOR_7,
    CURRENT_METER_ID_ESC_MOTOR_8,
    CURRENT_METER_ID_ESC_MOTOR_9,
    CURRENT_METER_ID_ESC_MOTOR_10,
    CURRENT_METER_ID_ESC_MOTOR_11,
    CURRENT_METER_ID_ESC_MOTOR_12,
#endif
#ifdef USE_MSP_CURRENT_METER
    CURRENT_METER_ID_MSP_1,
#endif
#ifdef USE_CURRENT_METER_INA226
    CURRENT_METER_ID_INA226_1,
#endif
};

const uint8_t supportedCurrentMeterCount = ARRAYLEN(currentMeterIds);

//
// ADC/Virtual/ESC/MSP shared
//

void currentMeterReset(currentMeter_t *meter)
{
    meter->amperage = 0;
    meter->amperageLatest = 0;
    meter->mAhDrawn = 0;
}

//
// ADC/Virtual shared
//

static pt1Filter_t adciBatFilter;

#ifndef DEFAULT_CURRENT_METER_SCALE
#define DEFAULT_CURRENT_METER_SCALE 400 // for Allegro ACS758LCB-100U (40mV/A)
#endif

#ifndef DEFAULT_CURRENT_METER_OFFSET
#define DEFAULT_CURRENT_METER_OFFSET 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig, PG_CURRENT_SENSOR_ADC_CONFIG, 0);

PG_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig,
    .scale = DEFAULT_CURRENT_METER_SCALE,
    .offset = DEFAULT_CURRENT_METER_OFFSET,
);

#ifdef USE_VIRTUAL_CURRENT_METER
PG_REGISTER(currentSensorVirtualConfig_t, currentSensorVirtualConfig, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, 0);
#endif

static int32_t currentMeterADCToCentiamps(const uint16_t src)
{

    const currentSensorADCConfig_t *config = currentSensorADCConfig();

    int32_t millivolts = ((uint32_t)src * getVrefMv()) / 4096;
    // y=x/m+b m is scale in (mV/10A) and b is offset in (mA)
    int32_t centiAmps = config->scale ? (millivolts * 10000 / (int32_t)config->scale + (int32_t)config->offset) / 10 : 0;

    DEBUG_SET(DEBUG_CURRENT_SENSOR, 0, millivolts);
    DEBUG_SET(DEBUG_CURRENT_SENSOR, 1, centiAmps);

    return centiAmps; // Returns Centiamps to maintain compatability with the rest of the code
}

#if defined(USE_ADC) || defined(USE_VIRTUAL_CURRENT_METER)
static void updateCurrentmAhDrawnState(currentMeterMAhDrawnState_t *state, int32_t amperageLatest, int32_t lastUpdateAt)
{
    state->mAhDrawnF = state->mAhDrawnF + (amperageLatest * lastUpdateAt / (100.0f * 1000 * 3600));
    state->mAhDrawn = state->mAhDrawnF;
}
#endif

//
// ADC
//

currentMeterADCState_t currentMeterADCState;

void currentMeterADCInit(void)
{
    memset(&currentMeterADCState, 0, sizeof(currentMeterADCState_t));
    pt1FilterInit(&adciBatFilter, pt1FilterGain(GET_BATTERY_LPF_FREQUENCY(batteryConfig()->ibatLpfPeriod), HZ_TO_INTERVAL(50)));
}

void currentMeterADCRefresh(int32_t lastUpdateAt)
{
#ifdef USE_ADC
    const uint16_t iBatSample = adcGetValue(ADC_CURRENT);
    currentMeterADCState.amperageLatest = currentMeterADCToCentiamps(iBatSample);
    currentMeterADCState.amperage = currentMeterADCToCentiamps(pt1FilterApply(&adciBatFilter, iBatSample));

    updateCurrentmAhDrawnState(&currentMeterADCState.mahDrawnState, currentMeterADCState.amperageLatest, lastUpdateAt);
#else
    UNUSED(lastUpdateAt);
    UNUSED(currentMeterADCToCentiamps);

    currentMeterADCState.amperageLatest = 0;
    currentMeterADCState.amperage = 0;
#endif
}

void currentMeterADCRead(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterADCState.amperageLatest;
    meter->amperage = currentMeterADCState.amperage;
    meter->mAhDrawn = currentMeterADCState.mahDrawnState.mAhDrawn;

    DEBUG_SET(DEBUG_CURRENT_SENSOR, 2, meter->amperageLatest);
    DEBUG_SET(DEBUG_CURRENT_SENSOR, 3, meter->mAhDrawn);
}

//
// VIRTUAL
//

#ifdef USE_VIRTUAL_CURRENT_METER
currentSensorVirtualState_t currentMeterVirtualState;

void currentMeterVirtualInit(void)
{
    memset(&currentMeterVirtualState, 0, sizeof(currentSensorVirtualState_t));
}

void currentMeterVirtualRefresh(int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset)
{
    currentMeterVirtualState.amperage = (int32_t)currentSensorVirtualConfig()->offset;
    if (armed) {
        if (throttleLowAndMotorStop) {
            throttleOffset = 0;
        }

        int throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50); // FIXME magic number 50. Possibly use thrustLinearization if configured.
        currentMeterVirtualState.amperage += throttleFactor * (int32_t)currentSensorVirtualConfig()->scale / 1000;
    }
    updateCurrentmAhDrawnState(&currentMeterVirtualState.mahDrawnState, currentMeterVirtualState.amperage, lastUpdateAt);
}

void currentMeterVirtualRead(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterVirtualState.amperage;
    meter->amperage = currentMeterVirtualState.amperage;
    meter->mAhDrawn = currentMeterVirtualState.mahDrawnState.mAhDrawn;
}
#endif

//
// ESC
//

#ifdef USE_ESC_SENSOR
currentMeterESCState_t currentMeterESCState;

void currentMeterESCInit(void)
{
    memset(&currentMeterESCState, 0, sizeof(currentMeterESCState_t));
}

void currentMeterESCRefresh(int32_t lastUpdateAt)
{
    UNUSED(lastUpdateAt);

    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData && escData->dataAge <= ESC_BATTERY_AGE_MAX) {
        currentMeterESCState.amperage = escData->current + escSensorConfig()->offset / 10;
        currentMeterESCState.mAhDrawn = escData->consumption + escSensorConfig()->offset * millis() / (1000.0f * 3600);
    } else {
        currentMeterESCState.amperage = 0;
        currentMeterESCState.mAhDrawn = 0;
    }
}

void currentMeterESCReadCombined(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterESCState.amperage;
    meter->amperage = currentMeterESCState.amperage;
    meter->mAhDrawn = currentMeterESCState.mAhDrawn;
}

void currentMeterESCReadMotor(uint8_t motorNumber, currentMeter_t *meter)
{
    escSensorData_t *escData = getEscSensorData(motorNumber);
    if (escData && escData->dataAge <= ESC_BATTERY_AGE_MAX) {
        meter->amperage = escData->current;
        meter->amperageLatest = escData->current;
        meter->mAhDrawn = escData->consumption;
    } else {
        currentMeterReset(meter);
    }
}
#endif

#ifdef USE_MSP_CURRENT_METER
#include "common/streambuf.h"

#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

currentMeterMSPState_t currentMeterMSPState;

void currentMeterMSPSet(uint16_t amperage, uint16_t mAhDrawn)
{
    // We expect the FC's MSP_ANALOG response handler to call this function
    currentMeterMSPState.amperage = amperage;
    currentMeterMSPState.mAhDrawn = mAhDrawn;
}

void currentMeterMSPInit(void)
{
    memset(&currentMeterMSPState, 0, sizeof(currentMeterMSPState_t));
}

void currentMeterMSPRefresh(timeUs_t currentTimeUs)
{
    // periodically request MSP_ANALOG
    static timeUs_t streamRequestAt = 0;
    if (cmp32(currentTimeUs, streamRequestAt) > 0) {
        streamRequestAt = currentTimeUs + ((1000 * 1000) / 10); // 10hz

        mspSerialPush(SERIAL_PORT_ALL, MSP_ANALOG, NULL, 0, MSP_DIRECTION_REQUEST, MSP_V1);
    }
}

void currentMeterMSPRead(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterMSPState.amperage;
    meter->amperage = currentMeterMSPState.amperage;
    meter->mAhDrawn = currentMeterMSPState.mAhDrawn;
}
#endif

//
// INA226
//

#ifdef USE_CURRENT_METER_INA226

#ifndef DEFAULT_INA226_I2C_DEVICE
#define DEFAULT_INA226_I2C_DEVICE 1  // 1 = I2CDEV_0 (internal I2C bus)
#endif

#ifndef DEFAULT_INA226_ADDRESS
#define DEFAULT_INA226_ADDRESS INA226_I2C_ADDR_DEFAULT  // 0x40
#endif

#ifndef DEFAULT_INA226_SHUNT_RESISTANCE
#define DEFAULT_INA226_SHUNT_RESISTANCE 1000  // 1mΩ = 1000µΩ
#endif

#ifndef DEFAULT_INA226_MAX_CURRENT
#define DEFAULT_INA226_MAX_CURRENT 50000  // 50A
#endif

#ifndef DEFAULT_INA226_VBAT_SCALE
#define DEFAULT_INA226_VBAT_SCALE 100  // 100 = 1.00x (no scaling)
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(currentSensorINA226Config_t, currentSensorINA226Config, PG_CURRENT_SENSOR_INA226_CONFIG, 1);

PG_RESET_TEMPLATE(currentSensorINA226Config_t, currentSensorINA226Config,
    .i2cDevice = DEFAULT_INA226_I2C_DEVICE,
    .address = DEFAULT_INA226_ADDRESS,
    .shuntResistanceMicroOhms = DEFAULT_INA226_SHUNT_RESISTANCE,
    .maxExpectedCurrentMa = DEFAULT_INA226_MAX_CURRENT,
    .vbatScale = DEFAULT_INA226_VBAT_SCALE,
);

static currentMeterINA226State_t currentMeterINA226State;
static ina226Config_t ina226Cfg;
static bool ina226Initialized = false;
static pt1Filter_t ina226Filter;
static uint16_t ina226LastVoltageMv = 0;  // Last read voltage in mV (shared with voltage meter)

// Store last detection result for diagnostics
static uint8_t ina226LastDetectResult = 0;  // 0=not tried, 1=success, 2=mfg_id fail, 3=die_id fail, 4=init fail

uint8_t ina226GetDetectResult(void) {
    return ina226LastDetectResult;
}

bool ina226IsInitialized(void) {
    return ina226Initialized;
}

uint16_t ina226GetLastVoltageMv(void) {
    return ina226LastVoltageMv;
}

void currentMeterINA226Init(void)
{
    memset(&currentMeterINA226State, 0, sizeof(currentMeterINA226State_t));
    ina226Initialized = false;
    ina226LastDetectResult = 0;

    const currentSensorINA226Config_t *config = currentSensorINA226Config();
    
    // Wait for INA226 to power up (datasheet says 10us typical, but give it more time)
    delay(10);
    
    // Configure INA226 - i2cDevice follows Betaflight convention: 1 = I2CDEV_0, 2 = I2CDEV_1, etc.
    if (config->i2cDevice < 1 || config->i2cDevice > I2CDEV_COUNT) {
        // Invalid device, use default
        ina226Cfg.i2cDevice = I2CDEV_0;
    } else {
        ina226Cfg.i2cDevice = I2C_CFG_TO_DEV(config->i2cDevice);
    }
    
    ina226Cfg.address = config->address;
    ina226Cfg.shuntResistorMicroOhms = config->shuntResistanceMicroOhms;
    ina226Cfg.maxCurrentMa = config->maxExpectedCurrentMa;
    
    // Try to initialize at configured address only
    // Note: If INA226 is not present, this will fail silently to avoid I2C bus errors
    if (ina226Init(&ina226Cfg)) {
        ina226Initialized = true;
        ina226LastDetectResult = 1;  // Success
        // Initialize the filter with a 10Hz cutoff (similar to ADC current filter)
        pt1FilterInit(&ina226Filter, pt1FilterGain(GET_BATTERY_LPF_FREQUENCY(batteryConfig()->ibatLpfPeriod), HZ_TO_INTERVAL(50)));
    } else {
        ina226LastDetectResult = 4;  // Init failed
    }
}

void currentMeterINA226Refresh(int32_t lastUpdateAt)
{
    if (!ina226Initialized) {
        currentMeterINA226State.amperageLatest = 0;
        currentMeterINA226State.amperage = 0;
        ina226LastVoltageMv = 0;
        return;
    }
    
    // Sanity check: limit lastUpdateAt to prevent overflow (max 10 seconds = 10,000,000 us)
    // This handles the case where ibatLastServiced was 0 on first call
    if (lastUpdateAt > 10000000 || lastUpdateAt < 0) {
        lastUpdateAt = 10000;  // Default to 10ms if unreasonable value
    }
    
    // Read ALL data at once (shunt voltage, bus voltage, current, power)
    // This avoids multiple I2C transactions
    ina226Data_t data;
    if (ina226Read(&ina226Cfg, &data)) {
        // Store voltage for voltage meter to use (no separate I2C call needed)
        // Apply calibration scale: vbatScale=100 means 1.00x, vbatScale=103 means 1.03x
        const currentSensorINA226Config_t *config = currentSensorINA226Config();
        uint8_t scale = config->vbatScale;
        if (scale == 0) {
            scale = 100;  // Safety: default to 1.00x if not configured
        }
        ina226LastVoltageMv = (uint16_t)((uint32_t)data.busVoltageMv * scale / 100);
        
        // Convert mA to centiamperes (1/100 A)
        int32_t centiAmps = data.currentMa / 10;
        
        currentMeterINA226State.amperageLatest = centiAmps;
        currentMeterINA226State.amperage = pt1FilterApply(&ina226Filter, centiAmps);
        
        // Update mAh drawn
        // Use float from the start to avoid integer overflow:
        // mAh = centiAmps * lastUpdateAt_us / (100 * 1000 * 3600) = centiAmps * lastUpdateAt_us / 360000000
        currentMeterINA226State.mahDrawnState.mAhDrawnF += 
            (float)currentMeterINA226State.amperageLatest * (float)lastUpdateAt / (100.0f * 1000.0f * 3600.0f);
        currentMeterINA226State.mahDrawnState.mAhDrawn = (int32_t)currentMeterINA226State.mahDrawnState.mAhDrawnF;
    }
}

void currentMeterINA226Read(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterINA226State.amperageLatest;
    meter->amperage = currentMeterINA226State.amperage;
    meter->mAhDrawn = currentMeterINA226State.mahDrawnState.mAhDrawn;
}

#endif // USE_CURRENT_METER_INA226

//
// API for current meters using IDs
//
// This API is used by MSP, for configuration/status.
//

void currentMeterRead(currentMeterId_e id, currentMeter_t *meter)
{
    if (id == CURRENT_METER_ID_BATTERY_1) {
        currentMeterADCRead(meter);
    }
#ifdef USE_VIRTUAL_CURRENT_METER
    else if (id == CURRENT_METER_ID_VIRTUAL_1) {
        currentMeterVirtualRead(meter);
    }
#endif
#ifdef USE_MSP_CURRENT_METER
    else if (id == CURRENT_METER_ID_MSP_1) {
        currentMeterMSPRead(meter);
    }
#endif
#ifdef USE_ESC_SENSOR
    else if (id == CURRENT_METER_ID_ESC_COMBINED_1) {
        currentMeterESCReadCombined(meter);
    }
    else if (id >= CURRENT_METER_ID_ESC_MOTOR_1 && id <= CURRENT_METER_ID_ESC_MOTOR_20 ) {
        int motor = id - CURRENT_METER_ID_ESC_MOTOR_1;
        currentMeterESCReadMotor(motor, meter);
    }
#endif
#ifdef USE_CURRENT_METER_INA226
    else if (id == CURRENT_METER_ID_INA226_1) {
        currentMeterINA226Read(meter);
    }
#endif
    else {
        currentMeterReset(meter);
    }
}
