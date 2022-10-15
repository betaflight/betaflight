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
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/adc.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"

#include "io/beeper.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "scheduler/scheduler.h"
#ifdef USE_BATTERY_CONTINUE
#include "pg/stats.h"
#endif

#include "sensors/battery.h"

/**
 * terminology: meter vs sensors
 *
 * voltage and current sensors are used to collect data.
 * - e.g. voltage at an MCU ADC input pin, value from an ESC sensor.
 *   sensors require very specific configuration, such as resistor values.
 * voltage and current meters are used to process and expose data collected from sensors to the rest of the system.
 * - e.g. a meter exposes normalized, and often filtered, values from a sensor.
 *   meters require different or little configuration.
 *   meters also have different precision concerns, and may use different units to the sensors.
 *
 */

#define VBAT_STABLE_MAX_DELTA 20
#define LVC_AFFECT_TIME 10000000 //10 secs for the LVC to slowly kick in

// Battery monitoring stuff
static uint8_t batteryCellCount; // Note: this can be 0 when no battery is detected or when the battery voltage sensor is missing or disabled.
static uint16_t batteryWarningVoltage;
static uint16_t batteryCriticalVoltage;
static uint16_t batteryWarningHysteresisVoltage;
static uint16_t batteryCriticalHysteresisVoltage;
static lowVoltageCutoff_t lowVoltageCutoff;

static currentMeter_t currentMeter;
static voltageMeter_t voltageMeter;

static batteryState_e batteryState;
static batteryState_e voltageState;
static batteryState_e consumptionState;
static float wattHoursDrawn;

#ifndef DEFAULT_CURRENT_METER_SOURCE
#ifdef USE_VIRTUAL_CURRENT_METER
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_VIRTUAL
#else
#ifdef USE_MSP_CURRENT_METER
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_MSP
#else
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_NONE
#endif
#endif
#endif

#ifndef DEFAULT_VOLTAGE_METER_SOURCE
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_NONE
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 3);

PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,
    // voltage
    .vbatmaxcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MAX,
    .vbatmincellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MIN,
    .vbatwarningcellvoltage = 350,
    .vbatnotpresentcellvoltage = 300, //A cell below 3 will be ignored
    .voltageMeterSource = DEFAULT_VOLTAGE_METER_SOURCE,
    .lvcPercentage = 100, //Off by default at 100%

    // current
    .batteryCapacity = 0,
    .currentMeterSource = DEFAULT_CURRENT_METER_SOURCE,

    // cells
    .forceBatteryCellCount = 0, //0 will be ignored

    // warnings / alerts
    .useVBatAlerts = true,
    .useConsumptionAlerts = false,
    .consumptionWarningPercentage = 10,
    .vbathysteresis = 1, // 0.01V

    .vbatfullcellvoltage = 410,

    .vbatDisplayLpfPeriod = 30,
    .vbatSagLpfPeriod = 2,
    .ibatLpfPeriod = 10,
    .vbatDurationForWarning = 0,
    .vbatDurationForCritical = 0,
);

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
        case VOLTAGE_METER_ADC:
            voltageMeterADCRefresh();
            voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VBAT, &voltageMeter);
            break;

        default:
        case VOLTAGE_METER_NONE:
            voltageMeterReset(&voltageMeter);
            break;
    }

    DEBUG_SET(DEBUG_BATTERY, 0, voltageMeter.unfiltered);
    DEBUG_SET(DEBUG_BATTERY, 1, voltageMeter.displayFiltered);
}

static void updateBatteryBeeperAlert(void)
{
    switch (getBatteryState()) {
        case BATTERY_WARNING:
            beeper(BEEPER_BAT_LOW);

            break;
        case BATTERY_CRITICAL:
            beeper(BEEPER_BAT_CRIT_LOW);

            break;
        case BATTERY_OK:
        case BATTERY_NOT_PRESENT:
        case BATTERY_INIT:
            break;
    }
}

//TODO: make all of these independent of voltage filtering for display

static bool isVoltageStable(void)
{
    return abs(voltageMeter.displayFiltered - voltageMeter.unfiltered) <= VBAT_STABLE_MAX_DELTA;
}

static bool isVoltageFromBat(void)
{
    // We want to disable battery getting detected around USB voltage or 0V

    return (voltageMeter.displayFiltered >= batteryConfig()->vbatnotpresentcellvoltage  // Above ~0V
        && voltageMeter.displayFiltered <= batteryConfig()->vbatmaxcellvoltage)  // 1s max cell voltage check
        || voltageMeter.displayFiltered > batteryConfig()->vbatnotpresentcellvoltage * 2; // USB voltage - 2s or more check
}

void batteryUpdatePresence(void)
{

    if ((voltageState == BATTERY_NOT_PRESENT || voltageState == BATTERY_INIT) && isVoltageFromBat() && isVoltageStable()) {
        // Battery has just been connected - calculate cells, warning voltages and reset state

        consumptionState = voltageState = BATTERY_OK;
        if (batteryConfig()->forceBatteryCellCount != 0) {
            batteryCellCount = batteryConfig()->forceBatteryCellCount;
        } else {
            unsigned cells = (voltageMeter.displayFiltered / batteryConfig()->vbatmaxcellvoltage) + 1;
            if (cells > MAX_AUTO_DETECT_CELL_COUNT) {
                // something is wrong, we expect MAX_CELL_COUNT cells maximum (and autodetection will be problematic at 6+ cells)
                cells = MAX_AUTO_DETECT_CELL_COUNT;
            }
            batteryCellCount = cells;

            if (!ARMING_FLAG(ARMED)) {
                changePidProfileFromCellCount(batteryCellCount);
            }
        }
        batteryWarningVoltage = batteryCellCount * batteryConfig()->vbatwarningcellvoltage;
        batteryCriticalVoltage = batteryCellCount * batteryConfig()->vbatmincellvoltage;
        batteryWarningHysteresisVoltage = (batteryWarningVoltage > batteryConfig()->vbathysteresis) ? batteryWarningVoltage - batteryConfig()->vbathysteresis : 0;
        batteryCriticalHysteresisVoltage = (batteryCriticalVoltage > batteryConfig()->vbathysteresis) ? batteryCriticalVoltage - batteryConfig()->vbathysteresis : 0;
        lowVoltageCutoff.percentage = 100;
        lowVoltageCutoff.startTime = 0;
    } else if (voltageState != BATTERY_NOT_PRESENT && isVoltageStable() && !isVoltageFromBat()) {
        /* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of batteryConfig()->vbatnotpresentcellvoltage */

        consumptionState = voltageState = BATTERY_NOT_PRESENT;

        batteryCellCount = 0;
        batteryWarningVoltage = 0;
        batteryCriticalVoltage = 0;
        batteryWarningHysteresisVoltage = 0;
        batteryCriticalHysteresisVoltage = 0;
        wattHoursDrawn = 0.0;
    }
}

void batteryUpdateWhDrawn(void) 
{
    static int32_t mAhDrawnPrev = 0;
    const int32_t mAhDrawnCurrent = getMAhDrawn();
    wattHoursDrawn += voltageMeter.displayFiltered * (mAhDrawnCurrent - mAhDrawnPrev) / 100000.0f;
    mAhDrawnPrev = mAhDrawnCurrent;
}

static void batteryUpdateVoltageState(void)
{
    // alerts are currently used by beeper, osd and other subsystems
    static uint32_t lastVoltageChangeMs;
    switch (voltageState) {
        case BATTERY_OK:
            if (voltageMeter.displayFiltered <= batteryWarningHysteresisVoltage) {
                if (cmp32(millis(), lastVoltageChangeMs) >= batteryConfig()->vbatDurationForWarning * 100) {
                    voltageState = BATTERY_WARNING;
                }
            } else {
                lastVoltageChangeMs = millis();
            }
            break;

        case BATTERY_WARNING:
            if (voltageMeter.displayFiltered <= batteryCriticalHysteresisVoltage) {
                if (cmp32(millis(), lastVoltageChangeMs) >= batteryConfig()->vbatDurationForCritical * 100) {
                    voltageState = BATTERY_CRITICAL;
                }
            } else {
                if (voltageMeter.displayFiltered > batteryWarningVoltage) {
                    voltageState = BATTERY_OK;
                }
                lastVoltageChangeMs = millis();
            }
            break;

        case BATTERY_CRITICAL:
            if (voltageMeter.displayFiltered > batteryCriticalVoltage) {
                voltageState = BATTERY_WARNING;
                lastVoltageChangeMs = millis();
            }
            break;

        default:
            break;
    }

}

static void batteryUpdateLVC(timeUs_t currentTimeUs)
{
    if (batteryConfig()->lvcPercentage < 100) {
        if (voltageState == BATTERY_CRITICAL && !lowVoltageCutoff.enabled) {
            lowVoltageCutoff.enabled = true;
            lowVoltageCutoff.startTime = currentTimeUs;
            lowVoltageCutoff.percentage = 100;
        }
        if (lowVoltageCutoff.enabled) {
            if (cmp32(currentTimeUs,lowVoltageCutoff.startTime) < LVC_AFFECT_TIME) {
                lowVoltageCutoff.percentage = 100 - (cmp32(currentTimeUs,lowVoltageCutoff.startTime) * (100 - batteryConfig()->lvcPercentage) / LVC_AFFECT_TIME);
            }
            else {
                lowVoltageCutoff.percentage = batteryConfig()->lvcPercentage;
            }
        }
    }

}

static void batteryUpdateConsumptionState(void)
{
    if (batteryConfig()->useConsumptionAlerts && batteryConfig()->batteryCapacity > 0 && batteryCellCount > 0) {
        uint8_t batteryPercentageRemaining = calculateBatteryPercentageRemaining();

        if (batteryPercentageRemaining == 0) {
            consumptionState = BATTERY_CRITICAL;
        } else if (batteryPercentageRemaining <= batteryConfig()->consumptionWarningPercentage) {
            consumptionState = BATTERY_WARNING;
        } else {
            consumptionState = BATTERY_OK;
        }
    }
}

void batteryUpdateStates(timeUs_t currentTimeUs)
{
    batteryUpdateVoltageState();
    batteryUpdateConsumptionState();
    batteryUpdateLVC(currentTimeUs);
    batteryState = MAX(voltageState, consumptionState);
    batteryUpdateWhDrawn();
}

const lowVoltageCutoff_t *getLowVoltageCutoff(void)
{
    return &lowVoltageCutoff;
}

batteryState_e getBatteryState(void)
{
    return batteryState;
}

batteryState_e getVoltageState(void)
{
    return voltageState;
}

batteryState_e getConsumptionState(void)
{
    return consumptionState;
}

const char * const batteryStateStrings[] = {"OK", "WARNING", "CRITICAL", "NOT PRESENT", "INIT"};

const char * getBatteryStateString(void)
{
    return batteryStateStrings[getBatteryState()];
}

void batteryInit(void)
{
    //
    // presence
    //
    batteryState = BATTERY_INIT;
    batteryCellCount = 0;

    //
    // Consumption
    //
    wattHoursDrawn = 0;

    //
    // voltage
    //
    voltageState = BATTERY_INIT;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;
    batteryWarningHysteresisVoltage = 0;
    batteryCriticalHysteresisVoltage = 0;
    lowVoltageCutoff.enabled = false;
    lowVoltageCutoff.percentage = 100;
    lowVoltageCutoff.startTime = 0;

    voltageMeterReset(&voltageMeter);

    voltageMeterGenericInit();
    switch (batteryConfig()->voltageMeterSource) {
        case VOLTAGE_METER_ESC:
#ifdef USE_ESC_SENSOR
            voltageMeterESCInit();
#endif
            break;

        case VOLTAGE_METER_ADC:
            voltageMeterADCInit();
            break;

        default:
            break;
    }

    //
    // current
    //
    consumptionState = BATTERY_OK;
    currentMeterReset(&currentMeter);
    switch (batteryConfig()->currentMeterSource) {
        case CURRENT_METER_ADC:
            currentMeterADCInit();
            break;

        case CURRENT_METER_VIRTUAL:
#ifdef USE_VIRTUAL_CURRENT_METER
            currentMeterVirtualInit();
#endif
            break;

        case CURRENT_METER_ESC:
#ifdef ESC_SENSOR
            currentMeterESCInit();
#endif
            break;
        case CURRENT_METER_MSP:
#ifdef USE_MSP_CURRENT_METER
            currentMeterMSPInit();
#endif
            break;

        default:
            break;
    }
}

void batteryUpdateCurrentMeter(timeUs_t currentTimeUs)
{
    if (batteryCellCount == 0) {
        currentMeterReset(&currentMeter);
        return;
    }

    static uint32_t ibatLastServiced = 0;
    const int32_t lastUpdateAt = cmp32(currentTimeUs, ibatLastServiced);
    ibatLastServiced = currentTimeUs;

    switch (batteryConfig()->currentMeterSource) {
        case CURRENT_METER_ADC:
            currentMeterADCRefresh(lastUpdateAt);
            currentMeterADCRead(&currentMeter);
            break;

        case CURRENT_METER_VIRTUAL: {
#ifdef USE_VIRTUAL_CURRENT_METER
            throttleStatus_e throttleStatus = calculateThrottleStatus();
            bool throttleLowAndMotorStop = (throttleStatus == THROTTLE_LOW && featureIsEnabled(FEATURE_MOTOR_STOP));
            const int32_t throttleOffset = lrintf(mixerGetThrottle() * 1000);

            currentMeterVirtualRefresh(lastUpdateAt, ARMING_FLAG(ARMED), throttleLowAndMotorStop, throttleOffset);
            currentMeterVirtualRead(&currentMeter);
#endif
            break;
        }

        case CURRENT_METER_ESC:
#ifdef USE_ESC_SENSOR
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                currentMeterESCRefresh(lastUpdateAt);
                currentMeterESCReadCombined(&currentMeter);
            }
#endif
            break;
        case CURRENT_METER_MSP:
#ifdef USE_MSP_CURRENT_METER
            currentMeterMSPRefresh(currentTimeUs);
            currentMeterMSPRead(&currentMeter);
#endif
            break;

        default:
        case CURRENT_METER_NONE:
            currentMeterReset(&currentMeter);
            break;
    }
}

uint8_t calculateBatteryPercentageRemaining(void)
{
    uint8_t batteryPercentage = 0;
    if (batteryCellCount > 0) {
        uint16_t batteryCapacity = batteryConfig()->batteryCapacity;

        if (batteryCapacity > 0) {
            batteryPercentage = constrain(((float)batteryCapacity - currentMeter.mAhDrawn) * 100 / batteryCapacity, 0, 100);
        } else {
            batteryPercentage = constrain((((uint32_t)voltageMeter.displayFiltered - (batteryConfig()->vbatmincellvoltage * batteryCellCount)) * 100) / ((batteryConfig()->vbatmaxcellvoltage - batteryConfig()->vbatmincellvoltage) * batteryCellCount), 0, 100);
        }
    }

    return batteryPercentage;
}

void batteryUpdateAlarms(void)
{
    // use the state to trigger beeper alerts
    if (batteryConfig()->useVBatAlerts) {
        updateBatteryBeeperAlert();
    }
}

bool isBatteryVoltageConfigured(void)
{
    return batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;
}

uint16_t getBatteryVoltage(void)
{
    return voltageMeter.displayFiltered;
}

uint16_t getLegacyBatteryVoltage(void)
{
    return (voltageMeter.displayFiltered + 5) / 10;
}

uint16_t getBatteryVoltageLatest(void)
{
    return voltageMeter.unfiltered;
}

uint8_t getBatteryCellCount(void)
{
    return batteryCellCount;
}

uint16_t getBatteryAverageCellVoltage(void)
{
    return (batteryCellCount ? voltageMeter.displayFiltered / batteryCellCount : 0);
}

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
uint16_t getBatterySagCellVoltage(void)
{
    return (batteryCellCount ? voltageMeter.sagFiltered / batteryCellCount : 0);
}
#endif

bool isAmperageConfigured(void)
{
    return batteryConfig()->currentMeterSource != CURRENT_METER_NONE;
}

int32_t getAmperage(void)
{
    return currentMeter.amperage;
}

int32_t getAmperageLatest(void)
{
    return currentMeter.amperageLatest;
}

int32_t getMAhDrawn(void)
{
#ifdef USE_BATTERY_CONTINUE
    return currentMeter.mAhDrawn + currentMeter.mAhDrawnOffset;
#else
    return currentMeter.mAhDrawn;
#endif
}

#ifdef USE_BATTERY_CONTINUE
bool hasUsedMAh(void)
{
    return batteryConfig()->isBatteryContinueEnabled
          && !(ARMING_FLAG(ARMED) || ARMING_FLAG(WAS_EVER_ARMED)) && (getBatteryState() == BATTERY_OK)
          && getBatteryAverageCellVoltage() < batteryConfig()->vbatfullcellvoltage
          && statsConfig()->stats_mah_used > 0;
}

void setMAhDrawn(uint32_t mAhDrawn)
{
    currentMeter.mAhDrawnOffset = mAhDrawn;
}
#endif

float getWhDrawn(void)
{
    return wattHoursDrawn;
}
