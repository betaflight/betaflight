/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "stdbool.h"
#include "stdint.h"

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "drivers/adc.h"
#include "drivers/system.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "config/feature.h"

#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "fc/rc_controls.h"
#include "io/beeper.h"

#include "rx/rx.h"

#include "common/utils.h"

#define VBAT_LPF_FREQ  0.4f
#define IBAT_LPF_FREQ  0.4f

#define VBAT_STABLE_MAX_DELTA 2

#define ADCVREF 3300   // in mV

#define MAX_ESC_BATTERY_AGE 10

// Battery monitoring stuff
uint8_t batteryCellCount;
uint16_t batteryWarningVoltage;
uint16_t batteryCriticalVoltage;

uint16_t vbat = 0;                  // battery voltage in 0.1V steps (filtered)
uint16_t vbatLatest = 0;            // most recent unsmoothed value

int32_t amperage = 0;               // amperage read by current sensor in centiampere (1/100th A)
uint16_t amperageLatest = 0;        // most recent value

int32_t mAhDrawn = 0;               // milliampere hours drawn from the battery since start

static batteryState_e vBatState;
static batteryState_e consumptionState;

static uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return ((((uint32_t)src * batteryConfig->vbatscale * 33 + (0xFFF * 5)) / (0xFFF * batteryConfig->vbatresdivval))/batteryConfig->vbatresdivmultiplier);
}

static void updateBatteryVoltage(void)
{
    static biquadFilter_t vBatFilter;
    static bool vBatFilterIsInitialised;

    if (!vBatFilterIsInitialised) {
        biquadFilterInitLPF(&vBatFilter, VBAT_LPF_FREQ, 50000); //50HZ Update
        vBatFilterIsInitialised = true;
    }

    #ifdef USE_ESC_SENSOR
    if (feature(FEATURE_ESC_SENSOR) && batteryConfig->batteryMeterType == BATTERY_SENSOR_ESC) {
        escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
        vbatLatest = escData->dataAge <= MAX_ESC_BATTERY_AGE ? escData->voltage / 10 : 0;
        if (debugMode == DEBUG_BATTERY) {
            debug[0] = -1;
        }
        vbat = biquadFilterApply(&vBatFilter, vbatLatest);
    }
    else
    #endif
    {
        uint16_t vBatSample = adcGetChannel(ADC_BATTERY);
        if (debugMode == DEBUG_BATTERY) {
            debug[0] = vBatSample;
        }
        vbat = batteryAdcToVoltage(biquadFilterApply(&vBatFilter, vBatSample));
        vbatLatest = batteryAdcToVoltage(vBatSample);
    }

    if (debugMode == DEBUG_BATTERY) {
        debug[1] = vbat;
    }
}

static void updateBatteryAlert(void)
{
    switch(getBatteryState()) {
        case BATTERY_WARNING:
            beeper(BEEPER_BAT_LOW);

            break;
        case BATTERY_CRITICAL:
            beeper(BEEPER_BAT_CRIT_LOW);

            break;
        case BATTERY_OK:
        case BATTERY_NOT_PRESENT:
            break;
    }
}

void updateBattery(void)
{
    uint16_t vBatPrevious = vbatLatest;
    updateBatteryVoltage();
    uint16_t vBatMeasured = vbatLatest;

    /* battery has just been connected*/
    if (vBatState == BATTERY_NOT_PRESENT && (ARMING_FLAG(ARMED) || (vbat > batteryConfig->batterynotpresentlevel && ABS(vBatMeasured - vBatPrevious) <= VBAT_STABLE_MAX_DELTA))) {
        /* Actual battery state is calculated below, this is really BATTERY_PRESENT */
        vBatState = BATTERY_OK;

        unsigned cells = (vBatMeasured / batteryConfig->vbatmaxcellvoltage) + 1;
        if (cells > 8) {
            // something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)
            cells = 8;
        }
        batteryCellCount = cells;
        batteryWarningVoltage = batteryCellCount * batteryConfig->vbatwarningcellvoltage;
        batteryCriticalVoltage = batteryCellCount * batteryConfig->vbatmincellvoltage;
    /* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of batteryConfig->batterynotpresentlevel */
    } else if (vBatState != BATTERY_NOT_PRESENT && !ARMING_FLAG(ARMED) && vbat <= batteryConfig->batterynotpresentlevel && ABS(vBatMeasured - vBatPrevious) <= VBAT_STABLE_MAX_DELTA) {
        vBatState = BATTERY_NOT_PRESENT;
        batteryCellCount = 0;
        batteryWarningVoltage = 0;
        batteryCriticalVoltage = 0;
    }

    if (debugMode == DEBUG_BATTERY) {
        debug[2] = vBatState;
        debug[3] = batteryCellCount;
    }

    if (batteryConfig->useVBatAlerts) {
        switch(vBatState) {
            case BATTERY_OK:
                if (vbat <= (batteryWarningVoltage - batteryConfig->vbathysteresis)) {
                    vBatState = BATTERY_WARNING;
                }

                break;
            case BATTERY_WARNING:
                if (vbat <= (batteryCriticalVoltage - batteryConfig->vbathysteresis)) {
                    vBatState = BATTERY_CRITICAL;
                } else if (vbat > batteryWarningVoltage) {
                    vBatState = BATTERY_OK;
                }

                break;
            case BATTERY_CRITICAL:
                if (vbat > batteryCriticalVoltage) {
                    vBatState = BATTERY_WARNING;
                }

                break;
            case BATTERY_NOT_PRESENT:
                break;
        }

        updateBatteryAlert();
    }
}

batteryState_e getBatteryState(void)
{
    batteryState_e batteryState = BATTERY_NOT_PRESENT;
    if (vBatState != BATTERY_NOT_PRESENT) {
        batteryState = MAX(vBatState, consumptionState);
    }

    return batteryState;
}

const char * const batteryStateStrings[] = {"OK", "WARNING", "CRITICAL", "NOT PRESENT"};

const char * getBatteryStateString(void)
{
    return batteryStateStrings[getBatteryState()];
}

void batteryInit(batteryConfig_t *initialBatteryConfig)
{
    batteryConfig = initialBatteryConfig;
    vBatState = BATTERY_NOT_PRESENT;
    consumptionState = BATTERY_OK;
    batteryCellCount = 0;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;
}

static int32_t currentSensorToCentiamps(uint16_t src)
{
    int32_t millivolts;

    millivolts = ((uint32_t)src * ADCVREF) / 4096;
    millivolts -= batteryConfig->currentMeterOffset;

    return (millivolts * 1000) / (int32_t)batteryConfig->currentMeterScale; // current in 0.01A steps
}

static void updateBatteryCurrent(void)
{
    static biquadFilter_t iBatFilter;
    static bool iBatFilterIsInitialised;

    if (!iBatFilterIsInitialised) {
        biquadFilterInitLPF(&iBatFilter, IBAT_LPF_FREQ, 50000); //50HZ Update
        iBatFilterIsInitialised = true;
    }

    uint16_t iBatSample = adcGetChannel(ADC_CURRENT);
    amperageLatest = currentSensorToCentiamps(iBatSample);
    amperage = currentSensorToCentiamps(biquadFilterApply(&iBatFilter, iBatSample));
}

static void updateCurrentDrawn(int32_t lastUpdateAt)
{
    static float mAhDrawnF = 0.0f; // used to get good enough resolution

    mAhDrawnF = mAhDrawnF + (amperageLatest * lastUpdateAt / (100.0f * 1000 * 3600));
    mAhDrawn = mAhDrawnF;
}

void updateConsumptionWarning(void)
{
    if (batteryConfig->useConsumptionAlerts && batteryConfig->batteryCapacity > 0 && getBatteryState() != BATTERY_NOT_PRESENT) {
        if (calculateBatteryPercentage() == 0) {
            vBatState = BATTERY_CRITICAL;
        } else if (calculateBatteryPercentage() <= batteryConfig->consumptionWarningPercentage) {
            consumptionState = BATTERY_WARNING;
        } else {
            consumptionState = BATTERY_OK;
        }

        updateBatteryAlert();
    }
}

void updateCurrentMeter(int32_t lastUpdateAt, rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    switch(batteryConfig->currentMeterType) {
        case CURRENT_SENSOR_ADC:
            updateBatteryCurrent();

            updateCurrentDrawn(lastUpdateAt);

            updateConsumptionWarning();

            break;
        case CURRENT_SENSOR_VIRTUAL:
            amperageLatest = (int32_t)batteryConfig->currentMeterOffset;
            if (ARMING_FLAG(ARMED)) {
                throttleStatus_e throttleStatus = calculateThrottleStatus(rxConfig, deadband3d_throttle);
                int throttleOffset = (int32_t)rcCommand[THROTTLE] - 1000;
                if (throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP)) {
                    throttleOffset = 0;
                }
                int throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50);
                amperageLatest += throttleFactor * (int32_t)batteryConfig->currentMeterScale  / 1000;
            }
            amperage = amperageLatest;

            updateCurrentDrawn(lastUpdateAt);

            updateConsumptionWarning();

            break;
        case CURRENT_SENSOR_ESC:
            #ifdef USE_ESC_SENSOR
            if (feature(FEATURE_ESC_SENSOR)) {
                escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
                if (escData->dataAge <= MAX_ESC_BATTERY_AGE) {
                    amperageLatest = escData->current;
                    mAhDrawn = escData->consumption;
                } else {
                    amperageLatest = 0;
                    mAhDrawn = 0;
                }
                amperage = amperageLatest;

                updateConsumptionWarning();
            }

            break;
            #endif
        case CURRENT_SENSOR_NONE:
            amperage = 0;
            amperageLatest = 0;

            break;
    }
}

float calculateVbatPidCompensation(void) {
    float batteryScaler =  1.0f;
    if (feature(FEATURE_VBAT) && batteryCellCount > 0) {
        // Up to 33% PID gain. Should be fine for 4,2to 3,3 difference
        batteryScaler =  constrainf((( (float)batteryConfig->vbatmaxcellvoltage * batteryCellCount ) / (float) vbat), 1.0f, 1.33f);
    }
    return batteryScaler;
}

uint8_t calculateBatteryPercentage(void)
{
    uint8_t batteryPercentage = 0;
    if (batteryCellCount > 0) {
        uint16_t batteryCapacity = batteryConfig->batteryCapacity;
        if (batteryCapacity > 0) {
            batteryPercentage = constrain(((float)batteryCapacity - mAhDrawn)  * 100 / batteryCapacity, 0, 100);
        } else {
            batteryPercentage = constrain((((uint32_t)vbat - (batteryConfig->vbatmincellvoltage * batteryCellCount)) * 100) / ((batteryConfig->vbatmaxcellvoltage - batteryConfig->vbatmincellvoltage) * batteryCellCount), 0, 100);
        }
    }

    return batteryPercentage;
}

uint16_t getVbat(void)
{
    return vbat;
}
