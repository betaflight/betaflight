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

#pragma once

#include "pg/pg.h"

#include "common/filter.h"
#include "common/time.h"
#include "sensors/current.h"
#include "sensors/voltage.h"

typedef struct batteryConfig_s {
    // voltage
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
    uint8_t vbatwarningcellvoltage;         // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
    uint8_t vbatnotpresentcellvoltage;      // Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
    uint8_t lvcPercentage;                  // Percentage of throttle when lvc is triggered
    voltageMeterSource_e voltageMeterSource; // source of battery voltage meter used, either ADC or ESC

    // current
    currentMeterSource_e currentMeterSource; // source of battery current meter used, either ADC, Virtual or ESC
    uint16_t batteryCapacity;               // mAh

    // warnings / alerts
    bool useVBatAlerts;                     // Issue alerts based on VBat readings
    bool useConsumptionAlerts;              // Issue alerts based on total power consumption
    uint8_t consumptionWarningPercentage;   // Percentage of remaining capacity that should trigger a battery warning
    uint8_t vbathysteresis;                 // hysteresis for alarm, default 1 = 0.1V

    uint8_t vbatfullcellvoltage;            // Cell voltage at which the battery is deemed to be "full" 0.1V units, default is 41 (4.1V)

} batteryConfig_t;

typedef struct lowVoltageCutoff_s {
    bool enabled;
    uint8_t percentage;
    timeUs_t startTime;
} lowVoltageCutoff_t;

PG_DECLARE(batteryConfig_t, batteryConfig);

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT,
    BATTERY_INIT
} batteryState_e;

void batteryInit(void);
void batteryUpdateVoltage(timeUs_t currentTimeUs);
void batteryUpdatePresence(void);

batteryState_e getBatteryState(void);
const  char * getBatteryStateString(void);

void batteryUpdateStates(timeUs_t currentTimeUs);
void batteryUpdateAlarms(void);

struct rxConfig_s;

float calculateVbatPidCompensation(void);
uint8_t calculateBatteryPercentageRemaining(void);
bool isBatteryVoltageConfigured(void);
uint16_t getBatteryVoltage(void);
uint16_t getBatteryVoltageLatest(void);
uint8_t getBatteryCellCount(void);
uint16_t getBatteryAverageCellVoltage(void);

bool isAmperageConfigured(void);
int32_t getAmperage(void);
int32_t getAmperageLatest(void);
int32_t getMAhDrawn(void);

void batteryUpdateCurrentMeter(timeUs_t currentTimeUs);

const lowVoltageCutoff_t *getLowVoltageCutoff(void);
