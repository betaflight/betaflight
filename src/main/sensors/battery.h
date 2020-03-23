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

#pragma once

#include "pg/pg.h"

#include "common/filter.h"
#include "common/time.h"
#include "sensors/current.h"
#include "sensors/voltage.h"

//TODO: Make the 'cell full' voltage user adjustble
#define CELL_VOLTAGE_FULL_CV 420

#define VBAT_CELL_VOTAGE_RANGE_MIN 100
#define VBAT_CELL_VOTAGE_RANGE_MAX 500

#define MAX_AUTO_DETECT_CELL_COUNT 8

#define GET_BATTERY_LPF_FREQUENCY(period) (1 / (period / 10.0f))

enum {
    AUTO_PROFILE_CELL_COUNT_STAY = 0, // Stay on this profile irrespective of the detected cell count. Use this profile if no other profile matches (default, i.e. auto profile switching is off)
    AUTO_PROFILE_CELL_COUNT_CHANGE = -1, // Always switch to a profile with matching cell count if there is one
};

typedef struct batteryConfig_s {
    // voltage
    uint16_t vbatmaxcellvoltage;            // maximum voltage per cell, used for auto-detecting battery voltage in 0.01V units, default is 430 (4.30V)
    uint16_t vbatmincellvoltage;            // minimum voltage per cell, this triggers battery critical alarm, in 0.01V units, default is 330 (3.30V)
    uint16_t vbatwarningcellvoltage;        // warning voltage per cell, this triggers battery warning alarm, in 0.01V units, default is 350 (3.50V)
    uint16_t vbatnotpresentcellvoltage;     // Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
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

    uint16_t vbatfullcellvoltage;           // Cell voltage at which the battery is deemed to be "full" 0.01V units, default is 410 (4.1V)

    uint8_t forceBatteryCellCount;          // Number of cells in battery, used for overwriting auto-detected cell count if someone has issues with it.
    uint8_t vbatDisplayLpfPeriod;           // Period of the cutoff frequency for the Vbat filter for display and startup (in 0.1 s)
    uint8_t ibatLpfPeriod;                  // Period of the cutoff frequency for the Ibat filter (in 0.1 s)
    uint8_t vbatDurationForWarning;         // Period voltage has to sustain before the battery state is set to BATTERY_WARNING (in 0.1 s)
    uint8_t vbatDurationForCritical;        // Period voltage has to sustain before the battery state is set to BATTERY_CRIT (in 0.1 s)
    uint8_t vbatSagLpfPeriod;               // Period of the cutoff frequency for the Vbat sag and PID compensation filter (in 0.1 s)
} batteryConfig_t;

PG_DECLARE(batteryConfig_t, batteryConfig);

typedef struct lowVoltageCutoff_s {
    bool enabled;
    uint8_t percentage;
    timeUs_t startTime;
} lowVoltageCutoff_t;

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
batteryState_e getVoltageState(void);
batteryState_e getConsumptionState(void);
const  char * getBatteryStateString(void);

void batteryUpdateStates(timeUs_t currentTimeUs);
void batteryUpdateAlarms(void);

struct rxConfig_s;

uint8_t calculateBatteryPercentageRemaining(void);
bool isBatteryVoltageConfigured(void);
uint16_t getBatteryVoltage(void);
uint16_t getLegacyBatteryVoltage(void);
uint16_t getBatteryVoltageLatest(void);
uint8_t getBatteryCellCount(void);
uint16_t getBatteryAverageCellVoltage(void);
uint16_t getBatterySagCellVoltage(void);

bool isAmperageConfigured(void);
int32_t getAmperage(void);
int32_t getAmperageLatest(void);
int32_t getMAhDrawn(void);

void batteryUpdateCurrentMeter(timeUs_t currentTimeUs);

const lowVoltageCutoff_t *getLowVoltageCutoff(void);
