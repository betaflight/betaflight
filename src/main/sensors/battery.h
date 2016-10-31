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

typedef struct batteryConfig_s {
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
    uint8_t vbatwarningcellvoltage;         // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
    uint16_t batteryCapacity;               // mAh
    uint8_t amperageMeterSource;             // see amperageMeter_e - used for telemetry, led strip, legacy MSP.
    uint8_t vbathysteresis;                 // hysteresis for alarm, default 1 = 0.1V
} batteryConfig_t;

PG_DECLARE(batteryConfig_t, batteryConfig);

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT
} batteryState_e;

extern uint16_t vbat;

extern uint8_t batteryCellCount;
extern uint16_t batteryWarningVoltage;

uint16_t batteryAdcToVoltage(uint16_t src);
batteryState_e getBatteryState(void);
const  char * getBatteryStateString(void);
void batteryUpdate(void);
void batteryInit(void);


uint8_t batteryVoltagePercentage(void);
uint8_t batteryCapacityRemainingPercentage(void);
