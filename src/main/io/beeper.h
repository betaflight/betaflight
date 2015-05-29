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

typedef enum {
    // IMPORTANT: these are in priority order, 0 = Highest
    BEEPER_SILENCE = 0,             // Silence, see beeperSilence()

    BEEPER_GYRO_CALIBRATED,
    BEEPER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
    BEEPER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
    BEEPER_DISARMING,               // Beep when disarming the board
    BEEPER_ARMING,                  // Beep when arming the board
    BEEPER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
    BEEPER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
    BEEPER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
    BEEPER_GPS_STATUS,
    BEEPER_RX_SET,                  // Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
    BEEPER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
    BEEPER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
    BEEPER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
    BEEPER_READY_BEEP,              // Ring a tone when GPS is locked and ready
    BEEPER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
    BEEPER_ARMED,                   // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
} beeperMode_e;

void beeper(beeperMode_e mode);
void beeperSilence(void);
void beeperUpdate(void);
void beeperConfirmationBeeps(uint8_t beepCount);
uint32_t getArmingBeepTimeMicros(void);
beeperMode_e beeperModeForTableIndex(int idx);
const char *beeperNameForTableIndex(int idx);
int beeperTableEntryCount(void);
