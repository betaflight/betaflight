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
    BEEPER_USB,                     // Disable beeper when connected to USB
    BEEPER_RX_SET,                  // Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
    BEEPER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
    BEEPER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
    BEEPER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
    BEEPER_READY_BEEP,              // Ring a tone when GPS is locked and ready
    BEEPER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
    BEEPER_ARMED,                   // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
    BEEPER_CASE_MAX
} beeperMode_e;

#define BEEPER_OFF_FLAGS_MIN  0
#define BEEPER_OFF_FLAGS_MAX  ((1 << (BEEPER_CASE_MAX - 1)) - 1)

typedef struct beeperOffConditions_t {
    uint32_t flags;
} beeperOffConditions_t;

void beeper(beeperMode_e mode);
void beeperSilence(void);
void beeperUpdate(void);
void beeperConfirmationBeeps(uint8_t beepCount);
uint32_t getArmingBeepTimeMicros(void);
beeperMode_e beeperModeForTableIndex(int idx);
const char *beeperNameForTableIndex(int idx);
int beeperTableEntryCount(void);

/* CLI  beeper_off_flags  =  sum of each desired beeper turned off case
BEEPER_GYRO_CALIBRATED,			1
BEEPER_RX_LOST_LANDING,			2		// Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
BEEPER_RX_LOST,         		4		// Beeps when TX is turned off or signal lost (repeat until TX is okay)
BEEPER_DISARMING,        		8		// Beep when disarming the board
BEEPER_ARMING,          		16    	// Beep when arming the board
BEEPER_ARMING_GPS_FIX, 			32    	// Beep a special tone when arming the board and GPS has fix
BEEPER_BAT_CRIT_LOW,       		64		// Longer warning beeps when battery is critically low (repeats)
BEEPER_BAT_LOW,            		128     // Warning beeps when battery is getting low (repeats)
BEEPER_USB_DISABLE,				256     // Disable beeper when connected to USB
BEEPER_RX_SET,              	512		// Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
BEEPER_DISARM_REPEAT,       	1024	// Beeps sounded while stick held in disarm position
BEEPER_ACC_CALIBRATION,     	2048	// ACC inflight calibration completed confirmation
BEEPER_ACC_CALIBRATION_FAIL,	4096	// ACC inflight calibration failed
BEEPER_READY_BEEP,          	8192	// Ring a tone when GPS is locked and ready
BEEPER_MULTI_BEEPS,         	16384	// Internal value used by 'beeperConfirmationBeeps()'.
BEEPER_ARMED,               	32768	// Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
*/
