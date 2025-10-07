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

#include "common/time.h"
#include "common/utils.h"

#define BEEPER_GET_FLAG(mode) (1U << ((mode) - 1))

#ifdef USE_DSHOT
#define DSHOT_BEACON_GUARD_DELAY_US 1200000  // Time to separate DShot beacon and arming/disarming events
                                             // to prevent interference with motor direction commands
#define DSHOT_BEACON_MODE_INTERVAL_US     450000  // at least 450ms between successive DShot beacon iterations to allow time for ESC to play tone
#endif

typedef enum {
    // IMPORTANT: the order of the elements should be preserved for backwards compatibility with the configurator.
    BEEPER_SILENCE = 0,             // Silence, see beeperSilence()

    BEEPER_GYRO_CALIBRATED,
    BEEPER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
    BEEPER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
    BEEPER_DISARMING,               // Beep when disarming the board
    BEEPER_ARMING,                  // Beep when arming the board
    BEEPER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
    BEEPER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
    BEEPER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
    BEEPER_GPS_STATUS,              // Use the number of beeps to indicate how many GPS satellites were found
    BEEPER_RX_SET,                  // Beeps when aux channel is set for beep
    BEEPER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
    BEEPER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
    BEEPER_READY_BEEP,              // Ring a tone when GPS is locked and ready
    BEEPER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
    BEEPER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
    BEEPER_ARMED,                   // Warning beeps when board is armed with motors off when idle (repeats until board is disarmed or throttle is increased)
    BEEPER_SYSTEM_INIT,             // Initialisation beeps when board is powered on
    BEEPER_USB,                     // Some boards have beeper powered USB connected
    BEEPER_BLACKBOX_ERASE,          // Beep when blackbox erase completes
    BEEPER_CRASHFLIP_MODE,          // Crashflip mode is active
    BEEPER_CAM_CONNECTION_OPEN,     // When the 5 key simulation stated
    BEEPER_CAM_CONNECTION_CLOSE,    // When the 5 key simulation stop
    BEEPER_ARMING_GPS_NO_FIX,       // Beep a special tone when arming the board and GPS has no fix
    BEEPER_ALL,                     // Turn ON or OFF all beeper conditions
    // BEEPER_ALL must remain at the bottom of this enum
} beeperMode_e;

STATIC_ASSERT(BEEPER_ALL < sizeof(uint32_t) * 8, "BEEPER bits exhausted");

#define BEEPER_ALLOWED_MODES ( \
    BEEPER_GET_FLAG(BEEPER_GYRO_CALIBRATED) \
    | BEEPER_GET_FLAG(BEEPER_RX_LOST) \
    | BEEPER_GET_FLAG(BEEPER_RX_LOST_LANDING) \
    | BEEPER_GET_FLAG(BEEPER_DISARMING) \
    | BEEPER_GET_FLAG(BEEPER_ARMING) \
    | BEEPER_GET_FLAG(BEEPER_ARMING_GPS_FIX) \
    | BEEPER_GET_FLAG(BEEPER_BAT_CRIT_LOW) \
    | BEEPER_GET_FLAG(BEEPER_BAT_LOW) \
    | BEEPER_GET_FLAG(BEEPER_GPS_STATUS) \
    | BEEPER_GET_FLAG(BEEPER_RX_SET) \
    | BEEPER_GET_FLAG(BEEPER_ACC_CALIBRATION) \
    | BEEPER_GET_FLAG(BEEPER_ACC_CALIBRATION_FAIL) \
    | BEEPER_GET_FLAG(BEEPER_READY_BEEP) \
    | BEEPER_GET_FLAG(BEEPER_MULTI_BEEPS) \
    | BEEPER_GET_FLAG(BEEPER_DISARM_REPEAT) \
    | BEEPER_GET_FLAG(BEEPER_ARMED) \
    | BEEPER_GET_FLAG(BEEPER_SYSTEM_INIT) \
    | BEEPER_GET_FLAG(BEEPER_USB) \
    | BEEPER_GET_FLAG(BEEPER_BLACKBOX_ERASE) \
    | BEEPER_GET_FLAG(BEEPER_CRASHFLIP_MODE) \
    | BEEPER_GET_FLAG(BEEPER_CAM_CONNECTION_OPEN) \
    | BEEPER_GET_FLAG(BEEPER_CAM_CONNECTION_CLOSE) \
    | BEEPER_GET_FLAG(BEEPER_ARMING_GPS_NO_FIX) \
    )

#define DSHOT_BEACON_ALLOWED_MODES ( \
    BEEPER_GET_FLAG(BEEPER_RX_SET) \
    | BEEPER_GET_FLAG(BEEPER_RX_LOST) \
    )

// record these modes as arming beep (for DShot)
#define BEEPER_ARMING_MODES (                   \
    BEEPER_GET_FLAG(BEEPER_ARMING)              \
    | BEEPER_GET_FLAG(BEEPER_ARMING_GPS_FIX)    \
    | BEEPER_GET_FLAG(BEEPER_ARMING_GPS_NO_FIX) \
    )
#ifdef USE_RACE_PRO
#define DEFAULT_DSHOT_BEACON_OFF_FLAGS BEEPER_RX_LOST
#else
#define DEFAULT_DSHOT_BEACON_OFF_FLAGS DSHOT_BEACON_ALLOWED_MODES
#endif // USE_RACE_PRO

void beeper(beeperMode_e mode);
void beeperSilence(void);
void beeperUpdate(timeUs_t currentTimeUs);
void beeperConfirmationBeeps(uint8_t beepCount);
void beeperWarningBeeps(uint8_t beepCount);
uint32_t getArmingBeepTimeMicros(void);
beeperMode_e beeperModeForTableIndex(int idx);
uint32_t beeperModeMaskForTableIndex(int idx);
const char *beeperNameForTableIndex(int idx);
int beeperTableEntryCount(void);
bool isBeeperOn(void);
timeUs_t getLastDshotBeaconCommandTimeUs(void);
