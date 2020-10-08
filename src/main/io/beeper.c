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

#include "stdbool.h"
#include "stdint.h"

#include "platform.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/dshot_command.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "flight/mixer.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/runtime_config.h"

#include "io/statusindicator.h"
#include "io/vtx_control.h"

#ifdef USE_GPS
#include "io/gps.h"
#endif

#include "pg/beeper.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "beeper.h"

#ifdef BEEPER_INVERTED
#define IS_OPEN_DRAIN   false
#define IS_INVERTED     true
#else
#define IS_OPEN_DRAIN   true
#define IS_INVERTED     false
#endif

#ifdef USE_BEEPER
#ifndef BEEPER_PWM_HZ
#define BEEPER_PWM_HZ   0
#endif
#else
#define BEEPER_PIN      NONE
#define BEEPER_PWM_HZ   0
#endif

#define MAX_MULTI_BEEPS 64   //size limit for 'beep_multiBeeps[]'

#define BEEPER_COMMAND_REPEAT 0xFE
#define BEEPER_COMMAND_STOP   0xFF

#ifdef USE_DSHOT
static timeUs_t lastDshotBeaconCommandTimeUs;
#endif

#ifdef USE_BEEPER
/* Beeper Sound Sequences: (Square wave generation)
 * Sequence must end with 0xFF or 0xFE. 0xFE repeats the sequence from
 * start when 0xFF stops the sound when it's completed.
 *
 * "Sound" Sequences are made so that 1st, 3rd, 5th.. are the delays how
 * long the beeper is on and 2nd, 4th, 6th.. are the delays how long beeper
 * is off. Delays are in milliseconds/10 (i.e., 5 => 50ms).
 */
// short fast beep
static const uint8_t beep_shortBeep[] = {
    10, 10, BEEPER_COMMAND_STOP
};
// arming beep
static const uint8_t beep_armingBeep[] = {
    30, 5, 5, 5, BEEPER_COMMAND_STOP
};
// Arming when GPS is fixed
static const uint8_t beep_armingGpsFix[] = {
    5, 5, 15, 5, 5, 5, 15, 30, BEEPER_COMMAND_STOP
};
// Arming when GPS is not fixed
static const uint8_t beep_armingGpsNoFix[] = {
    30, 5, 30, 5, 30, 5, BEEPER_COMMAND_STOP
};
// armed beep (first pause, then short beep)
static const uint8_t beep_armedBeep[] = {
    0, 245, 10, 5, BEEPER_COMMAND_STOP
};
// disarming beeps
static const uint8_t beep_disarmBeep[] = {
    15, 5, 15, 5, BEEPER_COMMAND_STOP
};
// beeps while stick held in disarm position (after pause)
static const uint8_t beep_disarmRepeatBeep[] = {
    0, 100, 10, BEEPER_COMMAND_STOP
};
// Long beep and pause after that
static const uint8_t beep_lowBatteryBeep[] = {
    25, 50, BEEPER_COMMAND_STOP
};
// critical battery beep
static const uint8_t beep_critBatteryBeep[] = {
    50, 2, BEEPER_COMMAND_STOP
};

// transmitter-signal-lost tone
static const uint8_t beep_txLostBeep[] = {
    50, 50, BEEPER_COMMAND_STOP
};
// SOS morse code:
static const uint8_t beep_sos[] = {
    10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70, BEEPER_COMMAND_STOP
};
// Ready beeps. When gps has fix and copter is ready to fly.
static const uint8_t beep_readyBeep[] = {
    4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, BEEPER_COMMAND_STOP
};
// 2 fast short beeps
static const uint8_t beep_2shortBeeps[] = {
    5, 5, 5, 5, BEEPER_COMMAND_STOP
};
// 2 longer beeps
static const uint8_t beep_2longerBeeps[] = {
    20, 15, 35, 5, BEEPER_COMMAND_STOP
};
// 3 beeps
static const uint8_t beep_gyroCalibrated[] = {
    20, 10, 20, 10, 20, 10, BEEPER_COMMAND_STOP
};

// Cam connection opened
static const uint8_t beep_camOpenBeep[] = {
    5, 15, 10, 15, 20, BEEPER_COMMAND_STOP
};

// Cam connection close
static const uint8_t beep_camCloseBeep[] = {
    10, 8, 5, BEEPER_COMMAND_STOP
};

// RC Smoothing filter not initialized - 3 short + 1 long
static const uint8_t beep_rcSmoothingInitFail[] = {
    10, 10, 10, 10, 10, 10, 50, 25, BEEPER_COMMAND_STOP
};

// array used for variable # of beeps (reporting GPS sat count, etc)
static uint8_t beep_multiBeeps[MAX_MULTI_BEEPS + 1];

#define BEEPER_CONFIRMATION_BEEP_DURATION 2
#define BEEPER_CONFIRMATION_BEEP_GAP_DURATION 20

#define BEEPER_WARNING_LONG_BEEP_MULTIPLIER 5

#define BEEPER_WARNING_BEEP_1_DURATION 20
#define BEEPER_WARNING_BEEP_2_DURATION 5
#define BEEPER_WARNING_BEEP_GAP_DURATION 10

static bool beeperIsOn = false;

// Place in current sequence
static uint16_t beeperPos = 0;
// Time when beeper routine must act next time
static uint32_t beeperNextToggleTime = 0;
// Time of last arming beep in microseconds (for blackbox)
static uint32_t armingBeepTimeMicros = 0;

static void beeperProcessCommand(timeUs_t currentTimeUs);

typedef struct beeperTableEntry_s {
    uint8_t mode;
    uint8_t priority; // 0 = Highest
    const uint8_t *sequence;
    const char *name;
} beeperTableEntry_t;

#define BEEPER_ENTRY(a,b,c,d) a,b,c,d

// IMPORTANT: these are in priority order, 0 = Highest
static const beeperTableEntry_t beeperTable[] = {
    { BEEPER_ENTRY(BEEPER_GYRO_CALIBRATED,       0, beep_gyroCalibrated,   "GYRO_CALIBRATED") },
    { BEEPER_ENTRY(BEEPER_RX_LOST,               1, beep_txLostBeep,       "RX_LOST") },
    { BEEPER_ENTRY(BEEPER_RX_LOST_LANDING,       2, beep_sos,              "RX_LOST_LANDING") },
    { BEEPER_ENTRY(BEEPER_DISARMING,             3, beep_disarmBeep,       "DISARMING") },
    { BEEPER_ENTRY(BEEPER_ARMING,                4, beep_armingBeep,       "ARMING")  },
    { BEEPER_ENTRY(BEEPER_ARMING_GPS_FIX,        5, beep_armingGpsFix,     "ARMING_GPS_FIX") },
    { BEEPER_ENTRY(BEEPER_ARMING_GPS_NO_FIX,     6, beep_armingGpsNoFix,   "ARMING_GPS_NO_FIX") },
    { BEEPER_ENTRY(BEEPER_BAT_CRIT_LOW,          7, beep_critBatteryBeep,  "BAT_CRIT_LOW") },
    { BEEPER_ENTRY(BEEPER_BAT_LOW,               8, beep_lowBatteryBeep,   "BAT_LOW") },
    { BEEPER_ENTRY(BEEPER_GPS_STATUS,            9, beep_multiBeeps,       "GPS_STATUS") },
    { BEEPER_ENTRY(BEEPER_RX_SET,                10, beep_shortBeep,       "RX_SET") },
    { BEEPER_ENTRY(BEEPER_ACC_CALIBRATION,       11, beep_2shortBeeps,     "ACC_CALIBRATION") },
    { BEEPER_ENTRY(BEEPER_ACC_CALIBRATION_FAIL,  12, beep_2longerBeeps,    "ACC_CALIBRATION_FAIL") },
    { BEEPER_ENTRY(BEEPER_READY_BEEP,            13, beep_readyBeep,       "READY_BEEP") },
    { BEEPER_ENTRY(BEEPER_MULTI_BEEPS,           14, beep_multiBeeps,      "MULTI_BEEPS") }, // FIXME having this listed makes no sense since the beep array will not be initialised.
    { BEEPER_ENTRY(BEEPER_DISARM_REPEAT,         15, beep_disarmRepeatBeep, "DISARM_REPEAT") },
    { BEEPER_ENTRY(BEEPER_ARMED,                 16, beep_armedBeep,       "ARMED") },
    { BEEPER_ENTRY(BEEPER_SYSTEM_INIT,           17, NULL,                 "SYSTEM_INIT") },
    { BEEPER_ENTRY(BEEPER_USB,                   18, NULL,                 "ON_USB") },
    { BEEPER_ENTRY(BEEPER_BLACKBOX_ERASE,        19, beep_2shortBeeps,     "BLACKBOX_ERASE") },
    { BEEPER_ENTRY(BEEPER_CRASH_FLIP_MODE,       20, beep_2longerBeeps,    "CRASH_FLIP") },
    { BEEPER_ENTRY(BEEPER_CAM_CONNECTION_OPEN,   21, beep_camOpenBeep,     "CAM_CONNECTION_OPEN") },
    { BEEPER_ENTRY(BEEPER_CAM_CONNECTION_CLOSE,  22, beep_camCloseBeep,    "CAM_CONNECTION_CLOSE") },
    { BEEPER_ENTRY(BEEPER_RC_SMOOTHING_INIT_FAIL,23, beep_rcSmoothingInitFail, "RC_SMOOTHING_INIT_FAIL") },
    { BEEPER_ENTRY(BEEPER_ALL,                   24, NULL,                 "ALL") },
};

static const beeperTableEntry_t *currentBeeperEntry = NULL;

#define BEEPER_TABLE_ENTRY_COUNT (sizeof(beeperTable) / sizeof(beeperTableEntry_t))

/*
 * Called to activate/deactivate beeper, using the given "BEEPER_..." value.
 * This function returns immediately (does not block).
 */
void beeper(beeperMode_e mode)
{
    if (
        mode == BEEPER_SILENCE || (
            (beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(BEEPER_USB))
            && getBatteryState() == BATTERY_NOT_PRESENT
        ) || IS_RC_MODE_ACTIVE(BOXBEEPERMUTE)
    ) {
        beeperSilence();
        return;
    }

    const beeperTableEntry_t *selectedCandidate = NULL;
    for (uint32_t i = 0; i < BEEPER_TABLE_ENTRY_COUNT; i++) {
        const beeperTableEntry_t *candidate = &beeperTable[i];
        if (candidate->mode != mode) {
            continue;
        }

        if (!currentBeeperEntry) {
            selectedCandidate = candidate;
            break;
        }

        if (candidate->priority < currentBeeperEntry->priority) {
            selectedCandidate = candidate;
        }

        break;
    }

    if (!selectedCandidate) {
        return;
    }

    currentBeeperEntry = selectedCandidate;

    beeperPos = 0;
    beeperNextToggleTime = 0;
}

void beeperSilence(void)
{
    BEEP_OFF;
    beeperIsOn = false;

    warningLedDisable();
    warningLedRefresh();

    beeperNextToggleTime = 0;
    beeperPos = 0;

    currentBeeperEntry = NULL;
}

/*
 * Emits the given number of 20ms beeps (with 200ms spacing).
 * This function returns immediately (does not block).
 */
void beeperConfirmationBeeps(uint8_t beepCount)
{
    uint32_t i = 0;
    uint32_t cLimit = beepCount * 2;
    if (cLimit > MAX_MULTI_BEEPS) {
        cLimit = MAX_MULTI_BEEPS;
    }
    do {
        beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_DURATION;
        beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_GAP_DURATION;
    } while (i < cLimit);
    beep_multiBeeps[i] = BEEPER_COMMAND_STOP;

    beeper(BEEPER_MULTI_BEEPS);
}

void beeperWarningBeeps(uint8_t beepCount)
{
    uint8_t longBeepCount = beepCount / BEEPER_WARNING_LONG_BEEP_MULTIPLIER;
    uint8_t shortBeepCount = beepCount % BEEPER_WARNING_LONG_BEEP_MULTIPLIER;

    unsigned i = 0;

    unsigned count = 0;
    while (i < MAX_MULTI_BEEPS - 1 && count < WARNING_FLASH_COUNT) {
        beep_multiBeeps[i++] = WARNING_FLASH_DURATION_MS / 10;
        if (++count < WARNING_FLASH_COUNT) {
            beep_multiBeeps[i++] = WARNING_FLASH_DURATION_MS / 10;
        } else {
            beep_multiBeeps[i++] = WARNING_PAUSE_DURATION_MS / 10;
        }
    }

    while (i < MAX_MULTI_BEEPS - 1 && longBeepCount > 0) {
        beep_multiBeeps[i++] = WARNING_CODE_DURATION_LONG_MS / 10;
        if (--longBeepCount > 0) {
            beep_multiBeeps[i++] = WARNING_CODE_DURATION_LONG_MS / 10;
        } else {
            beep_multiBeeps[i++] = WARNING_PAUSE_DURATION_MS / 10;
        }
    }

    while (i < MAX_MULTI_BEEPS - 1 && shortBeepCount > 0) {
        beep_multiBeeps[i++] = WARNING_CODE_DURATION_SHORT_MS / 10;
        if (--shortBeepCount > 0) {
            beep_multiBeeps[i++] = WARNING_CODE_DURATION_LONG_MS / 10;
        }
    }

    beep_multiBeeps[i] = BEEPER_COMMAND_STOP;

    beeper(BEEPER_MULTI_BEEPS);
}

#ifdef USE_GPS
static void beeperGpsStatus(void)
{
    if (!(beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(BEEPER_GPS_STATUS))) {
        // if GPS fix then beep out number of satellites
        if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
            uint8_t i = 0;
            do {
                beep_multiBeeps[i++] = 5;
                beep_multiBeeps[i++] = 10;
            } while (i < MAX_MULTI_BEEPS && gpsSol.numSat > i / 2);

            beep_multiBeeps[i - 1] = 50; // extend last pause
            beep_multiBeeps[i] = BEEPER_COMMAND_STOP;

            beeper(BEEPER_MULTI_BEEPS);    //initiate sequence
        }
    }
}
#endif

/*
 * Beeper handler function to be called periodically in loop. Updates beeper
 * state via time schedule.
 */
void beeperUpdate(timeUs_t currentTimeUs)
{
    // If beeper option from AUX switch has been selected
    if (IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
        beeper(BEEPER_RX_SET);
#ifdef USE_GPS
    } else if (featureIsEnabled(FEATURE_GPS) && IS_RC_MODE_ACTIVE(BOXBEEPGPSCOUNT)) {
        beeperGpsStatus();
#endif
    }

    // Beeper routine doesn't need to update if there aren't any sounds ongoing
    if (currentBeeperEntry == NULL) {
        return;
    }

    if (beeperNextToggleTime > currentTimeUs) {
        return;
    }

    if (!beeperIsOn) {
#ifdef USE_DSHOT
        if (!areMotorsRunning()
            && ((currentBeeperEntry->mode == BEEPER_RX_SET && !(beeperConfig()->dshotBeaconOffFlags & BEEPER_GET_FLAG(BEEPER_RX_SET)))
            || (currentBeeperEntry->mode == BEEPER_RX_LOST && !(beeperConfig()->dshotBeaconOffFlags & BEEPER_GET_FLAG(BEEPER_RX_LOST))))) {

            if ((currentTimeUs - getLastDisarmTimeUs() > DSHOT_BEACON_GUARD_DELAY_US) && !isTryingToArm()) {
                lastDshotBeaconCommandTimeUs = currentTimeUs;
                dshotCommandWrite(ALL_MOTORS, getMotorCount(), beeperConfig()->dshotBeaconTone, DSHOT_CMD_TYPE_INLINE);
            }
        }
#endif

        if (currentBeeperEntry->sequence[beeperPos] != 0) {
            if (!(beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(currentBeeperEntry->mode))) {
                BEEP_ON;
                beeperIsOn = true;
            }

            warningLedEnable();
            warningLedRefresh();
            // if this was arming beep then mark time (for blackbox)
            if (
                beeperPos == 0
                && (currentBeeperEntry->mode == BEEPER_ARMING || currentBeeperEntry->mode == BEEPER_ARMING_GPS_FIX
                || currentBeeperEntry->mode == BEEPER_ARMING_GPS_NO_FIX)) {
                armingBeepTimeMicros = micros();
            }
        }
    } else {
        if (currentBeeperEntry->sequence[beeperPos] != 0) {
            BEEP_OFF;
            beeperIsOn = false;

            warningLedDisable();
            warningLedRefresh();
        }
    }

    beeperProcessCommand(currentTimeUs);
}

/*
 * Calculates array position when next to change beeper state is due.
 */
static void beeperProcessCommand(timeUs_t currentTimeUs)
{
    if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_REPEAT) {
        beeperPos = 0;
    } else if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_STOP) {
        beeperSilence();
    } else {
        // Otherwise advance the sequence and calculate next toggle time
        beeperNextToggleTime = currentTimeUs + 1000 * 10 * currentBeeperEntry->sequence[beeperPos];
        beeperPos++;
    }
}

/*
 * Returns the time that the last arming beep occurred (in system-uptime
 * microseconds).  This is fetched and logged by blackbox.
 */
uint32_t getArmingBeepTimeMicros(void)
{
    return armingBeepTimeMicros;
}

/*
 * Returns the 'beeperMode_e' value for the given beeper-table index,
 * or BEEPER_SILENCE if none.
 */
beeperMode_e beeperModeForTableIndex(int idx)
{
    return (idx >= 0 && idx < (int)BEEPER_TABLE_ENTRY_COUNT) ? beeperTable[idx].mode : BEEPER_SILENCE;
}

/*
 * Returns the binary mask for the 'beeperMode_e' value corresponding to a given
 * beeper-table index, or 0 if the beeperMode is BEEPER_SILENCE.
 */
uint32_t beeperModeMaskForTableIndex(int idx)
{
    beeperMode_e beeperMode = beeperModeForTableIndex(idx);
    if (beeperMode == BEEPER_SILENCE)
        return 0;
    return BEEPER_GET_FLAG(beeperMode);
}

/*
 * Returns the name for the given beeper-table index, or NULL if none.
 */
const char *beeperNameForTableIndex(int idx)
{
    return (idx >= 0 && idx < (int)BEEPER_TABLE_ENTRY_COUNT) ? beeperTable[idx].name : NULL;
}

/*
 * Returns the number of entries in the beeper-sounds table.
 */
int beeperTableEntryCount(void)
{
    return (int)BEEPER_TABLE_ENTRY_COUNT;
}

/*
 * Returns true if the beeper is on, false otherwise
 */
bool isBeeperOn(void)
{
    return beeperIsOn;
}

#else

// Stub out beeper functions if #BEEPER not defined
void beeper(beeperMode_e mode) {UNUSED(mode);}
void beeperSilence(void) {}
void beeperConfirmationBeeps(uint8_t beepCount) {UNUSED(beepCount);}
void beeperWarningBeeps(uint8_t beepCount) {UNUSED(beepCount);}
void beeperUpdate(timeUs_t currentTimeUs) {UNUSED(currentTimeUs);}
uint32_t getArmingBeepTimeMicros(void) {return 0;}
beeperMode_e beeperModeForTableIndex(int idx) {UNUSED(idx); return BEEPER_SILENCE;}
uint32_t beeperModeMaskForTableIndex(int idx) {UNUSED(idx); return 0;}
const char *beeperNameForTableIndex(int idx) {UNUSED(idx); return NULL;}
int beeperTableEntryCount(void) {return 0;}
bool isBeeperOn(void) {return false;}

#endif

#ifdef USE_DSHOT
timeUs_t getLastDshotBeaconCommandTimeUs(void)
{
    return lastDshotBeaconCommandTimeUs;
}
#endif
