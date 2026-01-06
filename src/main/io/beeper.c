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

#include "platform.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/dshot_command.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_io.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/runtime_config.h"

#include "io/statusindicator.h"
#include "io/vtx_control.h"

#include "msp/msp_serial.h"

#ifdef USE_GPS
#include "io/gps.h"
#endif

#ifdef USE_OSD
#include "osd/osd.h"
#endif

#include "pg/beeper.h"

#include "scheduler/scheduler.h"

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

#define MAX_MULTI_BEEPS 32   // number of beeps (including pause) for 'beep_multiBeeps[]'

#define BEEPER_COMMAND_REPEAT 0xFE
#define BEEPER_COMMAND_STOP   0xFF

#ifdef USE_DSHOT
static timeUs_t lastDshotBeaconCommandTimeUs;
#endif

#ifdef USE_BEEPER

STATIC_ASSERT(BEEPER_ALL - 1 < sizeof(uint32_t) * 8, "BEEPER_GET_FLAG bits exceeded");

/* Beeper Sound Sequences: (Square wave generation)
 * Sequence must end with 0xFF or 0xFE. 0xFE repeats the sequence from
 * start when 0xFF stops the sound when it's completed.
 *
 * "Sound" Sequences are made so that 1st, 3rd, 5th.. are the delays how
 * long the beeper is on and 2nd, 4th, 6th.. are the delays how long beeper
 * is off. Delays are in milliseconds/10 (i.e., 5 => 50ms).
 *
 * if first value is zero, sequence starts with pause
 *
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

// array used for variable # of beeps (reporting GPS sat count, etc)
static uint8_t beep_multiBeeps[MAX_MULTI_BEEPS * 2 + 1];

#define BEEPER_CONFIRMATION_BEEP_DURATION 2
#define BEEPER_CONFIRMATION_BEEP_GAP_DURATION 20

#define BEEPER_WARNING_LONG_BEEP_MULTIPLIER 5

#define BEEPER_WARNING_BEEP_1_DURATION 20
#define BEEPER_WARNING_BEEP_2_DURATION 5
#define BEEPER_WARNING_BEEP_GAP_DURATION 10

static bool beeperIsOn = false;

// Index in current sequence (currentBeeperEntry->sequence[])
static uint16_t beeperPos = 0;
// Time when beeper routine must act next time (zero when not waiting)
static uint32_t beeperNextToggleTime = 0;
// Timestamp of last arming beep in microseconds (for blackbox)
static uint32_t armingBeepTimeMicros = 0;

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
    { BEEPER_ENTRY(BEEPER_MULTI_BEEPS,           14, beep_multiBeeps,      "MULTI_BEEPS") }, // FIXME This entry must not be called directly.
    { BEEPER_ENTRY(BEEPER_DISARM_REPEAT,         15, beep_disarmRepeatBeep,"DISARM_REPEAT") },
    { BEEPER_ENTRY(BEEPER_ARMED,                 16, beep_armedBeep,       "ARMED") },
    { BEEPER_ENTRY(BEEPER_SYSTEM_INIT,           17, NULL,                 "SYSTEM_INIT") },
    { BEEPER_ENTRY(BEEPER_USB,                   18, NULL,                 "ON_USB") },
    { BEEPER_ENTRY(BEEPER_BLACKBOX_ERASE,        19, beep_2shortBeeps,     "BLACKBOX_ERASE") },
    { BEEPER_ENTRY(BEEPER_CRASHFLIP_MODE,        20, beep_2longerBeeps,    "CRASHFLIP") },
    { BEEPER_ENTRY(BEEPER_CAM_CONNECTION_OPEN,   21, beep_camOpenBeep,     "CAM_CONNECTION_OPEN") },
    { BEEPER_ENTRY(BEEPER_CAM_CONNECTION_CLOSE,  22, beep_camCloseBeep,    "CAM_CONNECTION_CLOSE") },
    { BEEPER_ENTRY(BEEPER_ALL,                   23, NULL,                 "ALL") },
};

static const beeperTableEntry_t *currentBeeperEntry = NULL;

// find entry by mode
static const beeperTableEntry_t *beeperFind(beeperMode_e mode)
{
    for (const beeperTableEntry_t* e = beeperTable; e < ARRAYEND(beeperTable); e++) {
        if (e->mode == mode) {
            return e;
        }
    }
    return NULL;
}

/*
 * Called to activate/deactivate beeper, using the given "BEEPER_..." value.
 * This function returns immediately (does not block).
 * TODO - use led for beeps even for BEEPER_USB / BOXBEEPERMUTE
 */
void beeper(beeperMode_e mode)
{
    if (mode == BEEPER_SILENCE
        || ((beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(BEEPER_USB))
            && getBatteryState() == BATTERY_NOT_PRESENT )
        || IS_RC_MODE_ACTIVE(BOXBEEPERMUTE) ) {
        beeperSilence();
        return;
    }

    const beeperTableEntry_t *candidate = beeperFind(mode);
    if (!candidate || candidate->sequence == NULL) {
        // invalid mode for beeper()
        return;
    }
    if (currentBeeperEntry && candidate->priority >= currentBeeperEntry->priority) {
        // already got better beeper
        // this will also prevent restarting sequence for identical mode
        return;
    }

    currentBeeperEntry = candidate;
    beeperPos = 0;
    beeperNextToggleTime = 0;
}

void beeperSilence(void)
{
    BEEP_OFF;
    beeperIsOn = false;

    warningLedDisable();
    warningLedRefresh();

    currentBeeperEntry = NULL;
    beeperPos = 0;
    beeperNextToggleTime = 0;
}

// helper function, add count beeps starting at pos to beep_multiBeeps
// `off` is used as interval between beeps
// if offLast is nonzero, it is used after sequence
// if offLast is zero, no delay is appended after last beep (and BEEPER_COMMAND_STOP must follow)
// returns new pos
// will always keep enough space for stop-/repeat command
static unsigned beep_multiBeepsAdd(unsigned pos, int count, uint8_t on, uint8_t off, uint8_t offLast)
{
    while (pos < MAX_MULTI_BEEPS * 2 - 1 && count > 0) {   // make sure both entries fit
        beep_multiBeeps[pos++] = on;
        if (count > 1 || offLast) {
            beep_multiBeeps[pos++] = count > 1 ? off : offLast;
        }
        count--;
    }
    return pos;
}

/*
 * Emits the given number of 20ms beeps (with 200ms spacing).
 * This function returns immediately (does not block).
 */
void beeperConfirmationBeeps(uint8_t beepCount)
{
    unsigned i = 0;
    i = beep_multiBeepsAdd(i, beepCount,
                           BEEPER_CONFIRMATION_BEEP_DURATION,
                           BEEPER_CONFIRMATION_BEEP_GAP_DURATION, BEEPER_CONFIRMATION_BEEP_GAP_DURATION);
    beep_multiBeeps[i] = BEEPER_COMMAND_STOP;
    beeper(BEEPER_MULTI_BEEPS);
}

void beeperWarningBeeps(uint8_t beepCount)
{
    uint8_t longBeepCount = beepCount / BEEPER_WARNING_LONG_BEEP_MULTIPLIER;
    uint8_t shortBeepCount = beepCount % BEEPER_WARNING_LONG_BEEP_MULTIPLIER;

    unsigned i = 0;

    i = beep_multiBeepsAdd(i, WARNING_FLASH_COUNT - 1,
                           WARNING_FLASH_DURATION_MS / 10,
                           WARNING_FLASH_DURATION_MS / 10, WARNING_PAUSE_DURATION_MS / 10);
    i = beep_multiBeepsAdd(i, longBeepCount,
                           WARNING_CODE_DURATION_LONG_MS / 10,
                           WARNING_CODE_DURATION_LONG_MS / 10,  WARNING_PAUSE_DURATION_MS / 10);
    i = beep_multiBeepsAdd(i, shortBeepCount,
                           WARNING_CODE_DURATION_SHORT_MS / 10,
                           WARNING_CODE_DURATION_LONG_MS / 10, 0);
    beep_multiBeeps[i] = BEEPER_COMMAND_STOP;

    beeper(BEEPER_MULTI_BEEPS);
}

#ifdef USE_GPS
static void beeperGpsStatus(void)
{
    if (!(beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(BEEPER_GPS_STATUS))) {
        // if GPS 3D fix and at least the minimum number available, then beep out number of satellites
        if (STATE(GPS_FIX) && gpsSol.numSat > GPS_MIN_SAT_COUNT) {
            unsigned i = 0;
            i = beep_multiBeepsAdd(i, gpsSol.numSat,
                                   5,
                                   10, 50);
            beep_multiBeeps[i] = BEEPER_COMMAND_STOP;

            beeper(BEEPER_MULTI_BEEPS);
        }
    }
}
#endif

/*
 * Calculates array position when next to change beeper state is due.
 */
enum beeperState_e {
    BeepOnFirst,
    BeepOn,
    BeepOff,
    BeepDone,
    BeepError,
};

static enum beeperState_e beeperSequenceAdvance(timeUs_t currentTimeUs)
{
    unsigned origPos = beeperPos;
    bool wasRepeat = false;
    while (true) {  // evaluate commands until return statement
        switch (currentBeeperEntry->sequence[beeperPos]) {
        case BEEPER_COMMAND_REPEAT:
            if (wasRepeat) {     // prevent infinite loop on badly formed beep sequence
                return BeepError;
            }
            wasRepeat = true;
            beeperPos = 0;
            continue;
        case BEEPER_COMMAND_STOP:
            return BeepDone;
        case 0:
            beeperPos++;  // skip 0 in data
            continue;
        default:   // Otherwise advance the sequence and calculate next toggle time
            beeperNextToggleTime = currentTimeUs + 1000 * 10 * currentBeeperEntry->sequence[beeperPos];
            beeperPos++;
            // on/off is strictly index-based
            if ((beeperPos % 2) == 1) {
                return origPos ? BeepOn : BeepOnFirst;
            } else {
                return BeepOff;
            }
        }
    }
}

/*
 * Beeper handler function to be called periodically in loop. Updates beeper
 * state via time schedule.
 */
void beeperUpdate(timeUs_t currentTimeUs)
{
    // If beeper option from AUX switch has been selected
    if (IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
        beeper(BEEPER_RX_SET);
    }
#ifdef USE_GPS
    else if (featureIsEnabled(FEATURE_GPS) && IS_RC_MODE_ACTIVE(BOXBEEPGPSCOUNT)) {
        // Note: this may overwrite current beep_multiBeeps sequence
        beeperGpsStatus();
    }
#endif

    // Drive ESC beacons when requested:
    //  - RX link lost while configurator is not active (field retrieval), or
    //  - RX_SET via AUX with an active RX link (user-triggered beacon)
#ifdef USE_DSHOT
    static const timeDelta_t dshotBeaconIntervalUs = DSHOT_BEACON_MODE_INTERVAL_US;

    bool dshotBeaconRequested = false;

    if (!areMotorsRunning()) {
        const beeperMode_e activeMode = currentBeeperEntry ? currentBeeperEntry->mode : BEEPER_SILENCE;

        // Drive the ESC beacon whenever the beeper has entered the RX_LOST sequence.
        if (activeMode == BEEPER_RX_LOST
            && !mspSerialIsConfiguratorActive()
            && !(beeperConfig()->dshotBeaconOffFlags & BEEPER_GET_FLAG(BEEPER_RX_LOST)) ) {
            dshotBeaconRequested = true;
        }

        // Allow user-triggered beacon via AUX switch while the RX link is healthy.
        if (IS_RC_MODE_ACTIVE(BOXBEEPERON)
            && failsafeIsReceivingRxData()
            && !(beeperConfig()->dshotBeaconOffFlags & BEEPER_GET_FLAG(BEEPER_RX_SET)) ) {
            dshotBeaconRequested = true;
        }
    }

    if (dshotBeaconRequested) {
        if (cmpTimeUs(currentTimeUs, getLastDisarmTimeUs()) > DSHOT_BEACON_GUARD_DELAY_US
            && !isTryingToArm()) {
            if (cmpTimeUs(currentTimeUs, lastDshotBeaconCommandTimeUs) > dshotBeaconIntervalUs) {
                // at least 450ms between DShot beacons to allow time for the sound to fully complete
                // the DShot Beacon tone duration is determined by the ESC, and should not exceed 250ms
                lastDshotBeaconCommandTimeUs = currentTimeUs;
                dshotCommandWrite(ALL_MOTORS, getMotorCount(), beeperConfig()->dshotBeaconTone, DSHOT_CMD_TYPE_INLINE);
            }
        }
    }
#endif
    // Note: DShot beacon handling above must run even if no beeper sequence is active.
    // Beeper routine doesn't need to update if there aren't any sounds ongoing
    if (currentBeeperEntry == NULL) {
        schedulerIgnoreTaskExecTime();
        return;
    }

    if (beeperNextToggleTime && cmp32(beeperNextToggleTime, currentTimeUs) > 0) {
        schedulerIgnoreTaskExecTime();
        return;
    }

    bool visualBeep = false;
    switch (beeperSequenceAdvance(currentTimeUs)) {
    case BeepOnFirst:
        // if this is arming beep then mark time (for blackbox)
        // note that ARM sequence must start with beep
        if (BEEPER_GET_FLAG(currentBeeperEntry->mode) & BEEPER_ARMING_MODES) {
            armingBeepTimeMicros = micros();
        }
        FALLTHROUGH;
    case BeepOn:
        if (!(beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(currentBeeperEntry->mode))) {
            BEEP_ON;
            beeperIsOn = true;
            visualBeep = true;
        }
        warningLedEnable();
        warningLedRefresh();
        break;
    case BeepOff:
        BEEP_OFF;
        beeperIsOn = false;
        warningLedDisable();
        warningLedRefresh();
        break;
    case BeepDone:
    case BeepError:
        beeperSilence();
        break;
    }

#if defined(USE_OSD)
    static bool visualWasOn = false;
    if (visualBeep && !visualWasOn) {
        osdSetVisualBeeperState(true);
    }
    visualWasOn = visualBeep;
#else
    UNUSED(visualBeep);
#endif
}

/*
 * Returns the time that the last arming beep occurred (in system-uptime
 * microseconds).  This is fetched and logged by blackbox.
 */
uint32_t getArmingBeepTimeMicros(void)
{
    return armingBeepTimeMicros;
}

static bool beeperModeIndexValid(int idx)
{
    return (idx >= 0 && idx < (int)ARRAYLEN(beeperTable));
}

/*
 * Returns the 'beeperMode_e' value for the given beeper-table index,
 * or BEEPER_SILENCE if none.
 */
beeperMode_e beeperModeForTableIndex(int idx)
{
    return beeperModeIndexValid(idx) ? beeperTable[idx].mode : BEEPER_SILENCE;
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
    return beeperModeIndexValid(idx) ? beeperTable[idx].name : NULL;
}

/*
 * Returns the number of entries in the beeper-sounds table.
 */
int beeperTableEntryCount(void)
{
    return ARRAYLEN(beeperTable);
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
