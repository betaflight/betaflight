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
#include "stdlib.h"

#include "platform.h"

#include "drivers/gpio.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "flight/failsafe.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "rx/rx.h"
#include "io/rc_controls.h"

#ifdef GPS
#include "io/gps.h"
#endif

#include "config/runtime_config.h"
#include "config/config.h"

#include "io/beeper.h"

#define MAX_MULTI_BEEPS 20   //size limit for 'beep_multiBeeps[]'

#define BEEPER_COMMAND_REPEAT 0xFE
#define BEEPER_COMMAND_STOP   0xFF

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
    0, 35, 40, 5, BEEPER_COMMAND_STOP
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
// Arming when GPS is fixed
static const uint8_t beep_armedGpsFix[] = {
    5, 5, 15, 5, 5, 5, 15, 30, BEEPER_COMMAND_STOP
};
// Ready beeps. When gps has fix and copter is ready to fly.
static const uint8_t beep_readyBeep[] = {
    4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, BEEPER_COMMAND_STOP
};
// 2 fast short beeps
static const uint8_t beep_2shortBeeps[] = {
    5, 5, 5, 5, BEEPER_COMMAND_STOP
};
// 3 fast short beeps
static const uint8_t beep_3shortBeeps[] = {
    5, 5, 5, 5, 5, 5, BEEPER_COMMAND_STOP
};
// array used for variable # of beeps (reporting GPS sat count, etc)
static uint8_t beep_multiBeeps[MAX_MULTI_BEEPS + 2];

#define BEEPER_CONFIRMATION_BEEP_DURATION 2
#define BEEPER_CONFIRMATION_BEEP_GAP_DURATION 20


// Beeper off = 0 Beeper on = 1
static uint8_t beeperIsOn = 0;

// Place in current sequence
static uint16_t beeperPos = 0;
// Time when beeper routine must act next time
static uint32_t beeperNextToggleTime = 0;
// Time of last arming beep in microseconds (for blackbox)
static uint32_t armingBeepTimeMicros = 0;

static void beeperProcessCommand(void);

typedef struct beeperTableEntry_s {
    uint8_t mode;
    uint8_t priority; // 0 = Highest
    const uint8_t *sequence;
} beeperTableEntry_t;

static const beeperTableEntry_t const beeperTable[] = {
    { BEEPER_RX_LOST_LANDING,       0, beep_sos },
    { BEEPER_RX_LOST,               1, beep_txLostBeep },
    { BEEPER_DISARMING,             2, beep_disarmBeep },
    { BEEPER_ARMING,                3, beep_armingBeep },
    { BEEPER_ARMING_GPS_FIX,        4, beep_armedGpsFix },
    { BEEPER_BAT_CRIT_LOW,          5, beep_critBatteryBeep },
    { BEEPER_BAT_LOW,               6, beep_lowBatteryBeep },
    { BEEPER_GPS_STATUS,            7, beep_multiBeeps },
    { BEEPER_RX_SET,                8, beep_shortBeep },
    { BEEPER_DISARM_REPEAT,         9, beep_disarmRepeatBeep },
    { BEEPER_ACC_CALIBRATION,       10, beep_2shortBeeps },
    { BEEPER_ACC_CALIBRATION_FAIL,  11, beep_3shortBeeps },
    { BEEPER_READY_BEEP,            12, beep_readyBeep },
    { BEEPER_MULTI_BEEPS,           13, beep_multiBeeps }, // FIXME having this listed makes no sense since the beep array will not be initialised.
    { BEEPER_ARMED,                 14, beep_armedBeep },
};

static const beeperTableEntry_t *currentBeeperEntry = NULL;

#define BEEPER_TABLE_ENTRY_COUNT (sizeof(beeperTable) / sizeof(beeperTableEntry_t))

/*
 * Called to activate/deactivate beeper, using the given "BEEPER_..." value.
 * This function returns immediately (does not block).
 */
void beeper(beeperMode_e mode)
{
    if (mode == BEEPER_SILENCE) {
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
    beeperIsOn = 0;

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
    int i;
    int cLimit;

    i = 0;
    cLimit = beepCount * 2;
    if(cLimit > MAX_MULTI_BEEPS)
        cLimit = MAX_MULTI_BEEPS;  //stay within array size
    do {
        beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_DURATION;       // 20ms beep
        beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_GAP_DURATION;   // 200ms pause
    } while (i < cLimit);
    beep_multiBeeps[i] = BEEPER_COMMAND_STOP;     //sequence end
    beeper(BEEPER_MULTI_BEEPS);    //initiate sequence
}

#ifdef GPS
void beeperGpsStatus(void)
{
    // if GPS fix then beep out number of satellites
    if (STATE(GPS_FIX) && GPS_numSat >= 5) {
        uint8_t i = 0;
        do {
            beep_multiBeeps[i++] = 5;
            beep_multiBeeps[i++] = 10;
        } while (i < MAX_MULTI_BEEPS && GPS_numSat > i / 2);

        beep_multiBeeps[i-1] = 50; // extend last pause
        beep_multiBeeps[i] = BEEPER_COMMAND_STOP;

        beeper(BEEPER_MULTI_BEEPS);    //initiate sequence
    } else {
        beeper(BEEPER_RX_SET);
    }
}
#endif

/*
 * Beeper handler function to be called periodically in loop. Updates beeper
 * state via time schedule.
 */
void beeperUpdate(void)
{
    // If beeper option from AUX switch has been selected
    if (IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
#ifdef GPS
        if (feature(FEATURE_GPS)) {
            beeperGpsStatus();
        } else {
            beeper(BEEPER_RX_SET);
        }
#else
        beeper(BEEPER_RX_SET);
#endif
    }

    // Beeper routine doesn't need to update if there aren't any sounds ongoing
    if (currentBeeperEntry == NULL) {
        return;
    }

    uint32_t now = millis();
    if (beeperNextToggleTime > now) {
        return;
    }

    if (!beeperIsOn) {
        beeperIsOn = 1;
        if (currentBeeperEntry->sequence[beeperPos] != 0) {
            BEEP_ON;
                   //if this was arming beep then mark time (for blackbox)
            if (
                beeperPos == 0
                && (currentBeeperEntry->mode == BEEPER_ARMING || currentBeeperEntry->mode == BEEPER_ARMING_GPS_FIX)
            ) {
                armingBeepTimeMicros = micros();
            }
        }
    } else {
        beeperIsOn = 0;
        if (currentBeeperEntry->sequence[beeperPos] != 0) {
            BEEP_OFF;
        }
    }

    beeperProcessCommand();
}

/*
 * Calculates array position when next to change beeper state is due.
 */
static void beeperProcessCommand(void)
{
    if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_REPEAT) {
        beeperPos = 0;
    } else if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_STOP) {
        beeperSilence();
    } else {
        // Otherwise advance the sequence and calculate next toggle time
        beeperNextToggleTime = millis() + 10 * currentBeeperEntry->sequence[beeperPos];
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
