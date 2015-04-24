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
// single confirmation beep
static const uint8_t beep_confirmBeep[] = {
    2, 20, BEEPER_COMMAND_STOP
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
static uint8_t beep_multiBeeps[MAX_MULTI_BEEPS+2];

// Current Beeper mode
static uint8_t beeperMode = BEEPER_STOPPED;
// Beeper off = 0 Beeper on = 1
static uint8_t beeperIsOn = 0;
// Pointer to current sequence
static const uint8_t *beeperPtr = NULL;
// Place in current sequence
static uint16_t beeperPos = 0;
// Time when beeper routine must act next time
static uint32_t beeperNextToggleTime = 0;
// Time of last arming beep in microseconds (for blackbox)
static uint32_t armingBeepTimeMicros = 0;

static void beeperCalculations(void);

/*
 * Called to activate/deactivate beeper, using the given "BEEPER_..." value.
 * This function returns immediately (does not block).
 */
void beeper(beeperMode_e mode)
{
    // Just return if same or higher priority sound is active.
    if (beeperMode <= mode)
        return;

    switch (mode) {
        case BEEPER_READY_BEEP:
            beeperPtr = beep_readyBeep;
            break;
        case BEEPER_ARMING:
            beeperPtr = beep_armingBeep;
            break;
        case BEEPER_DISARMING:
            beeperPtr = beep_disarmBeep;
            break;
        case BEEPER_DISARM_REPEAT:
            beeperPtr = beep_disarmRepeatBeep;
            break;
        case BEEPER_ACC_CALIBRATION:
            beeperPtr = beep_2shortBeeps;
            break;
        case BEEPER_ACC_CALIBRATION_FAIL:
            beeperPtr = beep_3shortBeeps;
            break;
        case BEEPER_RX_LOST_LANDING:
            beeperPtr = beep_sos;
            break;
        case BEEPER_RX_LOST:
            beeperPtr = beep_txLostBeep;
            break;
        case BEEPER_BAT_LOW:
            beeperPtr = beep_lowBatteryBeep;
            break;
        case BEEPER_BAT_CRIT_LOW:
            beeperPtr = beep_critBatteryBeep;
            break;
        case BEEPER_ARMED:
            beeperPtr = beep_armedBeep;
            break;
        case BEEPER_ARMING_GPS_FIX:
            beeperPtr = beep_armedGpsFix;
            break;
        case BEEPER_CONFIRM_BEEP:
            beeperPtr = beep_confirmBeep;
            break;
        case BEEPER_MULTI_BEEPS:
            beeperPtr = beep_multiBeeps;
            break;
        case BEEPER_RX_SET:
#ifdef GPS
            // if GPS fix then beep out number of satellites
            if (feature(FEATURE_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5) {
                uint8_t i = 0;
                do {
                    beep_multiBeeps[i++] = 5;
                    beep_multiBeeps[i++] = 10;
                } while (i < MAX_MULTI_BEEPS && GPS_numSat > i / 2);
                beep_multiBeeps[i-1] = 50;    // extend last pause
                beep_multiBeeps[i] = BEEPER_COMMAND_STOP;
                beeperPtr = beep_multiBeeps;
                break;
            }
#endif
            beeperPtr = beep_shortBeep;
            break;

        default:
            return;
    }
    beeperPos = 0;
    beeperMode = mode;
    beeperNextToggleTime = 0;
}

void beeperSilence(void)
{
    BEEP_OFF;
    beeperIsOn = 0;

    beeperMode = BEEPER_STOPPED;
    beeperNextToggleTime = 0;
    beeperPtr = NULL;
    beeperPos = 0;
}
/*
 * Emits the given number of 20ms beeps (with 200ms spacing).
 * This function returns immediately (does not block).
 */
void queueConfirmationBeep(uint8_t beepCount)
{
    int i;
    int cLimit;

    if(beepCount <= 1)                 //if single beep then
        beeper(BEEPER_CONFIRM_BEEP);   //use dedicated array
    else {
        i = 0;
        cLimit = beepCount * 2;
        if(cLimit > MAX_MULTI_BEEPS)
            cLimit = MAX_MULTI_BEEPS;  //stay within array size
        do {
            beep_multiBeeps[i++] = 2;       //20ms beep
            beep_multiBeeps[i++] = 20;      //200ms pause
        } while (i < cLimit);
        beep_multiBeeps[i] = BEEPER_COMMAND_STOP;     //sequence end
        beeper(BEEPER_MULTI_BEEPS);    //initiate sequence
    }
}

/*
 * Beeper handler function to be called periodically in loop. Updates beeper
 * state via time schedule.
 */
void beeperUpdate(void)
{
    // If beeper option from AUX switch has been selected
    if (IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
        if (beeperMode > BEEPER_RX_SET)
            beeper(BEEPER_RX_SET);
    }

    // Beeper routine doesn't need to update if there aren't any sounds ongoing
    if (beeperMode == BEEPER_STOPPED || beeperPtr == NULL) {
        return;
    }

    uint32_t now = millis();
    if (beeperNextToggleTime > now) {
        return;
    }

    if (!beeperIsOn) {
        beeperIsOn = 1;
        if (beeperPtr[beeperPos] != 0) {
            BEEP_ON;
                   //if this was arming beep then mark time (for blackbox)
            if (beeperPos == 0 && (beeperMode == BEEPER_ARMING ||
                                   beeperMode == BEEPER_ARMING_GPS_FIX)) {
                armingBeepTimeMicros = micros();
            }
        }
    } else {
        beeperIsOn = 0;
        if (beeperPtr[beeperPos] != 0) {
            BEEP_OFF;
        }
    }

    beeperCalculations();
}

/*
 * Calculates array position when next to change beeper state is due.
 */
void beeperCalculations(void)
{
    if (beeperPtr[beeperPos] == BEEPER_COMMAND_REPEAT) {
        beeperPos = 0;
    } else if (beeperPtr[beeperPos] == BEEPER_COMMAND_STOP) {
        beeperMode = BEEPER_STOPPED;
        BEEP_OFF;
        beeperIsOn = 0;
    } else {
        // Otherwise advance the sequence and calculate next toggle time
        beeperNextToggleTime = millis() + 10 * beeperPtr[beeperPos];
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
