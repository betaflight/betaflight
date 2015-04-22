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

#include "platform.h"

#include "drivers/gpio.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "flight/failsafe.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "rx/rx.h"
#include "io/rc_controls.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "io/beeper.h"

#define LONG_PAUSE_DURATION_MILLIS 200
#define DOUBLE_PAUSE_DURATION_MILLIS (LONG_PAUSE_DURATION_MILLIS * 2)
#define SHORT_PAUSE_DURATION_MILLIS (LONG_PAUSE_DURATION_MILLIS / 4)
#define MEDIUM_PAUSE_DURATION_MILLIS (LONG_PAUSE_DURATION_MILLIS / 2)
#define SHORT_CONFIRMATION_BEEP_DURATION_MILLIS (SHORT_PAUSE_DURATION_MILLIS / 2)

static uint8_t beeperIsOn = 0, beepDone = 0;
static uint32_t beeperLastToggleTime;
static void beep(uint16_t pulseMillis);
static void beep_code(char first, char second, char third, char pause);

static uint8_t toggleBeep = 0;

typedef enum {
    FAILSAFE_WARNING_NONE = 0,
    FAILSAFE_WARNING_LANDING,
    FAILSAFE_WARNING_FIND_ME
} failsafeBeeperWarnings_e;

void beepcodeInit(void)
{
}

void beepcodeUpdateState(batteryState_e batteryState)
{
    static uint8_t beeperOnBox;
#ifdef GPS
    static uint8_t warn_noGPSfix = 0;
#endif
    static failsafeBeeperWarnings_e warn_failsafe = FAILSAFE_WARNING_NONE;

    //=====================  BeeperOn via rcOptions =====================
    if (IS_RC_MODE_ACTIVE(BOXBEEPERON)) {       // unconditional beeper on via AUXn switch
        beeperOnBox = 1;
    } else {
        beeperOnBox = 0;
    }
    //===================== Beeps for failsafe =====================
    if (feature(FEATURE_FAILSAFE)) {
        switch (failsafePhase()) {
            case FAILSAFE_LANDING:
                warn_failsafe = FAILSAFE_WARNING_LANDING;
                break;
            case FAILSAFE_LANDED:
                warn_failsafe = FAILSAFE_WARNING_FIND_ME;
                break;
            default:
                warn_failsafe = FAILSAFE_WARNING_NONE;
        }

        if (rxIsReceivingSignal()) {
            warn_failsafe = FAILSAFE_WARNING_NONE;
        }
    }

#ifdef GPS
    //===================== GPS fix notification handling =====================
    if (sensors(SENSOR_GPS)) {
        if ((IS_RC_MODE_ACTIVE(BOXGPSHOME) || IS_RC_MODE_ACTIVE(BOXGPSHOLD)) && !STATE(GPS_FIX)) {     // if no fix and gps funtion is activated: do warning beeps
            warn_noGPSfix = 1;
        } else {
            warn_noGPSfix = 0;
        }
    }
#endif

    //===================== Priority driven Handling =====================
    // beepcode(length1,length2,length3,pause)
    // D: Double, L: Long, M: Middle, S: Short, N: None
    if (warn_failsafe == 2)
        beep_code('L','N','N','D');                 // failsafe "find me" signal
    else if (warn_failsafe == 1)
        beep_code('S','M','L','M');                 // failsafe landing active
#ifdef GPS
    else if (warn_noGPSfix == 1)
        beep_code('S','S','N','M');
#endif
    else if (beeperOnBox == 1)
        beep_code('S','S','S','M');                 // beeperon
    else if (batteryState == BATTERY_CRITICAL)
        beep_code('S','S','M','D');
    else if (batteryState == BATTERY_WARNING)
        beep_code('S','M','M','D');
    else if (FLIGHT_MODE(AUTOTUNE_MODE))
        beep_code('S','M','S','M');
    else if (toggleBeep > 0)
        beep(SHORT_CONFIRMATION_BEEP_DURATION_MILLIS);  // fast confirmation beep
    else {
        beeperIsOn = 0;
        BEEP_OFF;
    }
}

// duration is specified in multiples of SHORT_CONFIRMATION_BEEP_DURATION_MILLIS
void queueConfirmationBeep(uint8_t duration) {
    toggleBeep = duration;
}

void beep_code(char first, char second, char third, char pause)
{
    char patternChar[4];
    uint16_t Duration;
    static uint8_t icnt = 0;

    patternChar[0] = first;
    patternChar[1] = second;
    patternChar[2] = third;
    patternChar[3] = pause;
    switch (patternChar[icnt]) {
        case 'N':
            Duration = 0;
            break;
        case 'S':
            Duration = LONG_PAUSE_DURATION_MILLIS / 4;
            break;
        case 'M':
            Duration = LONG_PAUSE_DURATION_MILLIS / 2;
            break;
        case 'D':
            Duration = LONG_PAUSE_DURATION_MILLIS * 2;
            break;
        case 'L':
        default:
            Duration = LONG_PAUSE_DURATION_MILLIS;
            break;
    }

    if (icnt < 3 && Duration != 0)
        beep(Duration);
    if (icnt >= 3 && (beeperLastToggleTime < millis() - Duration)) {
        icnt = 0;
        toggleBeep = 0;
    }
    if (beepDone == 1 || Duration == 0) {
        if (icnt < 3)
            icnt++;
        beepDone = 0;
        beeperIsOn = 0;
        BEEP_OFF;
    }
}

static void beep(uint16_t pulseMillis)
{
    if (beeperIsOn) {
        if (millis() >= beeperLastToggleTime + pulseMillis) {
            beeperIsOn = 0;
            BEEP_OFF;
            beeperLastToggleTime = millis();
            if (toggleBeep >0)
                toggleBeep--;
            beepDone = 1;
        }
        return;
    }

    if (millis() >= (beeperLastToggleTime + LONG_PAUSE_DURATION_MILLIS)) {         // Beeper is off and long pause time is up -> turn it on
        beeperIsOn = 1;
        BEEP_ON;
        beeperLastToggleTime = millis();      // save the time the buzer turned on
    }
}
