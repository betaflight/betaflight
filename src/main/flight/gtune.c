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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef GTUNE

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "config/config.h"
#include "blackbox/blackbox.h"

#include "io/rc_controls.h"

#include "config/runtime_config.h"

extern uint16_t cycleTime;
extern uint8_t motorCount;

/*
 ****************************************************************************
 ***                    G_Tune                                            ***
 ****************************************************************************
	G_Tune Mode
	This is the multiwii implementation of ZERO-PID Algorithm
	http://technicaladventure.blogspot.com/2014/06/zero-pids-tuner-for-multirotors.html
	The algorithm has been originally developed by Mohammad Hefny (mohammad.hefny@gmail.com)

	You may use/modify this algorithm on your own risk, kindly refer to above link in any future distribution.
 */

/*
   version 1.0.0: MIN & Maxis & Tuned Band
   version 1.0.1:
                a. error is gyro reading not rc - gyro.
                b. OldError = Error no averaging.
                c. No Min Maxis BOUNDRY
    version 1.0.2:
                a. no boundaries
                b. I - Factor tune.
                c. time_skip

   Crashpilot: Reduced to just P tuning in a predefined range - so it is not "zero pid" anymore.
   Tuning is limited to just work when stick is centered besides that YAW is tuned in non Acro as well.
   See also:
   http://diydrones.com/profiles/blogs/zero-pid-tunes-for-multirotors-part-2
   http://www.multiwii.com/forum/viewtopic.php?f=8&t=5190
   Gyrosetting 2000DPS
   GyroScale = (1 / 16,4 ) * RADX(see board.h) = 0,001064225154 digit per rad/s

    pidProfile->gtune_lolimP[ROLL]   = 10;  [0..200] Lower limit of ROLL P during G tune.
    pidProfile->gtune_lolimP[PITCH]  = 10;  [0..200] Lower limit of PITCH P during G tune.
    pidProfile->gtune_lolimP[YAW]    = 10;  [0..200] Lower limit of YAW P during G tune.
    pidProfile->gtune_hilimP[ROLL]   = 100; [0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axisis.
    pidProfile->gtune_hilimP[PITCH]  = 100; [0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axisis.
    pidProfile->gtune_hilimP[YAW]    = 100; [0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axisis.
    pidProfile->gtune_pwr            = 0;   [0..10] Strength of adjustment
    pidProfile->gtune_settle_time    = 450; [200..1000] Settle time in ms
    pidProfile->gtune_average_cycles = 16;  [8..128] Number of looptime cycles used for gyro average calculation
*/

static pidProfile_t *pidProfile;
static int16_t delay_cycles;
static int16_t time_skip[3];
static int16_t OldError[3], result_P64[3];
static int32_t AvgGyro[3];
static bool floatPID;

void updateDelayCycles(void)
{
    delay_cycles = -(((int32_t)pidProfile->gtune_settle_time * 1000) / cycleTime);
}

void init_Gtune(pidProfile_t *pidProfileToTune)
{
    uint8_t i;

    pidProfile = pidProfileToTune;
	if (pidProfile->pidController == 2) {
	    floatPID = true;                                                        // LuxFloat is using float values for PID settings
	} else {
	    floatPID = false;
	}
	updateDelayCycles();
	for (i = 0; i < 3; i++) {
        if ((pidProfile->gtune_hilimP[i] && pidProfile->gtune_lolimP[i] > pidProfile->gtune_hilimP[i]) || (motorCount < 4 && i == FD_YAW)) { // User config error disable axisis for tuning
            pidProfile->gtune_hilimP[i] = 0;                                    // Disable YAW tuning for everything below a quadcopter
        }
        if (floatPID) {
            if((pidProfile->P_f[i] * 10.0f) < pidProfile->gtune_lolimP[i]) {
                pidProfile->P_f[i] = (float)(pidProfile->gtune_lolimP[i] / 10.0f);
            }
            result_P64[i] = (int16_t)pidProfile->P_f[i] << 6;                   // 6 bit extra resolution for P.
        } else {
            if(pidProfile->P8[i] < pidProfile->gtune_lolimP[i]) {
                pidProfile->P8[i] = pidProfile->gtune_lolimP[i];
            }
            result_P64[i] = (int16_t)pidProfile->P8[i] << 6;                    // 6 bit extra resolution for P.
        }
        OldError[i] = 0;
        time_skip[i] = delay_cycles;
    }
}

void calculate_Gtune(uint8_t axis)
{
    int16_t error, diff_G, threshP;

    if(rcCommand[axis] || (axis != FD_YAW && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)))) {  // Block tuning on stick input. Always allow G-Tune on YAW, Roll & Pitch only in acromode
        OldError[axis] = 0;
        time_skip[axis] = delay_cycles;                                         // Some settle time after stick center. default 450ms
    } else {
        if (!time_skip[axis]) AvgGyro[axis] = 0;
        time_skip[axis]++;
        if (time_skip[axis] > 0) {
            if (axis == FD_YAW) {
                AvgGyro[axis] += 32 * ((int16_t)gyroADC[axis] / 32);           // Chop some jitter and average
            } else {
                AvgGyro[axis] += 128 * ((int16_t)gyroADC[axis] / 128);         // Chop some jitter and average
            }
        }
        if (time_skip[axis] == pidProfile->gtune_average_cycles) {              // Looptime cycles for gyro average calculation. default 16.
            AvgGyro[axis] /= time_skip[axis];                                   // AvgGyro[axis] has now very clean gyrodata
            time_skip[axis] = 0;
            if (axis == FD_YAW) {
                threshP = 20;
                error = -AvgGyro[axis];
            } else {
                threshP = 10;
                error = AvgGyro[axis];
            }
            if (pidProfile->gtune_hilimP[axis] && error && OldError[axis] && error != OldError[axis]) {  // Don't run when not needed or pointless to do so
                diff_G = ABS(error) - ABS(OldError[axis]);
                if ((error > 0 && OldError[axis] > 0) || (error < 0 && OldError[axis] < 0)) {
                    if (diff_G > threshP) {
                        if (axis == FD_YAW) {
                            result_P64[axis] += 256 + pidProfile->gtune_pwr;    // YAW ends up at low limit on float PID, give it some more to work with.
                        } else {
                            result_P64[axis] += 64 + pidProfile->gtune_pwr;     // Shift balance a little on the plus side.
                        }
                    } else {
                        if (diff_G < -threshP) {
                            if (axis == FD_YAW) {
                                result_P64[axis] -= 64 + pidProfile->gtune_pwr;
                            } else {
                                result_P64[axis] -= 32;
                            }
                        }
                    }
                } else {
                    if (ABS(diff_G) > threshP && axis != FD_YAW) {
                        result_P64[axis] -= 32;                                 // Don't use antiwobble for YAW
                    }
                }
                int16_t newP = constrain((result_P64[axis] >> 6), (int16_t)pidProfile->gtune_lolimP[axis], (int16_t)pidProfile->gtune_hilimP[axis]);

#ifdef BLACKBOX
                if (feature(FEATURE_BLACKBOX)) {
                    flightLogEvent_gtuneCycleResult_t eventData;

                    eventData.gtuneAxis = axis;
                    eventData.gtuneGyroAVG = AvgGyro[axis];
                    eventData.gtuneNewP = newP;                                 // for float PID the logged P value is still mutiplyed by 10
                    blackboxLogEvent(FLIGHT_LOG_EVENT_GTUNE_RESULT, (flightLogEventData_t*)&eventData);
                }
#endif

                if (floatPID) {
                    pidProfile->P_f[axis] = (float)newP / 10.0f;                // new P value for float PID
                } else {
                    pidProfile->P8[axis] = newP;                                // new P value
                }
            }
            OldError[axis] = error;
        }
    }
}

#endif

