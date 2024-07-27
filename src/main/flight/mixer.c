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
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer_init.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "mixer.h"

#define DYN_LPF_THROTTLE_STEPS           100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 5000 // minimum of 5ms between updates

static FAST_DATA_ZERO_INIT float motorMixRange;

float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT int throttleAngleCorrection;

float getMotorMixRange(void)
{
    return motorMixRange;
}

void writeMotors(void)
{
    motorWriteAll(motor);
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(mixerRuntime.disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT float mixerThrottle = 0;
static FAST_DATA_ZERO_INIT float motorOutputMin;
static FAST_DATA_ZERO_INIT float motorRangeMin;
static FAST_DATA_ZERO_INIT float motorRangeMax;
static FAST_DATA_ZERO_INIT float motorOutputRange;
static FAST_DATA_ZERO_INIT int8_t motorOutputMixSign;

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    // static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    // static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    static float motorRangeMinIncrease = 0;

    float currentThrottleInputRange = 0;
//     if (mixerRuntime.feature3dEnabled) {
//         uint16_t rcCommand3dDeadBandLow;
//         uint16_t rcCommand3dDeadBandHigh;

//         if (!ARMING_FLAG(ARMED)) {
//             rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
//         }

//         if (IS_RC_MODE_ACTIVE(BOX3D) || flight3DConfig()->switched_mode3d) {
//             // The min_check range is halved because the output throttle is scaled to 500us.
//             // So by using half of min_check we maintain the same low-throttle deadband
//             // stick travel as normal non-3D mode.
//             const int mincheckOffset = (rxConfig()->mincheck - PWM_RANGE_MIN) / 2;
//             rcCommand3dDeadBandLow = rxConfig()->midrc - mincheckOffset;
//             rcCommand3dDeadBandHigh = rxConfig()->midrc + mincheckOffset;
//         } else {
//             rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
//             rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;
//         }

//         const float rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
//         const float rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;

//         if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow || isFlipOverAfterCrashActive()) {
//             // INVERTED
//             motorRangeMin = mixerRuntime.motorOutputLow;
//             motorRangeMax = mixerRuntime.deadbandMotor3dLow;
// #ifdef USE_DSHOT
//             if (isMotorProtocolDshot()) {
//                 motorOutputMin = mixerRuntime.motorOutputLow;
//                 motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
//             } else
// #endif
//             {
//                 motorOutputMin = mixerRuntime.deadbandMotor3dLow;
//                 motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
//             }

//             if (motorOutputMixSign != -1) {
//                 reversalTimeUs = currentTimeUs;
//             }
//             motorOutputMixSign = -1;

//             rcThrottlePrevious = rcCommand[THROTTLE];
//             throttle = rcCommand3dDeadBandLow - rcCommand[THROTTLE];
//             currentThrottleInputRange = rcCommandThrottleRange3dLow;
//         } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
//             // NORMAL
//             motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
//             motorRangeMax = mixerRuntime.motorOutputHigh;
//             motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
//             motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
//             if (motorOutputMixSign != 1) {
//                 reversalTimeUs = currentTimeUs;
//             }
//             motorOutputMixSign = 1;
//             rcThrottlePrevious = rcCommand[THROTTLE];
//             throttle = rcCommand[THROTTLE] - rcCommand3dDeadBandHigh;
//             currentThrottleInputRange = rcCommandThrottleRange3dHigh;
//         } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
//                 !flight3DConfigMutable()->switched_mode3d) ||
//                 isMotorsReversed()) {
//             // INVERTED_TO_DEADBAND
//             motorRangeMin = mixerRuntime.motorOutputLow;
//             motorRangeMax = mixerRuntime.deadbandMotor3dLow;

// #ifdef USE_DSHOT
//             if (isMotorProtocolDshot()) {
//                 motorOutputMin = mixerRuntime.motorOutputLow;
//                 motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
//             } else
// #endif
//             {
//                 motorOutputMin = mixerRuntime.deadbandMotor3dLow;
//                 motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
//             }

//             if (motorOutputMixSign != -1) {
//                 reversalTimeUs = currentTimeUs;
//             }
//             motorOutputMixSign = -1;

//             throttle = 0;
//             currentThrottleInputRange = rcCommandThrottleRange3dLow;
//         } else {
//             // NORMAL_TO_DEADBAND
//             motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
//             motorRangeMax = mixerRuntime.motorOutputHigh;
//             motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
//             motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
//             if (motorOutputMixSign != 1) {
//                 reversalTimeUs = currentTimeUs;
//             }
//             motorOutputMixSign = 1;
//             throttle = 0;
//             currentThrottleInputRange = rcCommandThrottleRange3dHigh;
//         }
//         if (currentTimeUs - reversalTimeUs < 250000) {
//             // keep iterm zero for 250ms after motor reversal
//             pidResetIterm();
//         }
    // } else {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
        currentThrottleInputRange = PWM_RANGE;

#ifdef USE_DYN_IDLE
        if (mixerRuntime.dynIdleMinRps > 0.0f) {
            const float maxIncrease = isAirmodeActivated() ? mixerRuntime.dynIdleMaxIncrease : 0.05f;
            float minRps = getMinMotorFrequency();
            DEBUG_SET(DEBUG_DYN_IDLE, 3, lrintf(minRps * 10.0f));
            float rpsError = mixerRuntime.dynIdleMinRps - minRps;
            // PT1 type lowpass delay and smoothing for D
            minRps = mixerRuntime.prevMinRps + mixerRuntime.minRpsDelayK * (minRps - mixerRuntime.prevMinRps);
            float dynIdleD = (mixerRuntime.prevMinRps - minRps) * mixerRuntime.dynIdleDGain;
            mixerRuntime.prevMinRps = minRps;
            float dynIdleP = rpsError * mixerRuntime.dynIdlePGain;
            rpsError = MAX(-0.1f, rpsError); //I rises fast, falls slowly
            mixerRuntime.dynIdleI += rpsError * mixerRuntime.dynIdleIGain;
            mixerRuntime.dynIdleI = constrainf(mixerRuntime.dynIdleI, 0.0f, maxIncrease);
            motorRangeMinIncrease = constrainf((dynIdleP + mixerRuntime.dynIdleI + dynIdleD), 0.0f, maxIncrease);
            DEBUG_SET(DEBUG_DYN_IDLE, 0, MAX(-1000, lrintf(dynIdleP * 10000)));
            DEBUG_SET(DEBUG_DYN_IDLE, 1, lrintf(mixerRuntime.dynIdleI * 10000));
            DEBUG_SET(DEBUG_DYN_IDLE, 2, lrintf(dynIdleD * 10000));
        } else {
            motorRangeMinIncrease = 0;
        }
#endif

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
        float motorRangeAttenuationFactor = 0;
        // reduce motorRangeMax when battery is full
        if (!mixerConfig()->govenor && mixerRuntime.vbatSagCompensationFactor > 0.0f) {
            const uint16_t currentCellVoltage = getBatterySagCellVoltage();
            // batteryGoodness = 1 when voltage is above vbatFull, and 0 when voltage is below vbatLow
            float batteryGoodness = 1.0f - constrainf((mixerRuntime.vbatFull - currentCellVoltage) / mixerRuntime.vbatRangeToCompensate, 0.0f, 1.0f);
            motorRangeAttenuationFactor = (mixerRuntime.vbatRangeToCompensate / mixerRuntime.vbatFull) * batteryGoodness * mixerRuntime.vbatSagCompensationFactor;
            DEBUG_SET(DEBUG_BATTERY, 2, batteryGoodness * 100);
            DEBUG_SET(DEBUG_BATTERY, 3, motorRangeAttenuationFactor * 1000);
        }
        motorRangeMax = isFlipOverAfterCrashActive() ? mixerRuntime.motorOutputHigh : mixerRuntime.motorOutputHigh - motorRangeAttenuationFactor * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
#else
        motorRangeMax = mixerRuntime.motorOutputHigh;
#endif

        motorRangeMin = mixerRuntime.motorOutputLow + motorRangeMinIncrease * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
        motorOutputMin = motorRangeMin;
        motorOutputRange = motorRangeMax - motorRangeMin;
        motorOutputMixSign = 1;
    // }
    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

#define CRASH_FLIP_DEADBAND 20
#define CRASH_FLIP_STICK_MINF 0.15f

static void applyFlipOverAfterCrashModeToMotors(void)
{
    if (ARMING_FLAG(ARMED)) {
        const float flipPowerFactor = 1.0f - mixerConfig()->crashflip_expo / 100.0f;
        const float stickDeflectionPitchAbs = getRcDeflectionAbs(FD_PITCH);
        const float stickDeflectionRollAbs = getRcDeflectionAbs(FD_ROLL);
        const float stickDeflectionYawAbs = getRcDeflectionAbs(FD_YAW);

        const float stickDeflectionPitchExpo = flipPowerFactor * stickDeflectionPitchAbs + power3(stickDeflectionPitchAbs) * (1 - flipPowerFactor);
        const float stickDeflectionRollExpo = flipPowerFactor * stickDeflectionRollAbs + power3(stickDeflectionRollAbs) * (1 - flipPowerFactor);
        const float stickDeflectionYawExpo = flipPowerFactor * stickDeflectionYawAbs + power3(stickDeflectionYawAbs) * (1 - flipPowerFactor);

        float signPitch = getRcDeflection(FD_PITCH) < 0 ? 1 : -1;
        float signRoll = getRcDeflection(FD_ROLL) < 0 ? 1 : -1;
        float signYaw = (getRcDeflection(FD_YAW) < 0 ? 1 : -1) * (mixerConfig()->yaw_motors_reversed ? 1 : -1);

        float stickDeflectionLength = sqrtf(sq(stickDeflectionPitchAbs) + sq(stickDeflectionRollAbs));
        float stickDeflectionExpoLength = sqrtf(sq(stickDeflectionPitchExpo) + sq(stickDeflectionRollExpo));

        if (stickDeflectionYawAbs > MAX(stickDeflectionPitchAbs, stickDeflectionRollAbs)) {
            // If yaw is the dominant, disable pitch and roll
            stickDeflectionLength = stickDeflectionYawAbs;
            stickDeflectionExpoLength = stickDeflectionYawExpo;
            signRoll = 0;
            signPitch = 0;
        } else {
            // If pitch/roll dominant, disable yaw
            signYaw = 0;
        }

        const float cosPhi = (stickDeflectionLength > 0) ? (stickDeflectionPitchAbs + stickDeflectionRollAbs) / (sqrtf(2.0f) * stickDeflectionLength) : 0;
        const float cosThreshold = sqrtf(3.0f)/2.0f; // cos(PI/6.0f)

        if (cosPhi < cosThreshold) {
            // Enforce either roll or pitch exclusively, if not on diagonal
            if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
                signPitch = 0;
            } else {
                signRoll = 0;
            }
        }

        // Apply a reasonable amount of stick deadband
        const float crashFlipStickMinExpo = flipPowerFactor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flipPowerFactor);
        const float flipStickRange = 1.0f - crashFlipStickMinExpo;
        const float flipPower = MAX(0.0f, stickDeflectionExpoLength - crashFlipStickMinExpo) / flipStickRange;

        for (int i = 0; i < mixerRuntime.motorCount; ++i) {
            float motorOutputNormalised =
                signPitch * mixerRuntime.currentMixer[i].pitch +
                signRoll * mixerRuntime.currentMixer[i].roll +
                signYaw * mixerRuntime.currentMixer[i].yaw;

            if (motorOutputNormalised < 0) {
                if (mixerConfig()->crashflip_motor_percent > 0) {
                    motorOutputNormalised = -motorOutputNormalised * (float)mixerConfig()->crashflip_motor_percent / 100.0f;
                } else {
                    motorOutputNormalised = 0;
                }
            }
            motorOutputNormalised = MIN(1.0f, flipPower * motorOutputNormalised);
            float motorOutput = motorOutputMin + motorOutputNormalised * motorOutputRange;

            // Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
            motorOutput = (motorOutput < motorOutputMin + CRASH_FLIP_DEADBAND) ? mixerRuntime.disarmMotorOutput : (motorOutput - CRASH_FLIP_DEADBAND);

            motor[i] = motorOutput;
        }
    } else {
        // Disarmed mode
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

uint8_t getAfterburnerTanksRemaining(void)
{
    return mixerRuntime.afterburnerTanksRemaining;
}

float getAfterburnerTankPercent(void)
{
    return mixerRuntime.afterburnerTankPercent;
}

static void applyRPMLimiter(void)
{
    if (mixerRuntime.govenorEnabled) {
        float RPM_GOVENOR_LIMIT = 0.0f;
        float averageRPM = 0.0f;
        float averageRPM_smoothed = 0.0f;
        float PIDOutput = 0.0f;
        float rcCommandThrottle = (rcCommand[THROTTLE]-1000)/1000.0f;
        float maxRPMLimit = 0.0f;

        maxRPMLimit = mixerRuntime.RPMLimit;
        
        //afterburner code
        //if drone is armed
        if (ARMING_FLAG(ARMED)) {
            //if the afterburner switch is engaged
            if(IS_RC_MODE_ACTIVE(BOXBOOST)) {
                //if the afterburner isn't initiated
                if(mixerRuntime.afterburnerInitiated == false) {
                    //if there's charge in the tank
                    if(mixerRuntime.afterburnerTankPercent>0.0f) {
                        //if we have another tank left
                        if(mixerRuntime.afterburnerTanksRemaining>0) {
                            mixerRuntime.afterburnerInitiated = true;
                        }
                    }
                    
                }
            //if the afterburner switch is NOT engaged
            } else {
                //if hold to boost is enabled
                if(mixerRuntime.afterburnerHoldToBoost) {
                    mixerRuntime.afterburnerInitiated = false;
                }
            }

            //use afterburner
            if(mixerRuntime.afterburnerInitiated) {
                //if the tank is empty, reset
                if(mixerRuntime.afterburnerTankPercent<=0.0f) {
                    mixerRuntime.afterburnerInitiated = false;
                    //only refil the tank if we have one remaining
                    if(mixerRuntime.afterburnerTanksRemaining>0) {
                        mixerRuntime.afterburnerTanksRemaining -= 1;
                    }
                    if(mixerRuntime.afterburnerTanksRemaining>0) {
                        mixerRuntime.afterburnerTankPercent = 100.0f;
                    }

                } 
                //tank percent decreases linearly
                mixerRuntime.afterburnerTankPercent -= (pidGetDT()/(mixerRuntime.afterburnerDuration))*100.0f;
                //tank percent can never be above 100%
                mixerRuntime.afterburnerTankPercent = MIN(mixerRuntime.afterburnerTankPercent,100.0f);
                //tank percent can never be below 0%
                mixerRuntime.afterburnerTankPercent = MAX(mixerRuntime.afterburnerTankPercent,0.0f);
                //increase the rpm limit
                maxRPMLimit = mixerRuntime.RPMLimit+(mixerRuntime.afterburnerRPM*mixerRuntime.afterburnerTankPercent*0.01f);
            }
        }else{
            if(mixerRuntime.afterburnerReset) {
                mixerRuntime.afterburnerTankPercent = 100.0f;
                mixerRuntime.afterburnerTanksRemaining = mixerConfig()->govenor_rpm_afterburner_tank_count;
            }
        }

        //Street League customization
        if (mixerRuntime.rpmLinearization) {
            //scales rpm setpoint between idle rpm and max rpm limit based on throttle percent
            RPM_GOVENOR_LIMIT = ((maxRPMLimit - mixerConfig()->govenor_idle_rpm))*100.0f*(rcCommandThrottle) + mixerConfig()->govenor_idle_rpm * 100.0f;

            //limit the speed with which the rpm setpoint can change based on the rpm_limiter_acceleration_limit cli command
            float acceleration = RPM_GOVENOR_LIMIT - mixerRuntime.govenorPreviousRPMLimit;
            if(acceleration > 0) {
                acceleration = MIN(acceleration, mixerRuntime.govenorAccelerationLimit);
                RPM_GOVENOR_LIMIT = mixerRuntime.govenorPreviousRPMLimit + acceleration;
            }
            else if(acceleration < 0) {
                acceleration = MAX(acceleration, -mixerRuntime.govenorDecelerationLimit);
                RPM_GOVENOR_LIMIT = mixerRuntime.govenorPreviousRPMLimit + acceleration;
            }
        } else {
            throttle = throttle * mixerRuntime.govenorExpectedThrottleLimit;
            RPM_GOVENOR_LIMIT = ((maxRPMLimit))*100.0f;
        }

        //get the rpm averaged across the motors
        bool motorsSaturated = false;
        bool motorsDesaturated = false;
        for (int i = 0; i < getMotorCount(); i++) {
            averageRPM += (float)getDshotTelemetry(i);
            if (motor[i] >= motorConfig()->maxthrottle) {
                motorsSaturated = true;
            }
            if (motor[i] <= motorConfig()->minthrottle) {
                motorsDesaturated = false;
            }
        }
        averageRPM = 100.0f * averageRPM / (float)(getMotorCount()*mixerRuntime.motorPoleCount/2.0f);

        //get the smoothed rpm to avoid d term noise
        averageRPM_smoothed = mixerRuntime.govenorPreviousSmoothedRPM + (float)mixerRuntime.govenorDelayK * (averageRPM - mixerRuntime.govenorPreviousSmoothedRPM); //kinda braindead to convert to rps then back
        
        float smoothedRPMError = averageRPM_smoothed - RPM_GOVENOR_LIMIT;
        float govenorP = smoothedRPMError * mixerRuntime.govenorPGain; //+ when overspped
        float govenorD = (smoothedRPMError-mixerRuntime.govenorPreviousSmoothedRPMError) * mixerRuntime.govenorDGain; // + when quickly going overspeed
        
        if (mixerRuntime.rpmLinearization) {
            /*
            //don't let I term wind up if throttle is below the motor idle
            if (rcCommandThrottle < motorConfig()->digitalIdleOffsetValue / 10000.0f) {
                mixerRuntime.govenorI *= 1.0f/(1.0f+(pidGetDT()*10.0f)); //slowly ramp down i term instead of resetting to avoid throttle pulsing cheats
            } else {
                //don't let I term wind up if motors are saturated. Otherwise, motors may stay at high throttle even after low throttle is commanded
                if(!motorsSaturated)
                {
                    mixerRuntime.govenorI += smoothedRPMError * mixerRuntime.govenorIGain; // + when overspeed
                }
            }*/

            //don't let I term wind up if throttle is below the motor idle
            if (rcCommandThrottle < motorConfig()->digitalIdleOffsetValue / 10000.0f) {
                mixerRuntime.govenorI *= 1.0f/(1.0f+(pidGetDT()*10.0f)); //slowly ramp down i term
                //mixerRuntime.govenorI = 0.0f;
            } else {
                //don't let I term wind up if motors are saturated. Otherwise, motors may stay at high throttle even after low throttle is commanded
                if(!motorsSaturated && !motorsDesaturated)
                {
                    mixerRuntime.govenorI += smoothedRPMError * mixerRuntime.govenorIGain; // + when overspeed
                }else{
                    mixerRuntime.govenorI *= 1.0f/(1.0f+(pidGetDT()*10.0f)); //slowly ramp down i term
                }
            }

            //sum our pid terms
            PIDOutput = govenorP + mixerRuntime.govenorI + govenorD; //more + when overspeed, should be subtracted from throttle
        
        } else {
            throttle = throttle * mixerRuntime.govenorExpectedThrottleLimit;
            mixerRuntime.govenorI += smoothedRPMError * mixerRuntime.govenorIGain; // + when overspeed
            mixerRuntime.govenorI = MAX(mixerRuntime.govenorI, 0.0f);
            PIDOutput = govenorP + mixerRuntime.govenorI + govenorD; //more + when overspeed, should be subtracted from throttle
            if (PIDOutput > 0.05) {
                mixerRuntime.govenorExpectedThrottleLimit = 0.9994 * mixerRuntime.govenorExpectedThrottleLimit;
            }
            if (PIDOutput < -0.05 && rcCommand[THROTTLE] > 1950 && !motorsSaturated) {
                mixerRuntime.govenorExpectedThrottleLimit = (1+1-0.9994) * mixerRuntime.govenorExpectedThrottleLimit;
                mixerRuntime.govenorExpectedThrottleLimit = MAX(mixerRuntime.govenorExpectedThrottleLimit, 1.0f);
            }

            PIDOutput = MAX(PIDOutput,0.0f);
            
        }
        if (motorConfig()->dev.useDshotTelemetry) {
            if (mixerRuntime.govenor_init) {
                if (mixerRuntime.rpmLinearization) {
                    throttle = constrainf(-PIDOutput, 0.0f, 1.0f);
                } else {
                    throttle = constrainf(throttle-PIDOutput, 0.0f, 1.0f);
                }
            }
        } else {    //if dshot telemetry isn't enabled. Keep the throttle at zero
            throttle = 0.0f;
        }

        mixerRuntime.govenor_init = true;

        //update previous values for next loop
        mixerRuntime.prevAverageRPM = averageRPM;
        mixerRuntime.govenorPreviousSmoothedRPM = averageRPM_smoothed;
        mixerRuntime.govenorPreviousSmoothedRPMError = smoothedRPMError;
        mixerRuntime.govenorPreviousRPMLimit = RPM_GOVENOR_LIMIT;
        
        DEBUG_SET(DEBUG_RPM_LIMITER, 0, averageRPM);//unfiltered average rpm
        DEBUG_SET(DEBUG_RPM_LIMITER, 1, averageRPM_smoothed); //filtered average rpm
        DEBUG_SET(DEBUG_RPM_LIMITER, 2, smoothedRPMError); //P term
        DEBUG_SET(DEBUG_RPM_LIMITER, 3, mixerRuntime.govenorI*100.0f); // I term
        DEBUG_SET(DEBUG_RPM_LIMITER, 4, govenorD*10000.0f); // D term
        
        /*DEBUG_SET(DEBUG_RPM_LIMITER, 0, mixerRuntime.afterburnerInitiated);
        DEBUG_SET(DEBUG_RPM_LIMITER, 1, mixerRuntime.afterburnerTankPercent);
        DEBUG_SET(DEBUG_RPM_LIMITER, 2, mixerRuntime.afterburnerTanksRemaining);
        DEBUG_SET(DEBUG_RPM_LIMITER, 3, maxRPMLimit);*/
    }
}

static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float motorOutput = motorOutputMixSign * motorMix[i] + throttle * activeMixer[i].throttle;
        if (!mixerConfig()->govenor) {
            #ifdef USE_THRUST_LINEARIZATION
            motorOutput = pidApplyThrustLinearization(motorOutput);
            #endif
        }
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;
#ifdef USE_SERVOS
        if (mixerIsTricopter()) {
            motorOutput += mixerTricopterMotorCorrection(i);
        }
#endif
        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? mixerRuntime.disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrainf(motorOutput, mixerRuntime.disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrainf(motorOutput, motorRangeMin, motorRangeMax);
        }
        motor[i] = motorOutput;
    }

    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        mixerRuntime.govenorI = 0;
        mixerRuntime.govenor_init = false;
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static float applyThrottleLimit(float throttle)
{
    if (currentControlRateProfile->throttle_limit_percent < 100) {
        const float throttleLimitFactor = currentControlRateProfile->throttle_limit_percent / 100.0f;
        switch (currentControlRateProfile->throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }

    return throttle;
}

static void applyMotorStop(void)
{
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mixerRuntime.disarmMotorOutput;
    }
}

#ifdef USE_DYN_LPF
static void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
            dynLpfGyroUpdate(dynLpfThrottle);
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif

static void applyMixerAdjustmentLinear(float *motorMix, const bool airmodeEnabled) {
    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? motorMixRange : 1.0f;
    const float motorMixDelta = 0.5f * motorMixRange;

    for (int i = 0; i < mixerRuntime.motorCount; ++i) {
        if (airmodeEnabled || throttle > 0.5f) {
            if (mixerConfig()->mixer_type == MIXER_LINEAR) {
                motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + motorMixDelta, motorMix[i] - motorMixDelta);
            } else {
                motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + ABS(motorMix[i]), motorMix[i] - ABS(motorMix[i]));
            }
        }
        motorMix[i] /= motorMixNormalizationFactor;
    }
}

static void applyMixerAdjustment(float *motorMix, const float motorMixMin, const float motorMixMax, const bool airmodeEnabled) {
#ifdef USE_AIRMODE_LPF
    const float unadjustedThrottle = throttle;
    throttle += pidGetAirmodeThrottleOffset();
    float airmodeThrottleChange = 0;
#endif

    if (motorMixRange > 1.0f) {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motorMix[i] /= motorMixRange;
        }
        // Get the maximum correction by setting offset to center when airmode enabled
        if (airmodeEnabled) {
            throttle = 0.5f;
        }
    } else {
        if (airmodeEnabled || throttle > 0.5f) {
            throttle = constrainf(throttle, -motorMixMin, 1.0f - motorMixMax);
#ifdef USE_AIRMODE_LPF
            airmodeThrottleChange = constrainf(unadjustedThrottle, -motorMixMin, 1.0f - motorMixMax) - unadjustedThrottle;
#endif
        }
    }

#ifdef USE_AIRMODE_LPF
    pidUpdateAirmodeLpf(airmodeThrottleChange);
#endif
}

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    if (isFlipOverAfterCrashActive()) {
        applyFlipOverAfterCrashModeToMotors();

        return;
    }

    const bool launchControlActive = isLaunchControlActive();

    motorMixer_t * activeMixer = &mixerRuntime.currentMixer[0];
#ifdef USE_LAUNCH_CONTROL
    if (launchControlActive && (currentPidProfile->launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY)) {
        activeMixer = &mixerRuntime.launchControlMixer[0];
    }
#endif

    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinDetected = gyroYawSpinDetected();
    if (yawSpinDetected) {
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
#endif // USE_YAW_SPIN_RECOVERY

    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }

    // use scaled throttle, without dynamic idle throttle offset, as the input to antigravity
    pidUpdateAntiGravityThrottleFilter(throttle);

    // and for TPA
    pidUpdateTpaFactor(throttle);

#ifdef USE_DYN_LPF
    // keep the changes to dynamic lowpass clean, without unnecessary dynamic changes
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif

    // apply throttle boost when throttle moves quickly
#if defined(USE_THROTTLE_BOOST)
    if (throttleBoost > 0.0f) {
        const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    }
#endif

    // send throttle value to blackbox, including scaling and throttle boost, but not TL compensation, dyn idle or airmode 
    mixerThrottle = throttle;

#ifdef USE_DYN_IDLE
    // Set min throttle offset of 1% when stick is at zero and dynamic idle is active
    if (mixerRuntime.dynIdleMinRps > 0.0f) {
        throttle = MAX(throttle, 0.01f);
    }
#endif

#ifdef USE_THRUST_LINEARIZATION
    // reduce throttle to offset additional motor output
    throttle = pidCompensateThrustLinearization(throttle);
#endif
    applyRPMLimiter();
    // Find roll/pitch/yaw desired output
    // ??? Where is the optimal location for this code?
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float mix =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw;

        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

    //  The following fixed throttle values will not be shown in the blackbox log
    // ?? Should they be influenced by airmode?  If not, should go after the apply airmode code.
    const bool airmodeEnabled = airmodeIsEnabled() || launchControlActive;
#ifdef USE_YAW_SPIN_RECOVERY
    // 50% throttle provides the maximum authority for yaw recovery when airmode is not active.
    // When airmode is active the throttle setting doesn't impact recovery authority.
    if (yawSpinDetected && !airmodeEnabled) {
        throttle = 0.5f;
    }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
    // While launch control is active keep the throttle at minimum.
    // Once the pilot triggers the launch throttle control will be reactivated.
    if (launchControlActive) {
        throttle = 0.0f;
    }
#endif

#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif

    motorMixRange = motorMixMax - motorMixMin;
    if (mixerConfig()->mixer_type > MIXER_LEGACY) {
        applyMixerAdjustmentLinear(motorMix, airmodeEnabled);
    } else {
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
    }

    if (featureIsEnabled(FEATURE_MOTOR_STOP)
        && ARMING_FLAG(ARMED)
        && !mixerRuntime.feature3dEnabled
        && !airmodeEnabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable motor_stop while GPS Rescue is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        // motor_stop handling
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix, activeMixer);
    }
}

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}

float mixerGetThrottle(void)
{
    return mixerThrottle;
}