/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include "common/axis.h"
#include "common/time.h"
#include "common/vector.h"

#ifdef USE_WING

extern float autopilotAngle[RP_AXIS_COUNT]; // degrees

void autopilotInit(void);
void resetAltitudeControl(void);
void resetPositionControl(unsigned taskRateHz);
bool positionControl(void);
void altitudeControl(float targetAltitudeCm, timeUs_t taskIntervalUs, float targetAltitudeVelCmS, float velLimitCmS);
timeUs_t autopilotTaskIntervalUs(timeUs_t nominalIntervalUs);

bool isBelowLandingAltitude(void);
float getAutopilotThrottle(void);
// Body pitch rate a level turn at the measured bank needs, in the angle loop's sign (deg/s).
float autopilotGetTurnPitchRateDps(void);

// What a navigation leg, a landing or a descent may ask of the controllers on top of their
// configured bounds, for as long as it needs it: a gentler bank, wings held level, a pitch or
// throttle range of its own, a climb rate it sets directly, or the throttle for the path. Start from
// autopilotWingDefaultLimits(), which are the configured bounds, and change what is needed; the
// throttle range may go below ap_wing_min_throttle. Each owner's limits stand until that owner
// clears them, and the controllers fly within all of them at once.
typedef struct {
    float bankLimitDeg;
    float pitchMinDeg;      // nose-up from trim
    float pitchMaxDeg;
    float throttleMin;      // 0..1
    float throttleMax;
    bool climbRateOverride;
    float climbRateCmS;
    bool wingsLevel;
    bool motorStop;
    bool throttleForPath;   // the throttle by how far the nose stands above the path commanded over the ground
} autopilotWingLimits_t;

typedef enum {
    AP_WING_LIMITS_LEG = 0,         // the leg of a flight plan being flown
    AP_WING_LIMITS_LANDING,         // a landing, which takes precedence for a climb rate
    AP_WING_LIMITS_DESCENT,         // altitude hold coming down where it is
    AP_WING_LIMITS_OWNER_COUNT
} autopilotWingLimitsOwner_e;

void autopilotWingDefaultLimits(autopilotWingLimits_t *limits);
void autopilotWingSetLimits(autopilotWingLimitsOwner_e owner, const autopilotWingLimits_t *limits);
void autopilotWingClearLimits(autopilotWingLimitsOwner_e owner);
bool autopilotWingMotorStopRequested(void);
// The attitude's pitch, degrees nose-up from trim.
float autopilotWingAttitudePitchDeg(void);
// The most nose-up from trim at which the throttle for a climb rate's path still idles.
float autopilotWingIdlePitchDeg(float climbRateCmS);

#ifdef UNIT_TEST
float autopilotWingGetPitchIntegratorDeg(void);
float autopilotWingGetClimbDemandCmS(void);
#endif

// Lateral guidance geometry for the leg planner: how far ahead the guidance looks at the current
// groundspeed, the tightest turn it flies at the cruise speed, how early a turn through turnDeg has
// to start for the guidance to roll out on the new track, how far it flies after a right angle turn
// onto a line before it has settled on it, and the loiter radius it flies, no tighter than it can
// turn.
float autopilotWingL1DistanceM(void);
float autopilotWingMinTurnRadiusM(void);
float autopilotWingTurnDistanceM(float turnDeg);
float autopilotWingLineSettleDistanceM(void);
float autopilotWingLoiterRadiusM(void);

bool autopilotAltitudeControlAvailable(void);
bool autopilotPositionControlAvailable(void);
bool autopilotThrottleValid(void);
bool isAutopilotInControl(void);
void setSticksActiveStatus(bool areSticksActive);

float autopilotGetYawRate(void);
bool autopilotYawControlActive(void);
void autopilotSetYawRateLimit(float rateLimitDps);
void autopilotDisableYawControl(void);

#define HEADING_HOLD_TASK_RATE_HZ 100 // hz
void updateHeadingHold(timeUs_t currentTimeUs);

void autopilotSetNavHeadingOverride(bool valid, float headingDeg);
vector2_t autopilotGetPositionErrorCm(void);
void autopilotForceLevelPark(bool request);
void autopilotHeadingRecovery(bool active, float pitchDeg);

#endif // USE_WING
