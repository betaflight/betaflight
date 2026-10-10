/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/time.h"
#include "common/vector.h"

#include "flight/autopilot.h"

#ifdef USE_WING

// Sinking faster than this, the aircraft is still flying.
#define LANDING_WING_FLYING_SINK_CMS          50.0f
// The least a descent sinks at, well clear of the flying sink, so a cut throttle that was a wrong
// guess in the air shows as one.
#define LANDING_WING_DESCENT_MIN_SINK_CMS     (2.0f * LANDING_WING_FLYING_SINK_CMS)

// Lands a fixed wing on an approach: down in a loiter over the touchdown point to the approach
// altitude, round a pattern of downwind, base and final legs, down the glide slope, a flare, and
// a touchdown it detects and disarms on. It goes around and tries again from the top when the
// approach goes wrong, until it has tried ap_wing_land_attempts times, after which it is committed.
// Where the ground's height is not known it flies no approach: it comes down in the loiter onto
// wherever the ground turns out to be.

typedef enum {
    LANDING_WING_IDLE = 0,
    LANDING_WING_LOITER_DOWN,
    LANDING_WING_ALIGN,
    LANDING_WING_DOWNWIND,
    LANDING_WING_BASE,
    LANDING_WING_FINAL,
    LANDING_WING_FLARE,
    LANDING_WING_TOUCHDOWN,
    LANDING_WING_GO_AROUND,
    LANDING_WING_DESCEND,
} landingWingPhase_e;

typedef enum {
    LANDING_WING_GO_AROUND_NONE = 0,
    LANDING_WING_GO_AROUND_STICK,
    LANDING_WING_GO_AROUND_OVERSHOOT,
    LANDING_WING_GO_AROUND_SLOPE,
    LANDING_WING_GO_AROUND_CROSS_TRACK,
    LANDING_WING_GO_AROUND_POSITION,
} landingWingGoAround_e;

typedef struct {
    vector3_t touchdownEnuM;    // estimator frame, metres; up is the ground's height there
    bool groundKnown;           // false: up is no more than a guess, and no approach is flown
    float headingDeg;           // the course to land on, or < 0 for the landing to choose one
    float sinkRateMps;          // how fast the loiter comes down to the approach altitude
} landingWingSite_t;

// The touchdown corners of the approach pattern, east and north metres: final runs from finalEnuM
// to the touchdown, level until the glide slope, base from baseEnuM to finalEnuM, downwind from
// entryEnuM to baseEnuM.
typedef struct {
    vector2_t touchdownEnuM;
    vector2_t finalEnuM;
    vector2_t baseEnuM;
    vector2_t entryEnuM;
    float finalHeightM;         // above the touchdown, final's height until the glide slope
} landingWingPattern_t;

// The height the landing and the touchdown detector work from: the rangefinder's where it has a
// reading, otherwise the altitude above groundM in the estimator frame.
float landingWingHeightM(float groundM);
// Where the ground is at home in the estimator frame: the arming point, or below it by
// ap_wing_land_launch_height when the flight began with a throw.
float landingWingHomeGroundM(void);
// Within this of home, the ground is taken to be home's.
float landingWingHomeGroundRadiusM(void);

// The flare: the throttle closed for good, the sink easing with the height from the glide's to
// ap_wing_land_flare_sink, held by the nose moving between the attitude it glided in at and
// ap_wing_land_flare_pitch, so that it neither climbs nor stalls. Still not down long after it should
// have been, the height is wrong, and the nose comes back down to the attitude it glided in at.
// Over ground of unknown height it flares as if over home's, but under power, holding its sink
// until it meets the ground wherever that is; once the throttle has closed there, it glides on down
// at the attitude it came in at.
typedef struct {
    bool active;
    bool unknownGround;
    bool powered;               // over ground of unknown height, and not yet met
    timeUs_t startUs;
    timeUs_t lastUs;
    float entrySinkCmS;
    float glidePitchDeg;        // nose-up from trim, as it came into the flare
    float pitchDeg;             // the least nose-up the flare holds, or after it gives up the most
} landingWingFlare_t;

// Landed: the sink commanded has stopped, still on the ground, for ap_landing_detection_time, or
// a second after a hard contact. Not low enough by its height to be on the ground, it has landed
// once it has sat still with its wings flat for longer.
typedef struct {
    bool contact;               // coming down, the ground has been met: the throttle stays closed
    bool braking;
    timeUs_t brakingSinceUs;
    timeUs_t flyingSinceUs;     // since the contact, sinking with nothing to show it is on the ground
    bool still;
    timeUs_t stillSinceUs;
    bool hardContact;
    float rateDps[XYZ_AXIS_COUNT];
    timeUs_t rateUs;
    bool low;                   // below the flare height
    timeUs_t lowSinceUs;
    bool flareSpent;            // a descent's flare gave up: the ground was not where it was taken to be
    landingWingFlare_t flare;
} landingWingTouchdown_t;

void landingWingTouchdownReset(landingWingTouchdown_t *touchdown);
bool landingWingTouchdownUpdate(landingWingTouchdown_t *touchdown, timeUs_t nowUs, float heightM, float sinkDemandCmS);
// Coming down where it is with no approach, onto the ground at home's height or wherever it turns
// out to be: the throttle for the path, and the flare above home's ground. Where the ground under it
// is known, by the rangefinder or near home, the wings level near it and the flare closes the
// throttle. Elsewhere, or once a flare has shown the ground lower, it circles down at a gentle bank,
// gentler still near home's ground, and flares under power. The throttle closes once the ground is
// met. The owner's limits say so; true once landed.
bool landingWingDescend(autopilotWingLimitsOwner_e owner, landingWingTouchdown_t *touchdown, timeUs_t nowUs, float sinkDemandCmS);

// Where the aircraft headed off after it was armed, a landing heading of last resort.
void landingWingNoteDepartureCourse(timeUs_t nowUs);

void landingWingStart(const landingWingSite_t *site, timeUs_t nowUs);
// True once the aircraft has landed.
bool landingWingUpdate(timeUs_t nowUs);
void landingWingStop(void);
landingWingPhase_e landingWingGetPhase(void);

#ifdef UNIT_TEST
uint8_t landingWingGetAttempts(void);
landingWingGoAround_e landingWingGetGoAround(void);
float landingWingGetFinalCourseDeg(void);       // < 0 before the pattern is laid out
#endif

#endif // USE_WING
