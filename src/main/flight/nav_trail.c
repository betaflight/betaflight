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

// Breadcrumb trail recorder for the OSD navigation map: keeps a decimated
// polyline of flown positions, in local ENU metres relative to home. Points
// are laid down by distance flown, not by time — hovering adds nothing, so
// the stored line is the geometry of the flight, not its duration.

#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OSD_NAV_MAP

#include "build/build_config.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/vector.h"

#include "fc/runtime_config.h"

#include "io/gps.h"

#include "nav_trail.h"

#define NAV_TRAIL_UPDATE_INTERVAL_US 100000  // 10 Hz poll; the spacing gate does the real decimation
#define NAV_TRAIL_SPACING_M          5       // minimum distance between stored points
#define NAV_TRAIL_MAX_SPACING_M      5000    // spacing growth cap under adaptive decimation

static navTrailPoint_t trail[NAV_TRAIL_CAPACITY];
static unsigned trailCount = 0;
static uint16_t trailSpacingM = NAV_TRAIL_SPACING_M;
static bool wasArmed = false;
static timeUs_t lastUpdateUs = 0;

void navTrailReset(void)
{
    trailCount = 0;
    trailSpacingM = NAV_TRAIL_SPACING_M;
}

unsigned navTrailCount(void)
{
    return trailCount;
}

const navTrailPoint_t *navTrailPointAt(unsigned index)
{
    if (index >= trailCount) {
        return NULL;
    }
    return &trail[index];
}

static void pushTrailPoint(int32_t eastM, int32_t northM)
{
    // Adaptive decimation: when full, drop every second point and double the
    // spacing so the whole flight stays represented at reduced resolution.
    while (trailCount >= NAV_TRAIL_CAPACITY) {
        for (unsigned i = 0; i < trailCount / 2; i++) {
            trail[i] = trail[i * 2 + 1];
        }
        trailCount /= 2;
        trailSpacingM = MIN(trailSpacingM * 2, NAV_TRAIL_MAX_SPACING_M);
    }

    trail[trailCount].eastM = constrain(eastM, INT16_MIN, INT16_MAX);
    trail[trailCount].northM = constrain(northM, INT16_MIN, INT16_MAX);
    trailCount++;
}

// Feed one position (local ENU cm relative to home) through the distance
// gate. Split out so unit tests can drive it without GPS plumbing.
STATIC_UNIT_TESTED void navTrailIngestPosition(const vector2_t *posCm)
{
    const int32_t eastM = lrintf(posCm->x / 100.0f);
    const int32_t northM = lrintf(posCm->y / 100.0f);

    if (trailCount == 0) {
        pushTrailPoint(eastM, northM);
        return;
    }

    const navTrailPoint_t *last = &trail[trailCount - 1];
    const int64_t dx = eastM - last->eastM;
    const int64_t dy = northM - last->northM;
    const int64_t spacing = trailSpacingM;
    if (dx * dx + dy * dy >= spacing * spacing) {
        pushTrailPoint(eastM, northM);
    }
}

void navTrailUpdate(timeUs_t currentTimeUs)
{
    // the trail is per-flight: reset on the arm edge, before this cycle's
    // position can be recorded, so the new line starts at the new launch point
    const bool armed = ARMING_FLAG(ARMED);
    if (armed && !wasArmed) {
        navTrailReset();
    }
    wasArmed = armed;

    if (lastUpdateUs != 0 && cmpTimeUs(currentTimeUs, lastUpdateUs) < NAV_TRAIL_UPDATE_INTERVAL_US) {
        return;
    }
    lastUpdateUs = currentTimeUs;

    if (!armed || !STATE(GPS_FIX) || !STATE(GPS_FIX_HOME)) {
        return;
    }

    vector2_t posCm;
    GPS_distance2d(&GPS_home_llh, &gpsSol.llh, &posCm);
    navTrailIngestPosition(&posCm);
}

#ifdef UNIT_TEST
void navTrailResetStateForTest(void)
{
    navTrailReset();
    wasArmed = false;
    lastUpdateUs = 0;
}
#endif

#endif // USE_OSD_NAV_MAP
