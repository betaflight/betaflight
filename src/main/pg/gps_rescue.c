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

#include "platform.h"

#ifdef USE_GPS_RESCUE

#include "flight/gps_rescue.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "gps_rescue.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 6);

PG_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig,

    .minStartDistM = 15,
    .altitudeMode = GPS_RESCUE_ALT_MODE_MAX,
    .initialClimbM = 10,
    .ascendRate = 750,          // cm/s, for altitude corrections on ascent

    .returnAltitudeM = 30,
    .groundSpeedCmS = 750,
    .maxRescueAngle = 45,
    .rollMix = 150,
    .pitchCutoffHz = 75,

    .descentDistanceM = 20,
    .descendRate = 150,         // cm/s, minimum for descent and landing phase, or for descending if starting high ascent
    .disarmThreshold = 30,

    .throttleMin = 1100,
    .throttleMax = 1700,
    
    .allowArmingWithoutFix = false,
    .sanityChecks = RESCUE_SANITY_FS_ONLY,
    .minSats = 8,

    .throttleP = 15,
    .throttleI = 15,
    .throttleD = 20,
    .velP = 8,
    .velI = 40,
    .velD = 12,
    .yawP = 20,

    .useMag = GPS_RESCUE_USE_MAG,
    .imuYawGain = 10
);

#endif // USE_GPS_RESCUE
