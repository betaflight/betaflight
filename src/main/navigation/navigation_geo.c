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
#include <math.h>

#include "platform.h"

#if defined(NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"


#if defined(NAV_AUTO_MAG_DECLINATION)
/* Declination calculation code from PX4 project */
/* set this always to the sampling in degrees for the table below */
#define SAMPLING_RES        10.0f
#define SAMPLING_MIN_LAT    -60.0f
#define SAMPLING_MAX_LAT    60.0f
#define SAMPLING_MIN_LON    -180.0f
#define SAMPLING_MAX_LON    180.0f

static const int8_t declination_table[13][37] = \
{
    { 46, 45, 44, 42, 41, 40, 38, 36, 33, 28, 23, 16, 10, 4, -1, -5, -9, -14, -19, -26, -33, -40, -48, -55, -61, -66, -71, -74, -75, -72, -61, -25, 22, 40, 45, 47, 46 },
    { 30, 30, 30, 30, 29, 29, 29, 29, 27, 24, 18, 11, 3, -3, -9, -12, -15, -17, -21, -26, -32, -39, -45, -51, -55, -57, -56, -53, -44, -31, -14, 0, 13, 21, 26, 29, 30 },
    { 21, 22, 22, 22, 22, 22, 22, 22, 21, 18, 13, 5, -3, -11, -17, -20, -21, -22, -23, -25, -29, -35, -40, -44, -45, -44, -40, -32, -22, -12, -3, 3, 9, 14, 18, 20, 21 },
    { 16, 17, 17, 17, 17, 17, 16, 16, 16, 13, 8, 0, -9, -16, -21, -24, -25, -25, -23, -20, -21, -24, -28, -31, -31, -29, -24, -17, -9, -3, 0, 4, 7, 10, 13, 15, 16 },
    { 12, 13, 13, 13, 13, 13, 12, 12, 11, 9, 3, -4, -12, -19, -23, -24, -24, -22, -17, -12, -9, -10, -13, -17, -18, -16, -13, -8, -3, 0, 1, 3, 6, 8, 10, 12, 12 },
    { 10, 10, 10, 10, 10, 10, 10, 9, 9, 6, 0, -6, -14, -20, -22, -22, -19, -15, -10, -6, -2, -2, -4, -7, -8, -8, -7, -4, 0, 1, 1, 2, 4, 6, 8, 10, 10 },
    { 9, 9, 9, 9, 9, 9, 8, 8, 7, 4, -1, -8, -15, -19, -20, -18, -14, -9, -5, -2, 0, 1, 0, -2, -3, -4, -3, -2, 0, 0, 0, 1, 3, 5, 7, 8, 9 },
    { 8, 8, 8, 9, 9, 9, 8, 8, 6, 2, -3, -9, -15, -18, -17, -14, -10, -6, -2, 0, 1, 2, 2, 0, -1, -1, -2, -1, 0, 0, 0, 0, 1, 3, 5, 7, 8 },
    { 8, 9, 9, 10, 10, 10, 10, 8, 5, 0, -5, -11, -15, -16, -15, -12, -8, -4, -1, 0, 2, 3, 2, 1, 0, 0, 0, 0, 0, -1, -2, -2, -1, 0, 3, 6, 8 },
    { 6, 9, 10, 11, 12, 12, 11, 9, 5, 0, -7, -12, -15, -15, -13, -10, -7, -3, 0, 1, 2, 3, 3, 3, 2, 1, 0, 0, -1, -3, -4, -5, -5, -2, 0, 3, 6 },
    { 5, 8, 11, 13, 15, 15, 14, 11, 5, -1, -9, -14, -17, -16, -14, -11, -7, -3, 0, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -4, -7, -8, -8, -6, -2, 1, 5 },
    { 4, 8, 12, 15, 17, 18, 16, 12, 5, -3, -12, -18, -20, -19, -16, -13, -8, -4, -1, 1, 4, 6, 8, 9, 9, 9, 7, 3, -1, -6, -10, -12, -11, -9, -5, 0, 4 },
    { 3, 9, 14, 17, 20, 21, 19, 14, 4, -8, -19, -25, -26, -25, -21, -17, -12, -7, -2, 1, 5, 9, 13, 15, 16, 16, 13, 7, 0, -7, -12, -15, -14, -11, -6, -1, 3 },
};

static float get_lookup_table_val(unsigned lat_index, unsigned lon_index)
{
    return declination_table[lat_index][lon_index];
}

float geoCalculateMagDeclination(const gpsLocation_t * llh) // degrees units
{
    /*
     * If the values exceed valid ranges, return zero as default
     * as we have no way of knowing what the closest real value
     * would be.
     */
    const float lat = llh->lat / 10000000.0f;
    const float lon = llh->lon / 10000000.0f;

    if (lat < -90.0f || lat > 90.0f ||
        lon < -180.0f || lon > 180.0f) {
        return 0.0f;
    }

    /* round down to nearest sampling resolution */
    int min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES;
    int min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES;

    /* for the rare case of hitting the bounds exactly
     * the rounding logic wouldn't fit, so enforce it.
     */

    /* limit to table bounds - required for maxima even when table spans full globe range */
    if (lat <= SAMPLING_MIN_LAT) {
        min_lat = SAMPLING_MIN_LAT;
    }

    if (lat >= SAMPLING_MAX_LAT) {
        min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
    }

    if (lon <= SAMPLING_MIN_LON) {
        min_lon = SAMPLING_MIN_LON;
    }

    if (lon >= SAMPLING_MAX_LON) {
        min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
    }

    /* find index of nearest low sampling point */
    const unsigned min_lat_index = (-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES;
    const unsigned min_lon_index = (-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES;

    const float declination_sw = get_lookup_table_val(min_lat_index, min_lon_index);
    const float declination_se = get_lookup_table_val(min_lat_index, min_lon_index + 1);
    const float declination_ne = get_lookup_table_val(min_lat_index + 1, min_lon_index + 1);
    const float declination_nw = get_lookup_table_val(min_lat_index + 1, min_lon_index);

    /* perform bilinear interpolation on the four grid corners */

    const float declination_min = ((lon - min_lon) / SAMPLING_RES) * (declination_se - declination_sw) + declination_sw;
    const float declination_max = ((lon - min_lon) / SAMPLING_RES) * (declination_ne - declination_nw) + declination_nw;

    return ((lat - min_lat) / SAMPLING_RES) * (declination_max - declination_min) + declination_min;
}
#endif

void geoSetOrigin(gpsOrigin_s * origin, const gpsLocation_t * llh, geoOriginResetMode_e resetMode)
{
    if (resetMode == GEO_ORIGIN_SET) {
        origin->valid = true;
        origin->lat = llh->lat;
        origin->lon = llh->lon;
        origin->alt = llh->alt;
        origin->scale = constrainf(cos_approx((ABS(origin->lat) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);
    }
    else if (origin->valid && (resetMode == GEO_ORIGIN_RESET_ALTITUDE)) {
        origin->alt = llh->alt;
    }
}

void geoConvertGeodeticToLocal(gpsOrigin_s * origin, const gpsLocation_t * llh, t_fp_vector * pos, geoAltitudeConversionMode_e altConv)
{
    if (origin->valid) {
        pos->V.X = (llh->lat - origin->lat) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
        pos->V.Y = (llh->lon - origin->lon) * (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * origin->scale);

        // If flag GEO_ALT_RELATIVE, than llh altitude is already relative to origin
        if (altConv == GEO_ALT_RELATIVE) {
            pos->V.Z = llh->alt;
        } else {
            pos->V.Z = llh->alt - origin->alt;
        }
    }
    else {
        pos->V.X = 0.0f;
        pos->V.Y = 0.0f;
        pos->V.Z = 0.0f;
    }
}

void geoConvertLocalToGeodetic(const gpsOrigin_s * origin, const t_fp_vector * pos, gpsLocation_t * llh)
{
    float scaleLonDown;

    if (origin->valid) {
        llh->lat = origin->lat;
        llh->lon = origin->lon;
        llh->alt = origin->alt;
        scaleLonDown = origin->scale;
    }
    else {
        llh->lat = 0;
        llh->lon = 0;
        llh->alt = 0;
        scaleLonDown = 1.0f;
    }

    llh->lat += lrintf(pos->V.X / DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR);
    llh->lon += lrintf(pos->V.Y / (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * scaleLonDown));
    llh->alt += lrintf(pos->V.Z);
}


#endif  // NAV
