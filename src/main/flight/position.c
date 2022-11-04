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
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#if defined(USE_BARO) || defined(USE_GPS)

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "fc/runtime_config.h"

#include "io/gps.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#include "position.h"

#ifdef USE_GPS
static pt2Filter_t gpsUpsampleLpf;
#endif
#ifdef USE_BARO
static pt2Filter_t baroUpsampleLpf;
#endif

static bool altitudeAvailable = false;

static float altitudeCm = 0.0f;
static float displayAltitudeCm = 0.0f;

static bool wasArmed = false;
static bool isGpsAvailable = false;     // true if GPS is connected and while it has a 3D fix, set each run to false
static bool isBaroAvailable = false;    // true if baro exists and has been calibrated on power up

static float gpsAltCm = 0.0f;           // will hold last value on transient loss of 3D fix
static float gpsAltGroundCm = 0.0f;
static float baroAltCm = 0.0f;

#if defined(USE_GPS) && defined(USE_BARO)
static pt1Filter_t gpsFuseLpf;          // for complementary filter (lowpass part)
static pt1Filter_t baroFuseLpf;         // for complementary filter (highpass part)
static float gpsAltLowpassCm = 0.0f;
static float baroAltHighpassCm = 0.0f;
#endif

#ifdef USE_VARIO
static pt2Filter_t varioLpf;
static float climbRateCmS = 0.0f;
static int16_t estimatedVario = 0;      // in cm/s
#endif

void positionInit(void)
{
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

    float cutoffHz;
    float gain;

#ifdef USE_GPS
    // initialize GPS upsampling filter
    const float gpsRateHz = 1.0f / getGpsDataIntervalSeconds();
    cutoffHz = gpsRateHz / 4.0f;    // -3dB cutoff at nyquist/2
    gain = pt2FilterGain(cutoffHz, sampleTimeS);
    pt2FilterInit(&gpsUpsampleLpf, gain);
#endif

#ifdef USE_BARO
    // initialize baro upsampling filter
    const float baroRateHz = 1.0f / ((baro.dev.up_delay + 1000 * (BARO_STATE_COUNT - 1)) * 1e-6f);  // up_delay + 5x 1000us baro state delay
    cutoffHz = baroRateHz / 4.0f;   // -3dB cutoff at nyquist/2
    gain = pt2FilterGain(cutoffHz, sampleTimeS);
    pt2FilterInit(&baroUpsampleLpf, gain);
#endif

#if defined(USE_GPS) && defined(USE_BARO)
    // initialize complementary filter for fusing GPS and barometer
    cutoffHz = gpsRateHz / 2.0f * positionConfig()->altitude_fuse_ratio / 100.0f;
    gain = pt1FilterGain(cutoffHz, sampleTimeS);
    pt1FilterInit(&baroFuseLpf, gain);
    pt1FilterInit(&gpsFuseLpf, gain);
#endif

#ifdef USE_VARIO
    cutoffHz = positionConfig()->altitude_vario_lpf / 100.0f;
    gain = pt2FilterGain(cutoffHz, sampleTimeS);
    pt2FilterInit(&varioLpf, gain);
#endif
}

void positionUpdate(void)
{
    // ***  UPSAMPLE SENSOR DATA  ***
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        isBaroAvailable = isBaroReady(); // false only if no baro is on board, or if baro init has failed
        baroAltCm = pt2FilterApply(&baroUpsampleLpf, baroGetAltitudeCm());
    }
#endif
#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
        // GPS_FIX means a 3D fix, which requires min. 4 sats
        // On loss of 3D fix (or GPS altogether) hold last altitude value to prevent flying to the moon
        isGpsAvailable = gpsIsHealthy() && STATE(GPS_FIX);
        gpsAltCm = pt2FilterApply(&gpsUpsampleLpf, isGpsAvailable ? gpsSol.llh.altCm : gpsAltCm);
    }
#endif

    //  ***  DISARMED  ***
    if (!ARMING_FLAG(ARMED)) {

        // things to run once, on disarming, after being armed
        if (wasArmed) {
            wasArmed = false;
        }

        if (isGpsAvailable && positionConfig()->altitude_source != ALTITUDE_SOURCE_BARO_ONLY) {
            gpsAltGroundCm = gpsAltCm;      // watch for valid GPS altitude data to zero GPS altitude with
            displayAltitudeCm = gpsAltCm;   // estimatedAltitude shows most recent ASL GPS altitude in OSD and sensors, while disarmed
        } else if (isBaroAvailable && positionConfig()->altitude_source != ALTITUDE_SOURCE_GPS_ONLY) {
            displayAltitudeCm = baroAltCm;
        }

        altitudeCm = 0.0f; // always hold relative altitude at zero while disarmed

    //  ***  ARMED  ***
    } else {

        // things to run once, on arming, after being disarmed
        if (!wasArmed) {
#ifdef USE_BARO
            baroSetGroundLevel();  // zero barometer altitude
#endif
            wasArmed = true;
        }

        switch (positionConfig()->altitude_source) {
            case ALTITUDE_SOURCE_DEFAULT:
#if defined(USE_GPS) && defined(USE_BARO)
                // Fuse GPS and baro for more precise altitude
                if (isGpsAvailable && isBaroAvailable) {  // Complementary filter
                    gpsAltLowpassCm = pt1FilterApply(&gpsFuseLpf, gpsAltCm - gpsAltGroundCm);
                    baroAltHighpassCm = baroAltCm - pt1FilterApply(&baroFuseLpf, baroAltCm);
                    altitudeCm = gpsAltLowpassCm + baroAltHighpassCm;
                } else
#endif
                if (isGpsAvailable && !isBaroAvailable) {  // Fallback option if baro fails
                    altitudeCm = gpsAltCm - gpsAltGroundCm;
                } else if (!isGpsAvailable && isBaroAvailable) {  // Fallback option if GPS fails
                    altitudeCm = baroAltCm;
                }
                break;
            case ALTITUDE_SOURCE_GPS_ONLY:
                altitudeCm = gpsAltCm - gpsAltGroundCm;
                break;
            case ALTITUDE_SOURCE_BARO_ONLY:
                altitudeCm = baroAltCm;
                break;
            default:
                break;
        }
    }

    if (wasArmed) {
        displayAltitudeCm = altitudeCm; // while armed, show filtered relative altitude in OSD / sensors tab
    }

#ifdef USE_VARIO
    // calculate vario signal
    static float prevAltitudeCm = 0.0f;
    climbRateCmS = (altitudeCm - prevAltitudeCm) * TASK_ALTITUDE_RATE_HZ; // cm/s
    prevAltitudeCm = altitudeCm;

    climbRateCmS = pt2FilterApply(&varioLpf, climbRateCmS);

    estimatedVario = lrintf(climbRateCmS);
    estimatedVario = applyDeadband(estimatedVario, 10); // ignore climb rates less than 0.1 m/s
#endif

    // *** set debugs
    DEBUG_SET(DEBUG_ALTITUDE, 0, lrintf(altitudeCm * 0.1f));
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf(baroAltCm * 0.1f));
    DEBUG_SET(DEBUG_ALTITUDE, 2, lrintf(gpsAltCm * 0.1f));
#ifdef USE_VARIO
    DEBUG_SET(DEBUG_ALTITUDE, 3, estimatedVario);
#endif
    DEBUG_SET(DEBUG_RTH, 1, lrintf(displayAltitudeCm * 0.1f));
    DEBUG_SET(DEBUG_BARO, 3, lrintf(baroAltCm * 0.1f));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 2, lrintf(altitudeCm));

    altitudeAvailable = haveGpsAlt || haveBaroAlt;
}

float getAltitudeCm(void)
{
    return altitudeCm;
}

float getAltitudeDerivative(void)
{
    return zeroedAltitudeDerivative; // cm/s
}

bool isAltitudeAvailable(void) {
    return altitudeAvailable;
}

int32_t getEstimatedAltitudeCm(void)
{
    return lrintf(displayAltitudeCm);
}

#ifdef USE_GPS
float getAltitudeAsl(void)
{
    return gpsSol.llh.altCm;
}
#endif

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif

#endif // defined(USE_BARO) || defined(USE_GPS)
