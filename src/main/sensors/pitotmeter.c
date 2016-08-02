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

#include "common/maths.h"

#include "drivers/pitotmeter.h"
#include "config/config.h"

#include "sensors/pitotmeter.h"

extern int16_t debug[4];

pitot_t pitot;                  // pitotmeter access functions
int32_t AirSpeed = 0;

#ifdef PITOT

static float pitotPressureSum = 0;
static uint16_t calibratingP = 0;      // baro calibration = get new ground pressure value
static float pitotPressure = 0;
static float pitotTemperature = 0;
static float CalibratedAirspeed = 0;

pitotmeterConfig_t *pitotmeterConfig;

void usePitotmeterConfig(pitotmeterConfig_t *pitotmeterConfigToUse)
{
    pitotmeterConfig = pitotmeterConfigToUse;
}

bool isPitotCalibrationComplete(void)
{
    return calibratingP == 0;
}

void pitotSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingP = calibrationCyclesRequired;
}

static bool pitotReady = false;

#define PRESSURE_SAMPLE_COUNT (pitotmeterConfig->pitot_sample_count - 1)

static float recalculatePitotmeterTotal(uint8_t pitotSampleCount, float pressureTotal, float newPressureReading)
{
    static float pitotmeterSamples[PITOT_SAMPLE_COUNT_MAX];
    static int currentSampleIndex = 0;
    int nextSampleIndex;

    // store current pressure in pitotmeterSamples
    nextSampleIndex = (currentSampleIndex + 1);
    if (nextSampleIndex == pitotSampleCount) {
        nextSampleIndex = 0;
        pitotReady = true;
    }
    pitotmeterSamples[currentSampleIndex] = newPressureReading;

    // recalculate pressure total
    // Note, the pressure total is made up of pitotSampleCount - 1 samples - See PRESSURE_SAMPLE_COUNT
    pressureTotal += pitotmeterSamples[currentSampleIndex];
    pressureTotal -= pitotmeterSamples[nextSampleIndex];

    currentSampleIndex = nextSampleIndex;

    return pressureTotal;
}

typedef enum {
    PITOTMETER_NEEDS_SAMPLES = 0,
    PITOTMETER_NEEDS_CALCULATION,
    PITOTMETER_NEEDS_PROCESSING
} pitotmeterState_e;


bool isPitotReady(void) {
    return pitotReady;
}

uint32_t pitotUpdate(void)
{
    static pitotmeterState_e state = PITOTMETER_NEEDS_SAMPLES;

    switch (state) {
        default:
        case PITOTMETER_NEEDS_SAMPLES:
            pitot.start();
            state = PITOTMETER_NEEDS_CALCULATION;
            return pitot.delay;
        break;

        case PITOTMETER_NEEDS_CALCULATION:
            pitot.get();
            pitot.calculate(&pitotPressure, &pitotTemperature);
            state = PITOTMETER_NEEDS_PROCESSING;
            return pitot.delay;
        break;

        case PITOTMETER_NEEDS_PROCESSING:
            state = PITOTMETER_NEEDS_SAMPLES;
            pitotPressureSum = recalculatePitotmeterTotal(pitotmeterConfig->pitot_sample_count, pitotPressureSum, pitotPressure);
            return pitot.delay;
        break;
    }
}

#define P0          101325.0f           // standard pressure
#define CCEXPONENT  0.2857142857f       // exponent of compressibility correction 2/7
#define CASFACTOR   760.8802669f        // sqrt(5) * speed of sound at standard
#define TASFACTOR   0.05891022589f      // 1/sqrt(T0)

int32_t pitotCalculateAirSpeed(void)
{
    float CalibratedAirspeed_tmp;
    CalibratedAirspeed_tmp = pitotmeterConfig->pitot_scale * CASFACTOR * sqrtf(powf(fabsf(pitotPressureSum / PRESSURE_SAMPLE_COUNT) / P0 + 1.0f, CCEXPONENT) - 1.0f);
    //const float CalibratedAirspeed = 1 * CASFACTOR * sqrtf(powf(fabsf(pitotPressure) / P0 + 1.0f, CCEXPONENT) - 1.0f);

    CalibratedAirspeed = CalibratedAirspeed * pitotmeterConfig->pitot_noise_lpf + CalibratedAirspeed_tmp * (1.0f - pitotmeterConfig->pitot_noise_lpf); // additional LPF to reduce baro noise
    float TrueAirspeed = CalibratedAirspeed * TASFACTOR * sqrtf(pitotTemperature);

    AirSpeed = TrueAirspeed*100;
    //debug[0] = (int16_t)(CalibratedAirspeed*100);
    //debug[1] = (int16_t)(TrueAirspeed*100);
    //debug[2] = (int16_t)((pitotTemperature-273.15f)*100);
    debug[3] = AirSpeed;

    return AirSpeed;
}

#endif /* PITOT */
