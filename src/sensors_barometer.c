#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "drivers/barometer_common.h"
#include "config.h"

#include "sensors_barometer.h"

baro_t baro;                        // barometer access functions
uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value
int32_t baroPressure = 0;
int32_t baroTemperature = 0;
uint32_t baroPressureSum = 0;
int32_t BaroAlt = 0;

#ifdef BARO

static int32_t baroGroundAltitude = 0;
static int32_t baroGroundPressure = 0;

barometerConfig_t *barometerConfig;

void useBarometerConfig(barometerConfig_t *barometerConfigToUse)
{
    barometerConfig = barometerConfigToUse;
}

bool isBaroCalibrationComplete(void)
{
    return calibratingB == 0;
}

void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingB = calibrationCyclesRequired;
}

static uint32_t recalculateBarometerTotal(uint8_t baroSampleCount, uint32_t pressureTotal, int32_t newPressureReading)
{
    static int32_t barometerSamples[BARO_SAMPLE_COUNT_MAX];
    static int currentSampleIndex = 0;
    int nextSampleIndex;

    // store current pressure in barometerSamples
    nextSampleIndex = (currentSampleIndex + 1);
    if (nextSampleIndex == baroSampleCount) {
        nextSampleIndex = 0;
    }
    barometerSamples[currentSampleIndex] = newPressureReading;

    // recalculate pressure total
    pressureTotal += barometerSamples[currentSampleIndex];
    pressureTotal -= barometerSamples[nextSampleIndex];

    currentSampleIndex = nextSampleIndex;

    return pressureTotal;
}

typedef enum {
    BAROMETER_NEEDS_SAMPLES = 0,
    BAROMETER_NEEDS_CALCULATION
} barometerState_e;

barometerAction_e baroUpdate(uint32_t currentTime)
{
    static uint32_t baroDeadline = 0;
    static barometerState_e state = BAROMETER_NEEDS_SAMPLES;

    if ((int32_t)(currentTime - baroDeadline) < 0)
        return BAROMETER_ACTION_NOT_READY;

    baroDeadline = currentTime;

    switch (state) {
        case BAROMETER_NEEDS_CALCULATION:
            baro.get_up();
            baro.start_ut();
            baroDeadline += baro.ut_delay;
            baro.calculate(&baroPressure, &baroTemperature);
            state = BAROMETER_NEEDS_SAMPLES;
            return BAROMETER_ACTION_PERFORMED_CALCULATION;

        case BAROMETER_NEEDS_SAMPLES:
        default:
            baro.get_ut();
            baro.start_up();
            baroPressureSum = recalculateBarometerTotal(barometerConfig->baro_sample_count, baroPressureSum, baroPressure);
            state = BAROMETER_NEEDS_CALCULATION;
            baroDeadline += baro.up_delay;
            return BAROMETER_ACTION_OBTAINED_SAMPLES;
    }
}

int32_t baroCalculateAltitude(void)
{
    int32_t BaroAlt_tmp;

    // calculates height from ground via baro readings
    // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
    BaroAlt_tmp = lrintf((1.0f - powf((float)(baroPressureSum / (barometerConfig->baro_sample_count - 1)) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
    BaroAlt_tmp -= baroGroundAltitude;
    BaroAlt = lrintf((float)BaroAlt * barometerConfig->baro_noise_lpf + (float)BaroAlt_tmp * (1.0f - barometerConfig->baro_noise_lpf)); // additional LPF to reduce baro noise

    return BaroAlt;
}

void performBaroCalibrationCycle(void)
{
    baroGroundPressure -= baroGroundPressure / 8;
    baroGroundPressure += baroPressureSum / (barometerConfig->baro_sample_count - 1);
    baroGroundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

    calibratingB--;
}

#endif /* BARO */
