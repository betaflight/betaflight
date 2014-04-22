#include "board.h"
#include "mw.h"

#include "drivers/barometer_common.h"

baro_t baro;                        // barometer access functions

#ifdef BARO
uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value

static int32_t baroGroundAltitude = 0;
static int32_t baroGroundPressure = 0;

void Baro_SetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingB = calibrationCyclesRequired;
}

void Baro_Common(void)
{
    static int32_t baroHistTab[BARO_TAB_SIZE_MAX];
    static int baroHistIdx;
    int indexplus1;

    indexplus1 = (baroHistIdx + 1);
    if (indexplus1 == currentProfile.baro_tab_size)
        indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;
}


int Baro_update(void)
{
    static uint32_t baroDeadline = 0;
    static int state = 0;

    if ((int32_t)(currentTime - baroDeadline) < 0)
        return 0;

    baroDeadline = currentTime;

    if (state) {
        baro.get_up();
        baro.start_ut();
        baroDeadline += baro.ut_delay;
        baro.calculate(&baroPressure, &baroTemperature);
        state = 0;
        return 2;
    } else {
        baro.get_ut();
        baro.start_up();
        Baro_Common();
        state = 1;
        baroDeadline += baro.up_delay;
        return 1;
    }
}

int32_t Baro_calculateAltitude(void)
{
    int32_t BaroAlt_tmp;

    // calculates height from ground via baro readings
    // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
    BaroAlt_tmp = lrintf((1.0f - powf((float)(baroPressureSum / (currentProfile.baro_tab_size - 1)) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
    BaroAlt_tmp -= baroGroundAltitude;
    BaroAlt = lrintf((float)BaroAlt * currentProfile.baro_noise_lpf + (float)BaroAlt_tmp * (1.0f - currentProfile.baro_noise_lpf)); // additional LPF to reduce baro noise

    return BaroAlt;
}

void performBaroCalibrationCycle(void)
{
    baroGroundPressure -= baroGroundPressure / 8;
    baroGroundPressure += baroPressureSum / (currentProfile.baro_tab_size - 1);
    baroGroundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

    calibratingB--;
}

bool isBaroCalibrationComplete(void)
{
    return calibratingB == 0;
}
#endif /* BARO */
