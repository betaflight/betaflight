#pragma once

#define BARO_SAMPLE_COUNT_MAX   48

typedef struct barometerConfig_s {
    uint8_t baro_sample_count;                  // size of baro filter array
    float baro_noise_lpf;                   // additional LPF to reduce baro noise
    float baro_cf_vel;                      // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
    float baro_cf_alt;                      // apply CF to use ACC for height estimation
} barometerConfig_t;

typedef enum {
    BAROMETER_ACTION_NOT_READY = 0,
    BAROMETER_ACTION_OBTAINED_SAMPLES,
    BAROMETER_ACTION_PERFORMED_CALCULATION
} barometerAction_e;

extern int32_t BaroAlt;

#ifdef BARO
void useBarometerConfig(barometerConfig_t *barometerConfigToUse);
bool isBaroCalibrationComplete(void);
void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
barometerAction_e baroUpdate(uint32_t currentTime);
int32_t baroCalculateAltitude(void);
void performBaroCalibrationCycle(void);
#endif
