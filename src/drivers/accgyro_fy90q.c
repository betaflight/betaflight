#ifdef FY90Q

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "accgyro_common.h"
#include "sensors_common.h"

#include "system_common.h"

#include "adc_fy90q.h"

static void adcAccRead(int16_t *accelData);
static void adcGyroRead(int16_t *gyroData);
static void adcDummyInit(sensor_align_e align);

void adcSensorInit(sensor_t *acc, sensor_t *gyro)
{
    acc->init = adcDummyInit;
    acc->read = adcAccRead;

    gyro->init = adcDummyInit;
    gyro->read = adcGyroRead;
    gyro->scale = 1.0f;

    acc_1G = 376;
}

static void adcAccRead(int16_t *accelData)
{
    // ADXL335
    // 300mV/g
    // Vcc 3.0V
    accelData[0] = adcData[3];
    accelData[1] = adcData[4];
    accelData[2] = adcData[5];
}

static void adcGyroRead(int16_t *gyroData)
{
    // Vcc: 3.0V
    // Pitch/Roll: LPR550AL, 2000dps mode.
    // 0.5mV/dps
    // Zero-rate: 1.23V
    // Yaw: LPY550AL, 2000dps mode.
    // 0.5mV/dps
    // Zero-rate: 1.23V

    // Need to match with: 14.375lsb per dps
    // 12-bit ADC

    gyroData[0] = adcData[0] * 2;
    gyroData[1] = adcData[1] * 2;
    gyroData[2] = adcData[2] * 2;
}

static void adcDummyInit(sensor_align_e align)
{
    // nothing to init here
}

uint16_t adcGetBattery(void)
{
    return 0;
}
#endif
