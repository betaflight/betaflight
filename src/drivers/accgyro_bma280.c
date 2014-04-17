#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "accgyro_common.h"
#include <sensors_common.h>

#include "accgyro_bma280.h"

#include "bus_i2c.h"

#include "boardalignment.h"


// BMA280, default I2C address mode 0x18
#define BMA280_ADDRESS     0x18
#define BMA280_ACC_X_LSB   0x02
#define BMA280_PMU_BW      0x10
#define BMA280_PMU_RANGE   0x0F

static void bma280Init(sensor_align_e align);
static void bma280Read(int16_t *accelData);

static sensor_align_e accAlign = CW0_DEG;

bool bma280Detect(sensor_t *acc)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(BMA280_ADDRESS, 0x00, 1, &sig);
    if (!ack || sig != 0xFB)
        return false;

    acc->init = bma280Init;
    acc->read = bma280Read;
    return true;
}

static void bma280Init(sensor_align_e align)
{
    i2cWrite(BMA280_ADDRESS, BMA280_PMU_RANGE, 0x08); // +-8g range
    i2cWrite(BMA280_ADDRESS, BMA280_PMU_BW, 0x0E); // 500Hz BW

    acc_1G = 512 * 8;

    if (align > 0)
        accAlign = align;
}

static void bma280Read(int16_t *accelData)
{
    uint8_t buf[6];
    int16_t data[3];

    i2cRead(BMA280_ADDRESS, BMA280_ACC_X_LSB, 6, buf);

    // Data format is lsb<5:0><crap><new_data_bit> | msb<13:6>
    data[0] = (int16_t)((buf[0] >> 2) + (buf[1] << 8));
    data[1] = (int16_t)((buf[2] >> 2) + (buf[3] << 8));
    data[2] = (int16_t)((buf[4] >> 2) + (buf[5] << 8));

    alignSensors(data, accelData, accAlign);
}
