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

#include <platform.h>

#include "build/build_config.h"

#include "drivers/barometer/barometer.h"

#include "drivers/gpio.h"
#include "drivers/time.h"
#include "drivers/bus_i2c.h"
#include "drivers/nvic.h"
#include "drivers/exti.h"
#include "drivers/io.h"

#include "drivers/barometer/barometer_bmp085.h"

#ifdef BARO

#if defined(BARO_EOC_GPIO)

static IO_t eocIO;

static bool isConversionComplete = false;
static bool isEOCConnected = true;

// EXTI14 for BMP085 End of Conversion Interrupt
void bmp085_extiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);
    isConversionComplete = true;
}

bool bmp085TestEOCConnected(const bmp085Config_t *config);
# endif

typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} bmp085_smd500_calibration_param_t;

typedef struct {
    bmp085_smd500_calibration_param_t cal_param;
    uint8_t mode;
    uint8_t chip_id, ml_version, al_version;
    uint8_t dev_addr;
    int32_t param_b5;
    int16_t oversampling_setting;
} bmp085_t;

#define BMP085_I2C_ADDR         0x77
#define BMP085_CHIP_ID          0x55
#define BOSCH_PRESSURE_BMP085   85
#define BMP085_CHIP_ID_REG      0xD0
#define BMP085_VERSION_REG      0xD1
#define E_SENSOR_NOT_DETECTED   (char) 0
#define BMP085_PROM_START__ADDR 0xaa
#define BMP085_PROM_DATA__LEN   22
#define BMP085_T_MEASURE        0x2E                // temperature measurent
#define BMP085_P_MEASURE        0x34                // pressure measurement
#define BMP085_CTRL_MEAS_REG    0xF4
#define BMP085_ADC_OUT_MSB_REG  0xF6
#define BMP085_ADC_OUT_LSB_REG  0xF7
#define BMP085_CHIP_ID__POS     0
#define BMP085_CHIP_ID__MSK     0xFF
#define BMP085_CHIP_ID__LEN     8
#define BMP085_CHIP_ID__REG     BMP085_CHIP_ID_REG

#define BMP085_ML_VERSION__POS      0
#define BMP085_ML_VERSION__LEN      4
#define BMP085_ML_VERSION__MSK      0x0F
#define BMP085_ML_VERSION__REG      BMP085_VERSION_REG

#define BMP085_AL_VERSION__POS      4
#define BMP085_AL_VERSION__LEN      4
#define BMP085_AL_VERSION__MSK      0xF0
#define BMP085_AL_VERSION__REG      BMP085_VERSION_REG

#define BMP085_GET_BITSLICE(regvar, bitname) (regvar & bitname##__MSK) >> bitname##__POS
#define BMP085_SET_BITSLICE(regvar, bitname, val) (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

#define SMD500_PARAM_MG      3038        //calibration parameter
#define SMD500_PARAM_MH     -7357        //calibration parameter
#define SMD500_PARAM_MI      3791        //calibration parameter

STATIC_UNIT_TESTED bmp085_t bmp085;

#define UT_DELAY    6000        // 1.5ms margin according to the spec (4.5ms T conversion time)
#define UP_DELAY    27000       // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P conversion time with OSS=3)

static bool bmp085InitDone = false;
STATIC_UNIT_TESTED uint16_t bmp085_ut;  // static result of temperature measurement
STATIC_UNIT_TESTED uint32_t bmp085_up;  // static result of pressure measurement

static void bmp085_get_cal_param(void);
static void bmp085_start_ut(void);
static void bmp085_get_ut(void);
static void bmp085_start_up(void);
static void bmp085_get_up(void);
static int32_t bmp085_get_temperature(uint32_t ut);
static int32_t bmp085_get_pressure(uint32_t up);
STATIC_UNIT_TESTED void bmp085_calculate(int32_t *pressure, int32_t *temperature);

static IO_t xclrIO;

#ifdef BARO_XCLR_PIN
#define BMP085_OFF  IOLo(xclrIO);
#define BMP085_ON   IOHi(xclrIO);
#else
#define BMP085_OFF
#define BMP085_ON
#endif


void bmp085InitXclrIO(const bmp085Config_t *config)
{
    if (!xclrIO && config && config->xclrIO) {
        xclrIO = IOGetByTag(config->xclrIO);
        IOInit(xclrIO, OWNER_BARO, RESOURCE_OUTPUT, 0);
        IOConfigGPIO(xclrIO, IOCFG_OUT_PP);
    }
}

void bmp085Disable(const bmp085Config_t *config)
{
    bmp085InitXclrIO(config);
    BMP085_OFF;
}

#define DETECTION_MAX_RETRY_COUNT   5
bool bmp085Detect(const bmp085Config_t *config, baroDev_t *baro)
{
    uint8_t data;
    bool ack;
#if defined(BARO_EOC_GPIO)
    IO_t eocIO = IO_NONE;
#endif

    if (bmp085InitDone)
        return true;

    bmp085InitXclrIO(config);
    BMP085_ON;   // enable baro

#if defined(BARO_EOC_GPIO) && defined(USE_EXTI)
    if (config && config->eocIO) {
        eocIO = IOGetByTag(config->eocIO);
        // EXTI interrupt for barometer EOC
        IOInit(eocIO, OWNER_SYSTEM, RESOURCE_INPUT | RESOURCE_EXTI);
        IOConfigGPIO(eocIO, Mode_IN_FLOATING);
        EXTIHandlerInit(&bmp085_extiCallbackRec, bmp085_extiHandler);
        EXTIConfig(eocIO, &bmp085_extiCallbackRec, NVIC_PRIO_BARO_EXTI, EXTI_Trigger_Rising);
        EXTIEnable(eocIO, true);
    }
#else
    UNUSED(config);
#endif

    delay(20); // datasheet says 10ms, we'll be careful and do 20.

    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        ack = i2cRead(BARO_I2C_INSTANCE, BMP085_I2C_ADDR, BMP085_CHIP_ID__REG, 1, &data); /* read Chip Id */
        if (ack) {
            bmp085.chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);
            bmp085.oversampling_setting = 3;

            if (bmp085.chip_id == BMP085_CHIP_ID) { /* get bitslice */
                i2cRead(BARO_I2C_INSTANCE, BMP085_I2C_ADDR, BMP085_VERSION_REG, 1, &data); /* read Version reg */
                bmp085.ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION); /* get ML Version */
                bmp085.al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION); /* get AL Version */
                bmp085_get_cal_param(); /* readout bmp085 calibparam structure */
                baro->ut_delay = UT_DELAY;
                baro->up_delay = UP_DELAY;
                baro->start_ut = bmp085_start_ut;
                baro->get_ut = bmp085_get_ut;
                baro->start_up = bmp085_start_up;
                baro->get_up = bmp085_get_up;
                baro->calculate = bmp085_calculate;
#if defined(BARO_EOC_GPIO)
                isEOCConnected = bmp085TestEOCConnected(config);
#endif
                bmp085InitDone = true;
                return true;
            }
        }
    }

#if defined(BARO_EOC_GPIO)
    if (eocIO)
        EXTIRelease(eocIO);
#endif

    BMP085_OFF;

    return false;
}

static int32_t bmp085_get_temperature(uint32_t ut)
{
    int32_t temperature;
    int32_t x1, x2;

    x1 = (((int32_t) ut - (int32_t) bmp085.cal_param.ac6) * (int32_t) bmp085.cal_param.ac5) >> 15;
    x2 = ((int32_t) bmp085.cal_param.mc << 11) / (x1 + bmp085.cal_param.md);
    bmp085.param_b5 = x1 + x2;
    temperature = ((bmp085.param_b5 * 10 + 8) >> 4);  // temperature in 0.01 C (make same as MS5611)

    return temperature;
}

static int32_t bmp085_get_pressure(uint32_t up)
{
    int32_t pressure, x1, x2, x3, b3, b6;
    uint32_t b4, b7;

    b6 = bmp085.param_b5 - 4000;
    // *****calculate B3************
    x1 = (b6 * b6) >> 12;
    x1 *= bmp085.cal_param.b2;
    x1 >>= 11;

    x2 = (bmp085.cal_param.ac2 * b6);
    x2 >>= 11;

    x3 = x1 + x2;

    b3 = (((((int32_t) bmp085.cal_param.ac1) * 4 + x3) << bmp085.oversampling_setting) + 2) >> 2;

    // *****calculate B4************
    x1 = (bmp085.cal_param.ac3 * b6) >> 13;
    x2 = (bmp085.cal_param.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (bmp085.cal_param.ac4 * (uint32_t)(x3 + 32768)) >> 15;

    b7 = ((uint32_t)(up - b3) * (50000 >> bmp085.oversampling_setting));
    if (b7 < 0x80000000) {
        pressure = (b7 << 1) / b4;
    } else {
        pressure = (b7 / b4) << 1;
    }

    x1 = pressure >> 8;
    x1 *= x1;
    x1 = (x1 * SMD500_PARAM_MG) >> 16;
    x2 = (pressure * SMD500_PARAM_MH) >> 16;
    pressure += (x1 + x2 + SMD500_PARAM_MI) >> 4;   // pressure in Pa

    return pressure;
}

static void bmp085_start_ut(void)
{
#if defined(BARO_EOC_GPIO)
    isConversionComplete = false;
#endif
    i2cWrite(BARO_I2C_INSTANCE, BMP085_I2C_ADDR, BMP085_CTRL_MEAS_REG, BMP085_T_MEASURE);
}

static void bmp085_get_ut(void)
{
    uint8_t data[2];

#if defined(BARO_EOC_GPIO)
    // return old baro value if conversion time exceeds datasheet max when EOC is connected
    if ((isEOCConnected) && (!isConversionComplete)) {
        return;
    }
#endif

    i2cRead(BARO_I2C_INSTANCE, BMP085_I2C_ADDR, BMP085_ADC_OUT_MSB_REG, 2, data);
    bmp085_ut = (data[0] << 8) | data[1];
}

static void bmp085_start_up(void)
{
    uint8_t ctrl_reg_data;

    ctrl_reg_data = BMP085_P_MEASURE + (bmp085.oversampling_setting << 6);

#if defined(BARO_EOC_GPIO)
    isConversionComplete = false;
#endif

    i2cWrite(BARO_I2C_INSTANCE, BMP085_I2C_ADDR, BMP085_CTRL_MEAS_REG, ctrl_reg_data);
}

/** read out up for pressure conversion
 depending on the oversampling ratio setting up can be 16 to 19 bit
 \return up parameter that represents the uncompensated pressure value
 */
static void bmp085_get_up(void)
{
    uint8_t data[3];

#if defined(BARO_EOC_GPIO)
    // return old baro value if conversion time exceeds datasheet max when EOC is connected
    if ((isEOCConnected) && (!isConversionComplete)) {
        return;
    }
#endif

    i2cRead(BARO_I2C_INSTANCE, BMP085_I2C_ADDR, BMP085_ADC_OUT_MSB_REG, 3, data);
    bmp085_up = (((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[2])
            >> (8 - bmp085.oversampling_setting);
}

STATIC_UNIT_TESTED void bmp085_calculate(int32_t *pressure, int32_t *temperature)
{
    int32_t temp, press;

    temp = bmp085_get_temperature(bmp085_ut);
    press = bmp085_get_pressure(bmp085_up);
    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
}

static void bmp085_get_cal_param(void)
{
    uint8_t data[BMP085_PROM_DATA__LEN];
    i2cRead(BARO_I2C_INSTANCE, BMP085_I2C_ADDR, BMP085_PROM_START__ADDR, BMP085_PROM_DATA__LEN, data);

    /*parameters AC1-AC6*/
    bmp085.cal_param.ac1 = (data[0] << 8) | data[1];
    bmp085.cal_param.ac2 = (data[2] << 8) | data[3];
    bmp085.cal_param.ac3 = (data[4] << 8) | data[5];
    bmp085.cal_param.ac4 = (data[6] << 8) | data[7];
    bmp085.cal_param.ac5 = (data[8] << 8) | data[9];
    bmp085.cal_param.ac6 = (data[10] << 8) | data[11];

    /*parameters B1,B2*/
    bmp085.cal_param.b1 = (data[12] << 8) | data[13];
    bmp085.cal_param.b2 = (data[14] << 8) | data[15];

    /*parameters MB,MC,MD*/
    bmp085.cal_param.mb = (data[16] << 8) | data[17];
    bmp085.cal_param.mc = (data[18] << 8) | data[19];
    bmp085.cal_param.md = (data[20] << 8) | data[21];
}

#if defined(BARO_EOC_GPIO)
bool bmp085TestEOCConnected(const bmp085Config_t *config)
{
    UNUSED(config);

    if (!bmp085InitDone && eocIO) {
        bmp085_start_ut();
        delayMicroseconds(UT_DELAY * 2); // wait twice as long as normal, just to be sure

        // conversion should have finished now so check if EOC is high
        uint8_t status = IORead(eocIO);
        if (status) {
            return true;
        }
    }
    return false; // assume EOC is not connected
}
#endif

#endif /* BARO */
