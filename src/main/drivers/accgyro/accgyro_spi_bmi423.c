/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_BMI423

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi423.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#define BMI423_MAX_SPI_CLK_HZ 10000000
#define BMI423_CHIP_ID 0xAB

#define BMI423_READ_DUMMY_BYTES 2
#define BMI423_SENSOR_DATA_BYTES 6
#define BMI423_REGISTER_READ_BYTES (BMI423_READ_DUMMY_BYTES + sizeof(uint16_t))
#define BMI423_SENSOR_READ_BYTES (1 + BMI423_READ_DUMMY_BYTES + BMI423_SENSOR_DATA_BYTES)
#define BMI423_DMA_READ_BYTES (1 + BMI423_READ_DUMMY_BYTES + (2 * BMI423_SENSOR_DATA_BYTES))
#define BMI423_PAYLOAD_OFFSET (1 + BMI423_READ_DUMMY_BYTES)

typedef enum {
    BMI423_REG_CHIP_ID = 0x00,
    BMI423_REG_FILTER_CONF = 0x25,
    BMI423_REG_ACC_CONF = 0x26,
    BMI423_REG_GYR_CONF = 0x27,
    BMI423_REG_IO_INT_CTRL = 0x33,
    BMI423_REG_INT_CONF = 0x34,
    BMI423_REG_INT_MAP_HW = 0x35,
    BMI423_REG_LEG_ACC_DATA_X = 0x56,
    BMI423_REG_LEG_GYR_DATA_X = 0x59,
    BMI423_REG_LEGACY_CONF = 0x7B,
    BMI423_REG_CMD = 0x7E,
} bmi423Register_e;

typedef enum {
    BMI423_VAL_FILTER_CONF_ACC_IIR2 = 0x0002, // second-order accelerometer IIR filter
    BMI423_VAL_FILTER_CONF_GYR_IIR2 = 0x0200, // second-order gyroscope IIR filter
    BMI423_VAL_ACC_CONF = 0xE03B,         // high-performance, 800Hz, 16g
    BMI423_VAL_GYR_CONF_BASE = 0xE04E,    // high-performance, 6.4kHz, 2000dps
    BMI423_VAL_IO_INT_CTRL = 0x0005,      // INT1 active-high, push-pull, output enabled
    BMI423_VAL_INT_CONF = 0x0003,         // INT1 long-pulse mode (39us typical)
    BMI423_VAL_INT_MAP_HW = 0x0004,       // gyroscope data-ready mapped to INT1
    BMI423_VAL_LEGACY_CONF = 0x0001,      // use 16-bit accelerometer and gyroscope samples
    BMI423_VAL_CMD_SOFTRESET = 0xDEAF,
} bmi423ConfigValue_e;

typedef enum {
    BMI423_GYR_BW_AUTO = 0,
    BMI423_GYR_BW_200HZ = 4,
    BMI423_GYR_BW_400HZ = 5,
    BMI423_GYR_BW_800HZ = 6,
} bmi423GyroBandwidth_e;

#define BMI423_GYR_BW_SHIFT 7

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity.
#define GYRO_EXTI_DETECT_THRESHOLD 1000

static bool bmi423RegisterRead(const extDevice_t *dev, bmi423Register_e registerId, uint16_t *value)
{
    uint8_t data[BMI423_REGISTER_READ_BYTES] = { 0 };

    if (!spiReadRegMskBufRB(dev, registerId, data, sizeof(data))) {
        return false;
    }

    *value = (uint16_t)data[BMI423_READ_DUMMY_BYTES]
        | ((uint16_t)data[BMI423_READ_DUMMY_BYTES + 1] << 8);
    return true;
}

static void bmi423RegisterWrite(const extDevice_t *dev, bmi423Register_e registerId, uint16_t value, unsigned delayMs)
{
    uint8_t data[sizeof(value)] = {
        value & 0xFF,
        value >> 8,
    };

    spiWriteRegBuf(dev, registerId, data, sizeof(data));
    if (delayMs) {
        delay(delayMs);
    }
}

// The device powers up in I2C mode. A CS rising edge followed by an initial
// register read selects SPI; the first read result must be discarded.
static void bmi423EnableSPI(const extDevice_t *dev)
{
    uint16_t discard;

    IOLo(dev->busType_u.spi.csnPin);
    delay(1);
    IOHi(dev->busType_u.spi.csnPin);
    delay(2);
    bmi423RegisterRead(dev, BMI423_REG_CHIP_ID, &discard);
}

uint8_t bmi423Detect(const extDevice_t *dev)
{
    uint16_t chipId;

    bmi423EnableSPI(dev);

    if (bmi423RegisterRead(dev, BMI423_REG_CHIP_ID, &chipId) && (uint8_t)chipId == BMI423_CHIP_ID) {
        return BMI_423_SPI;
    }

    return MPU_NONE;
}

static uint16_t bmi423GyroBandwidth(const gyroHardwareLpf_e hardwareLpf)
{
    switch (hardwareLpf) {
    case GYRO_HARDWARE_LPF_NORMAL:
        return BMI423_GYR_BW_200HZ;
    case GYRO_HARDWARE_LPF_OPTION_1:
        return BMI423_GYR_BW_400HZ;
    case GYRO_HARDWARE_LPF_OPTION_2:
        return BMI423_GYR_BW_800HZ;
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    case GYRO_HARDWARE_LPF_EXPERIMENTAL:
        // Bandwidth is ignored when the gyroscope IIR filter is bypassed.
        return BMI423_GYR_BW_AUTO;
#endif
    default:
        return BMI423_GYR_BW_200HZ;
    }
}

static uint16_t bmi423FilterConfig(const gyroHardwareLpf_e hardwareLpf)
{
    uint16_t filterConfig = BMI423_VAL_FILTER_CONF_ACC_IIR2 | BMI423_VAL_FILTER_CONF_GYR_IIR2;

#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    if (hardwareLpf == GYRO_HARDWARE_LPF_EXPERIMENTAL) {
        // FILTER_CONF.gyr_filter_type = 0 bypasses the gyroscope IIR filter.
        filterConfig &= ~BMI423_VAL_FILTER_CONF_GYR_IIR2;
    }
#else
    UNUSED(hardwareLpf);
#endif

    return filterConfig;
}

static void bmi423Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    bmi423RegisterWrite(dev, BMI423_REG_CMD, BMI423_VAL_CMD_SOFTRESET, 2);
    bmi423EnableSPI(dev);

    // Betaflight consumes signed 16-bit samples, so select the sensor's legacy
    // representation before configuring the 16g/2000dps ranges.
    bmi423RegisterWrite(dev, BMI423_REG_LEGACY_CONF, BMI423_VAL_LEGACY_CONF, 1);

    const uint16_t filterConfig = bmi423FilterConfig(gyro->hardware_lpf);

    // FILTER_CONF settings are latched separately by the next write to each
    // sensor's configuration register (BMI423 datasheet section 5.6.3).
    bmi423RegisterWrite(dev, BMI423_REG_FILTER_CONF, filterConfig, 1);
    bmi423RegisterWrite(dev, BMI423_REG_ACC_CONF, BMI423_VAL_ACC_CONF, 1);
    bmi423RegisterWrite(dev, BMI423_REG_FILTER_CONF, filterConfig, 1);
    bmi423RegisterWrite(dev, BMI423_REG_GYR_CONF,
        BMI423_VAL_GYR_CONF_BASE | (bmi423GyroBandwidth(gyro->hardware_lpf) << BMI423_GYR_BW_SHIFT), 55);

    bmi423RegisterWrite(dev, BMI423_REG_IO_INT_CTRL, BMI423_VAL_IO_INT_CTRL, 1);
    bmi423RegisterWrite(dev, BMI423_REG_INT_CONF, BMI423_VAL_INT_CONF, 1);
    bmi423RegisterWrite(dev, BMI423_REG_INT_MAP_HW, BMI423_VAL_INT_MAP_HW, 1);
}

static void bmi423DecodeSample(const uint8_t *data, int16_t sample[XYZ_AXIS_COUNT])
{
    sample[X] = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    sample[Y] = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
    sample[Z] = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
}

extiCallbackRec_t bmi423IntCallbackRec;

#ifdef USE_DMA
static busStatus_e bmi423IntCallback(uintptr_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    const int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;
    return BUS_READY;
}
#endif

static void bmi423ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    extDevice_t *dev = &gyro->dev;
    const uint32_t nowCycles = getCycleCounter();

    gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    gyro->gyroLastEXTI = nowCycles;

    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(dev, gyro->segments);
    }

    gyro->detectedEXTI++;
}

static void bmi423IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi423ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}

static bool bmi423AccRead(accDev_t *acc)
{
    extDevice_t *dev = &acc->gyro->dev;

    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        memset(dev->txBuf, 0, BMI423_SENSOR_READ_BYTES);
        dev->txBuf[0] = BMI423_REG_LEG_ACC_DATA_X | 0x80;

        busSegment_t segments[] = {
            { .u.buffers = { dev->txBuf, dev->rxBuf }, BMI423_SENSOR_READ_BYTES, true, NULL },
            { .u.link = { NULL, NULL }, 0, true, NULL },
        };

        spiSequence(dev, segments);
        spiWait(dev);
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
        bmi423DecodeSample(&dev->rxBuf[BMI423_PAYLOAD_OFFSET], acc->ADCRaw);
        break;

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

static bool bmi423GyroRead(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
        memset(dev->txBuf, 0, BMI423_DMA_READ_BYTES);
        gyro->gyroDmaMaxDuration = 5;

        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
#ifdef USE_DMA
            if (spiUseDMA(dev)) {
                dev->callbackArg = (uintptr_t)gyro;
                dev->txBuf[0] = BMI423_REG_LEG_ACC_DATA_X | 0x80;
                gyro->segments[0].len = BMI423_DMA_READ_BYTES;
                gyro->segments[0].callback = bmi423IntCallback;
                gyro->segments[0].u.buffers.txData = dev->txBuf;
                gyro->segments[0].u.buffers.rxData = dev->rxBuf;
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else
#endif
            {
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        memset(dev->txBuf, 0, BMI423_SENSOR_READ_BYTES);
        dev->txBuf[0] = BMI423_REG_LEG_GYR_DATA_X | 0x80;

        busSegment_t segments[] = {
            { .u.buffers = { dev->txBuf, dev->rxBuf }, BMI423_SENSOR_READ_BYTES, true, NULL },
            { .u.link = { NULL, NULL }, 0, true, NULL },
        };

        spiSequence(dev, segments);
        spiWait(dev);
        bmi423DecodeSample(&dev->rxBuf[BMI423_PAYLOAD_OFFSET], gyro->gyroADCRaw);
        break;
    }

    case GYRO_EXTI_INT_DMA:
        bmi423DecodeSample(&dev->rxBuf[BMI423_PAYLOAD_OFFSET + BMI423_SENSOR_DATA_BYTES], gyro->gyroADCRaw);
        break;

    default:
        break;
    }

    return true;
}

static void bmi423SpiGyroInit(gyroDev_t *gyro)
{
    bmi423Config(gyro);
    bmi423IntExtiInit(gyro);
    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(BMI423_MAX_SPI_CLK_HZ));
}

static void bmi423SpiAccInit(accDev_t *acc)
{
    // Sensor configuration is shared and performed during gyro initialisation.
    acc->acc_1G = 2048;
}

bool bmi423SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_423_SPI) {
        return false;
    }

    acc->initFn = bmi423SpiAccInit;
    acc->readFn = bmi423AccRead;
    return true;
}

bool bmi423SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_423_SPI) {
        return false;
    }

    gyro->initFn = bmi423SpiGyroInit;
    gyro->readFn = bmi423GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;
    return true;
}

#endif // USE_ACCGYRO_BMI423
