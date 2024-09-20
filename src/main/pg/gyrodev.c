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

#include "platform.h"

#include "common/sensor_alignment.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/gyrodev.h"

#include "drivers/io.h"
#include "drivers/bus_spi.h"
#include "drivers/sensor.h"
#include "sensors/gyro.h"

#ifndef GYRO_1_CS_PIN
#define GYRO_1_CS_PIN NONE
#endif

#ifndef GYRO_1_EXTI_PIN
#define GYRO_1_EXTI_PIN NONE
#endif

#ifndef GYRO_2_CS_PIN
#define GYRO_2_CS_PIN NONE
#endif

#ifndef GYRO_2_EXTI_PIN
#define GYRO_2_EXTI_PIN NONE
#endif

#ifndef GYRO_CLKIN_PIN
#define GYRO_CLKIN_PIN NONE
#endif

#ifdef MPU_ADDRESS
#define GYRO_I2C_ADDRESS MPU_ADDRESS
#else
 // 0 == AUTO
#define GYRO_I2C_ADDRESS 0
#endif

// gyro alignments

#ifndef GYRO_1_ALIGN
#define GYRO_1_ALIGN            CW0_DEG
#endif

#ifndef GYRO_2_ALIGN
#define GYRO_2_ALIGN            CW0_DEG
#endif

#if defined(USE_SPI_GYRO) && (defined(GYRO_1_SPI_INSTANCE) || defined(GYRO_2_SPI_INSTANCE))
static void gyroResetSpiDeviceConfig(gyroDeviceConfig_t *devconf, SPI_TypeDef *instance, ioTag_t csnTag, ioTag_t extiTag, uint8_t alignment, sensorAlignment_t customAlignment)
{
    devconf->busType = BUS_TYPE_SPI;
    devconf->spiBus = SPI_DEV_TO_CFG(spiDeviceByInstance(instance));
    devconf->csnTag = csnTag;
    devconf->extiTag = extiTag;
    devconf->alignment = alignment;
    devconf->customAlignment = customAlignment;
    devconf->clkIn = IO_TAG(GYRO_CLKIN_PIN);
}
#endif

#if defined(USE_I2C_GYRO) && !defined(USE_MULTI_GYRO)
static void gyroResetI2cDeviceConfig(gyroDeviceConfig_t *devconf, I2CDevice i2cbus, ioTag_t extiTag, uint8_t alignment, sensorAlignment_t customAlignment)
{
    devconf->busType = BUS_TYPE_I2C;
    devconf->i2cBus = I2C_DEV_TO_CFG(i2cbus);
    devconf->i2cAddress = GYRO_I2C_ADDRESS;
    devconf->extiTag = extiTag;
    devconf->alignment = alignment;
    devconf->customAlignment = customAlignment;
}
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(gyroDeviceConfig_t, MAX_GYRODEV_COUNT, gyroDeviceConfig, PG_GYRO_DEVICE_CONFIG, 0);

void pgResetFn_gyroDeviceConfig(gyroDeviceConfig_t *devconf)
{
    devconf[0].index = 0;
    sensorAlignment_t customAlignment1 = CUSTOM_ALIGN_CW0_DEG;
#ifdef GYRO_1_CUSTOM_ALIGN
    customAlignment1 = GYRO_1_CUSTOM_ALIGN;
#else
    buildAlignmentFromStandardAlignment(&customAlignment1, GYRO_1_ALIGN);
#endif // GYRO_1_CUSTOM_ALIGN

    // All multi-gyro boards use SPI based gyros.
#ifdef USE_SPI_GYRO
#ifdef GYRO_1_SPI_INSTANCE
    gyroResetSpiDeviceConfig(&devconf[0], GYRO_1_SPI_INSTANCE, IO_TAG(GYRO_1_CS_PIN), IO_TAG(GYRO_1_EXTI_PIN), GYRO_1_ALIGN, customAlignment1);
#else
    devconf[0].busType = BUS_TYPE_NONE;
#endif
#ifdef USE_MULTI_GYRO
    devconf[1].index = 1;
    sensorAlignment_t customAlignment2 = CUSTOM_ALIGN_CW0_DEG;
#ifdef GYRO_2_CUSTOM_ALIGN
    customAlignment2 = GYRO_2_CUSTOM_ALIGN;
#else
    buildAlignmentFromStandardAlignment(&customAlignment2, GYRO_2_ALIGN);
#endif // GYRO_2_CUSTOM_ALIGN

#ifdef GYRO_2_SPI_INSTANCE
    gyroResetSpiDeviceConfig(&devconf[1], GYRO_2_SPI_INSTANCE, IO_TAG(GYRO_2_CS_PIN), IO_TAG(GYRO_2_EXTI_PIN), GYRO_2_ALIGN, customAlignment2);
#else
    devconf[1].busType = BUS_TYPE_NONE;
#endif

#endif // USE_MULTI_GYRO
#endif // USE_SPI_GYRO

    // I2C gyros appear as a sole gyro in single gyro boards.
#if defined(USE_I2C_GYRO) && !defined(USE_MULTI_GYRO)
    devconf[0].i2cBus = I2C_DEV_TO_CFG(I2CINVALID); // XXX Not required?
    gyroResetI2cDeviceConfig(&devconf[0], I2C_DEVICE, IO_TAG(GYRO_1_EXTI_PIN), GYRO_1_ALIGN, customAlignment1);
#endif
}
