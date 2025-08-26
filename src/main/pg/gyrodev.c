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
#include "common/sensor_alignment_impl.h"

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

#ifndef GYRO_1_CLKIN_PIN
#define GYRO_1_CLKIN_PIN NONE
#endif

#ifndef GYRO_2_CS_PIN
#define GYRO_2_CS_PIN NONE
#endif

#ifndef GYRO_2_EXTI_PIN
#define GYRO_2_EXTI_PIN NONE
#endif

#ifndef GYRO_2_CLKIN_PIN
#define GYRO_2_CLKIN_PIN NONE
#endif

#ifndef GYRO_3_CS_PIN
#define GYRO_3_CS_PIN NONE
#endif

#ifndef GYRO_3_EXTI_PIN
#define GYRO_3_EXTI_PIN NONE
#endif

#ifndef GYRO_3_CLKIN_PIN
#define GYRO_3_CLKIN_PIN NONE
#endif

#ifndef GYRO_4_CS_PIN
#define GYRO_4_CS_PIN NONE
#endif

#ifndef GYRO_4_EXTI_PIN
#define GYRO_4_EXTI_PIN NONE
#endif

#ifndef GYRO_4_CLKIN_PIN
#define GYRO_4_CLKIN_PIN NONE
#endif

#ifdef MPU_ADDRESS
#define GYRO_I2C_ADDRESS MPU_ADDRESS
#else
 // 0 == AUTO
#define GYRO_I2C_ADDRESS 0
#endif

// gyro alignments

#if defined(GYRO_1_ALIGN_ROLL) || defined(GYRO_1_ALIGN_PITCH) || defined(GYRO_1_ALIGN_YAW)
#ifndef GYRO_1_ALIGN_ROLL
#define GYRO_1_ALIGN_ROLL 0
#endif
#ifndef GYRO_1_ALIGN_PITCH
#define GYRO_1_ALIGN_PITCH 0
#endif
#ifndef GYRO_1_ALIGN_YAW
#define GYRO_1_ALIGN_YAW 0
#endif
#ifndef GYRO_1_CUSTOM_ALIGN
#define GYRO_1_CUSTOM_ALIGN     SENSOR_ALIGNMENT( GYRO_1_ALIGN_ROLL / 10, GYRO_1_ALIGN_PITCH / 10, GYRO_1_ALIGN_YAW / 10 )
#else
#error "GYRO_1_ALIGN_x and GYRO_1_CUSTOM_ALIGN are mutually exclusive"
#endif
#endif // GYRO_1_ALIGN_ROLL || GYRO_1_ALIGN_PITCH || GYRO_1_ALIGN_YAW

#if defined(GYRO_2_ALIGN_ROLL) || defined(GYRO_2_ALIGN_PITCH) || defined(GYRO_2_ALIGN_YAW)
#ifndef GYRO_2_ALIGN_ROLL
#define GYRO_2_ALIGN_ROLL 0
#endif
#ifndef GYRO_2_ALIGN_PITCH
#define GYRO_2_ALIGN_PITCH 0
#endif
#ifndef GYRO_2_ALIGN_YAW
#define GYRO_2_ALIGN_YAW 0
#endif
#ifndef GYRO_2_CUSTOM_ALIGN
#define GYRO_2_CUSTOM_ALIGN     SENSOR_ALIGNMENT( GYRO_2_ALIGN_ROLL / 10, GYRO_2_ALIGN_PITCH / 10, GYRO_2_ALIGN_YAW / 10 )
#else
#error "GYRO_2_ALIGN_x and GYRO_2_CUSTOM_ALIGN are mutually exclusive"
#endif
#endif // GYRO_2_ALIGN_ROLL || GYRO_2_ALIGN_PITCH || GYRO_2_ALIGN_YAW

#if defined(GYRO_3_ALIGN_ROLL) || defined(GYRO_3_ALIGN_PITCH) || defined(GYRO_3_ALIGN_YAW)
#ifndef GYRO_3_ALIGN_ROLL
#define GYRO_3_ALIGN_ROLL 0
#endif
#ifndef GYRO_3_ALIGN_PITCH
#define GYRO_3_ALIGN_PITCH 0
#endif
#ifndef GYRO_3_ALIGN_YAW
#define GYRO_3_ALIGN_YAW 0
#endif
#ifndef GYRO_3_CUSTOM_ALIGN
#define GYRO_3_CUSTOM_ALIGN     SENSOR_ALIGNMENT( GYRO_3_ALIGN_ROLL / 10, GYRO_3_ALIGN_PITCH / 10, GYRO_3_ALIGN_YAW / 10 )
#else
#error "GYRO_3_ALIGN_x and GYRO_3_CUSTOM_ALIGN are mutually exclusive"
#endif
#endif // GYRO_3_ALIGN_ROLL || GYRO_3_ALIGN_PITCH || GYRO_3_ALIGN_YAW

#if defined(GYRO_4_ALIGN_ROLL) || defined(GYRO_4_ALIGN_PITCH) || defined(GYRO_4_ALIGN_YAW)
#ifndef GYRO_4_ALIGN_ROLL
#define GYRO_4_ALIGN_ROLL 0
#endif
#ifndef GYRO_4_ALIGN_PITCH
#define GYRO_4_ALIGN_PITCH 0
#endif
#ifndef GYRO_4_ALIGN_YAW
#define GYRO_4_ALIGN_YAW 0
#endif
#ifndef GYRO_4_CUSTOM_ALIGN
#define GYRO_4_CUSTOM_ALIGN     SENSOR_ALIGNMENT( GYRO_4_ALIGN_ROLL / 10, GYRO_4_ALIGN_PITCH / 10, GYRO_4_ALIGN_YAW / 10 )
#else
#error "GYRO_4_ALIGN_x and GYRO_4_CUSTOM_ALIGN are mutually exclusive"
#endif
#endif // GYRO_4_ALIGN_ROLL || GYRO_4_ALIGN_PITCH || GYRO_4_ALIGN_YAW

#ifndef GYRO_1_ALIGN
#ifdef GYRO_1_CUSTOM_ALIGN
#define GYRO_1_ALIGN            ALIGN_CUSTOM
#else
#define GYRO_1_ALIGN            CW0_DEG
#endif
#endif // GYRO_1_ALIGN

#ifndef GYRO_1_CUSTOM_ALIGN
#define GYRO_1_CUSTOM_ALIGN     SENSOR_ALIGNMENT_FROM_STD(GYRO_1_ALIGN)
#else
STATIC_ASSERT(GYRO_1_ALIGN == ALIGN_CUSTOM, "GYRO_1_ALIGN and GYRO_1_CUSTOM_ALIGN mixed");
#endif // GYRO_1_CUSTOM_ALIGN

#ifndef GYRO_2_ALIGN
#ifdef GYRO_2_CUSTOM_ALIGN
#define GYRO_2_ALIGN            ALIGN_CUSTOM
#else
#define GYRO_2_ALIGN            CW0_DEG
#endif
#endif // GYRO_2_ALIGN

#ifndef GYRO_2_CUSTOM_ALIGN
#define GYRO_2_CUSTOM_ALIGN     SENSOR_ALIGNMENT_FROM_STD(GYRO_2_ALIGN)
#else
STATIC_ASSERT(GYRO_2_ALIGN == ALIGN_CUSTOM, "GYRO_2_ALIGN and GYRO_2_CUSTOM_ALIGN mixed");
#endif // GYRO_2_CUSTOM_ALIGN

#ifndef GYRO_3_ALIGN
#ifdef GYRO_3_CUSTOM_ALIGN
#define GYRO_3_ALIGN            ALIGN_CUSTOM
#else
#define GYRO_3_ALIGN            CW0_DEG
#endif
#endif // GYRO_3_ALIGN

#ifndef GYRO_3_CUSTOM_ALIGN
#define GYRO_3_CUSTOM_ALIGN     SENSOR_ALIGNMENT_FROM_STD(GYRO_3_ALIGN)
#else
STATIC_ASSERT(GYRO_3_ALIGN == ALIGN_CUSTOM, "GYRO_3_ALIGN and GYRO_3_CUSTOM_ALIGN mixed");
#endif // GYRO_3_CUSTOM_ALIGN

#ifndef GYRO_4_ALIGN
#ifdef GYRO_4_CUSTOM_ALIGN
#define GYRO_4_ALIGN            ALIGN_CUSTOM
#else
#define GYRO_4_ALIGN            CW0_DEG
#endif
#endif // GYRO_4_ALIGN

#ifndef GYRO_4_CUSTOM_ALIGN
#define GYRO_4_CUSTOM_ALIGN     SENSOR_ALIGNMENT_FROM_STD(GYRO_4_ALIGN)
#else
STATIC_ASSERT(GYRO_4_ALIGN == ALIGN_CUSTOM, "GYRO_4_ALIGN and GYRO_4_CUSTOM_ALIGN mixed");
#endif // GYRO_4_CUSTOM_ALIGN

#if defined(USE_SPI_GYRO) && (defined(GYRO_1_SPI_INSTANCE) || defined(GYRO_2_SPI_INSTANCE))
static void gyroResetSpiDeviceConfig(gyroDeviceConfig_t *devconf, SPI_TypeDef *instance, ioTag_t csnTag, ioTag_t extiTag, ioTag_t clkInTag, uint8_t alignment, sensorAlignment_t customAlignment)
{
    devconf->busType = BUS_TYPE_SPI;
    devconf->spiBus = SPI_DEV_TO_CFG(spiDeviceByInstance(instance));
    devconf->csnTag = csnTag;
    devconf->extiTag = extiTag;
    devconf->alignment = alignment;
    devconf->customAlignment = customAlignment;
    devconf->clkIn = clkInTag;
}
#endif

#if defined(USE_I2C_GYRO) && !(GYRO_COUNT > 1)
static void gyroResetI2cDeviceConfig(gyroDeviceConfig_t *devconf, i2cDevice_e i2cbus, ioTag_t extiTag, uint8_t alignment, sensorAlignment_t customAlignment)
{
    devconf->busType = BUS_TYPE_I2C;
    devconf->i2cBus = I2C_DEV_TO_CFG(i2cbus);
    devconf->i2cAddress = GYRO_I2C_ADDRESS;
    devconf->extiTag = extiTag;
    devconf->alignment = alignment;
    devconf->customAlignment = customAlignment;
}
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(gyroDeviceConfig_t, MAX_GYRODEV_COUNT, gyroDeviceConfig, PG_GYRO_DEVICE_CONFIG, 1);

void pgResetFn_gyroDeviceConfig(gyroDeviceConfig_t *devconf)
{
    devconf[0].index = 0;
    // All multi-gyro boards use SPI based gyros.
#ifdef USE_SPI_GYRO

#define GYRO_RESET(index, num) \
    gyroResetSpiDeviceConfig(&devconf[index], GYRO_##num##_SPI_INSTANCE, \
        IO_TAG(GYRO_##num##_CS_PIN), IO_TAG(GYRO_##num##_EXTI_PIN), \
        IO_TAG(GYRO_##num##_CLKIN_PIN), GYRO_##num##_ALIGN, GYRO_##num##_CUSTOM_ALIGN)

#ifdef GYRO_1_SPI_INSTANCE
    GYRO_RESET(0, 1);
#else
    devconf[0].busType = BUS_TYPE_NONE;
#endif

#if GYRO_COUNT > 1
    devconf[1].index = 1;
#ifdef GYRO_2_SPI_INSTANCE
    // TODO: CLKIN gyro 2 on separate pin is not supported yet. need to implement it
    GYRO_RESET(1, 2);
#else
    devconf[1].busType = BUS_TYPE_NONE;
#endif
#endif // GYRO_COUNT 2

#if GYRO_COUNT > 2
    devconf[2].index = 2;
#ifdef GYRO_3_SPI_INSTANCE
    // TODO: CLKIN gyro 3 on separate pin is not supported yet. need to implement it
    GYRO_RESET(2, 3);
#else
    devconf[2].busType = BUS_TYPE_NONE;
#endif
#endif // GYRO_COUNT 3

#if GYRO_COUNT > 3
    devconf[3].index = 3;
#ifdef GYRO_4_SPI_INSTANCE
    // TODO: CLKIN gyro 4 on separate pin is not supported yet. need to implement it
    GYRO_RESET(3, 4);
#else
    devconf[3].busType = BUS_TYPE_NONE;
#endif
#endif // GYRO_COUNT 4

#if GYRO_COUNT > 4
    // reset all gyro greater than gyro 5
    for (int i = 4; i < 8; i++) {
        devconf[i].busType = BUS_TYPE_NONE;
    }
#endif
#endif // USE_SPI_GYRO

    // I2C gyros appear as a sole gyro in single gyro boards.
#if defined(USE_I2C_GYRO) && !(GYRO_COUNT > 1)
    devconf[0].i2cBus = I2C_DEV_TO_CFG(I2CINVALID); // XXX Not required?
    gyroResetI2cDeviceConfig(&devconf[0], I2C_DEVICE, IO_TAG(GYRO_1_EXTI_PIN), GYRO_1_ALIGN, customAlignment1);
#endif
}
