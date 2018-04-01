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

/*
 * Created by jflyper
 */

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "common/utils.h"

#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/bus_i2c.h"

#ifdef I2C_FULL_RECONFIGURABILITY
#if I2CDEV_COUNT >= 1
#ifndef I2C1_SCL
#define I2C1_SCL NONE
#endif
#ifndef I2C1_SDA
#define I2C1_SDA NONE
#endif
#endif

#if I2CDEV_COUNT >= 2
#ifndef I2C2_SCL
#define I2C2_SCL NONE
#endif
#ifndef I2C2_SDA
#define I2C2_SDA NONE
#endif
#endif

#if I2CDEV_COUNT >= 3
#ifndef I2C3_SCL
#define I2C3_SCL NONE
#endif
#ifndef I2C3_SDA
#define I2C3_SDA NONE
#endif
#endif

#if I2CDEV_COUNT >= 4
#ifndef I2C4_SCL
#define I2C4_SCL NONE
#endif
#ifndef I2C4_SDA
#define I2C4_SDA NONE
#endif
#endif

#else // I2C_FULL_RECONFIGURABILITY

// Backward compatibility for exisiting targets

#ifdef STM32F1
#ifndef I2C1_SCL
#define I2C1_SCL PB8
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB9
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#endif // STM32F1

#ifdef STM32F3
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PA9
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PA10
#endif
#endif // STM32F3

#ifdef STM32F4
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PC9
#endif
#endif // STM32F4

#ifdef STM32F7
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PB4
#endif
#ifndef I2C4_SCL
#define I2C4_SCL PD12
#endif
#ifndef I2C4_SDA
#define I2C4_SDA PD13
#endif
#endif // STM32F7

#endif // I2C_FULL_RECONFIGURABILITY

// Default values for internal pullup

#if defined(USE_I2C_PULLUP)
#define I2C1_PULLUP true
#define I2C2_PULLUP true
#define I2C3_PULLUP true
#define I2C4_PULLUP true
#else
#define I2C1_PULLUP false
#define I2C2_PULLUP false
#define I2C3_PULLUP false
#define I2C4_PULLUP false
#endif

typedef struct i2cDefaultConfig_s {
    I2CDevice device;
    ioTag_t ioTagScl, ioTagSda;
    bool overClock;
    bool pullUp;
} i2cDefaultConfig_t;

static const i2cDefaultConfig_t i2cDefaultConfig[] = {
#ifdef USE_I2C_DEVICE_1
    { I2CDEV_1, IO_TAG(I2C1_SCL), IO_TAG(I2C1_SDA), I2C1_OVERCLOCK, I2C1_PULLUP },
#endif
#ifdef USE_I2C_DEVICE_2
    { I2CDEV_2, IO_TAG(I2C2_SCL), IO_TAG(I2C2_SDA), I2C2_OVERCLOCK, I2C2_PULLUP },
#endif
#ifdef USE_I2C_DEVICE_3
    { I2CDEV_3, IO_TAG(I2C3_SCL), IO_TAG(I2C3_SDA), I2C3_OVERCLOCK, I2C3_PULLUP },
#endif
#ifdef USE_I2C_DEVICE_4
    { I2CDEV_4, IO_TAG(I2C4_SCL), IO_TAG(I2C4_SDA), I2C4_OVERCLOCK, I2C4_PULLUP },
#endif
};

void pgResetFn_i2cConfig(i2cConfig_t *i2cConfig)
{
    memset(i2cConfig, 0, sizeof(*i2cConfig));

    for (size_t index = 0 ; index < ARRAYLEN(i2cDefaultConfig) ; index++) {
        const i2cDefaultConfig_t *defconf = &i2cDefaultConfig[index];
        i2cConfig->ioTagScl[defconf->device] = defconf->ioTagScl;
        i2cConfig->ioTagSda[defconf->device] = defconf->ioTagSda;
        i2cConfig->overClock[defconf->device] = defconf->overClock;
        i2cConfig->pullUp[defconf->device] = defconf->pullUp;
    }
}

PG_REGISTER_WITH_RESET_FN(i2cConfig_t, i2cConfig, PG_I2C_CONFIG, 0);

#endif // defined(USE_I2C) && !defined(USE_SOFT_I2C)
