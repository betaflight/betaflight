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

#pragma once

#define I2C_SHORT_TIMEOUT            ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT             ((uint32_t)(10 * I2C_SHORT_TIMEOUT))
#define I2C_DEFAULT_TIMEOUT          I2C_SHORT_TIMEOUT

#include "io_types.h"
#include "rcc_types.h"

#ifndef I2C_DEVICE
#define I2C_DEVICE I2CINVALID
#endif

typedef enum I2CDevice {
    I2CINVALID = -1,
    I2CDEV_1   = 0,
    I2CDEV_2,
    I2CDEV_3,
    I2CDEV_4,
    I2CDEV_COUNT
} I2CDevice;

// Maximum device instance count; consolidate with I2CDevice enum?
#if defined(STM32F1) || defined(STM32F3)
# ifdef USE_I2C1
#  define I2CDEV_MAX 1
# endif
# ifdef USE_I2C2
#  undef  I2CDEV_MAX
#  define I2CDEV_MAX 2
# endif
#endif

#if defined(STM32F4)
# ifdef USE_I2C3
#  undef  I2CDEV_MAX
#  define I2CDEV_MAX 3
# endif
#endif

#if defined(STM32F7)
# ifdef USE_I2C4
#  undef  I2CDEV_MAX
#  define I2CDEV_MAX 4
# endif
#endif

typedef struct i2cDevice_s {
    bool configured;
    I2C_TypeDef *dev;
    ioTag_t scl;
    ioTag_t sda;
    rccPeriphTag_t rcc;
    bool overClock;
#if !defined(STM32F303xC)
    uint8_t ev_irq;
    uint8_t er_irq;
#endif
#if defined(STM32F7)
    uint8_t af;
#endif
} i2cDevice_t;

typedef struct i2cState_s {
    volatile bool error;
    volatile bool busy;
    volatile uint8_t addr;
    volatile uint8_t reg;
    volatile uint8_t bytes;
    volatile uint8_t writing;
    volatile uint8_t reading;
    volatile uint8_t* write_p;
    volatile uint8_t* read_p;
} i2cState_t;

#ifdef STM32F1
# define I2CDEV_MAX 2
#endif
#ifdef STM32F3
# define I2CDEV_MAX 2
#endif
#ifdef STM32F4
# define I2CDEV_MAX 3
#endif
#ifdef STM32F7
# define I2CDEV_MAX 4
#endif

typedef struct i2cPinConfig_s {
    ioTag_t ioTagSCL[I2CDEV_MAX];
    ioTag_t ioTagSDA[I2CDEV_MAX];
} i2cPinConfig_t;

void i2cInit(I2CDevice device);
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t data);
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);

uint16_t i2cGetErrorCounter(void);
void i2cInitAll();
void i2cPinConfigDefault(void);
