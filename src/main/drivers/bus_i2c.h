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

#include <string.h> // size_t
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

#ifdef UNIT_TEST
# define I2C_TypeDef unsigned long
#endif

// Default pins

#ifdef I2C_FULL_CONFIGURABILITY

# ifndef I2C1_SCL
#  define I2C1_SCL  NONE
# endif
# ifndef I2C1_SDA
#  define I2C1_SDA  NONE
# endif

# ifndef I2C2_SCL
#  define I2C2_SCL  NONE
# endif
# ifndef I2C2_SDA
#  define I2C2_SDA  NONE
# endif

# ifndef I2C3_SCL
#  define I2C3_SCL  NONE
# endif
# ifndef I2C3_SDA
#  define I2C3_SDA  NONE
# endif

# ifndef I2C4_SCL
#  define I2C4_SCL  NONE
# endif
# ifndef I2C4_SDA
#  define I2C4_SDA  NONE
# endif

#else // !I2C_FULL_CONFIGURABILITY

# ifdef STM32F1

#  ifdef USE_I2C1
#   ifndef I2C1_SCL
#    define I2C1_SCL PB6
#   endif
#   ifndef I2C1_SDA
#    define I2C1_SDA PB7
#   endif
#  endif

#  ifdef USE_I2C2
#   ifndef I2C2_SCL
#    define I2C2_SCL PB10
#   endif
#   ifndef I2C2_SDA
#    define I2C2_SDA PB11
#   endif
#  endif

# endif // STM32F1

# ifdef STM32F3

#  ifdef USE_I2C1
#   ifndef I2C1_SCL
#    define I2C1_SCL PB6
#   endif
#   ifndef I2C1_SDA
#    define I2C1_SDA PB7
#   endif
#  endif

#  ifdef USE_I2C2
#   ifndef I2C2_SCL
#    define I2C2_SCL PA9
#   endif
#   ifndef I2C2_SDA
#    define I2C2_SDA PA10
#   endif
#  endif

# endif // STM32F3

# ifdef STM32F4

#  ifdef USE_I2C1
#   ifndef I2C1_SCL
#    define I2C1_SCL PB8
#   endif
#   ifndef I2C1_SDA
#    define I2C1_SDA PB9
#   endif
#  endif

#  ifdef USE_I2C2
#   ifndef I2C2_SCL
#    define I2C2_SCL PB10
#   endif
#   ifndef I2C2_SDA
#    define I2C2_SDA PB11
#   endif
#  endif

#  ifdef USE_I2C3
#   ifndef I2C3_SCL
#    define I2C3_SCL PA8
#   endif
#   ifndef I2C3_SDA
#    define I2C3_SDA PC9
#   endif
#  endif

# endif // STM32F4

# ifdef STM32F7

#  ifdef USE_I2C1
#   ifndef I2C1_SCL
#    define I2C1_SCL PB6
#   endif
#   ifndef I2C1_SDA
#    define I2C1_SDA PB7
#   endif
#  endif

#  ifdef USE_I2C2
#   ifndef I2C2_SCL
#    define I2C2_SCL PB10
#   endif
#   ifndef I2C2_SDA
#    define I2C2_SDA PB11
#   endif
#  endif

#  ifdef USE_I2C3
#   ifndef I2C3_SCL
#    define I2C3_SCL PA8
#   endif
#   ifndef I2C3_SDA
#    define I2C3_SDA PB4
#   endif
#  endif

#  ifdef USE_I2C4
#   ifndef I2C4_SCL
#    define I2C4_SCL PD12
#   endif
#   ifndef I2C4_SDA
#    define I2C4_SDA PD13
#   endif
#  endif

# endif // STM32F7

#endif // I2C_FULL_CONFIGURABILITY

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

// Target configured pins (mostly from target.h)

typedef struct i2cTargetConfig_s {
    I2CDevice bus;
    ioTag_t scl;
    ioTag_t sda;
} i2cTargetConfig_t;

// This is how i2cTargetConfig is stored in master config ... (sigh)
// XXX PG should take care of this. Rewrite when possible.

typedef struct i2cPinConfig_s {
    ioTag_t ioTagSCL[I2CDEV_COUNT];
    ioTag_t ioTagSDA[I2CDEV_COUNT];
} i2cPinConfig_t;

void i2cInitBus(I2CDevice device);
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t data);
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);

uint16_t i2cGetErrorCounter(void);

size_t i2cPinMapSize(void);
size_t i2cTargetConfigSize(void);
void i2cTargetConfigInit(void);
void i2cInitAll();

extern i2cTargetConfig_t i2cTargetConfig[];
extern i2cDevice_t i2cPinMap[];
extern i2cDevice_t i2cHardwareConfig[];
