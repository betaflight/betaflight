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

#include "drivers/io.h"
#include "drivers/time.h"

#include "drivers/bus_i2c.h"
#include "drivers/nvic.h"
#include "io_impl.h"
#include "rcc.h"

#if !defined(SOFT_I2C) && defined(USE_I2C)

#define CLOCKSPEED 800000    // i2c clockspeed 400kHz default (conform specs), 800kHz  and  1200kHz (Betaflight default)

static void i2cUnstick(IO_t scl, IO_t sda);

#if defined(USE_I2C_PULLUP)
#define IOCFG_I2C IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#else
#define IOCFG_I2C IOCFG_AF_OD
#endif

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

#if defined(USE_I2C_DEVICE_4)
#ifndef I2C4_SCL
#define I2C4_SCL PD12
#endif
#ifndef I2C4_SDA
#define I2C4_SDA PD13
#endif
#endif

static i2cDevice_t i2cHardwareMap[] = {
    { .dev = I2C1, .scl = IO_TAG(I2C1_SCL), .sda = IO_TAG(I2C1_SDA), .rcc = RCC_APB1(I2C1), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C1_EV_IRQn, .er_irq = I2C1_ER_IRQn, .af = GPIO_AF4_I2C1 },
    { .dev = I2C2, .scl = IO_TAG(I2C2_SCL), .sda = IO_TAG(I2C2_SDA), .rcc = RCC_APB1(I2C2), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C2_EV_IRQn, .er_irq = I2C2_ER_IRQn, .af = GPIO_AF4_I2C2 },
    { .dev = I2C3, .scl = IO_TAG(I2C3_SCL), .sda = IO_TAG(I2C3_SDA), .rcc = RCC_APB1(I2C3), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C3_EV_IRQn, .er_irq = I2C3_ER_IRQn, .af = GPIO_AF4_I2C3 },
#if defined(USE_I2C_DEVICE_4)
    { .dev = I2C4, .scl = IO_TAG(I2C4_SCL), .sda = IO_TAG(I2C4_SDA), .rcc = RCC_APB1(I2C4), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C4_EV_IRQn, .er_irq = I2C4_ER_IRQn, .af = GPIO_AF4_I2C4 }
#endif
};

static volatile uint16_t i2cErrorCount = 0;

#define I2C_DEFAULT_TIMEOUT     10000

typedef struct i2cState_s {
    volatile bool initialised;
    volatile bool error;
    volatile bool busError;
    volatile bool busy;
    volatile uint8_t addr;
    volatile uint8_t reg;
    volatile uint8_t bytes;
    volatile uint8_t writing;
    volatile uint8_t reading;
    volatile uint8_t* write_p;
    volatile uint8_t* read_p;
} i2cState_t;

static i2cState_t i2cState[] = {
    { false, false, false, 0, 0, 0, 0, 0, 0, 0, 0 },
    { false, false, false, 0, 0, 0, 0, 0, 0, 0, 0 },
    { false, false, false, 0, 0, 0, 0, 0, 0, 0, 0 },
    { false, false, false, 0, 0, 0, 0, 0, 0, 0, 0 }
};

void i2cSetSpeed(uint8_t speed)
{
    for (unsigned int i = 0; i < sizeof(i2cHardwareMap) / sizeof(i2cHardwareMap[0]); i++) {
        i2cHardwareMap[i].speed = speed;
    }
}

typedef struct{
    I2C_HandleTypeDef Handle;
}i2cHandle_t;
static i2cHandle_t i2cHandle[I2CDEV_COUNT];

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cHandle[I2CDEV_1].Handle);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cHandle[I2CDEV_1].Handle);
}

void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cHandle[I2CDEV_2].Handle);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cHandle[I2CDEV_2].Handle);
}

void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cHandle[I2CDEV_3].Handle);
}

void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cHandle[I2CDEV_3].Handle);
}

#ifdef USE_I2C_DEVICE_4
void I2C4_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cHandle[I2CDEV_4].Handle);
}

void I2C4_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cHandle[I2CDEV_4].Handle);
}
#endif


static bool i2cHandleHardwareFailure(I2CDevice device)
{
    i2cErrorCount++;

    // reinit peripheral + clock out garbage
    i2cInit(device);

    return false;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID)
        return false;

    i2cState_t *state;
    state = &(i2cState[device]);

    if (!state->initialised)
        return false;

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Transmit(&i2cHandle[device].Handle,addr_ << 1,data, len_, I2C_DEFAULT_TIMEOUT);
    else
        status = HAL_I2C_Mem_Write(&i2cHandle[device].Handle,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,data, len_, I2C_DEFAULT_TIMEOUT);

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID)
        return false;

    i2cState_t *state;
    state = &(i2cState[device]);

    if (!state->initialised)
        return false;

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Receive(&i2cHandle[device].Handle,addr_ << 1,buf, len, I2C_DEFAULT_TIMEOUT);
    else
        status = HAL_I2C_Mem_Read(&i2cHandle[device].Handle,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,buf, len, I2C_DEFAULT_TIMEOUT);

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

void i2cInit(I2CDevice device)
{
    /*## Configure the I2C clock source. The clock is derived from the SYSCLK #*/
//    RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
//    RCC_PeriphCLKInitStruct.PeriphClockSelection = i2cHardwareMap[device].clk;
//    RCC_PeriphCLKInitStruct.I2c1ClockSelection = i2cHardwareMap[device].clk_src;
//    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    switch (device) {
    case I2CDEV_1:
        __HAL_RCC_I2C1_CLK_ENABLE();
        break;
    case I2CDEV_2:
        __HAL_RCC_I2C2_CLK_ENABLE();
        break;
    case I2CDEV_3:
        __HAL_RCC_I2C3_CLK_ENABLE();
        break;
#ifdef USE_I2C_DEVICE_4
    case I2CDEV_4:
        __HAL_RCC_I2C4_CLK_ENABLE();
        break;
#endif
    default:
        break;
    }
    if (device == I2CINVALID)
        return;

    i2cDevice_t *i2c;
    i2c = &(i2cHardwareMap[device]);

    i2cState_t *state;
    state = &(i2cState[device]);

    IO_t scl = IOGetByTag(i2c->scl);
    IO_t sda = IOGetByTag(i2c->sda);

    IOInit(scl, OWNER_I2C, RESOURCE_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C, RESOURCE_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    RCC_ClockCmd(i2c->rcc, ENABLE);


    i2cUnstick(scl, sda);

    // Init pins
    IOConfigGPIOAF(scl, IOCFG_I2C, i2c->af);
    IOConfigGPIOAF(sda, IOCFG_I2C, i2c->af);

    // Init I2C peripheral
    i2cHandle[device].Handle.Instance             = i2cHardwareMap[device].dev;
    /// TODO: HAL check if I2C timing is correct

    switch (i2c->speed) {
        case I2C_SPEED_400KHZ:
        default:
            i2cHandle[device].Handle.Init.Timing = 0x00A01B5B;  // 400kHz, Rise 100ns, Fall 10ns    0x00500B6A
            break;

        case I2C_SPEED_800KHZ:
            i2cHandle[device].Handle.Init.Timing = 0x00401B1B;  // 800khz, Rise 40, Fall 4
            break;

        case I2C_SPEED_100KHZ:
            i2cHandle[device].Handle.Init.Timing = 0x60100544;  // 100kHz, Rise 100ns, Fall 10ns
            break;

        case I2C_SPEED_200KHZ:
            i2cHandle[device].Handle.Init.Timing = 0x00A01EDF;  // 200kHz, Rise 100ns, Fall 10ns
            break;
    }

    i2cHandle[device].Handle.Init.OwnAddress1     = 0x0;
    i2cHandle[device].Handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    i2cHandle[device].Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2cHandle[device].Handle.Init.OwnAddress2     = 0x0;
    i2cHandle[device].Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2cHandle[device].Handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;


    HAL_I2C_Init(&i2cHandle[device].Handle);
    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&i2cHandle[device].Handle,I2C_ANALOGFILTER_ENABLE);

    HAL_NVIC_SetPriority(i2cHardwareMap[device].er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    HAL_NVIC_EnableIRQ(i2cHardwareMap[device].er_irq);
    HAL_NVIC_SetPriority(i2cHardwareMap[device].ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
    HAL_NVIC_EnableIRQ(i2cHardwareMap[device].ev_irq);
    state->initialised = true;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

static void i2cUnstick(IO_t scl, IO_t sda)
{
    int i;

    IOHi(scl);
    IOHi(sda);

    IOConfigGPIO(scl, IOCFG_OUT_OD);
    IOConfigGPIO(sda, IOCFG_OUT_OD);

    // Analog Devices AN-686
    // We need 9 clock pulses + STOP condition
    for (i = 0; i < 9; i++) {
        // Wait for any clock stretching to finish
        int timeout = 100;
        while (!IORead(scl) && timeout) {
            delayMicroseconds(5);
            timeout--;
        }

        // Pull low
        IOLo(scl); // Set bus low
        delayMicroseconds(5);
        IOHi(scl); // Set bus high
        delayMicroseconds(5);
    }

    // Generate a stop condition in case there was none
    IOLo(scl);
    delayMicroseconds(5);
    IOLo(sda);
    delayMicroseconds(5);

    IOHi(scl); // Set bus scl high
    delayMicroseconds(5);
    IOHi(sda); // Set bus sda high
}

#endif
