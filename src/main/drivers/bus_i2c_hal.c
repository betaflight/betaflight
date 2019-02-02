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
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

#define CLOCKSPEED 800000    // i2c clockspeed 400kHz default (conform specs), 800kHz  and  1200kHz (Betaflight default)

// Number of bits in I2C protocol phase
#define LEN_ADDR 7
#define LEN_RW 1
#define LEN_ACK 1

// Clock period in us during unstick transfer
#define UNSTICK_CLK_US 10

// Allow 500us for clock strech to complete during unstick
#define UNSTICK_CLK_STRETCH (500/UNSTICK_CLK_US)

static void i2cUnstick(IO_t scl, IO_t sda);

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)

#define GPIO_AF4_I2C GPIO_AF4_I2C1

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = { I2CPINDEF(PB6), I2CPINDEF(PB8) },
        .sdaPins = { I2CPINDEF(PB7), I2CPINDEF(PB9) },
        .rcc = RCC_APB1(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = { I2CPINDEF(PB10), I2CPINDEF(PF1) },
        .sdaPins = { I2CPINDEF(PB11), I2CPINDEF(PF0) },
        .rcc = RCC_APB1(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = I2C3,
        .sclPins = { I2CPINDEF(PA8) },
        .sdaPins = { I2CPINDEF(PC9) },
        .rcc = RCC_APB1(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = I2C4,
        .sclPins = { I2CPINDEF(PD12), I2CPINDEF(PF14) },
        .sdaPins = { I2CPINDEF(PD13), I2CPINDEF(PF15) },
        .rcc = RCC_APB1(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

#ifdef USE_I2C_DEVICE_1
void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_1].handle);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_1].handle);
}
#endif

#ifdef USE_I2C_DEVICE_2
void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_2].handle);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_2].handle);
}
#endif

#ifdef USE_I2C_DEVICE_3
void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_3].handle);
}

void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_3].handle);
}
#endif

#ifdef USE_I2C_DEVICE_4
void I2C4_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_4].handle);
}

void I2C4_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_4].handle);
}
#endif

static volatile uint16_t i2cErrorCount = 0;

static bool i2cOverClock;

void i2cSetOverclock(uint8_t OverClock) {
    i2cOverClock = (OverClock) ? true : false;
}

static bool i2cHandleHardwareFailure(I2CDevice device)
{
    (void)device;
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    //i2cInit(device);
    return false;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Transmit(pHandle ,addr_ << 1, data, len_, I2C_DEFAULT_TIMEOUT);
    else
        status = HAL_I2C_Mem_Write(pHandle ,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,data, len_, I2C_DEFAULT_TIMEOUT);

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
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Receive(pHandle ,addr_ << 1, buf, len, I2C_DEFAULT_TIMEOUT);
    else
        status = HAL_I2C_Mem_Read(pHandle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,buf, len, I2C_DEFAULT_TIMEOUT);

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];

    const i2cHardware_t *hardware = pDev->hardware;

    if (!hardware) {
        return;
    }

    IO_t scl = pDev->scl;
    IO_t sda = pDev->sda;

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    RCC_ClockCmd(hardware->rcc, ENABLE);

    i2cUnstick(scl, sda);

    // Init pins
#ifdef STM32F7
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4_I2C);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4_I2C);
#else
    IOConfigGPIO(scl, IOCFG_AF_OD);
    IOConfigGPIO(sda, IOCFG_AF_OD);
#endif

    // Init I2C peripheral

    I2C_HandleTypeDef *pHandle = &pDev->handle;

    memset(pHandle, 0, sizeof(*pHandle));

    pHandle->Instance = pDev->hardware->reg;

    /// TODO: HAL check if I2C timing is correct

    if (pDev->overClock) {
        // 800khz Maximum speed tested on various boards without issues
        pHandle->Init.Timing = 0x00500D1D;
    } else {
        pHandle->Init.Timing = 0x00500C6F;
    }

    pHandle->Init.OwnAddress1 = 0x0;
    pHandle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    pHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    pHandle->Init.OwnAddress2 = 0x0;
    pHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    pHandle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(pHandle);

    // Enable the Analog I2C Filter
    HAL_I2CEx_ConfigAnalogFilter(pHandle, I2C_ANALOGFILTER_ENABLE);

    // Setup interrupt handlers
    HAL_NVIC_SetPriority(hardware->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    HAL_NVIC_EnableIRQ(hardware->er_irq);
    HAL_NVIC_SetPriority(hardware->ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
    HAL_NVIC_EnableIRQ(hardware->ev_irq);
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

    // Clock out, with SDA high:
    //   7 data bits
    //   1 READ bit
    //   1 cycle for the ACK
    for (i = 0; i < (LEN_ADDR + LEN_RW + LEN_ACK); i++) {
        // Wait for any clock stretching to finish
        int timeout = UNSTICK_CLK_STRETCH;
        while (!IORead(scl) && timeout) {
            delayMicroseconds(UNSTICK_CLK_US);
            timeout--;
        }

        // Pull low
        IOLo(scl); // Set bus low
        delayMicroseconds(UNSTICK_CLK_US/2);
        IOHi(scl); // Set bus high
        delayMicroseconds(UNSTICK_CLK_US/2);
    }

    // Generate a stop condition in case there was none
    IOLo(scl);
    delayMicroseconds(UNSTICK_CLK_US/2);
    IOLo(sda);
    delayMicroseconds(UNSTICK_CLK_US/2);

    IOHi(scl); // Set bus scl high
    delayMicroseconds(UNSTICK_CLK_US/2);
    IOHi(sda); // Set bus sda high
}

#endif
