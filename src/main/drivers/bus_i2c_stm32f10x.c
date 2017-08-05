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
#include <stdlib.h>

#include <platform.h>

#include "build/atomic.h"

#include "drivers/io.h"
#include "drivers/time.h"

#include "drivers/bus_i2c.h"
#include "drivers/nvic.h"
#include "io_impl.h"
#include "rcc.h"
#include "drivers/light_led.h"

#ifndef SOFT_I2C

static void i2cUnstick(IO_t scl, IO_t sda);

#define GPIO_AF_I2C GPIO_AF_I2C1

#ifdef STM32F4

#if defined(USE_I2C_PULLUP)
#define IOCFG_I2C IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_UP)
#else
#define IOCFG_I2C IOCFG_AF_OD
#endif

#ifndef I2C1_SCL
#define I2C1_SCL PB8
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB9
#endif

#else

#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#define IOCFG_I2C   IO_CONFIG(GPIO_Mode_AF_OD, GPIO_Speed_50MHz)

#endif

#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif

#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif

#ifdef STM32F4
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PB4
#endif
#endif

typedef enum {
    I2C_STATE_STOPPED = 0,
    I2C_STATE_STOPPING,
    I2C_STATE_STARTING,
    I2C_STATE_STARTING_WAIT,

    I2C_STATE_R_ADDR,
    I2C_STATE_R_ADDR_WAIT,
    I2C_STATE_R_REGISTER,
    I2C_STATE_R_REGISTER_WAIT,
    I2C_STATE_R_RESTARTING,
    I2C_STATE_R_RESTARTING_WAIT,
    I2C_STATE_R_RESTART_ADDR,
    I2C_STATE_R_RESTART_ADDR_WAIT,
    I2C_STATE_R_TRANSFER_EQ1,
    I2C_STATE_R_TRANSFER_EQ2,
    I2C_STATE_R_TRANSFER_GE2,

    I2C_STATE_W_ADDR,
    I2C_STATE_W_ADDR_WAIT,
    I2C_STATE_W_REGISTER,
    I2C_STATE_W_TRANSFER_WAIT,
    I2C_STATE_W_TRANSFER,

    I2C_STATE_NACK,
    I2C_STATE_BUS_ERROR,
} i2cState_t;

typedef enum {
    I2C_TXN_READ,
    I2C_TXN_WRITE
} i2cTransferDirection_t;

typedef struct i2cBusState_s {
    I2CDevice       device;
    bool            initialized;
    i2cState_t      state;
    uint32_t        timeout;

    /* Active transfer */
    uint8_t                     addr;   // device address
    i2cTransferDirection_t      rw;     // direction
    uint8_t                     reg;    // register
    uint32_t                    len;    // buffer length
    uint8_t                    *buf;    // buffer
    bool                        txnOk;
} i2cBusState_t;

static volatile uint16_t i2cErrorCount = 0;

static i2cDevice_t i2cHardwareMap[] = {
    { .dev = I2C1, .scl = IO_TAG(I2C1_SCL), .sda = IO_TAG(I2C1_SDA), .rcc = RCC_APB1(I2C1), .speed = I2C_SPEED_400KHZ },
    { .dev = I2C2, .scl = IO_TAG(I2C2_SCL), .sda = IO_TAG(I2C2_SDA), .rcc = RCC_APB1(I2C2), .speed = I2C_SPEED_400KHZ },
#ifdef STM32F4
    { .dev = I2C3, .scl = IO_TAG(I2C3_SCL), .sda = IO_TAG(I2C3_SDA), .rcc = RCC_APB1(I2C3), .speed = I2C_SPEED_400KHZ }
#endif
};

static i2cBusState_t busState[I2CDEV_COUNT] = { { 0 } };

static void i2cResetInterface(i2cBusState_t * i2cBusState)
{
    const i2cDevice_t * i2c = &(i2cHardwareMap[i2cBusState->device]);
    IO_t scl = IOGetByTag(i2c->scl);
    IO_t sda = IOGetByTag(i2c->sda);

    i2cErrorCount++;
    i2cUnstick(scl, sda);
    i2cInit(i2cBusState->device);
}

static void i2cStateMachine(i2cBusState_t * i2cBusState, const uint32_t currentTicks)
{
    I2C_TypeDef * I2Cx = i2cHardwareMap[i2cBusState->device].dev;

    switch (i2cBusState->state) {
        case I2C_STATE_BUS_ERROR:
            i2cResetInterface(i2cBusState);
            i2cBusState->state = I2C_STATE_STOPPED;
            break;

        case I2C_STATE_STOPPING:
            // Wait for stop bit to clear
            // RM0090: When the STOP, START or PEC bit is set, the software must not perform any write access
            // to I2C_CR1 before this bit is cleared by hardware. Otherwise there is a risk of setting a second STOP, START or PEC request.
            if ((I2Cx->CR1 & I2C_CR1_STOP) == 0) {
                i2cBusState->state = I2C_STATE_STOPPED;
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_STOPPED:
            // Stick here
            break;

        case I2C_STATE_STARTING:
            I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
            I2C_AcknowledgeConfig(I2Cx, ENABLE);
            I2C_GenerateSTART(I2Cx, ENABLE);
            i2cBusState->state = I2C_STATE_STARTING_WAIT;
            i2cBusState->timeout = currentTicks;
            // Fallthrough

        case I2C_STATE_STARTING_WAIT:
            if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) != ERROR) {
                if (i2cBusState->rw == I2C_TXN_READ) {
                    i2cBusState->state = I2C_STATE_R_ADDR;
                }
                else {
                    i2cBusState->state = I2C_STATE_W_ADDR;
                }
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_R_ADDR:
            I2C_Send7bitAddress(I2Cx, i2cBusState->addr, I2C_Direction_Transmitter);
            i2cBusState->state = I2C_STATE_R_ADDR_WAIT;
            i2cBusState->timeout = currentTicks;
            // Fallthrough

        case I2C_STATE_R_ADDR_WAIT:
            if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != ERROR) {
                i2cBusState->state = I2C_STATE_R_REGISTER;
            }
            else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF) != RESET) {
                i2cBusState->state = I2C_STATE_NACK;
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_R_REGISTER:      /* Send Register address */
            I2C_SendData(I2Cx, i2cBusState->reg);
            i2cBusState->state = I2C_STATE_R_REGISTER_WAIT;
            i2cBusState->timeout = currentTicks;
            /* Fallthrough */

        case I2C_STATE_R_REGISTER_WAIT:
            if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != ERROR) {
                if (i2cBusState->len == 0) {
                    I2C_GenerateSTOP(I2Cx, ENABLE);
                    i2cBusState->timeout = currentTicks;
                    i2cBusState->state = I2C_STATE_STOPPING;
                }
                else {
                    i2cBusState->state = I2C_STATE_R_RESTARTING;
                }
            }
            else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF) != RESET) {
                i2cBusState->state = I2C_STATE_NACK;
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_R_RESTARTING:
            I2C_GenerateSTART(I2Cx, ENABLE);
            i2cBusState->state = I2C_STATE_R_RESTARTING_WAIT;
            i2cBusState->timeout = currentTicks;
            // Fallthrough

        case I2C_STATE_R_RESTARTING_WAIT:
            if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) != ERROR) {
                i2cBusState->state = I2C_STATE_R_RESTART_ADDR;
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_R_RESTART_ADDR:
            I2C_Send7bitAddress(I2Cx, i2cBusState->addr, I2C_Direction_Receiver);
            i2cBusState->state = I2C_STATE_R_RESTART_ADDR_WAIT;
            i2cBusState->timeout = currentTicks;
            // Fallthrough

        case I2C_STATE_R_RESTART_ADDR_WAIT:
            if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != ERROR) {
                if (i2cBusState->len == 1) {
                    // This TXN is 1-byte, disable ACK and generate stop early
                    I2C_AcknowledgeConfig(I2Cx, DISABLE);

                    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
                        (void) I2Cx->SR2;
                        I2C_GenerateSTOP(I2Cx, ENABLE);
                    }

                    i2cBusState->state = I2C_STATE_R_TRANSFER_EQ1;
                }
                else if (i2cBusState->len == 2) {
                    // 2-byte transaction, disable ACK
                    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);
                    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
                        (void) I2Cx->SR2;
                        I2C_AcknowledgeConfig(I2Cx, DISABLE);
                    }

                    i2cBusState->state = I2C_STATE_R_TRANSFER_EQ2;
                }
                else {
                    (void) I2Cx->SR2;   // Clear ADDR flag
                    i2cBusState->state = I2C_STATE_R_TRANSFER_GE2;
                }

                i2cBusState->timeout = currentTicks;
            }
            else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF) != RESET) {
                i2cBusState->state = I2C_STATE_NACK;
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_R_TRANSFER_EQ1:
            if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE) != RESET) {
                *i2cBusState->buf++ = I2C_ReceiveData(I2Cx);
                i2cBusState->len--;

                // This was the last successful byte
                i2cBusState->txnOk = true;
                i2cBusState->timeout = currentTicks;
                i2cBusState->state = I2C_STATE_STOPPING;
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_R_TRANSFER_EQ2:
            if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF) != RESET) {
                ATOMIC_BLOCK(NVIC_PRIO_MAX) {
                    I2C_GenerateSTOP(I2Cx,ENABLE);
                    *i2cBusState->buf++ = I2C_ReceiveData(I2Cx);
                }

                *i2cBusState->buf++ = I2C_ReceiveData(I2Cx);
                i2cBusState->len =- 2;

                // This was the last successful byte
                i2cBusState->txnOk = true;
                i2cBusState->timeout = currentTicks;
                i2cBusState->state = I2C_STATE_STOPPING;
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_R_TRANSFER_GE2:
            if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF) != RESET) {
                if (i2cBusState->len == 3) {
                    I2C_AcknowledgeConfig(I2Cx, DISABLE);           // clear ack bit

                    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
                        *i2cBusState->buf++ = I2C_ReceiveData(I2Cx);
                        I2C_GenerateSTOP(I2Cx,ENABLE);
                    }

                    *i2cBusState->buf++ = I2C_ReceiveData(I2Cx);
                    i2cBusState->len -= 2;

                    // Last byte remaining
                    i2cBusState->state = I2C_STATE_R_TRANSFER_EQ1;
                    i2cBusState->timeout = currentTicks;
                }
                else if (i2cBusState->len < 3) {
                    // Shouldn't happen - abort
                    I2C_AcknowledgeConfig(I2Cx, DISABLE);
                    I2C_GenerateSTOP(I2Cx,ENABLE);
                    I2C_ReceiveData(I2Cx);

                    i2cBusState->txnOk = false;
                    i2cBusState->timeout = currentTicks;
                    i2cBusState->state = I2C_STATE_STOPPING;
                }
                else {
                    // 4 or more extra bytes remaining
                    *i2cBusState->buf++ = I2C_ReceiveData(I2Cx);
                    i2cBusState->len--;

                    // Restart timeout and stay in this state
                    i2cBusState->timeout = currentTicks;
                }
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_W_ADDR:
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
            I2C_Send7bitAddress(I2Cx, i2cBusState->addr, I2C_Direction_Transmitter);
            i2cBusState->state = I2C_STATE_W_ADDR_WAIT;
            i2cBusState->timeout = currentTicks;
            // Fallthrough

        case I2C_STATE_W_ADDR_WAIT:
            if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != ERROR) {
                i2cBusState->state = I2C_STATE_W_REGISTER;
            }
            else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF) != RESET) {
                i2cBusState->state = I2C_STATE_NACK;
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_W_REGISTER:      /* Send Register address */
            I2C_SendData(I2Cx, i2cBusState->reg);
            i2cBusState->state = I2C_STATE_W_TRANSFER_WAIT;
            i2cBusState->timeout = currentTicks;
            /* Fallthrough */

        case I2C_STATE_W_TRANSFER_WAIT:
            if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != ERROR) {
                i2cBusState->state = I2C_STATE_W_TRANSFER;
            }
            else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF) != RESET) {
                i2cBusState->state = I2C_STATE_NACK;
            }
            else if (ticks_diff_us(i2cBusState->timeout, currentTicks) >= I2C_TIMEOUT) {
                i2cBusState->state = I2C_STATE_BUS_ERROR;
            }
            break;

        case I2C_STATE_W_TRANSFER:
            if (i2cBusState->len > 0) {
                I2C_SendData(I2Cx, *i2cBusState->buf);
                i2cBusState->buf++;
                i2cBusState->len--;
                i2cBusState->timeout = currentTicks;
                i2cBusState->state = I2C_STATE_W_TRANSFER_WAIT;
            }
            else {
                I2C_GenerateSTOP(I2Cx, ENABLE);
                i2cBusState->timeout = currentTicks;
                i2cBusState->txnOk = true;
                i2cBusState->state = I2C_STATE_STOPPING;
            }
            break;

        case I2C_STATE_NACK:
            I2C_GenerateSTOP(I2Cx, ENABLE);
            I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
            i2cBusState->timeout = currentTicks;
            i2cBusState->state = I2C_STATE_STOPPING;
            break;
    }
}

void i2cSetSpeed(uint8_t speed)
{
    for (unsigned int i = 0; i < sizeof(i2cHardwareMap) / sizeof(i2cHardwareMap[0]); i++) {
        i2cHardwareMap[i].speed = speed;
    }
}

uint32_t i2cTimeoutUserCallback(void)
{
    i2cErrorCount++;
    return false;
}

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID)
        return;

    i2cDevice_t *i2c = &(i2cHardwareMap[device]);

    IO_t scl = IOGetByTag(i2c->scl);
    IO_t sda = IOGetByTag(i2c->sda);

    RCC_ClockCmd(i2c->rcc, ENABLE);

    IOInit(scl, OWNER_I2C, RESOURCE_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C, RESOURCE_I2C_SDA, RESOURCE_INDEX(device));

#ifdef STM32F4
    IOConfigGPIOAF(scl, IOCFG_I2C, GPIO_AF_I2C);
    IOConfigGPIOAF(sda, IOCFG_I2C, GPIO_AF_I2C);
#else
    IOConfigGPIO(scl, IOCFG_I2C);
    IOConfigGPIO(sda, IOCFG_I2C);
#endif

    I2C_DeInit(i2c->dev);

    I2C_InitTypeDef i2cInit;
    I2C_StructInit(&i2cInit);

    i2cInit.I2C_Mode = I2C_Mode_I2C;
    i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
    i2cInit.I2C_OwnAddress1 = 0x00;
    i2cInit.I2C_Ack = I2C_Ack_Enable;
    i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    switch (i2c->speed) {
        case I2C_SPEED_400KHZ:
        default:
            i2cInit.I2C_ClockSpeed = 400000;
            break;

        case I2C_SPEED_800KHZ:
            i2cInit.I2C_ClockSpeed = 800000;
            break;

        case I2C_SPEED_100KHZ:
            i2cInit.I2C_ClockSpeed = 100000;
            break;

        case I2C_SPEED_200KHZ:
            i2cInit.I2C_ClockSpeed = 200000;
            break;
    }

    I2C_Init(i2c->dev, &i2cInit);
    I2C_StretchClockCmd(i2c->dev, ENABLE);
    I2C_Cmd(i2c->dev, ENABLE);

    busState[device].device = device;
    busState[device].initialized = true;
    busState[device].state = I2C_STATE_STOPPED;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

static void i2cWaitForCompletion(I2CDevice device)
{
    do {
        const uint32_t currentTicks = ticks();
        i2cStateMachine(&busState[device], currentTicks);
    } while (busState[device].state != I2C_STATE_STOPPED);
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    // Don't try to access the non-initialized device
    if (!busState[device].initialized)
        return false;

    // Set up write transaction
    busState[device].addr = addr << 1;
    busState[device].reg = reg;
    busState[device].rw = I2C_TXN_WRITE;
    busState[device].len = len;
    busState[device].buf = data;
    busState[device].txnOk = false;
    busState[device].state = I2C_STATE_STARTING;

    // Inject I2C_EVENT_START
    i2cWaitForCompletion(device);

    return busState[device].txnOk;
}

bool i2cWrite(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t data)
{
    return i2cWriteBuffer(device, addr, reg, 1, &data);
}

bool i2cRead(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    // Don't try to access the non-initialized device
    if (!busState[device].initialized)
        return false;

    // Set up read transaction
    busState[device].addr = addr << 1;
    busState[device].reg = reg;
    busState[device].rw = I2C_TXN_READ;
    busState[device].len = len;
    busState[device].buf = buf;
    busState[device].txnOk = false;
    busState[device].state = I2C_STATE_STARTING;

    // Inject I2C_EVENT_START
    i2cWaitForCompletion(device);

    return busState[device].txnOk;
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
