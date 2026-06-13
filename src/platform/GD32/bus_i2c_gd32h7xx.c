/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_utils.h"

static void i2c_er_handler(i2cDevice_e device);
static void i2c_ev_handler(i2cDevice_e device);

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_60MHZ, GPIO_OTYPE_OD, GPIO_PUPD_PULLUP)
#define IOCFG_I2C   IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_60MHZ, GPIO_OTYPE_OD, GPIO_PUPD_NONE)

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_0
    {
        .device = I2CDEV_0,
        .reg = (i2cResource_t *)I2C0,
        .sclPins = {
            I2CPINDEF(PB6, GPIO_AF_4),
            I2CPINDEF(PB8, GPIO_AF_4),
        },
         .sdaPins = {
            I2CPINDEF(PB7, GPIO_AF_4),
            I2CPINDEF(PB9, GPIO_AF_4),
         },
        .rcc = RCC_APB1(I2C0),
        .ev_irq = I2C0_EV_IRQn,
        .er_irq = I2C0_ER_IRQn,
    },
#endif
 #ifdef USE_I2C_DEVICE_1
     {
         .device = I2CDEV_1,
         .reg = (i2cResource_t *)I2C1,
         .sclPins = {
             I2CPINDEF(PB10, GPIO_AF_4),
             I2CPINDEF(PF1,  GPIO_AF_4),
         },
         .sdaPins = {
             I2CPINDEF(PB11, GPIO_AF_4),
             I2CPINDEF(PF0,  GPIO_AF_4),
         },
         .rcc = RCC_APB1(I2C1),
         .ev_irq = I2C1_EV_IRQn,
         .er_irq = I2C1_ER_IRQn,
     },
 #endif
#ifdef USE_I2C_DEVICE_2
     {
         .device = I2CDEV_2,
         .reg = (i2cResource_t *)I2C2,
         .sclPins = {
             I2CPINDEF(PA8, GPIO_AF_4),
         },
         .sdaPins = {
             I2CPINDEF(PC9, GPIO_AF_4),
         },
         .rcc = RCC_APB1(I2C2),
         .ev_irq = I2C2_EV_IRQn,
         .er_irq = I2C2_ER_IRQn,
     },
#endif
#ifdef USE_I2C_DEVICE_3
     {
         .device = I2CDEV_3,
         .reg = (i2cResource_t *)I2C3,
         .sclPins = {
             I2CPINDEF(PD12, GPIO_AF_4),
         },
         .sdaPins = {
             I2CPINDEF(PD13, GPIO_AF_4),
         },
         .rcc = RCC_APB1(I2C3),
         .ev_irq = I2C3_EV_IRQn,
         .er_irq = I2C3_ER_IRQn,
     },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

/* I2C transfer context - tracks transfer progress in interrupt handlers */
typedef struct {
    volatile int8_t index;              // Current transfer index, -1 means need to send register address first
    volatile uint8_t subaddress_sent;   // Read operation: whether register address has been sent
    volatile uint8_t final_stop;        // Whether final STOP is needed
    volatile uint8_t phase;             // Transfer phase: 0=not started, 1=Phase1(send reg), 2=Phase2(recv/send data)
} i2cContext_t;

static volatile i2cContext_t i2cContext[I2CDEV_COUNT];

static volatile uint16_t i2cErrorCount = 0;

void I2C0_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_0);
}

void I2C0_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_0);
}

void I2C1_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_1);
}

void I2C2_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_2);
}

void I2C3_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_3);
}

void I2C3_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_3);
}

/* Disable all I2C interrupts */
static inline void i2cDisableAllInterrupts(uint32_t I2Cx)
{
    I2C_CTL0(I2Cx) &= ~(I2C_CTL0_TIE | I2C_CTL0_RBNEIE | I2C_CTL0_TCIE |
                        I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);
}

/* Clear all status flags (write 1 to clear) */
static inline void i2cClearAllFlags(uint32_t I2Cx)
{
    I2C_STATC(I2Cx) = I2C_STATC_ADDSENDC | I2C_STATC_NACKC | I2C_STATC_STPDETC |
                      I2C_STATC_BERRC | I2C_STATC_LOSTARBC | I2C_STATC_OUERRC |
                      I2C_STATC_PECERRC | I2C_STATC_TIMEOUTC | I2C_STATC_SMBALTC;
}

/* Hardware failure handling: reinitialize I2C peripheral */
static bool i2cHandleHardwareFailure(i2cDevice_e device)
{
    i2cErrorCount++;
    i2cInit(device);
    return false;
}

/*
 * I2C write operation
 * Flow: START -> address -> register(optional) -> data -> AUTOEND auto STOP -> STPDET
 */
bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    uint32_t I2Cx = (uint32_t)(i2cDevice[device].reg);

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &(i2cDevice[device].state);
    volatile i2cContext_t *ctx = &i2cContext[device];
    if (state->busy) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 1;
    state->reading = 0;
    state->write_p = data;
    state->read_p = data;
    state->bytes = len_;
    state->busy = 1;
    state->error = false;

    // Wait for bus to be idle
    while (I2C_STAT(I2Cx) & I2C_STAT_I2CBSY) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            state->busy = 0;
            return i2cHandleHardwareFailure(device);
        }
    }

    // Wait for previous STOP to finish sending
    while (I2C_CTL1(I2Cx) & I2C_CTL1_STOP) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            state->busy = 0;
            return i2cHandleHardwareFailure(device);
        }
    }

    // Disable interrupts and clear residual flags
    i2cDisableAllInterrupts(I2Cx);
    i2cClearAllFlags(I2Cx);

    // Initialize transfer context
    ctx->subaddress_sent = 0;
    ctx->index = (reg_ != 0xFF) ? -1 : 0;  // -1: need to send reg first; 0: send data directly
    ctx->final_stop = 1;                   // Write operation requires STOP at end
    ctx->phase = 1;                        // Write operation has only one phase

    // Clear transmit buffer
    I2C_STAT(I2Cx) |= I2C_STAT_TBE;

    // Calculate total transfer bytes (including register address)
    uint32_t total_bytes = state->bytes;
    if (state->reg != 0xFF) {
        total_bytes++;  // Add 1 byte for register address
    }

    // Configure CTL1 register atomically (avoid read-modify-write race)
    uint32_t ctl1_new = 0;
    ctl1_new |= (state->addr & I2C_CTL1_SADDRESS);  // Slave address
    ctl1_new |= (total_bytes << 16);                // BYTENUM: bytes to transfer
    ctl1_new |= I2C_CTL1_AUTOEND;                   // AUTOEND=1: hardware auto sends STOP
    ctl1_new |= I2C_CTL1_START;                     // START

    __DSB();
    I2C_CTL1(I2Cx) = ctl1_new;
    __DSB();
    __ISB();

    // Enable interrupts: transmit buffer empty + NACK + STOP detect + error
    I2C_CTL0(I2Cx) |= (I2C_CTL0_TIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);

    return true;
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    i2cState_t *state = &i2cDevice[device].state;

    if (error) {
        *error = state->error;
    }
    return state->busy;
}

/* Wait for I2C transfer to complete */
static bool i2cWait(i2cDevice_e device)
{
    i2cState_t *state = &(i2cDevice[device].state);
    timeUs_t timeoutStartUs = microsISR();

    while (state->busy) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device) && i2cWait(device);
        }
    }

    return !(state->error);
}

bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data) && i2cWait(device);
}

/*
 * I2C read operation
 * With register: Phase1(send reg,AUTOEND=0) -> TC -> Phase2(ReSTART receive,AUTOEND=1) -> STPDET
 * Without register: direct receive(AUTOEND=1) -> STPDET
 */
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    uint32_t I2Cx = (uint32_t)(i2cDevice[device].reg);
    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    volatile i2cContext_t *ctx = &i2cContext[device];

    if (state->busy) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    // Setup transfer parameters
    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 0;
    state->reading = 1;
    state->read_p = buf;
    state->write_p = buf;
    state->bytes = len;
    state->busy = 1;
    state->error = false;

    // Wait for bus to be idle
    while (I2C_STAT(I2Cx) & I2C_STAT_I2CBSY) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            state->busy = 0;
            return i2cHandleHardwareFailure(device);
        }
    }

    // Wait for previous STOP to finish sending
    while (I2C_CTL1(I2Cx) & I2C_CTL1_STOP) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            state->busy = 0;
            return i2cHandleHardwareFailure(device);
        }
    }

    // Disable interrupts and clear flags
    i2cDisableAllInterrupts(I2Cx);
    i2cClearAllFlags(I2Cx);

    // Initialize transfer context - ensure all states are clean
    ctx->index = 0;
    ctx->subaddress_sent = (reg_ == 0xFF) ? 1 : 0;
    ctx->final_stop = 1;  // Read operation requires STOP at end
    ctx->phase = (reg_ == 0xFF) ? 2 : 1;  // Phase1=send reg, Phase2=receive

    // Clear transmit buffer
    I2C_STAT(I2Cx) |= I2C_STAT_TBE;

    // Configure CTL1 register atomically
    uint32_t ctl1_new = 0;
    ctl1_new |= (state->addr & I2C_CTL1_SADDRESS);  // Slave address

    if (reg_ == 0xFF) {
        // Direct read mode: START -> receive data -> AUTOEND auto NACK+STOP
        ctl1_new |= I2C_CTL1_TRDIR;             // TRDIR=1: master receive
        ctl1_new |= ((uint32_t)len << 16);      // BYTENUM
        ctl1_new |= I2C_CTL1_AUTOEND;           // Hardware auto NACK+STOP
        ctl1_new |= I2C_CTL1_START;             // START

        __DSB();
        I2C_CTL1(I2Cx) = ctl1_new;
        __DSB();
        __ISB();

        // Enable: receive buffer not empty + NACK + STOP detect + error
        I2C_CTL0(I2Cx) |= (I2C_CTL0_RBNEIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);
    } else {
        // Two-phase read: Phase1 send register address first
        ctl1_new |= ((uint32_t)1 << 16);        // BYTENUM=1 (send reg only)
        // TRDIR=0: master transmit; AUTOEND=0: send ReSTART after TC
        ctl1_new |= I2C_CTL1_START;             // START

        __DSB();
        I2C_CTL1(I2Cx) = ctl1_new;
        __DSB();
        __ISB();

        // Enable: transmit buffer empty + transfer complete + NACK + STOP detect + error
        I2C_CTL0(I2Cx) |= (I2C_CTL0_TIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);
    }

    return true;
}

bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf) && i2cWait(device);
}

/*
 * I2C error interrupt handler
 * Handles: BERR(bus error), LOSTARB(arbitration lost), OUERR(over/underrun), TIMEOUT
 * Error recovery: disable interrupts -> clear flags -> send STOP(except arbitration loss) -> release bus
 */
static void i2c_er_handler(i2cDevice_e device)
{
    uint32_t I2Cx = (uint32_t)(i2cDevice[device].hardware->reg);
    i2cState_t *state = &i2cDevice[device].state;
    volatile i2cContext_t *ctx = &i2cContext[device];

    // Read status register
    uint32_t status = I2C_STAT(I2Cx);
    uint32_t ctl1 = I2C_CTL1(I2Cx);

    // Check error flags
    if (status & (I2C_STAT_BERR | I2C_STAT_LOSTARB | I2C_STAT_OUERR | I2C_STAT_PECERR | I2C_STAT_TIMEOUT)) {
        state->error = true;

        // Disable interrupts first to prevent interrupt nesting
        i2cDisableAllInterrupts(I2Cx);

        // Clear corresponding error flags
        uint32_t clearMask = 0;
        if (status & I2C_STAT_BERR)    clearMask |= I2C_STATC_BERRC;
        if (status & I2C_STAT_LOSTARB) clearMask |= I2C_STATC_LOSTARBC;
        if (status & I2C_STAT_OUERR)   clearMask |= I2C_STATC_OUERRC;
        if (status & I2C_STAT_PECERR)  clearMask |= I2C_STATC_PECERRC;
        if (status & I2C_STAT_TIMEOUT) clearMask |= I2C_STATC_TIMEOUTC;

        I2C_STATC(I2Cx) = clearMask;

        // No need to send STOP on arbitration loss
        if (!(status & I2C_STAT_LOSTARB)) {
            // If currently sending START, need to wait for completion before sending STOP
            if (ctl1 & I2C_CTL1_START) {
                uint32_t timeout = 10000;
                while ((I2C_CTL1(I2Cx) & I2C_CTL1_START) && --timeout) {
                    // Wait for START to complete
                }
                I2C_CTL1(I2Cx) |= I2C_CTL1_STOP;    // Send STOP to terminate bus transaction
                timeout = 10000;
                while ((I2C_CTL1(I2Cx) & I2C_CTL1_STOP) && --timeout) {
                    // Wait for STOP to complete
                }
                if (timeout == 0) {
                    i2c_deinit(I2Cx);               // Reinitialize hardware on timeout
                }
            } else {
                // Send STOP to release bus
                if (!(I2C_CTL1(I2Cx) & I2C_CTL1_STOP)) {
                    I2C_CTL1(I2Cx) |= I2C_CTL1_STOP;
                }
            }
        }

        // Check if STPDET is already set
        status = I2C_STAT(I2Cx);
        if (status & I2C_STAT_STPDET) {
            I2C_STATC(I2Cx) = I2C_STATC_STPDETC;
        }

        // Clean up state
        ctx->phase = 0;
        state->busy = 0;
    }
}

/*
 * I2C event interrupt handler - state machine implementation
 *
 * Event priority: STPDET > NACK > TC > RBNE > TI
 *
 * STPDET: STOP detected, transfer end
 * NACK:   Slave not acknowledged, transfer failed
 * TC:     Transfer complete, used for mid-phase switching in two-phase read
 * RBNE:   Receive buffer not empty, read data
 * TI:     Transmit buffer empty, send data or register address
 */
void i2c_ev_handler(i2cDevice_e device)
{
    uint32_t I2Cx = (uint32_t)(i2cDevice[device].hardware->reg);
    i2cState_t *state = &i2cDevice[device].state;
    volatile i2cContext_t *ctx = &i2cContext[device];

    // Read current status
    uint32_t status = I2C_STAT(I2Cx);
    uint32_t ctl0 = I2C_CTL0(I2Cx);

    // 1. STPDET: STOP detected, transfer end
    if ((ctl0 & I2C_CTL0_STPDETIE) && (status & I2C_STAT_STPDET)) {
        I2C_STATC(I2Cx) = I2C_STATC_STPDETC;  // Clear STPDET flag

        // Also clear possible NACK flag
        if (status & I2C_STAT_NACK) {
            I2C_STATC(I2Cx) = I2C_STATC_NACKC;
            state->error = true;
        }

        // Transfer complete, disable all interrupts
        i2cDisableAllInterrupts(I2Cx);
        ctx->phase = 0;  // Reset phase
        state->busy = 0;
        return;
    }

    // 2. NACK: slave not acknowledged
    if ((ctl0 & I2C_CTL0_NACKIE) && (status & I2C_STAT_NACK)) {
        I2C_STATC(I2Cx) = I2C_STATC_NACKC;
        state->error = true;

        // Disable all buffer interrupts
        I2C_CTL0(I2Cx) &= ~(I2C_CTL0_TIE | I2C_CTL0_RBNEIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_ERRIE);

        // Check if STPDET is already set
        status = I2C_STAT(I2Cx);
        if (status & I2C_STAT_STPDET) {
            I2C_STATC(I2Cx) = I2C_STATC_STPDETC;
            i2cDisableAllInterrupts(I2Cx);
            ctx->phase = 0;
            state->busy = 0;
        }
        return;
    }

    // 3. TC: transfer complete, used for mid-phase switching in two-phase read
    if ((ctl0 & I2C_CTL0_TCIE) && (status & I2C_STAT_TC)) {
        if (state->reading && ctx->subaddress_sent && ctx->phase == 1) {
            // Phase1 complete (register address sent), configure Phase2 to receive data

            // Disable all interrupts first to prevent race conditions
            i2cDisableAllInterrupts(I2Cx);

            ctx->index = 0;
            ctx->phase = 2;  // Mark entering Phase2

            // Configure CTL1 register atomically to send Repeated Start
            uint32_t ctl1_new = 0;
            ctl1_new |= (state->addr & I2C_CTL1_SADDRESS);  // Slave address
            ctl1_new |= I2C_CTL1_TRDIR;                      // TRDIR=1: master receive
            ctl1_new |= ((uint32_t)state->bytes << 16);      // BYTENUM=bytes to receive
            ctl1_new |= I2C_CTL1_AUTOEND;                    // AUTOEND=1: auto NACK+STOP on last byte
            ctl1_new |= I2C_CTL1_START;                      // START bit (Repeated Start)

            // Verify BYTENUM is not 0
            if (state->bytes == 0) {
                state->error = true;
                state->busy = 0;
                return;
            }

            __DSB();
            I2C_CTL1(I2Cx) = ctl1_new;
            __DSB();
            __ISB();

            // Enable receive interrupts
            I2C_CTL0(I2Cx) |= (I2C_CTL0_RBNEIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);
        }
        return;
    }

    // 4. RBNE: receive buffer not empty
    if ((ctl0 & I2C_CTL0_RBNEIE) && (status & I2C_STAT_RBNE)) {
        // Read data (must read, otherwise RBNE flag won't clear)
        uint8_t data = (uint8_t)I2C_RDATA(I2Cx);
        if (ctx->index < state->bytes) {
            state->read_p[ctx->index++] = data;
        }
        // AUTOEND will auto send NACK+STOP, STPDET interrupt ends transfer
        return;
    }

    // 5. TI: transmit buffer empty
    if ((ctl0 & I2C_CTL0_TIE) && (status & I2C_STAT_TI)) {
        if (state->writing) {
            // Write operation
            if (ctx->index == -1) {
                // Send register address first
                I2C_TDATA(I2Cx) = state->reg;
                ctx->index = 0;
            } else if (ctx->index < state->bytes) {
                // Send data
                while(!i2c_flag_get(I2Cx, I2C_FLAG_TI));
                I2C_TDATA(I2Cx) = state->write_p[ctx->index++];
            }
            // After all data sent, disable TI interrupt, wait for AUTOEND to auto send STOP
            if (ctx->index >= state->bytes) {
                I2C_CTL0(I2Cx) &= ~I2C_CTL0_TIE;
            }
        } else if (state->reading && !ctx->subaddress_sent) {
            // Read operation Phase1: send register address
            I2C_TDATA(I2Cx) = state->reg;
            ctx->subaddress_sent = 1;
            // Disable TI interrupt, wait for TC to trigger Phase2
            I2C_CTL0(I2Cx) &= ~I2C_CTL0_TIE;
        }
        return;
    }
}

/* I2C initialization */
void i2cInit(i2cDevice_e device)
{
    if (device == I2CINVALID)
        return;

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hw = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    if (!hw || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }

    uint32_t I2Cx = (uint32_t)i2cDevice[device].reg;

    // Clear transfer state
    memset(&pDev->state, 0, sizeof(pDev->state));

    // Initialize transfer context - ensure all fields are zeroed
    volatile i2cContext_t *ctx = &i2cContext[device];
    ctx->index = 0;
    ctx->subaddress_sent = 0;
    ctx->final_stop = 0;
    ctx->phase = 0;

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable I2C clock
    RCC_ClockCmd(hw->rcc, ENABLE);

    // Recover bus from stuck condition
    i2cUnstick(scl, sda);

    // Configure GPIO as open-drain alternate function
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);

    // Reset I2C peripheral
    i2c_deinit(I2Cx);

    i2c_timing_config(I2Cx, 0x4, 0x4, 0x2);
    i2c_master_clock_config(I2Cx, 0x2D, 0xE0);

    // Configure as 7-bit address mode
    i2c_address_config(I2Cx, 0, I2C_ADDFORMAT_7BITS);

    // Enable clock stretching (slave can pull SCL low)
    i2c_stretch_scl_low_enable(I2Cx);

    // Enable analog and digital noise filter
    i2c_analog_noise_filter_enable(I2Cx);
    i2c_digital_noise_filter_config(I2Cx,FILTER_LENGTH_15);

    // Enable I2C peripheral
    i2c_enable(I2Cx);

    // Configure and enable interrupt priorities
    nvic_irq_enable(hw->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    nvic_irq_enable(hw->ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
}


uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

#endif
