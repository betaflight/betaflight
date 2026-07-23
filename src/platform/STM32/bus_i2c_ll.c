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
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C) && !defined(USE_I3C_AS_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

static volatile uint16_t i2cErrorCount = 0;

#if defined(STM32H5)
// TEMPORARY (Development Instrumentation) — see bus_i2c.h. Per-bus capture of
// the most recent failed transaction plus per-reason counters, so the CLI can
// tell a NACK ("bus clocks, target silent") apart from a TIMEOUT ("peripheral
// never advanced") without another flash. Remove with the rest of the H5 I2C3
// bring-up instrumentation.
typedef struct {
    uint32_t lastIsr;    // I2Cx->ISR sampled at the failure (raw, nothing lost)
    uint32_t lastCr2;    // I2Cx->CR2 sampled at the failure
    uint16_t total;
    uint16_t nack;
    uint16_t berr;
    uint16_t arlo;
    uint16_t ovr;
    uint16_t timeout;
    uint8_t  lastReason; // i2cFailReason_e
} i2cFailDiag_t;
static i2cFailDiag_t i2cFailDiag[I2CDEV_COUNT];
#endif

// Record a failed transaction. reason is authoritative (the flag that triggered
// the failure may already be cleared by the caller); lastIsr/lastCr2 preserve
// the surrounding peripheral state (BUSY, START pending, byte counters).
static void i2cCaptureFail(i2cDevice_e device, i2cFailReason_e reason)
{
#if defined(STM32H5)
    if (device < 0 || device >= I2CDEV_COUNT) {
        return;
    }
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;
    if (!I2Cx) {
        return;
    }
    i2cFailDiag_t *d = &i2cFailDiag[device];
    d->lastIsr = I2Cx->ISR;
    d->lastCr2 = I2Cx->CR2;
    d->lastReason = reason;
    d->total++;
    switch (reason) {
    case I2C_FAIL_NACK:    d->nack++;    break;
    case I2C_FAIL_BERR:    d->berr++;    break;
    case I2C_FAIL_ARLO:    d->arlo++;    break;
    case I2C_FAIL_OVR:     d->ovr++;     break;
    case I2C_FAIL_TIMEOUT: d->timeout++; break;
    default: break;
    }
#else
    (void)device;
    (void)reason;
#endif
}

#if defined(STM32H5)
void i2cGetFailDiag(i2cDevice_e device, i2cDebugRegs_t *regs)
{
    if (device < 0 || device >= I2CDEV_COUNT || !regs) {
        return;
    }
    const i2cFailDiag_t *d = &i2cFailDiag[device];
    regs->lastIsr = d->lastIsr;
    regs->lastCr2 = d->lastCr2;
    regs->lastReason = d->lastReason;
    regs->failTotal = d->total;
    regs->failNack = d->nack;
    regs->failBerr = d->berr;
    regs->failArlo = d->arlo;
    regs->failOvr = d->ovr;
    regs->failTimeout = d->timeout;
}

void i2cResetFailDiag(i2cDevice_e device)
{
    if (device < 0 || device >= I2CDEV_COUNT) {
        return;
    }
    memset(&i2cFailDiag[device], 0, sizeof(i2cFailDiag[device]));
}
#endif

static void i2cRecoverFromISRError(I2C_TypeDef *I2Cx, i2cState_t *state)
{
    // Disable all transfer interrupts
    LL_I2C_DisableIT_TX(I2Cx);
    LL_I2C_DisableIT_RX(I2Cx);
    LL_I2C_DisableIT_TC(I2Cx);
    LL_I2C_DisableIT_STOP(I2Cx);
    LL_I2C_DisableIT_NACK(I2Cx);
    LL_I2C_DisableIT_ERR(I2Cx);

    // Clear pending STOPF (AUTOEND generates STOP after NACK)
    if (LL_I2C_IsActiveFlag_STOP(I2Cx)) {
        LL_I2C_ClearFlag_STOP(I2Cx);
    }

    // Flush TXDR to clear stale TXIS flag (matches HAL I2C_Flush_TXDR)
    if (LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
        LL_I2C_TransmitData8(I2Cx, 0x00);
    }
    if (!LL_I2C_IsActiveFlag_TXE(I2Cx)) {
        LL_I2C_ClearFlag_TXE(I2Cx);
    }

    // Reset CR2 to clear stale transfer configuration
    I2Cx->CR2 = 0;

    // PE-cycle to clear sticky ISR.BUSY=1 left after a NACK. CR2=0 alone
    // doesn't drop BUSY; without this the next transaction sees BUSY high
    // and stalls. Bounded NOP delay between Disable+Enable — polling
    // LL_I2C_IsEnabled() can hang in IRQ context.
    LL_I2C_Disable(I2Cx);
    for (volatile int i = 0; i < 64; i++) { __NOP(); }
    LL_I2C_Enable(I2Cx);

    state->error = true;
    state->busy = false;
    i2cErrorCount++;
}

// I2C event IRQ handler
static void i2cEVIRQHandler(i2cDevice_e device)
{
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].hardware->reg;
    i2cState_t *state = &i2cDevice[device].state;

    // NACK received
    if (LL_I2C_IsEnabledIT_NACK(I2Cx) && LL_I2C_IsActiveFlag_NACK(I2Cx)) {
        i2cCaptureFail(device, I2C_FAIL_NACK);  // capture before clearing NACKF
        LL_I2C_ClearFlag_NACK(I2Cx);
        i2cRecoverFromISRError(I2Cx, state);
        return;
    }

    // Transmit data register empty
    if (LL_I2C_IsEnabledIT_TX(I2Cx) && LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
        if (state->writing && state->bytes > 0) {
            LL_I2C_TransmitData8(I2Cx, *state->write_p++);
            state->bytes--;
        }
        // Disable IT_TX once the last byte has shifted out (TXIS reasserts
        // immediately otherwise and the IRQ tail-chains until IWDG fires).
        // Also disable if this IRQ fired during a read transaction
        // (state->writing=0); a stray IT_TX surviving a prior write would
        // otherwise tail-chain through the SOFTEND read setup.
        if (state->bytes == 0 || !state->writing) {
            LL_I2C_DisableIT_TX(I2Cx);
        }
        return;
    }

    // Receive data register not empty
    if (LL_I2C_IsEnabledIT_RX(I2Cx) && LL_I2C_IsActiveFlag_RXNE(I2Cx)) {
        if (state->reading && state->bytes > 0) {
            *state->read_p++ = LL_I2C_ReceiveData8(I2Cx);
            state->bytes--;
        }
        if (state->bytes == 0) {
            LL_I2C_DisableIT_RX(I2Cx);
        }
        return;
    }

    // Transfer complete (used for restart between write-reg-addr and read-data)
    if (LL_I2C_IsEnabledIT_TC(I2Cx) && LL_I2C_IsActiveFlag_TC(I2Cx)) {
        if (state->reading) {
            // Register address phase complete, now start read phase
            LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                                  state->bytes, LL_I2C_MODE_AUTOEND,
                                  LL_I2C_GENERATE_START_READ);
            // Switch from TX to RX interrupts
            LL_I2C_DisableIT_TX(I2Cx);
            LL_I2C_EnableIT_RX(I2Cx);
        }
        return;
    }

    // Stop condition detected
    if (LL_I2C_IsEnabledIT_STOP(I2Cx) && LL_I2C_IsActiveFlag_STOP(I2Cx)) {
        LL_I2C_ClearFlag_STOP(I2Cx);
        // Disable all transfer interrupts
        LL_I2C_DisableIT_TX(I2Cx);
        LL_I2C_DisableIT_RX(I2Cx);
        LL_I2C_DisableIT_TC(I2Cx);
        LL_I2C_DisableIT_STOP(I2Cx);
        LL_I2C_DisableIT_NACK(I2Cx);
        LL_I2C_DisableIT_ERR(I2Cx);
        state->busy = false;
        return;
    }

    // Catch-all: no flag matched but the IRQ fired. Disable every IT to
    // break out of any tail-chain we don't recognise; the transaction
    // state machine will time out cleanly from the foreground.
    LL_I2C_DisableIT_TX(I2Cx);
    LL_I2C_DisableIT_RX(I2Cx);
    LL_I2C_DisableIT_TC(I2Cx);
    LL_I2C_DisableIT_STOP(I2Cx);
    LL_I2C_DisableIT_NACK(I2Cx);
    LL_I2C_DisableIT_ERR(I2Cx);
}

// I2C error IRQ handler
static void i2cERIRQHandler(i2cDevice_e device)
{
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].hardware->reg;
    i2cState_t *state = &i2cDevice[device].state;

    if (!LL_I2C_IsEnabledIT_ERR(I2Cx)) {
        return;
    }

    if (LL_I2C_IsActiveFlag_BERR(I2Cx)) {
        i2cCaptureFail(device, I2C_FAIL_BERR);  // capture before clearing the flag
        LL_I2C_ClearFlag_BERR(I2Cx);
        state->error = true;
    }

    if (LL_I2C_IsActiveFlag_ARLO(I2Cx)) {
        i2cCaptureFail(device, I2C_FAIL_ARLO);
        LL_I2C_ClearFlag_ARLO(I2Cx);
        state->error = true;
    }

    if (LL_I2C_IsActiveFlag_OVR(I2Cx)) {
        i2cCaptureFail(device, I2C_FAIL_OVR);
        LL_I2C_ClearFlag_OVR(I2Cx);
        state->error = true;
    }

    if (state->error) {
        i2cRecoverFromISRError(I2Cx, state);
    }
}

#ifdef USE_I2C_DEVICE_1
void I2C1_ER_IRQHandler(void)
{
    i2cERIRQHandler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void)
{
    i2cEVIRQHandler(I2CDEV_1);
}
#endif

#ifdef USE_I2C_DEVICE_2
void I2C2_ER_IRQHandler(void)
{
    i2cERIRQHandler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void)
{
    i2cEVIRQHandler(I2CDEV_2);
}
#endif

#ifdef USE_I2C_DEVICE_3
void I2C3_ER_IRQHandler(void)
{
    i2cERIRQHandler(I2CDEV_3);
}

void I2C3_EV_IRQHandler(void)
{
    i2cEVIRQHandler(I2CDEV_3);
}
#endif

#ifdef USE_I2C_DEVICE_4
void I2C4_ER_IRQHandler(void)
{
    i2cERIRQHandler(I2CDEV_4);
}

void I2C4_EV_IRQHandler(void)
{
    i2cEVIRQHandler(I2CDEV_4);
}
#endif

static bool i2cHandleHardwareFailure(i2cDevice_e device, i2cFailReason_e reason)
{
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;
    i2cState_t *state = &i2cDevice[device].state;
    i2cCaptureFail(device, reason);
    i2cRecoverFromISRError(I2Cx, state);
    return false;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

// Blocking write
bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;

    if (state->busy) {
        return false;
    }

    state->busy = true;
    state->error = false;
    state->transactionStartUs = microsISR();

    timeUs_t timeoutStartUs = microsISR();

    if (reg_ == 0xFF) {
        // No register address, just send data byte
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              1, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
            }
        }
        LL_I2C_TransmitData8(I2Cx, data);
    } else {
        // Send register address + data byte
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              2, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS then send register address
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
            }
        }
        LL_I2C_TransmitData8(I2Cx, reg_);

        // Wait for TXIS then send data byte
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
            }
        }
        LL_I2C_TransmitData8(I2Cx, data);
    }

    // Wait for STOP flag (AUTOEND generates stop automatically)
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {
        if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
            LL_I2C_ClearFlag_NACK(I2Cx);
            return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
        }
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
        }
    }
    LL_I2C_ClearFlag_STOP(I2Cx);

    state->busy = false;

    return true;
}

// Non-blocking write
bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;

    if (state->busy) {
        return false;
    }

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 1;
    state->reading = 0;
    state->write_p = data;
    state->read_p = data;
    state->bytes = len_;
    state->busy = true;
    state->error = false;
    state->transactionStartUs = microsISR();

    if (reg_ == 0xFF) {
        // No register address, just send data bytes
        LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                              len_, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_WRITE);
    } else {
        // Send register address + data bytes
        // First byte is register address, transmitted in ISR before data
        LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                              1 + len_, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS to send the register address byte synchronously
        // so the ISR only deals with data bytes
        timeUs_t timeoutStartUs = microsISR();
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
            }
        }
        LL_I2C_TransmitData8(I2Cx, reg_);
    }

    // Enable interrupts in a single atomic CR1 write — the LL Enable*
    // helpers are RMW (LDR/ORR/STR). The IRQ runs at the highest NVIC
    // priority, so a single ISR-driven Disable* between the foreground
    // LDR and STR makes the stale STR re-set bits the IRQ just cleared,
    // leaving stray ITs enabled that tail-chain the next transaction.
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        I2Cx->CR1 |= I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE;
        __set_PRIMASK(primask);
    }

    return true;
}

// Blocking read
bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;

    if (state->busy) {
        return false;
    }

    state->busy = true;
    state->error = false;
    state->transactionStartUs = microsISR();

    timeUs_t timeoutStartUs = microsISR();

    if (reg_ == 0xFF) {
        // No register address, just receive data
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              len, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_READ);
    } else {
        // First write the register address with SOFTEND (no stop, restart follows)
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              1, LL_I2C_MODE_SOFTEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS then send register address
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
            }
        }
        LL_I2C_TransmitData8(I2Cx, reg_);

        // Wait for Transfer Complete (TC) - SOFTEND means no auto-stop
        while (!LL_I2C_IsActiveFlag_TC(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
            }
        }

        // Now send restart for the read phase
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              len, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_READ);
    }

    // Read all bytes
    for (uint8_t i = 0; i < len; i++) {
        while (!LL_I2C_IsActiveFlag_RXNE(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
            }
        }
        buf[i] = LL_I2C_ReceiveData8(I2Cx);
    }

    // Wait for STOP flag (AUTOEND generates stop automatically)
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {
        if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
            LL_I2C_ClearFlag_NACK(I2Cx);
            return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
        }
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
        }
    }
    LL_I2C_ClearFlag_STOP(I2Cx);

    state->busy = false;

    return true;
}

// Non-blocking read
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;

    if (state->busy) {
        return false;
    }

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 0;
    state->reading = 1;
    state->write_p = buf;
    state->read_p = buf;
    state->bytes = len;
    state->busy = true;
    state->error = false;
    state->transactionStartUs = microsISR();

    uint32_t crBitsToEnable;

    if (reg_ == 0xFF) {
        // No register address, directly start read
        LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                              len, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_READ);

        crBitsToEnable = I2C_CR1_RXIE;
    } else {
        // First send register address with SOFTEND, TC interrupt will trigger restart
        LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                              1, LL_I2C_MODE_SOFTEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS to send the register address byte synchronously
        timeUs_t timeoutStartUs = microsISR();
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device, I2C_FAIL_NACK);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);
            }
        }
        LL_I2C_TransmitData8(I2Cx, reg_);

        // TC interrupt fires after slave ACKs the reg byte; ISR handles
        // the RESTART then switches to RX interrupts for the read phase.
        crBitsToEnable = I2C_CR1_TCIE;
    }

    // Single atomic CR1 write — see comment in i2cWriteBuffer for why
    // separate Enable* RMWs can race the ISR's Disable* and leave stray
    // ITs enabled that tail-chain the next transaction.
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        I2Cx->CR1 |= crBitsToEnable | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE;
        __set_PRIMASK(primask);
    }

    return true;
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        if (error) {
            *error = true;
        }
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;

    if (state->busy && cmpTimeUs(microsISR(), state->transactionStartUs) >= I2C_TIMEOUT_US) {
        // Transfer has stalled — forcefully recover.
        // Disable I2C NVIC IRQs to prevent the ISR from racing with recovery.
        const i2cHardware_t *hardware = i2cDevice[device].hardware;
        if (hardware) {
            HAL_NVIC_DisableIRQ(hardware->ev_irq);
            HAL_NVIC_DisableIRQ(hardware->er_irq);
        }

        i2cHandleHardwareFailure(device, I2C_FAIL_TIMEOUT);

        if (hardware) {
            HAL_NVIC_EnableIRQ(hardware->ev_irq);
            HAL_NVIC_EnableIRQ(hardware->er_irq);
        }
    }

    if (error) {
        *error = state->error;
    }

    return state->busy;
}

#endif
