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

#pragma once

#include <stdbool.h>

#include "platform.h"

#include "drivers/io_types.h"

#ifndef I2C_DEVICE
#define I2C_DEVICE I2CINVALID
#endif

typedef enum {
    I2CINVALID = -1,
    I2CDEV_FIRST = 0,
#if defined(USE_I2C_DEVICE_0)
    I2CDEV_0   = I2CDEV_FIRST,
    I2CDEV_1,
#else
    I2CDEV_1   = I2CDEV_FIRST,
#endif
    I2CDEV_2,
    I2CDEV_3,
    I2CDEV_4,
    I2CDEV_5,
    I2CDEV_6,
    I2CDEV_7,
    I2CDEV_8,
    I2CDEV_9,
    I2CDEV_10,
} i2cDevice_e;

// Macros to convert between CLI bus number and i2cDevice_e.
#define I2C_CFG_TO_DEV(x)   ((x) - 1)
#define I2C_DEV_TO_CFG(x)   ((x) + 1)

// I2C device address range in 7-bit address mode
#define I2C_ADDR7_MIN       8
#define I2C_ADDR7_MAX       119

struct i2cConfig_s;
void i2cPinConfigure(const struct i2cConfig_s *i2cConfig);
void i2cInit(i2cDevice_e device);
bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg, uint8_t data);
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
bool i2cBusy(i2cDevice_e device, bool *error);

uint16_t i2cGetErrorCounter(void);
uint8_t i2cGetRegisteredDeviceCount(void);

// Startup bus-health diagnostic recorded by i2cInit() and surfaced in CLI
// status so a dead bus is visible instead of failing silently. Two independent
// checks, kept separate so a board that legitimately relies on the MCU internal
// pull-up is never mis-reported as faulty:
//
//   1. Usability (pull-strategy agnostic): engage the internal pull-UP and read.
//      An unloaded, functional line reaches HIGH; if it stays LOW something is
//      holding it down — a short, a stuck device, or a non-functional pin. This
//      is the actual "bus unusable" signal (*_LOW bits).
//   2. External pull-up presence: only when the board is NOT using the internal
//      pull-up. Engage the internal pull-DOWN — a real external pull-up (few
//      kOhm) overrides it and reads HIGH, its absence reads LOW. Purely
//      informational (a wiring note), NOT an unusable-bus condition (*_NOPULL).
#define I2C_HEALTH_CHECKED     (1 << 0)  // bus was configured and checked at init
#define I2C_HEALTH_SCL_LOW     (1 << 1)  // SCL held low despite internal pull-up (unusable)
#define I2C_HEALTH_SDA_LOW     (1 << 2)  // SDA held low despite internal pull-up (unusable)
#define I2C_HEALTH_SCL_NOPULL  (1 << 3)  // no external SCL pull-up detected (info only)
#define I2C_HEALTH_SDA_NOPULL  (1 << 4)  // no external SDA pull-up detected (info only)
void i2cReportBusHealth(i2cDevice_e device, uint8_t health);
uint8_t i2cGetBusHealth(i2cDevice_e device);

// Classification of the last failed transaction, recorded by the transaction
// path so a dead bus can be told apart from a silent (but healthy) bus:
//   NACK    - address/data byte not acknowledged; the peripheral clocked the
//             bus correctly, the target simply did not answer (wiring/address/
//             sensor-rail issue, NOT the peripheral).
//   BERR    - misplaced START/STOP: bus contention / protocol violation.
//   ARLO    - arbitration lost: another master or a line stuck low mid-frame.
//   OVR     - over/under-run (slave mode; unexpected here).
//   TIMEOUT - the foreground state machine gave up with no completion IRQ; the
//             peripheral never advanced (e.g. START never issued, BUSY stuck,
//             no kernel clock reaching the bus).
typedef enum {
    I2C_FAIL_NONE = 0,
    I2C_FAIL_NACK,
    I2C_FAIL_BERR,
    I2C_FAIL_ARLO,
    I2C_FAIL_OVR,
    I2C_FAIL_TIMEOUT,
} i2cFailReason_e;

#if defined(STM32H5)
// TEMPORARY (Development Instrumentation): live register snapshot for an I2C bus,
// to localise a dead-bus fault to clock-mux vs peripheral-enable vs pin/AF in a
// single flash. Remove once the H5 I2C3 bring-up issue is resolved.
typedef struct i2cDebugRegs_s {
    uint8_t clkSel;      // RCC_CCIPR4 I2CxSEL field (0 = PCLKx, the expected default)
    bool busClockOn;     // RCC APBxENR I2CxEN
    bool peEnabled;      // I2Cx->CR1 PE
    uint32_t timingr;    // I2Cx->TIMINGR (0 = uninitialised)
    uint8_t sclMode;     // SCL pin MODER (2 = alternate function)
    uint8_t sclAf;       // SCL pin AF selection (expect 4 for I2C)
    uint8_t sdaMode;     // SDA pin MODER
    uint8_t sdaAf;       // SDA pin AF selection
    bool sclLevel;       // live SCL line level from IDR now (input buffer stays live in AF-OD; 1 = idle high)
    bool sdaLevel;       // live SDA line level from IDR now (1 = idle high)
    uint8_t initHealth;  // bus-health bitmask recorded at i2cInit() (I2C_HEALTH_* bits)
    // Kernel-clock health — closes the "is HSI actually the running I2C3/4
    // kernel source?" question on the record (I2C1/2 stay on PCLK, so these are
    // only meaningful for the HSI-sourced buses).
    bool hsiReady;       // RCC_CR HSIRDY (kernel oscillator locked)
    uint8_t hsiDiv;      // RCC_CR HSIDIV (0=>/1 .. 3=>/8); kernel = 64 MHz >> hsiDiv
    // Last-failure capture (from the transaction path). lastIsr/lastCr2 are the
    // raw I2Cx->ISR / I2Cx->CR2 sampled at the moment of failure, so no bit is
    // lost to interpretation; the per-reason counters summarise the boot-time
    // device-detection sweep.
    uint32_t lastIsr;
    uint32_t lastCr2;
    uint8_t  lastReason; // i2cFailReason_e
    uint16_t failTotal;
    uint16_t failNack;
    uint16_t failBerr;
    uint16_t failArlo;
    uint16_t failOvr;
    uint16_t failTimeout;
} i2cDebugRegs_t;
bool i2cGetDebugRegs(i2cDevice_e device, i2cDebugRegs_t *regs);
// Copy the transaction-path failure capture for a bus into regs (lives in the
// LL transaction unit, separate from the init unit that fills the rest).
void i2cGetFailDiag(i2cDevice_e device, i2cDebugRegs_t *regs);
// Zero a bus's failure capture — used by "i2c_regs scan" so the live sweep's
// result is not confused with the boot-time counts.
void i2cResetFailDiag(i2cDevice_e device);
#endif
