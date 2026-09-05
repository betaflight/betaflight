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

#if defined(USE_ACCGYRO_ADIS1657X)

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_adis1657x.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/gyrodev.h"

#include "sensors/gyro.h"

/*
 * Analog Devices ADIS16575 / ADIS16576 / ADIS16577.
 *
 * The part is configured for the 32-bit burst and readFn keeps the high word of
 * each axis, so gyroADCRaw[] and ADCRaw[] get one int16_t per channel with no
 * shifting. The FIFO is left disabled, so one data-ready pulse is one sample.
 */

#define GYRO_EXTI_DETECT_THRESHOLD      1000

#define ADIS1657X_RESET_DELAY_MS        350
#define ADIS1657X_RESET_RETRIES         10
#define ADIS1657X_PROD_ID_RETRIES       5
#define ADIS1657X_WRITE_RETRIES         16

// PROD_ID is read once during detection and needed again by the accelerometer's
// initFn, which has no bus transaction of its own. Two ADIS1657x variants on one
// board would share this; that combination is not supported.
static uint16_t adis1657xProdId;

/*
 * Reads are pipelined: the frame carrying the address clocks out whatever the
 * previous frame asked for, so the answer needs a second frame of its own.
 */
static uint16_t adis1657xReadReg(const extDevice_t *dev, uint8_t reg)
{
    uint8_t txBuf[2] = { reg, 0 };
    uint8_t rxBuf[2] = { 0 };

    spiReadWriteBuf(dev, txBuf, NULL, sizeof(txBuf));
    delayMicroseconds(ADIS1657X_STALL_US);
    spiReadWriteBuf(dev, NULL, rxBuf, sizeof(rxBuf));

    return (rxBuf[0] << 8) | rxBuf[1];
}

static void adis1657xWriteReg(const extDevice_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t txBuf[2];

    txBuf[0] = reg | ADIS1657X_WRITE_BIT;
    txBuf[1] = value & 0xff;
    spiReadWriteBuf(dev, txBuf, NULL, sizeof(txBuf));
    delayMicroseconds(ADIS1657X_STALL_US);

    txBuf[0] = (reg + 1) | ADIS1657X_WRITE_BIT;
    txBuf[1] = value >> 8;
    spiReadWriteBuf(dev, txBuf, NULL, sizeof(txBuf));
    delayMicroseconds(ADIS1657X_STALL_US);
}

static bool adis1657xWriteRegVerify(const extDevice_t *dev, uint8_t reg, uint16_t value)
{
    for (int i = 0; i < ADIS1657X_WRITE_RETRIES; i++) {
        adis1657xWriteReg(dev, reg, value);
        if (adis1657xReadReg(dev, reg) == value) {
            return true;
        }
    }

    return false;
}

static bool adis1657xProdIdValid(uint16_t prodId)
{
    return (prodId == ADIS1657X_PROD_ID_16575)
        || (prodId == ADIS1657X_PROD_ID_16576)
        || (prodId == ADIS1657X_PROD_ID_16577);
}

uint8_t adis1657xSpiDetect(const extDevice_t *dev)
{
    /* Deliberately last in gyroSpiDetectFnTable[]. An ADI address byte with bit 7
     * clear is a read, but that is the *write* encoding on every Invensense part
     * here, so probing this one sends what an ICM would take as a write of 0x00 to
     * its register 0x72. Running last means a known part has already claimed the
     * bus.
     */
    for (int tries = 0; tries < ADIS1657X_PROD_ID_RETRIES; tries++) {
        const uint16_t prodId = adis1657xReadReg(dev, ADIS1657X_REG_PROD_ID);

        if (adis1657xProdIdValid(prodId)) {
            adis1657xProdId = prodId;
            return ADIS1657X_SPI;
        }
    }

    return MPU_NONE;
}

static void adis1657xGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    // Default off: the filter's group delay is undocumented, and an unquantified
    // phase lag is not acceptable in a 2 kHz PID loop.
    static const uint16_t filterOptions[GYRO_HARDWARE_LPF_COUNT] = {
        [GYRO_HARDWARE_LPF_NORMAL] = ADIS1657X_FILT_CTRL_OFF,
        [GYRO_HARDWARE_LPF_OPTION_1] = ADIS1657X_FILT_CTRL_4_TAP,
        [GYRO_HARDWARE_LPF_OPTION_2] = ADIS1657X_FILT_CTRL_16_TAP,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        [GYRO_HARDWARE_LPF_EXPERIMENTAL] = ADIS1657X_FILT_CTRL_64_TAP,
#endif
    };

    spiSetClkDivisor(dev, spiCalculateDivider(ADIS1657X_MAX_SPI_CLK_HZ));

    for (int tries = 0; tries < ADIS1657X_RESET_RETRIES; tries++) {
        adis1657xWriteReg(dev, ADIS1657X_REG_GLOB_CMD, ADIS1657X_GLOB_CMD_SW_RESET);
        delay(ADIS1657X_RESET_DELAY_MS);

        const uint16_t prodId = adis1657xReadReg(dev, ADIS1657X_REG_PROD_ID);
        if (adis1657xProdIdValid(prodId)) {
            adis1657xProdId = prodId;
            break;
        }
    }

    /* RNG_MDL reports the fitted gyro range, so there is no build-time variant to
     * get wrong. Anything but the high-range code falls back to the low range,
     * which under-reports rather than over-reports.
     */
    const uint16_t rngMdl = adis1657xReadReg(dev, ADIS1657X_REG_RNG_MDL) & ADIS1657X_RNG_MDL_RANGE_MASK;
    if (rngMdl == ADIS1657X_RNG_MDL_HIGH_RANGE) {
        gyro->scale = ADIS1657X_GYRO_SCALE_HIGH;
    } else {
        gyro->scale = ADIS1657X_GYRO_SCALE_LOW;
    }

    adis1657xWriteRegVerify(dev, ADIS1657X_REG_MSC_CTRL, ADIS1657X_MSC_CTRL_CFG);

    adis1657xWriteRegVerify(dev, ADIS1657X_REG_FILT_CTRL,
                            filterOptions[gyroConfig()->gyro_hardware_lpf]);

    adis1657xWriteRegVerify(dev, ADIS1657X_REG_DEC_RATE, ADIS1657X_DEC_RATE_2000HZ);

    adis1657xWriteRegVerify(dev, ADIS1657X_REG_FIFO_CTRL, ADIS1657X_FIFO_CTRL_DISABLED);

    gyro->tempScale = ADIS1657X_TEMP_SCALE;
    gyro->tempZero = ADIS1657X_TEMP_ZERO;

    mpuGyroInit(gyro);

    /* Only dmaReadRegStart is corrected. The per-axis output registers are never
     * read - the burst is the only transaction that carries samples - so their
     * addresses are deliberately absent rather than guessed.
     */
    gyro->dmaReadRegStart = ADIS1657X_BURST_CMD;
}

static void adis1657xAccInit(accDev_t *acc)
{
    switch (adis1657xProdId) {
    case ADIS1657X_PROD_ID_16576:
        acc->acc_1G = ADIS1657X_ACC_1G_16576;
        break;
    case ADIS1657X_PROD_ID_16577:
        acc->acc_1G = ADIS1657X_ACC_1G_16577;
        break;
    case ADIS1657X_PROD_ID_16575:
    default:
        acc->acc_1G = ADIS1657X_ACC_1G_16575;
        break;
    }
}

static bool adis1657xBurstChecksumValid(const uint16_t *frame)
{
    const uint8_t *bytes = (const uint8_t *)&frame[ADIS1657X_BURST_CHECKSUM_FIRST_WORD];
    const uint8_t *end = (const uint8_t *)&frame[ADIS1657X_BURST_CHECKSUM_END_WORD];
    uint16_t sum = 0;
    uint8_t bits = 0;

    while (bytes < end) {
        sum += *bytes;
        bits |= *bytes;
        bytes++;
    }

    /* Zero is a fixed point of a truncated sum, so an all-zero frame validates -
     * and that is what a bus with MISO held low returns. No real sample has every
     * bit clear: the accelerometer cannot read zero on all three axes.
     */
    if (bits == 0) {
        return false;
    }

    return sum == __builtin_bswap16(frame[ADIS1657X_BURST_WORD_CHECKSUM]);
}

/*
 * DIAG_STAT is carried in the frame but deliberately not acted on. Its bits are
 * sticky, and Betaflight has no way to report or recover from a sensor fault -
 * readFn returning false only means "no new data" - so rejecting flagged frames
 * would turn one transient into a permanent freeze on the last good sample.
 */
static bool adis1657xUnpackGyro(gyroDev_t *gyro)
{
    static uint16_t lastDataCntr;

    const uint16_t *frame = (const uint16_t *)gyro->dev.rxBuf;

    if (!adis1657xBurstChecksumValid(frame)) {
        return false;
    }

    // The counter only advances when the part produces a sample, so this is what
    // distinguishes a fresh burst from a re-read of the same output registers.
    const uint16_t dataCntr = __builtin_bswap16(frame[ADIS1657X_BURST_WORD_DATA_CNTR]);
    if (dataCntr == lastDataCntr) {
        return false;
    }
    lastDataCntr = dataCntr;

    gyro->gyroADCRaw[X] = (int16_t)__builtin_bswap16(frame[ADIS1657X_BURST_GYRO_HIGH(X)]);
    gyro->gyroADCRaw[Y] = (int16_t)__builtin_bswap16(frame[ADIS1657X_BURST_GYRO_HIGH(Y)]);
    gyro->gyroADCRaw[Z] = (int16_t)__builtin_bswap16(frame[ADIS1657X_BURST_GYRO_HIGH(Z)]);

    // Same frame, so no temperatureFn and no extra transaction.
    gyro->temperature = (int16_t)(((int16_t)__builtin_bswap16(frame[ADIS1657X_BURST_WORD_TEMPERATURE]))
                                 * gyro->tempScale + gyro->tempZero);

    return true;
}

static void adis1657xStartBurst(gyroDev_t *gyro)
{
    gyro->dev.txBuf[0] = ADIS1657X_BURST_CMD;

    busSegment_t segments[] = {
        {.u.buffers = {NULL, NULL}, ADIS1657X_BURST_FRAME_LEN, true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    segments[0].u.buffers.txData = gyro->dev.txBuf;
    segments[0].u.buffers.rxData = gyro->dev.rxBuf;

    spiSequence(&gyro->dev, &segments[0]);
    spiWait(&gyro->dev);
}

static bool adis1657xGyroReadSPI(gyroDev_t *gyro)
{
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Zero, not 0xff: trailing bytes must not look like another command.
        memset(gyro->dev.txBuf, 0, ADIS1657X_BURST_FRAME_LEN);

        gyro->gyroDmaMaxDuration = 5;
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
#ifdef USE_DMA
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uintptr_t)gyro;
                gyro->dev.txBuf[0] = ADIS1657X_BURST_CMD;
                gyro->segments[0].len = ADIS1657X_BURST_FRAME_LEN;
                gyro->segments[0].callback = mpuIntCallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                // No &rxBuf[1] offset: the command echo aligns the payload itself.
                gyro->segments[0].u.buffers.rxData = gyro->dev.rxBuf;
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else
#endif
            {
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
        adis1657xStartBurst(gyro);
        return adis1657xUnpackGyro(gyro);

    case GYRO_EXTI_INT_DMA:
        // Started by the EXTI handler; don't wait. Worst case is a stale sample,
        // which the data counter check then rejects.
        return adis1657xUnpackGyro(gyro);

    default:
        break;
    }

    return true;
}

static bool adis1657xAccReadSPI(accDev_t *acc)
{
    // Same frame the gyro read already fetched, in every mode - no separate accel
    // transaction. The data counter is not checked here: the gyro owns it, and a
    // repeated sample is harmless at the rate the accelerometer is consumed.
    const uint16_t *frame = (const uint16_t *)acc->gyro->dev.rxBuf;

    if (acc->gyro->gyroModeSPI == GYRO_EXTI_INIT) {
        return true;
    }

    if (!adis1657xBurstChecksumValid(frame)) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)__builtin_bswap16(frame[ADIS1657X_BURST_ACCEL_HIGH(X)]);
    acc->ADCRaw[Y] = (int16_t)__builtin_bswap16(frame[ADIS1657X_BURST_ACCEL_HIGH(Y)]);
    acc->ADCRaw[Z] = (int16_t)__builtin_bswap16(frame[ADIS1657X_BURST_ACCEL_HIGH(Z)]);

    return true;
}

bool adis1657xSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ADIS1657X_SPI) {
        return false;
    }

    gyro->initFn = adis1657xGyroInit;
    gyro->readFn = adis1657xGyroReadSPI;

    return true;
}

bool adis1657xSpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ADIS1657X_SPI) {
        return false;
    }

    acc->initFn = adis1657xAccInit;
    acc->readFn = adis1657xAccReadSPI;

    return true;
}

#endif // USE_ACCGYRO_ADIS1657X
