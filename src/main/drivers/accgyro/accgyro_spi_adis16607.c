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

#if defined(USE_ACCGYRO_ADIS16607)

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_adis16607.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "pg/gyrodev.h"

#include "sensors/gyro.h"

/*
 * Analog Devices ADIS16607 precision MEMS IMU, gyro and accelerometer.
 *
 * Driven in the 16-bit output format, so readFn gets one int16_t per channel
 * and the LOW registers never cross the bus. The FIFO is disabled, so the burst
 * reads the live output registers and one data-ready pulse gives exactly one
 * fresh sample.
 */

#define ADIS16607_DEC_RATE_SETTING      ADIS16607_DEC_RATE_4780HZ

#define GYRO_EXTI_DETECT_THRESHOLD      1000

#define ADIS16607_RESET_DELAY_MS        200
#define ADIS16607_STATUS_RETRIES        20
#define ADIS16607_STATUS_DELAY_MS       10
#define ADIS16607_DEV_ID_RETRIES        5
#define ADIS16607_WRITE_RETRIES         8
#define ADIS16607_CONFIG_RETRIES        3

static bool adis16607ConfigOk;

// Half duplex: the response arrives in the same 32-bit frame as the command.
static uint16_t adis16607ReadReg(const extDevice_t *dev, uint8_t reg)
{
    uint8_t txBuf[4] = { reg | ADIS16607_READ_BIT, 0, 0, 0 };
    uint8_t rxBuf[4] = { 0 };

    spiReadWriteBuf(dev, txBuf, rxBuf, sizeof(txBuf));

    return (rxBuf[2] << 8) | rxBuf[3];
}

static void adis16607WriteReg(const extDevice_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t txBuf[3] = { reg, value >> 8, value & 0xff };

    spiReadWriteBuf(dev, txBuf, NULL, sizeof(txBuf));
}

// The part silently drops writes while locked or while the bootloader runs.
static bool adis16607WriteRegVerify(const extDevice_t *dev, uint8_t reg, uint16_t value)
{
    for (int i = 0; i < ADIS16607_WRITE_RETRIES; i++) {
        adis16607WriteReg(dev, reg, value);
        if (adis16607ReadReg(dev, reg) == value) {
            return true;
        }
    }

    return false;
}

static void adis16607Unlock(const extDevice_t *dev)
{
    adis16607WriteReg(dev, ADIS16607_REG_WRITE_LOCK, ADIS16607_WRITE_LOCK_KEY_A);
    adis16607WriteReg(dev, ADIS16607_REG_WRITE_LOCK, ADIS16607_WRITE_LOCK_KEY_B);
}

static void adis16607Lock(const extDevice_t *dev)
{
    adis16607WriteReg(dev, ADIS16607_REG_WRITE_LOCK, ADIS16607_WRITE_LOCK_KEY_B);
    adis16607WriteReg(dev, ADIS16607_REG_WRITE_LOCK, ADIS16607_WRITE_LOCK_KEY_A);
}

uint8_t adis16607SpiDetect(const extDevice_t *dev)
{
    /* This probe must write before it can read. Out of reset the part tells full
     * from half duplex by frame length, and a read is 32 bits either way, so it
     * resolves a bare read as full duplex and answers with a CRC error. The
     * 24-bit key write is the only unambiguously half-duplex frame. With no ADIS
     * fitted it writes 0xB4B4 to register 0x32 of whatever else is on the bus,
     * so this probe is deliberately last in gyroSpiDetectFnTable[].
     */
    for (int tries = 0; tries < ADIS16607_DEV_ID_RETRIES; tries++) {
        adis16607WriteReg(dev, ADIS16607_REG_SPI_HALFDUPLEX_KEY, ADIS16607_SPI_HALFDUPLEX_KEY_VAL);

        if (adis16607ReadReg(dev, ADIS16607_REG_DEV_ID) == ADIS16607_DEV_ID_CONST) {
            return ADIS16607_SPI;
        }
    }

    return MPU_NONE;
}

/*
 * The data counter is the only thing in a burst that separates a fresh sample
 * from a re-read of output registers the part has not rewritten yet: the
 * checksum matches either way. Held per device, since two of these can share a
 * build.
 */
typedef struct {
    const gyroDev_t *gyro;
    uint16_t dataCntr;
    bool valid;
} adis16607FrameState_t;

static adis16607FrameState_t frameStates[MAX_GYRODEV_COUNT];

static adis16607FrameState_t *adis16607FrameState(const gyroDev_t *gyro)
{
    for (unsigned i = 0; i < ARRAYLEN(frameStates); i++) {
        if (frameStates[i].gyro == gyro) {
            return &frameStates[i];
        }
    }

    for (unsigned i = 0; i < ARRAYLEN(frameStates); i++) {
        if (!frameStates[i].gyro) {
            frameStates[i].gyro = gyro;
            return &frameStates[i];
        }
    }

    return NULL;
}

// Every write here changes how the burst is laid out or how fast it arrives, so
// each one is verified and the caller retries the whole block on failure.
static bool adis16607Configure(const extDevice_t *dev, uint16_t filterBandwidth)
{
    bool ok = true;

    // Data ready on GPIO3, the pin wired to GYRO_1_EXTI_PIN.
    ok = adis16607WriteRegVerify(dev, ADIS16607_REG_USER_GPIO_CFG1, ADIS16607_GPIO_CFG1_GPIO3_DATA_RDY) && ok;

    ok = adis16607WriteRegVerify(dev, ADIS16607_REG_USER_DATA_CFG, ADIS16607_USER_DATA_CFG) && ok;

    ok = adis16607WriteRegVerify(dev, ADIS16607_REG_MSC_CTRL, filterBandwidth) && ok;

    // Zero DEC_RATE first so the decimation accumulator resets.
    ok = adis16607WriteRegVerify(dev, ADIS16607_REG_DEC_RATE, ADIS16607_DEC_RATE_9560HZ) && ok;
    ok = adis16607WriteRegVerify(dev, ADIS16607_REG_DEC_RATE, ADIS16607_DEC_RATE_SETTING) && ok;

    ok = adis16607WriteRegVerify(dev, ADIS16607_REG_USER_FIFO_CFG, ADIS16607_FIFO_CFG_DISABLED) && ok;

    return ok;
}

static void adis16607GyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    // Default off: the filter's phase lag is unquantified, which is not
    // acceptable in a 4kHz PID loop.
    static const uint16_t filterBandwidthOptions[GYRO_HARDWARE_LPF_COUNT] = {
        [GYRO_HARDWARE_LPF_NORMAL] = ADIS16607_MSC_CTRL_FILT_BW_OFF,
        [GYRO_HARDWARE_LPF_OPTION_1] = ADIS16607_MSC_CTRL_FILT_BW_500HZ,
        [GYRO_HARDWARE_LPF_OPTION_2] = ADIS16607_MSC_CTRL_FILT_BW_100HZ,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        [GYRO_HARDWARE_LPF_EXPERIMENTAL] = ADIS16607_MSC_CTRL_FILT_BW_20HZ,
#endif
    };

    spiSetClkDivisor(dev, spiCalculateDivider(ADIS16607_MAX_SPI_CLK_HZ));

    // SOFT_RESET restores register defaults, which forgets the SPI mode key.
    for (int tries = 0; tries < ADIS16607_DEV_ID_RETRIES; tries++) {
        adis16607Unlock(dev);
        adis16607WriteReg(dev, ADIS16607_REG_SOFT_RESET, ADIS16607_SOFT_RESET_REG_RESET);
        delay(ADIS16607_RESET_DELAY_MS);
        adis16607WriteReg(dev, ADIS16607_REG_SPI_HALFDUPLEX_KEY, ADIS16607_SPI_HALFDUPLEX_KEY_VAL);

        if (adis16607ReadReg(dev, ADIS16607_REG_DEV_ID) == ADIS16607_DEV_ID_CONST) {
            break;
        }
    }

    for (int tries = 0; tries < ADIS16607_STATUS_RETRIES; tries++) {
        if (!(adis16607ReadReg(dev, ADIS16607_REG_DIGITAL_STATUS) & ADIS16607_DIGITAL_STATUS_BOOTLOAD_BUSY)) {
            break;
        }
        delay(ADIS16607_STATUS_DELAY_MS);
    }

    // Let the startup errors settle. These reads may not clear the flags, so do
    // not rely on them to.
    for (int tries = 0; tries < ADIS16607_STATUS_RETRIES; tries++) {
        if (adis16607ReadReg(dev, ADIS16607_REG_DIAG_STAT) == 0) {
            break;
        }
        delay(ADIS16607_STATUS_DELAY_MS);
    }

    /* A configuration write the part drops is not cosmetic: the wrong
     * USER_DATA_CFG shifts every word of the burst, and the wrong DEC_RATE runs
     * the part at an ODR the scheduler was not told about. Unlock and retry the
     * block until it reads back. If it never does, the reads that follow will
     * fail their checksum or their data counter rather than publish a
     * misinterpreted frame.
     */
    adis16607ConfigOk = false;
    for (int tries = 0; tries < ADIS16607_CONFIG_RETRIES; tries++) {
        adis16607Unlock(dev);
        if (adis16607Configure(dev, filterBandwidthOptions[gyroConfig()->gyro_hardware_lpf])) {
            adis16607ConfigOk = true;
            break;
        }
    }

    // Locking is what leaves the initialisation state and starts data-ready.
    adis16607Lock(dev);

    adis16607FrameState_t *state = adis16607FrameState(gyro);
    if (state) {
        state->valid = false;
    }

    gyro->scale = ADIS16607_GYRO_SCALE;
    gyro->tempScale = ADIS16607_TEMP_SCALE;
    gyro->tempZero = ADIS16607_TEMP_ZERO;

    mpuGyroInit(gyro);

    // Replace the MPU-style defaults mpuGyroInit() left behind.
    gyro->gyroDataReg = ADIS16607_REG_X_GYRO_HIGH;
    gyro->accDataReg = ADIS16607_REG_X_ACCEL_HIGH;
    gyro->tempDataReg = ADIS16607_REG_TEMPERATURE;
    gyro->dmaReadRegStart = ADIS16607_BURST_CMD;

    /* mpuIntExtiInit() arms a rising edge, but the output registers are still
     * updating then and a burst can tear across a sample boundary, so sample on
     * the falling edge. Only where mpuIntExtiInit() claimed the pin and installed
     * the handler, since it returns early otherwise.
     */
    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);
#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    if (mpuIntIO && !IORead(mpuIntIO)) {
#else
    if (mpuIntIO) {
#endif
        EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING,
                   BETAFLIGHT_EXTI_TRIGGER_FALLING);
        EXTIEnable(mpuIntIO);
    }
}

static void adis16607AccInit(accDev_t *acc)
{
    acc->acc_1G = ADIS16607_ACC_1G;
}

// Byte-wise so the frame needs no alignment guarantee on gyro->dev.rxBuf, which
// is a plain uint8_t DMA buffer.
static uint16_t adis16607Word(const uint8_t *frame, int word)
{
    return (frame[word * 2] << 8) | frame[word * 2 + 1];
}

static bool adis16607BurstChecksumValid(const uint8_t *frame)
{
    uint16_t sum = 0;
    uint16_t orBits = 0;
    uint16_t andBits = 0xffff;

    for (int i = ADIS16607_BURST_CHECKSUM_FIRST_WORD; i < ADIS16607_BURST_WORD_CHECKSUM; i++) {
        const uint16_t word = adis16607Word(frame, i);
        sum += word;
        orBits |= word;
        andBits &= word;
    }

    /* A truncated sum has fixed points at both rails, so an all-zero and an
     * all-ones frame both validate - and those are exactly what a bus with MISO
     * stuck low or high returns. Neither is a real sample: the accel cannot read
     * zero on all three axes, and 0xffff on every word would need DIAG_STAT to
     * report every fault at once.
     */
    if (orBits == 0 || andBits == 0xffff) {
        return false;
    }

    return sum == adis16607Word(frame, ADIS16607_BURST_WORD_CHECKSUM);
}

static bool adis16607UnpackGyro(gyroDev_t *gyro)
{
    const uint8_t *frame = gyro->dev.rxBuf;

    if (!adis16607BurstChecksumValid(frame)) {
        return false;
    }

    /* An unchanged data counter means the output registers were read twice
     * between two of the part's updates, so this frame repeats the last one. The
     * polled path hits that whenever the gyro task and the 4780Hz ODR drift out
     * of phase, and the DMA path hits it when a transfer completes before the
     * next data-ready edge; publishing it either way feeds the PID loop a rate
     * it has already acted on.
     */
    adis16607FrameState_t *state = adis16607FrameState(gyro);
    const uint16_t dataCntr = adis16607Word(frame, ADIS16607_BURST_WORD_DATA_CNTR);
    if (state) {
        if (state->valid && dataCntr == state->dataCntr) {
            return false;
        }
        state->dataCntr = dataCntr;
        state->valid = true;
    }

    gyro->gyroADCRaw[X] = (int16_t)adis16607Word(frame, ADIS16607_BURST_WORD_GYRO + X);
    gyro->gyroADCRaw[Y] = (int16_t)adis16607Word(frame, ADIS16607_BURST_WORD_GYRO + Y);
    gyro->gyroADCRaw[Z] = (int16_t)adis16607Word(frame, ADIS16607_BURST_WORD_GYRO + Z);

    gyro->temperature = (int16_t)(((int16_t)adis16607Word(frame, ADIS16607_BURST_WORD_TEMPERATURE))
                                 * gyro->tempScale + gyro->tempZero);

    return true;
}

static void adis16607StartBurst(gyroDev_t *gyro)
{
    gyro->dev.txBuf[0] = ADIS16607_BURST_CMD;

    busSegment_t segments[] = {
        {.u.buffers = {NULL, NULL}, ADIS16607_BURST_FRAME_LEN, true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    segments[0].u.buffers.txData = gyro->dev.txBuf;
    segments[0].u.buffers.rxData = gyro->dev.rxBuf;

    spiSequence(&gyro->dev, &segments[0]);
    spiWait(&gyro->dev);
}

static bool adis16607GyroReadSPI(gyroDev_t *gyro)
{
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Zero, not 0xff: trailing bytes must not look like another command.
        memset(gyro->dev.txBuf, 0, ADIS16607_BURST_FRAME_LEN);

        gyro->gyroDmaMaxDuration = 5;
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
#ifdef USE_DMA
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uintptr_t)gyro;
                gyro->dev.txBuf[0] = ADIS16607_BURST_CMD;
                gyro->segments[0].len = ADIS16607_BURST_FRAME_LEN;
                gyro->segments[0].callback = mpuIntCallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                // The burst payload starts at byte 0, so no offset.
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
        adis16607StartBurst(gyro);
        return adis16607UnpackGyro(gyro);

    case GYRO_EXTI_INT_DMA:
        // Started by the EXTI handler; don't wait. Worst case is a stale sample.
        return adis16607UnpackGyro(gyro);

    default:
        break;
    }

    return true;
}

static bool adis16607AccReadSPI(accDev_t *acc)
{
    // Same frame the gyro read already fetched - no separate accel transaction.
    const uint8_t *frame = acc->gyro->dev.rxBuf;

    if (acc->gyro->gyroModeSPI == GYRO_EXTI_INIT) {
        return true;
    }

    if (!adis16607BurstChecksumValid(frame)) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)adis16607Word(frame, ADIS16607_BURST_WORD_ACCEL + X);
    acc->ADCRaw[Y] = (int16_t)adis16607Word(frame, ADIS16607_BURST_WORD_ACCEL + Y);
    acc->ADCRaw[Z] = (int16_t)adis16607Word(frame, ADIS16607_BURST_WORD_ACCEL + Z);

    return true;
}

bool adis16607SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ADIS16607_SPI) {
        return false;
    }

    gyro->initFn = adis16607GyroInit;
    gyro->readFn = adis16607GyroReadSPI;

    return true;
}

bool adis16607SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ADIS16607_SPI) {
        return false;
    }

    acc->initFn = adis16607AccInit;
    acc->readFn = adis16607AccReadSPI;

    return true;
}

#endif // USE_ACCGYRO_ADIS16607
