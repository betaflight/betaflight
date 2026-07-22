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

/*
 * Driver for the TDK InvenSense ICM-56686 6-axis accel/gyro.
 *
 * The ICM-56686 shares the two-tier register architecture of the ICM-456xx
 * family (see accgyro_spi_icm456xx.c), which this driver is closely modelled on:
 *
 *   - DREG_BANK1 : directly addressable over SPI (sensor data, power, ODR/FSR,
 *                  interrupt and FIFO configuration).
 *   - IREG       : indirect register access for everything else (filters,
 *                  offsets, SREG_CTRL). To touch an IREG register the host
 *                  writes the 16-bit target address into IREG_ADDR_15_8 /
 *                  IREG_ADDR_7_0, then writes/reads IREG_DATA. A minimum 4us gap
 *                  is required between consecutive IREG accesses; completion is
 *                  signalled by the IREG_DONE bit in REG_MISC2.
 *
 * IREG bank base addresses (added to the per-bank register offset):
 *   IPREG_SYS1 = 0xA400 (gyro filters/offsets)
 *   IPREG_SYS2 = 0xA500 (accel filters/offsets)
 *   IPREG_TOP1 = 0xA200 (SREG_CTRL, trim, ...)
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_ACCGYRO_ICM56686)

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm56686.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/system.h"

#include "fc/runtime_config.h"

#include "sensors/gyro.h"
#include "pg/gyrodev.h"

// Initial version for testing / verification: use bprintf (PICO trace helper)
#ifndef bprintf
#define bprintf(fmt, ...) do {} while (0)
#endif

// Forward declarations of the SPI read functions (defined further down).
bool icm56686AccReadSPI(accDev_t *acc);
bool icm56686GyroReadSPI(gyroDev_t *gyro);

// ---------------------------------------------------------------------------
// DREG_BANK1 - directly addressable registers (datasheet 14.9)
// ---------------------------------------------------------------------------
#define ICM56686_ACCEL_DATA_X1                  0x00 // 6 bytes accel (X,Y,Z)
#define ICM56686_GYRO_DATA_X1                   0x06 // 6 bytes gyro  (X,Y,Z)
#define ICM56686_TEMP_DATA0                     0x0C // 2 bytes temperature

#define ICM56686_PWR_MGMT0                      0x14
#define ICM56686_INT1_CONFIG0                   0x1A
#define ICM56686_INT1_CONFIG2                   0x1C
#define ICM56686_ACCEL_CONFIG0                  0x1F
#define ICM56686_GYRO_CONFIG0                   0x20
#define ICM56686_WHO_AM_I                       0x72

#define ICM56686_REG_IREG_ADDR_15_8             0x7C
#define ICM56686_REG_IREG_ADDR_7_0              0x7D
#define ICM56686_REG_IREG_DATA                  0x7E
#define ICM56686_REG_MISC2                      0x7F

// REG_MISC2 (0x7F) bits
#define ICM56686_SOFT_RESET                     (1 << 1)
#define ICM56686_BIT_IREG_DONE                  (1 << 0)

// PWR_MGMT0 (0x14): GYRO_MODE[3:2], ACCEL_MODE[1:0], 0b11 = Low-Noise
#define ICM56686_GYRO_MODE_OFF                  (0x00 << 2)
#define ICM56686_GYRO_MODE_STANDBY              (0x01 << 2)
#define ICM56686_GYRO_MODE_LP                   (0x02 << 2)
#define ICM56686_GYRO_MODE_LN                   (0x03 << 2)
#define ICM56686_ACCEL_MODE_OFF                 (0x00)
#define ICM56686_ACCEL_MODE_LP                  (0x02)
#define ICM56686_ACCEL_MODE_LN                  (0x03)

// ACCEL_CONFIG0 (0x1F): AP_ACCEL_FS_SEL[6:4], ACCEL_ODR[3:0]
#define ICM56686_ACCEL_FS_SEL_32G               (0x00 << 4)
#define ICM56686_ACCEL_FS_SEL_16G               (0x01 << 4)
#define ICM56686_ACCEL_FS_SEL_8G                (0x02 << 4)
#define ICM56686_ACCEL_FS_SEL_4G                (0x03 << 4)
#define ICM56686_ACCEL_FS_SEL_2G                (0x04 << 4)
#define ICM56686_ACCEL_ODR_6K4_LN               0x03 // 0..3 all map to 6.4 kHz
#define ICM56686_ACCEL_ODR_3K2_LN               0x04
#define ICM56686_ACCEL_ODR_1K6_LN               0x05
#define ICM56686_ACCEL_ODR_800_LN               0x06

// GYRO_CONFIG0 (0x20): AP_GYRO_FS_SEL[7:4], GYRO_ODR[3:0]
#define ICM56686_GYRO_FS_SEL_4000DPS            (0x00 << 4)
#define ICM56686_GYRO_FS_SEL_2000DPS            (0x01 << 4)
#define ICM56686_GYRO_FS_SEL_1000DPS            (0x02 << 4)
#define ICM56686_GYRO_ODR_6K4_LN                0x03 // 0..3 all map to 6.4 kHz
#define ICM56686_GYRO_ODR_3K2_LN                0x04
#define ICM56686_GYRO_ODR_1K6_LN                0x05
#define ICM56686_GYRO_ODR_800_LN                0x06

// INT1_CONFIG0 (0x1A) - interrupt source enables
#define ICM56686_INT1_STATUS_EN_DRDY            (1 << 2)

// INT1_CONFIG2 (0x1C) - pin electrical configuration
#define ICM56686_INT1_DRIVE_CIRCUIT_PP          (0 << 2)
#define ICM56686_INT1_DRIVE_CIRCUIT_OD          (1 << 2)
#define ICM56686_INT1_MODE_PULSED               (0 << 1)
#define ICM56686_INT1_MODE_LATCHED              (1 << 1)
#define ICM56686_INT1_POLARITY_ACTIVE_LOW       (0 << 0)
#define ICM56686_INT1_POLARITY_ACTIVE_HIGH      (1 << 0)

// ---------------------------------------------------------------------------
// IREG (indirect) register addresses = bank base + register offset
// ---------------------------------------------------------------------------
#define ICM56686_IPREG_SYS1_BASE                0xA400
#define ICM56686_IPREG_SYS2_BASE                0xA500
#define ICM56686_IPREG_TOP1_BASE                0xA200

// SREG_CTRL : IPREG_TOP1 offset 0x60. Controls sensor data resolution/endianness.
#define ICM56686_SREG_CTRL_IREG_ADDR            (ICM56686_IPREG_TOP1_BASE + 0x60)
#define ICM56686_SREG_SIFS_20BITS_EN            (1 << 3) // 1 = 20-bit (default), 0 = 16-bit
#define ICM56686_SREG_DATA_ENDIAN_SEL_BIG       (1 << 1) // 1 = big-endian (default), 0 = little
// Desired: 16-bit, little-endian -> 0x00
#define ICM56686_SREG_CTRL_16BIT_LE             0x00

// Gyro SRC control : IPREG_SYS1 offset 0x9A, GYRO_SRC_CTRL[3:2]
#define ICM56686_GYRO_SRC_CTRL_IREG_ADDR        (ICM56686_IPREG_SYS1_BASE + 0x9A)
#define ICM56686_GYRO_SRC_CTRL_MASK             (0x03 << 2)
#define ICM56686_GYRO_SRC_CTRL_SRC_PREFILT_ON   (0x02 << 2) // SRC on + pre-filter on

// Gyro UI LPF : IPREG_SYS1 offset 0x9E, GYRO_UI_LPFBW_SEL[6:4], 3rd-order = bit7
#define ICM56686_GYRO_UI_LPF_CFG_IREG_ADDR      (ICM56686_IPREG_SYS1_BASE + 0x9E)
#define ICM56686_GYRO_UI_LPFBW_MASK             (0x07 << 4)
#define ICM56686_GYRO_UI_3RD_ORD_SEL            (1 << 7)

// Accel SRC control : IPREG_SYS2 offset 0x6D, ACCEL_SRC_CTRL[1:0]
#define ICM56686_ACCEL_SRC_CTRL_IREG_ADDR       (ICM56686_IPREG_SYS2_BASE + 0x6D)
#define ICM56686_ACCEL_SRC_CTRL_MASK            (0x03 << 0)
#define ICM56686_ACCEL_SRC_CTRL_SRC_PREFILT_ON  (0x02 << 0)

// Accel UI LPF : IPREG_SYS2 offset 0x70, ACCEL_UI_LPFBW_SEL[2:0]
#define ICM56686_ACCEL_UI_LPF_CFG_IREG_ADDR     (ICM56686_IPREG_SYS2_BASE + 0x70)
#define ICM56686_ACCEL_UI_LPFBW_MASK            (0x07 << 0)

// UI LPF bandwidth selections (ODR-relative). Encoding is identical for the
// gyro [6:4] and accel [2:0] fields; the macros above place them in the field.
#define ICM56686_UI_LPFBW_ODR_DIV_2_BYPASS      0x00 // LP bypass (== Nyquist)
#define ICM56686_UI_LPFBW_ODR_DIV_4             0x01
#define ICM56686_UI_LPFBW_ODR_DIV_8             0x02
#define ICM56686_UI_LPFBW_ODR_DIV_16            0x03
#define ICM56686_UI_LPFBW_ODR_DIV_32            0x04
#define ICM56686_UI_LPFBW_ODR_DIV_64            0x05
#define ICM56686_UI_LPFBW_ODR_DIV_128           0x06

// ---------------------------------------------------------------------------
// Timing / misc
// ---------------------------------------------------------------------------
#ifndef ICM56686_CLOCK
#define ICM56686_MAX_SPI_CLK_HZ                 24000000
#else
#define ICM56686_MAX_SPI_CLK_HZ                 ICM56686_CLOCK
#endif

#define HZ_TO_US(hz)                            ((int32_t)((1000 * 1000) / (hz)))

#define ICM56686_RESET_TIMEOUT_US               20000 // power-on reset is 5 ms typ
#define ICM56686_IREG_TIMEOUT_US                5000
#define ICM56686_SENSOR_ENABLE_DELAY_MS         1
#define ICM56686_ACCEL_STARTUP_TIME_MS          10
#define ICM56686_GYRO_STARTUP_TIME_MS           35  // gyro startup 35 ms typ

#define ICM56686_DATA_LENGTH                    6   // 3 axes * 2 bytes
#define ICM56686_SPI_BUFFER_SIZE                (1 + ICM56686_DATA_LENGTH)

// ---------------------------------------------------------------------------
// IREG access helpers
// ---------------------------------------------------------------------------

// Wait for the IREG_DONE bit (REG_MISC2 bit0) to indicate the internal IREG
// transfer has completed. Returns true on success, false on timeout.
static bool icm56686_waitIregDone(const extDevice_t *dev)
{
    // IREG_DONE (REG_MISC2 bit0) reads 1 at idle (reset 0x01), so polling
    // immediately after writing IREG_DATA can observe a stale "done" before the
    // internal transfer has even started. Honour the datasheet minimum 4us gap
    // first so the operation is genuinely underway before we poll.
    delayMicroseconds(4);

    for (uint32_t waited_us = 0; waited_us < ICM56686_IREG_TIMEOUT_US; waited_us += 10) {
        const uint8_t misc2 = spiReadRegMsk(dev, ICM56686_REG_MISC2);
        if (misc2 & ICM56686_BIT_IREG_DONE) {
            return true;
        }
        delayMicroseconds(10);
    }
    bprintf("[icm56686] IREG_DONE timeout");
    return false;
}

// Write one byte to an indirect (IREG) register. See datasheet 13.x.
// Refuses to run while armed: IREG writes reconfigure the signal path and must
// not happen mid-flight
// (seems a bit spurious but mirrors the ICM-456xx driver).
static bool icm56686_write_ireg(const extDevice_t *dev, uint16_t reg, uint8_t value)
{
    if (ARMING_FLAG(ARMED)) {
        bprintf("[icm56686] refusing IREG write 0x%04X while ARMED", reg);
        return false;
    }

    // An IREG write must be a single auto-incrementing SPI burst: the register
    // byte (IREG_ADDR_15_8 = 0x7C) followed by addr_msb (-> 0x7C), addr_lsb
    // (-> 0x7D) and the data (-> 0x7E), all with CS held low. Writing the three
    // registers as separate transactions does NOT trigger the internal transfer
    // (confirmed on hardware: writes reported done but never changed the target).
    uint8_t buf[3] = { (uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), value };
    spiWriteRegBuf(dev, ICM56686_REG_IREG_ADDR_15_8, buf, sizeof(buf));

    const bool ok = icm56686_waitIregDone(dev);
    bprintf("[icm56686] IREG wr 0x%04X = 0x%02X -> %s", reg, value, ok ? "ok" : "FAIL");
    return ok;
}

// Read one byte from an indirect (IREG) register. The host programs the target
// address, the device pre-fetches into IREG_DATA, and after IREG_DONE the host
// reads IREG_DATA. Returns true on success.
static bool icm56686_read_ireg(const extDevice_t *dev, uint16_t reg, uint8_t *value)
{
    // Programming the registers ICM56686_REG_IREG_ADDR_15_8 and ICM56686_REG_IREG_ADDR_7_0 sequentially also seems to work.
    // This might prevent a spurious read request (in the case of writing separately) after writing the MSB (15_8).
    // Note also, icm56686_write_reg requires programming the reqisters and data all at once, without releasing CS until the end.
    uint8_t buf[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
    spiWriteRegBuf(dev, ICM56686_REG_IREG_ADDR_15_8, buf, sizeof(buf));

    if (!icm56686_waitIregDone(dev)) {
        return false;
    }

    *value = spiReadRegMsk(dev, ICM56686_REG_IREG_DATA);
    // The address auto-increments and a new pre-fetch is triggered; let it settle.
    icm56686_waitIregDone(dev);

    bprintf("[icm56686] IREG rd 0x%04X = 0x%02X", reg, *value);
    return true;
}

// Read-modify-write a field of an IREG register, preserving the reserved/OIS bits
// at their reset values.
static bool icm56686_modify_ireg(const extDevice_t *dev, uint16_t reg, uint8_t mask, uint8_t value)
{
    uint8_t cur = 0;
    if (!icm56686_read_ireg(dev, reg, &cur)) {
        bprintf("[icm56686] modify 0x%04X: read failed, writing field directly", reg);
        return icm56686_write_ireg(dev, reg, value & mask);
    }
    const uint8_t next = (cur & ~mask) | (value & mask);
    return icm56686_write_ireg(dev, reg, next);
}

// Map the configurator gyro_hardware_lpf setting to a UI LPF bandwidth code.
// At 6.4 kHz ODR: ODR/16 ~= 400 Hz, ODR/32 ~= 200 Hz, ODR/8 ~= 800 Hz, ODR/4 ~= 1600 Hz.
static uint8_t getGyroLpfConfig(const gyroHardwareLpf_e hardwareLpf)
{
    switch (hardwareLpf) {
    case GYRO_HARDWARE_LPF_NORMAL:
        return ICM56686_UI_LPFBW_ODR_DIV_16; // ~400 Hz
    case GYRO_HARDWARE_LPF_OPTION_1:
        return ICM56686_UI_LPFBW_ODR_DIV_32; // ~200 Hz
    case GYRO_HARDWARE_LPF_OPTION_2:
        return ICM56686_UI_LPFBW_ODR_DIV_8;  // ~800 Hz
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    case GYRO_HARDWARE_LPF_EXPERIMENTAL:
        return ICM56686_UI_LPFBW_ODR_DIV_4;  // ~1600 Hz
#endif
    default:
        return ICM56686_UI_LPFBW_ODR_DIV_16;
    }
}

static void icm56686_enableSensors(const extDevice_t *dev, bool enable)
{
    const uint8_t value = enable
        ? (ICM56686_GYRO_MODE_LN | ICM56686_ACCEL_MODE_LN)
        : (ICM56686_GYRO_MODE_OFF | ICM56686_ACCEL_MODE_OFF);

    bprintf("[icm56686] PWR_MGMT0 <= 0x%02X (%s)", value, enable ? "LN/LN" : "OFF");
    spiWriteReg(dev, ICM56686_PWR_MGMT0, value);
}

// ---------------------------------------------------------------------------
// Detection
// ---------------------------------------------------------------------------
uint8_t icm56686SpiDetect(const extDevice_t *dev)
{
    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
    uint32_t waited_us = 0;

    bprintf("[icm56686] detect: soft reset via REG_MISC2");

    // The ICM-56686 has no bank-select register; soft reset is REG_MISC2 bit 1.
    spiWriteReg(dev, ICM56686_REG_MISC2, ICM56686_SOFT_RESET);

    // Wait for the soft-reset bit to self-clear (power-on reset is ~5 ms).
    while ((spiReadRegMsk(dev, ICM56686_REG_MISC2) & ICM56686_SOFT_RESET) != 0) {
        delayMicroseconds(10);
        waited_us += 10;
        if (waited_us >= ICM56686_RESET_TIMEOUT_US) {
            bprintf("[icm56686] detect: soft reset TIMEOUT");
            return MPU_NONE;
        }
    }
    bprintf("[icm56686] detect: reset done after %u us", waited_us);

    // Put power management into a known (off) state after reset.
    spiWriteReg(dev, ICM56686_PWR_MGMT0, 0x00);

    do {
        delay(1);
        const uint8_t whoAmI = spiReadRegMsk(dev, ICM56686_WHO_AM_I);
        bprintf("[icm56686] WHO_AM_I = 0x%02X (expecting 0x%02X), attempts left %u",
                whoAmI, ICM56686_WHO_AM_I_CONST, attemptsRemaining);
        if (whoAmI == ICM56686_WHO_AM_I_CONST) {
            icmDetected = ICM_56686_SPI;
            break;
        }
    } while (attemptsRemaining--);

    bprintf("[icm56686] detect result: %s", icmDetected == ICM_56686_SPI ? "FOUND" : "not found");
    return icmDetected;
}

// ---------------------------------------------------------------------------
// Accel
// ---------------------------------------------------------------------------
void icm56686AccInit(accDev_t *acc)
{
    const extDevice_t *dev = &acc->gyro->dev;

    bprintf("[icm56686] accInit");

    // Accel sensitivity 16-bit 16G in icm56686GyroInit (which runs first)
    // 16-bit mode uses +/-16g -> 2048 LSB/g.
    acc->acc_1G = 2048;
    acc->gyro->accSampleRateHz = 1600;    // accel ODR set to 1.6 kHz below

    // Enable accel SRC + pre-filter.
    icm56686_modify_ireg(dev, ICM56686_ACCEL_SRC_CTRL_IREG_ADDR,
                         ICM56686_ACCEL_SRC_CTRL_MASK, ICM56686_ACCEL_SRC_CTRL_SRC_PREFILT_ON);

    // Set the Accel UI LPF bandwidth cut-off to ODR/8 (~ 1600/8 = 200Hz)
    icm56686_modify_ireg(dev, ICM56686_ACCEL_UI_LPF_CFG_IREG_ADDR,
                         ICM56686_ACCEL_UI_LPFBW_MASK, ICM56686_UI_LPFBW_ODR_DIV_8);

    // Set up register address (might want for DMA reads later).
    acc->gyro->accDataReg = ICM56686_ACCEL_DATA_X1;
}

bool icm56686SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ICM_56686_SPI) {
        return false;
    }

    acc->initFn = icm56686AccInit;
    acc->readFn = icm56686AccReadSPI;
    return true;
}

bool icm56686AccReadSPI(accDev_t *acc)
{
    uint8_t raw[ICM56686_DATA_LENGTH];
    const bool ack = spiReadRegMskBufRB(&acc->gyro->dev, ICM56686_ACCEL_DATA_X1, raw, ICM56686_DATA_LENGTH);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((raw[1] << 8) | raw[0]);
    acc->ADCRaw[Y] = (int16_t)((raw[3] << 8) | raw[2]);
    acc->ADCRaw[Z] = (int16_t)((raw[5] << 8) | raw[4]);

    return true;
}

// ---------------------------------------------------------------------------
// Gyro
// ---------------------------------------------------------------------------
void icm56686GyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    bprintf("[icm56686] gyroInit: SPI clk <= %d Hz", ICM56686_MAX_SPI_CLK_HZ);
    spiSetClkDivisor(dev, spiCalculateDivider(ICM56686_MAX_SPI_CLK_HZ));

    mpuGyroInit(gyro);

    // Configure 16-bit, little-endian sensor output (chip default is 20-bit big-endian).
    bprintf("[icm56686] setting SREG_CTRL to 16-bit little-endian (was 0x0A at reset)");
    icm56686_write_ireg(dev, ICM56686_SREG_CTRL_IREG_ADDR, ICM56686_SREG_CTRL_16BIT_LE);

    // Power up both sensors in Low-Noise mode.
    icm56686_enableSensors(dev, true);
    delay(ICM56686_SENSOR_ENABLE_DELAY_MS);

    // Accel ODR 1.6 kHz with 16G range.
    bprintf("[icm56686] ACCEL_CONFIG0 <= 16G / 1.6kHz");
    spiWriteReg(dev, ICM56686_ACCEL_CONFIG0, ICM56686_ACCEL_FS_SEL_16G | ICM56686_ACCEL_ODR_1K6_LN);
    delay(ICM56686_ACCEL_STARTUP_TIME_MS);

    // Gyro filters: enable SRC + pre-filter, then UI LPF per gyro_hardware_lpf.
    icm56686_modify_ireg(dev, ICM56686_GYRO_SRC_CTRL_IREG_ADDR,
                         ICM56686_GYRO_SRC_CTRL_MASK, ICM56686_GYRO_SRC_CTRL_SRC_PREFILT_ON);

    // Diagnostic: does this IREG write stick? (SREG_CTRL above did not.) If this
    // reads back 0x28 the filter writes are applying and the SREG_CTRL failure is
    // register-specific; if it reads 0x20 (reset) then IREG writes fail in general.
    uint8_t srcChk = 0xFF;
    if (icm56686_read_ireg(dev, ICM56686_GYRO_SRC_CTRL_IREG_ADDR, &srcChk)) {
        bprintf("[icm56686] GYRO_SRC_CTRL readback = 0x%02X (expect 0x28)", srcChk);
    }

    const uint8_t lpfSel = getGyroLpfConfig(gyroConfig()->gyro_hardware_lpf);
    bprintf("[icm56686] gyro UI LPF select = %u (field<<4)", lpfSel);
    icm56686_modify_ireg(dev, ICM56686_GYRO_UI_LPF_CFG_IREG_ADDR,
                         ICM56686_GYRO_UI_LPFBW_MASK, (uint8_t)(lpfSel << 4));

    // Gyro full-scale range + max 6.4 kHz ODR, Low-Noise.
    // In 16-bit mode FS_SEL is honoured, so use +/-2000 dps.
    gyro->scale = GYRO_SCALE_2000DPS;
    gyro->gyroRateKHz = GYRO_RATE_6400_Hz;
    gyro->gyroSampleRateHz = 6400;

    // Temperature: 128 LSB/°C, 25 °C zero (datasheet).
    // (Note, not currently reading temperature.)
    gyro->tempScale = 1.0f / 128.0f;
    gyro->tempZero = 25.0f;

    bprintf("[icm56686] GYRO_CONFIG0 <= 2000dps / 6.4kHz");
    spiWriteReg(dev, ICM56686_GYRO_CONFIG0, ICM56686_GYRO_FS_SEL_2000DPS | ICM56686_GYRO_ODR_6K4_LN);
    delay(ICM56686_GYRO_STARTUP_TIME_MS);

    gyro->gyroShortPeriod = clockMicrosToCycles(HZ_TO_US(gyro->gyroSampleRateHz));

    // Data-ready interrupt on INT1: push-pull, active-high, pulsed.
    bprintf("[icm56686] configuring INT1 (PP, active-high, pulsed, DRDY)");
    spiWriteReg(dev, ICM56686_INT1_CONFIG2,
                ICM56686_INT1_MODE_PULSED | ICM56686_INT1_DRIVE_CIRCUIT_PP | ICM56686_INT1_POLARITY_ACTIVE_HIGH);
    spiWriteReg(dev, ICM56686_INT1_CONFIG0, ICM56686_INT1_STATUS_EN_DRDY);

    // Contiguous data: accel at 0x00, gyro at 0x06.
    gyro->accDataReg = ICM56686_ACCEL_DATA_X1;
    gyro->gyroDataReg = ICM56686_GYRO_DATA_X1;
    gyro->gyroDmaMaxDuration = 0; // DRDY interrupt paces reads

    bprintf("[icm56686] gyroInit complete");
}

bool icm56686GyroReadSPI(gyroDev_t *gyro)
{
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
        bprintf("[icm56686] icm56686GyroReadSPI GYRO_EXTI_INIT");
        memset(gyro->dev.txBuf, 0xff, ICM56686_SPI_BUFFER_SIZE);
        gyro->gyroDmaMaxDuration = 0;
        // DMA path not yet enabled for this device; use direct reads.
        // TODO
        gyro->gyroModeSPI = GYRO_EXTI_INT;
        break;

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT: {
        uint8_t raw[ICM56686_DATA_LENGTH];
        const bool ack = spiReadRegMskBufRB(&gyro->dev, ICM56686_GYRO_DATA_X1, raw, ICM56686_DATA_LENGTH);
        if (!ack) {
            bprintf("[icm56686] read reg no ack");
            return false;
        }

        gyro->gyroADCRaw[X] = (int16_t)((raw[1] << 8) | raw[0]);
        gyro->gyroADCRaw[Y] = (int16_t)((raw[3] << 8) | raw[2]);
        gyro->gyroADCRaw[Z] = (int16_t)((raw[5] << 8) | raw[4]);

        break;
    }

    case GYRO_EXTI_INT_DMA:
        // TODO (future)
        bprintf("*** unexpected DMA ***");
        // rxBuf[0] is the dummy byte from the register address phase
        gyro->gyroADCRaw[X] = (int16_t)((gyro->dev.rxBuf[2] << 8) | gyro->dev.rxBuf[1]);
        gyro->gyroADCRaw[Y] = (int16_t)((gyro->dev.rxBuf[4] << 8) | gyro->dev.rxBuf[3]);
        gyro->gyroADCRaw[Z] = (int16_t)((gyro->dev.rxBuf[6] << 8) | gyro->dev.rxBuf[5]);
        break;

    default:
        break;
    }

    return true;
}

bool icm56686SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ICM_56686_SPI) {
        return false;
    }

    gyro->initFn = icm56686GyroInit;
    gyro->readFn = icm56686GyroReadSPI;
    return true;
}

#endif // USE_ACCGYRO_ICM56686
