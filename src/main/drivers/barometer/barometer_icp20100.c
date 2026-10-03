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

#include "platform.h"

#include "build/build_config.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_icp20100.h"
#include "drivers/bus.h"
#include "drivers/time.h"

#if defined(USE_BARO) && defined(USE_BARO_ICP20100)

/*
 * TDK InvenSense ICP-20100 datasheet, DS-000416 revision 1.5:
 * https://invensense.tdk.com/wp-content/uploads/2022/08/DS-000416-ICP-20100-v1.4.pdf
 *
 * Revision 1.5 is also distributed with TDK's official ICP201xx driver:
 * https://github.com/tdk-invn-oss/pressure.arduino.ICP201xx
 */

#define ICP20100_I2C_ADDRESS_AD0_LOW          0x63

#define ICP20100_REG_TRIM1_MSB                0x05
#define ICP20100_REG_TRIM2_LSB                0x06
#define ICP20100_REG_TRIM2_MSB                0x07
#define ICP20100_REG_DEVICE_ID                0x0C
#define ICP20100_REG_OTP_CONFIG1              0xAC
#define ICP20100_REG_OTP_MR_LSB               0xAD
#define ICP20100_REG_OTP_MR_MSB               0xAE
#define ICP20100_REG_OTP_MRA_LSB              0xAF
#define ICP20100_REG_OTP_MRA_MSB              0xB0
#define ICP20100_REG_OTP_MRB_LSB              0xB1
#define ICP20100_REG_OTP_MRB_MSB              0xB2
#define ICP20100_REG_OTP_ADDRESS              0xB5
#define ICP20100_REG_OTP_COMMAND              0xB6
#define ICP20100_REG_OTP_RDATA                0xB8
#define ICP20100_REG_OTP_STATUS               0xB9
#define ICP20100_REG_OTP_DBG2                 0xBC
#define ICP20100_REG_MASTER_LOCK              0xBE
#define ICP20100_REG_OTP_STATUS2              0xBF
#define ICP20100_REG_MODE_SELECT              0xC0
#define ICP20100_REG_INTERRUPT_STATUS         0xC1
#define ICP20100_REG_INTERRUPT_MASK           0xC2
#define ICP20100_REG_FIFO_FILL                0xC4
#define ICP20100_REG_DEVICE_STATUS            0xCD
#define ICP20100_REG_VERSION                  0xD3
#define ICP20100_REG_PRESS_DATA_0             0xFA

#define ICP20100_DEVICE_ID                    0x63
#define ICP20100_VERSION_A                    0x00
#define ICP20100_VERSION_B2                   0xB2

#define ICP20100_TRIM1_OFFSET_MASK            0x3F
#define ICP20100_TRIM2_HFOSC_MASK             0x7F
#define ICP20100_TRIM2_GAIN_MASK              0x70
#define ICP20100_TRIM2_GAIN_SHIFT             4

#define ICP20100_OTP_CONFIG_ENABLE            0x01
#define ICP20100_OTP_CONFIG_WRITE_SWITCH      0x02
#define ICP20100_OTP_DBG2_RESET               0x80
#define ICP20100_OTP_STATUS_BUSY              0x01
#define ICP20100_OTP_STATUS2_BOOTED           0x01
#define ICP20100_OTP_READ_COMMAND             0x10

#define ICP20100_MODE_MEAS_CONFIG_MODE2       0x40
#define ICP20100_MODE_CONTINUOUS              0x08
#define ICP20100_MODE_POWER_ACTIVE            0x04
#define ICP20100_MODE2_CONTINUOUS             (ICP20100_MODE_MEAS_CONFIG_MODE2 | ICP20100_MODE_CONTINUOUS)

#define ICP20100_FIFO_FLUSH                   0x80
#define ICP20100_FIFO_EMPTY                   0x40
#define ICP20100_FIFO_FULL                    0x20
#define ICP20100_FIFO_LEVEL_MASK              0x1F
#define ICP20100_INTERRUPT_FIFO_OVERFLOW      0x01
#define ICP20100_MODE_SYNC_READY              0x01

#define ICP20100_DATA_FRAME_SIZE              6
#define ICP20100_FIFO_MAX_FRAMES              16
#define ICP20100_DATA_BUFFER_SIZE             (ICP20100_FIFO_MAX_FRAMES * ICP20100_DATA_FRAME_SIZE)
#define ICP20100_FIR_SETTLING_SAMPLES         14
#define ICP20100_POLL_DELAY_US                22000
#define ICP20100_MODE_SYNC_TIMEOUT_US         10000
#define ICP20100_OTP_TIMEOUT_US               10000
#define ICP20100_DETECTION_RETRY_COUNT        5

STATIC_ASSERT(ICP20100_DATA_BUFFER_SIZE <= UINT8_MAX, ICP20100_fifo_burst_must_fit_bus_length);

STATIC_UNIT_TESTED int32_t icp20100RawPressure;
STATIC_UNIT_TESTED int32_t icp20100RawTemperature;
STATIC_UNIT_TESTED bool icp20100SampleValid;

STATIC_UNIT_TESTED uint8_t icp20100SamplesToDiscard;
static uint8_t icp20100FramesPending;
static DMA_DATA_ZERO_INIT uint8_t icp20100Data[ICP20100_DATA_BUFFER_SIZE];

static bool icp20100StartUT(baroDev_t *baro);
static bool icp20100ReadUT(baroDev_t *baro);
static bool icp20100GetUT(baroDev_t *baro);
static bool icp20100StartUP(baroDev_t *baro);
STATIC_UNIT_TESTED bool icp20100ReadUP(baroDev_t *baro);
STATIC_UNIT_TESTED bool icp20100GetUP(baroDev_t *baro);

STATIC_UNIT_TESTED bool icp20100VersionSupported(uint8_t version)
{
    return version == ICP20100_VERSION_A || version == ICP20100_VERSION_B2;
}

STATIC_UNIT_TESTED bool icp20100VersionRequiresOtp(uint8_t version, uint8_t bootStatus)
{
    return version == ICP20100_VERSION_A && !(bootStatus & ICP20100_OTP_STATUS2_BOOTED);
}

STATIC_UNIT_TESTED int32_t icp20100SignExtend20(uint32_t value)
{
    value &= 0xFFFFF;
    return (value & 0x80000) ? (int32_t)(value - 0x100000) : (int32_t)value;
}

STATIC_UNIT_TESTED void icp20100DecodeFrame(const uint8_t data[ICP20100_DATA_FRAME_SIZE], int32_t *rawPressure, int32_t *rawTemperature)
{
    const uint32_t pressure = data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)(data[2] & 0x0F) << 16);
    const uint32_t temperature = data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)(data[5] & 0x0F) << 16);

    *rawPressure = icp20100SignExtend20(pressure);
    *rawTemperature = icp20100SignExtend20(temperature);
}

static int32_t icp20100DivideRounded(int64_t numerator, int32_t denominator)
{
    const int64_t halfDenominator = denominator / 2;
    numerator += numerator >= 0 ? halfDenominator : -halfDenominator;
    return (int32_t)(numerator / denominator);
}

STATIC_UNIT_TESTED void icp20100ConvertRaw(int32_t rawPressure, int32_t rawTemperature, int32_t *pressure, int32_t *temperature)
{
    // P = (POUT / 2^17) * 40 kPa + 70 kPa.
    if (pressure) {
        *pressure = 70000 + icp20100DivideRounded((int64_t)rawPressure * 40000, 131072);
    }

    // T = (TOUT / 2^18) * 65 degrees Celsius + 25 degrees Celsius.
    if (temperature) {
        *temperature = 2500 + icp20100DivideRounded((int64_t)rawTemperature * 6500, 262144);
    }
}

STATIC_UNIT_TESTED bool icp20100AcceptSample(uint8_t *samplesToDiscard)
{
    if (*samplesToDiscard) {
        (*samplesToDiscard)--;
        return false;
    }

    return true;
}

STATIC_UNIT_TESTED void icp20100Calculate(int32_t *pressure, int32_t *temperature)
{
    if (!icp20100SampleValid) {
        if (pressure) {
            *pressure = 0;
        }
        if (temperature) {
            *temperature = 0;
        }
        return;
    }

    icp20100ConvertRaw(icp20100RawPressure, icp20100RawTemperature, pressure, temperature);
}

static bool icp20100ReadRegister(const extDevice_t *dev, uint8_t reg, uint8_t *value)
{
    return busReadRegisterBuffer(dev, reg, value, 1);
}

static bool icp20100UpdateRegister(const extDevice_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current;

    if (!icp20100ReadRegister(dev, reg, &current)) {
        return false;
    }

    current = (current & ~mask) | (value & mask);
    return busWriteRegister(dev, reg, current);
}

static bool icp20100DummyRead(const extDevice_t *dev)
{
    uint8_t unused;
    return icp20100ReadRegister(dev, 0x00, &unused);
}

static bool icp20100WaitForModeSync(const extDevice_t *dev)
{
    for (unsigned elapsed = 0; elapsed < ICP20100_MODE_SYNC_TIMEOUT_US; elapsed += 10) {
        uint8_t status;
        if (!icp20100ReadRegister(dev, ICP20100_REG_DEVICE_STATUS, &status)) {
            return false;
        }
        if (status & ICP20100_MODE_SYNC_READY) {
            return true;
        }
        delayMicroseconds(10);
    }

    return false;
}

static bool icp20100WaitForOtp(const extDevice_t *dev)
{
    for (unsigned elapsed = 0; elapsed < ICP20100_OTP_TIMEOUT_US; elapsed++) {
        uint8_t status;
        if (!icp20100ReadRegister(dev, ICP20100_REG_OTP_STATUS, &status)) {
            return false;
        }
        if (!(status & ICP20100_OTP_STATUS_BUSY)) {
            return true;
        }
        delayMicroseconds(1);
    }

    return false;
}

static bool icp20100ReadOtp(const extDevice_t *dev, uint8_t address, uint8_t *value)
{
    return busWriteRegister(dev, ICP20100_REG_OTP_ADDRESS, address)
        && busWriteRegister(dev, ICP20100_REG_OTP_COMMAND, ICP20100_OTP_READ_COMMAND)
        && icp20100WaitForOtp(dev)
        && icp20100ReadRegister(dev, ICP20100_REG_OTP_RDATA, value);
}

STATIC_UNIT_TESTED bool icp20100LoadOtpTrim(const extDevice_t *dev, uint8_t version)
{
    if (!icp20100VersionSupported(version)) {
        return false;
    }

    if (version == ICP20100_VERSION_B2) {
        return icp20100DummyRead(dev);
    }

    uint8_t bootStatus;
    if (!icp20100ReadRegister(dev, ICP20100_REG_OTP_STATUS2, &bootStatus)) {
        return false;
    }

    if (!icp20100VersionRequiresOtp(version, bootStatus)) {
        return icp20100DummyRead(dev);
    }

    bool success = false;
    bool registersUnlocked = false;
    bool otpEnabled = false;
    uint8_t offset = 0;
    uint8_t gain = 0;
    uint8_t hfosc = 0;

    if (!icp20100WaitForModeSync(dev)
        || !busWriteRegister(dev, ICP20100_REG_MODE_SELECT, ICP20100_MODE_POWER_ACTIVE)) {
        goto cleanup;
    }
    delay(4);

    if (!busWriteRegister(dev, ICP20100_REG_MASTER_LOCK, 0x1F)) {
        goto cleanup;
    }
    registersUnlocked = true;

    if (!icp20100UpdateRegister(dev, ICP20100_REG_OTP_CONFIG1,
            ICP20100_OTP_CONFIG_ENABLE | ICP20100_OTP_CONFIG_WRITE_SWITCH,
            ICP20100_OTP_CONFIG_ENABLE | ICP20100_OTP_CONFIG_WRITE_SWITCH)) {
        goto cleanup;
    }
    otpEnabled = true;
    delayMicroseconds(10);

    if (!icp20100UpdateRegister(dev, ICP20100_REG_OTP_DBG2, ICP20100_OTP_DBG2_RESET, ICP20100_OTP_DBG2_RESET)) {
        goto cleanup;
    }
    delayMicroseconds(10);
    if (!icp20100UpdateRegister(dev, ICP20100_REG_OTP_DBG2, ICP20100_OTP_DBG2_RESET, 0)) {
        goto cleanup;
    }
    delayMicroseconds(10);

    if (!busWriteRegister(dev, ICP20100_REG_OTP_MRA_LSB, 0x04)
        || !busWriteRegister(dev, ICP20100_REG_OTP_MRA_MSB, 0x04)
        || !busWriteRegister(dev, ICP20100_REG_OTP_MRB_LSB, 0x21)
        || !busWriteRegister(dev, ICP20100_REG_OTP_MRB_MSB, 0x20)
        || !busWriteRegister(dev, ICP20100_REG_OTP_MR_LSB, 0x10)
        || !busWriteRegister(dev, ICP20100_REG_OTP_MR_MSB, 0x80)
        || !icp20100ReadOtp(dev, 0xF8, &offset)
        || !icp20100ReadOtp(dev, 0xF9, &gain)
        || !icp20100ReadOtp(dev, 0xFA, &hfosc)) {
        goto cleanup;
    }

    if (!icp20100UpdateRegister(dev, ICP20100_REG_OTP_CONFIG1,
            ICP20100_OTP_CONFIG_ENABLE | ICP20100_OTP_CONFIG_WRITE_SWITCH, 0)) {
        goto cleanup;
    }
    otpEnabled = false;
    delayMicroseconds(10);

    if (!icp20100UpdateRegister(dev, ICP20100_REG_TRIM1_MSB,
            ICP20100_TRIM1_OFFSET_MASK, offset)
        || !icp20100UpdateRegister(dev, ICP20100_REG_TRIM2_MSB,
            ICP20100_TRIM2_GAIN_MASK, (gain & 0x07) << ICP20100_TRIM2_GAIN_SHIFT)
        || !icp20100UpdateRegister(dev, ICP20100_REG_TRIM2_LSB,
            ICP20100_TRIM2_HFOSC_MASK, hfosc)) {
        goto cleanup;
    }

    success = true;

cleanup:
    if (otpEnabled) {
        if (!icp20100UpdateRegister(dev, ICP20100_REG_OTP_CONFIG1,
                ICP20100_OTP_CONFIG_ENABLE | ICP20100_OTP_CONFIG_WRITE_SWITCH, 0)) {
            success = false;
        }
        delayMicroseconds(10);
    }

    if (registersUnlocked && !busWriteRegister(dev, ICP20100_REG_MASTER_LOCK, 0x00)) {
        success = false;
    }

    if (!icp20100WaitForModeSync(dev)
        || !busWriteRegister(dev, ICP20100_REG_MODE_SELECT, 0x00)) {
        success = false;
    }

    if (success && !icp20100UpdateRegister(dev, ICP20100_REG_OTP_STATUS2,
            ICP20100_OTP_STATUS2_BOOTED, ICP20100_OTP_STATUS2_BOOTED)) {
        success = false;
    }

    if (!icp20100DummyRead(dev)) {
        success = false;
    }

    return success;
}

static bool icp20100Configure(const extDevice_t *dev)
{
    bool success = icp20100WaitForModeSync(dev)
        && busWriteRegister(dev, ICP20100_REG_MODE_SELECT, 0x00);

    if (success) {
        delayMicroseconds(10);
        success = icp20100WaitForModeSync(dev)
            && busWriteRegister(dev, ICP20100_REG_FIFO_FILL, ICP20100_FIFO_FLUSH)
            && busWriteRegister(dev, ICP20100_REG_INTERRUPT_MASK, 0xFF);
    }

    uint8_t interruptStatus = 0;
    if (success) {
        success = icp20100ReadRegister(dev, ICP20100_REG_INTERRUPT_STATUS, &interruptStatus);
    }
    if (success && interruptStatus) {
        success = busWriteRegister(dev, ICP20100_REG_INTERRUPT_STATUS, interruptStatus);
    }

    if (success) {
        success = icp20100WaitForModeSync(dev)
            && busWriteRegister(dev, ICP20100_REG_MODE_SELECT, ICP20100_MODE2_CONTINUOUS)
            && icp20100WaitForModeSync(dev);
    }

    if (!icp20100DummyRead(dev)) {
        success = false;
    }

    if (success) {
        icp20100SamplesToDiscard = ICP20100_FIR_SETTLING_SAMPLES;
        icp20100FramesPending = 0;
        icp20100SampleValid = false;
        icp20100RawPressure = 0;
        icp20100RawTemperature = 0;
    }

    return success;
}

static void icp20100InvalidatePipeline(void)
{
    /*
     * A failed asynchronous transfer may have consumed only part of a frame.
     * Do not publish data from that transfer, and allow the hardware FIR to
     * settle again before publishing a later sample.
     */
    icp20100SamplesToDiscard = ICP20100_FIR_SETTLING_SAMPLES;
    icp20100FramesPending = 0;
    icp20100SampleValid = false;
    icp20100RawPressure = 0;
    icp20100RawTemperature = 0;
}

static bool icp20100StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool icp20100ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool icp20100GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool icp20100StartUP(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

STATIC_UNIT_TESTED bool icp20100ReadUP(baroDev_t *baro)
{
    bool error = false;
    if (busBusy(&baro->dev, &error)) {
        return false;
    }
    if (error) {
        icp20100InvalidatePipeline();
        (void)icp20100DummyRead(&baro->dev);
        return false;
    }

    uint8_t fifoFill;
    if (!icp20100ReadRegister(&baro->dev, ICP20100_REG_FIFO_FILL, &fifoFill)) {
        icp20100DummyRead(&baro->dev);
        return false;
    }

    uint8_t interruptStatus;
    if (!icp20100ReadRegister(&baro->dev, ICP20100_REG_INTERRUPT_STATUS, &interruptStatus)) {
        icp20100DummyRead(&baro->dev);
        return false;
    }

    const uint8_t fifoLevel = fifoFill & ICP20100_FIFO_LEVEL_MASK;
    const bool fifoEmpty = fifoFill & ICP20100_FIFO_EMPTY;
    const bool fifoFull = fifoFill & ICP20100_FIFO_FULL;
    const bool fifoStateInvalid = fifoLevel > ICP20100_FIFO_MAX_FRAMES
        || (fifoEmpty != (fifoLevel == 0))
        || (fifoFull != (fifoLevel == ICP20100_FIFO_MAX_FRAMES));
    if (fifoStateInvalid) {
        icp20100InvalidatePipeline();
        (void)icp20100DummyRead(&baro->dev);
        return false;
    }

    /*
     * FIFO_FULL with level 16 is a normal readable state.  On overflow the
     * sensor keeps the existing FIFO contents and ignores newer samples, so
     * clear the W1C status and drain those valid frames without changing mode
     * or writing FIFO_FLUSH while a measurement is active.
     */
    if ((interruptStatus & ICP20100_INTERRUPT_FIFO_OVERFLOW)
        && !busWriteRegister(&baro->dev, ICP20100_REG_INTERRUPT_STATUS, ICP20100_INTERRUPT_FIFO_OVERFLOW)) {
        icp20100InvalidatePipeline();
        (void)icp20100DummyRead(&baro->dev);
        return false;
    }

    if (fifoLevel == 0) {
        (void)icp20100DummyRead(&baro->dev);
        return false;
    }

    icp20100FramesPending = 0;
    const bool readStarted = busReadRegisterBufferStart(&baro->dev, ICP20100_REG_PRESS_DATA_0,
        icp20100Data, fifoLevel * ICP20100_DATA_FRAME_SIZE);
    if (!readStarted) {
        icp20100InvalidatePipeline();
        (void)icp20100DummyRead(&baro->dev);
    } else {
        icp20100FramesPending = fifoLevel;
    }
    return readStarted;
}

STATIC_UNIT_TESTED bool icp20100GetUP(baroDev_t *baro)
{
    bool error = false;
    if (busBusy(&baro->dev, &error)) {
        return false;
    }

    if (error) {
        icp20100InvalidatePipeline();
        (void)icp20100DummyRead(&baro->dev);
        return true;
    }

    // End this ICP-20100 transaction sequence with the dummy read required by DS-000416.
    const bool dummyReadOk = icp20100DummyRead(&baro->dev);
    if (!dummyReadOk || icp20100FramesPending == 0
        || icp20100FramesPending > ICP20100_FIFO_MAX_FRAMES) {
        icp20100InvalidatePipeline();
        return true;
    }

    const uint8_t framesRead = icp20100FramesPending;
    const uint8_t *latestFrame = &icp20100Data[(framesRead - 1) * ICP20100_DATA_FRAME_SIZE];
    icp20100DecodeFrame(latestFrame, &icp20100RawPressure, &icp20100RawTemperature);

    bool latestSampleValid = false;
    for (uint8_t frame = 0; frame < framesRead; frame++) {
        latestSampleValid = icp20100AcceptSample(&icp20100SamplesToDiscard);
    }
    icp20100SampleValid = latestSampleValid;
    icp20100FramesPending = 0;
    return true;
}

bool icp20100Detect(baroDev_t *baro)
{
    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    if (dev->bus->busType != BUS_TYPE_I2C) {
        return false;
    }

    if (dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = ICP20100_I2C_ADDRESS_AD0_LOW;
        defaultAddressApplied = true;
    }

    uint8_t deviceId = 0;
    bool detected = false;
    for (unsigned retry = 0; retry < ICP20100_DETECTION_RETRY_COUNT; retry++) {
        // The device needs at least ten SCL cycles before its I2C interface is ready.
        busWriteRegister(dev, 0xEE, 0xF0);
        delayMicroseconds(10);

        if (icp20100ReadRegister(dev, ICP20100_REG_DEVICE_ID, &deviceId)
            && deviceId == ICP20100_DEVICE_ID) {
            detected = true;
            break;
        }
        delay(1);
    }

    uint8_t version = 0;
    if (!detected
        || !icp20100ReadRegister(dev, ICP20100_REG_VERSION, &version)
        || !icp20100VersionSupported(version)
        || !icp20100LoadOtpTrim(dev, version)
        || !icp20100Configure(dev)) {
        icp20100DummyRead(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = icp20100StartUT;
    baro->read_ut = icp20100ReadUT;
    baro->get_ut = icp20100GetUT;

    /*
     * Mode 2 runs at 40 Hz.  READ, SAMPLE, and the next START each add a
     * scheduler interval, so poll early enough that the complete state-machine
     * cycle remains below 25 ms.  read_up drains any occasional backlog.
     */
    baro->up_delay = ICP20100_POLL_DELAY_US;
    baro->start_up = icp20100StartUP;
    baro->read_up = icp20100ReadUP;
    baro->get_up = icp20100GetUP;
    baro->calculate = icp20100Calculate;

    return true;
}

#endif
