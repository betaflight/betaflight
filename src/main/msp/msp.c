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
#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include <ctype.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "cli/cli.h"

#include "common/axis.h"
#include "common/bitarray.h"
#include "common/color.h"
#include "common/huffman.h"
#include "common/maths.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/simplified_tuning.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/camera_control_impl.h"
#include "drivers/compass/compass.h"
#include "drivers/display.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/flash/flash.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/osd.h"
#include "drivers/pwm_output.h"
#include "drivers/sdcard.h"
#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/system.h"
#include "drivers/transponder_ir.h"
#include "drivers/usb_msc.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "fc/board_info.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/dispatch.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_4way.h"
#include "io/transponder_ir.h"
#include "io/usb_msc.h"
#include "io/vtx_control.h"
#include "io/vtx.h"
#include "io/vtx_msp.h"

#include "msp/msp_box.h"
#include "msp/msp_build_info.h"
#include "msp/msp_protocol.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp_protocol_v2_common.h"
#include "msp/msp_serial.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "osd/osd_warnings.h"

#include "pg/beeper.h"
#include "pg/board.h"
#include "pg/dyn_notch.h"
#include "pg/gyrodev.h"
#include "pg/motor.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"
#ifdef USE_RX_EXPRESSLRS
#include "pg/rx_spi_expresslrs.h"
#endif
#include "pg/usb.h"
#include "pg/vcd.h"
#include "pg/vtx_table.h"

#include "rx/rx.h"
#include "rx/rx_bind.h"
#include "rx/msp.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/rangefinder.h"

#include "telemetry/msp_shared.h"
#include "telemetry/telemetry.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "msp.h"


static const char * const flightControllerIdentifier = FC_FIRMWARE_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.

enum {
    MSP_REBOOT_FIRMWARE = 0,
    MSP_REBOOT_BOOTLOADER_ROM,
    MSP_REBOOT_MSC,
    MSP_REBOOT_MSC_UTC,
    MSP_REBOOT_BOOTLOADER_FLASH,
    MSP_REBOOT_COUNT,
};

static uint8_t rebootMode;

typedef enum {
    MSP_SDCARD_STATE_NOT_PRESENT = 0,
    MSP_SDCARD_STATE_FATAL       = 1,
    MSP_SDCARD_STATE_CARD_INIT   = 2,
    MSP_SDCARD_STATE_FS_INIT     = 3,
    MSP_SDCARD_STATE_READY       = 4
} mspSDCardState_e;

typedef enum {
    MSP_SDCARD_FLAG_SUPPORTED   = 1
} mspSDCardFlags_e;

typedef enum {
    MSP_FLASHFS_FLAG_READY       = 1,
    MSP_FLASHFS_FLAG_SUPPORTED  = 2
} mspFlashFsFlags_e;

typedef enum {
    MSP_PASSTHROUGH_ESC_SIMONK = PROTOCOL_SIMONK,
    MSP_PASSTHROUGH_ESC_BLHELI = PROTOCOL_BLHELI,
    MSP_PASSTHROUGH_ESC_KISS = PROTOCOL_KISS,
    MSP_PASSTHROUGH_ESC_KISSALL = PROTOCOL_KISSALL,
    MSP_PASSTHROUGH_ESC_CASTLE = PROTOCOL_CASTLE,

    MSP_PASSTHROUGH_SERIAL_ID = 0xFD,
    MSP_PASSTHROUGH_SERIAL_FUNCTION_ID = 0xFE,

    MSP_PASSTHROUGH_ESC_4WAY = 0xFF,
} mspPassthroughType_e;

#define RATEPROFILE_MASK (1 << 7)

#define RTC_NOT_SUPPORTED 0xff

typedef enum {
    DEFAULTS_TYPE_BASE = 0,
    DEFAULTS_TYPE_CUSTOM,
} defaultsType_e;

#ifdef USE_VTX_TABLE
static bool vtxTableNeedsInit = false;
#endif

static int mspDescriptor = 0;

mspDescriptor_t mspDescriptorAlloc(void)
{
    return (mspDescriptor_t)mspDescriptor++;
}

static uint32_t mspArmingDisableFlags = 0;

#ifndef SIMULATOR_BUILD
static void mspArmingDisableByDescriptor(mspDescriptor_t desc)
{
    mspArmingDisableFlags |= (1 << desc);
}
#endif

static void mspArmingEnableByDescriptor(mspDescriptor_t desc)
{
    mspArmingDisableFlags &= ~(1 << desc);
}

static bool mspIsMspArmingEnabled(void)
{
    return mspArmingDisableFlags == 0;
}

#define MSP_PASSTHROUGH_ESC_4WAY 0xff

static uint8_t mspPassthroughMode;
static uint8_t mspPassthroughArgument;

#if defined(USE_ESCSERIAL) && defined(USE_SERIAL_4WAY_BLHELI_INTERFACE)
static void mspEscPassthroughFn(serialPort_t *serialPort)
{
    escEnablePassthrough(serialPort, &motorConfig()->dev, mspPassthroughArgument, mspPassthroughMode);
}
#endif

static serialPort_t *mspFindPassthroughSerialPort(void)
{
    serialPortUsage_t *portUsage = NULL;

    switch (mspPassthroughMode) {
    case MSP_PASSTHROUGH_SERIAL_ID:
    {
        portUsage = findSerialPortUsageByIdentifier(mspPassthroughArgument);
        break;
    }
    case MSP_PASSTHROUGH_SERIAL_FUNCTION_ID:
    {
        const serialPortConfig_t *portConfig = findSerialPortConfig(1 << mspPassthroughArgument);
        if (portConfig) {
            portUsage = findSerialPortUsageByIdentifier(portConfig->identifier);
        }
        break;
    }
    }
    return portUsage ? portUsage->serialPort : NULL;
}

static void mspSerialPassthroughFn(serialPort_t *serialPort)
{
    serialPort_t *passthroughPort = mspFindPassthroughSerialPort();
    if (passthroughPort && serialPort) {
        serialPassthrough(passthroughPort, serialPort, NULL, NULL);
    }
}

static void mspFcSetPassthroughCommand(sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    if (dataSize == 0) {
        // Legacy format
        mspPassthroughMode = MSP_PASSTHROUGH_ESC_4WAY;
    } else {
        mspPassthroughMode = sbufReadU8(src);
        mspPassthroughArgument = sbufReadU8(src);
    }

    switch (mspPassthroughMode) {
    case MSP_PASSTHROUGH_SERIAL_ID:
    case MSP_PASSTHROUGH_SERIAL_FUNCTION_ID:
        if (mspFindPassthroughSerialPort()) {
            if (mspPostProcessFn) {
                *mspPostProcessFn = mspSerialPassthroughFn;
            }
            sbufWriteU8(dst, 1);
        } else {
            sbufWriteU8(dst, 0);
        }
        break;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    case MSP_PASSTHROUGH_ESC_4WAY:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        sbufWriteU8(dst, esc4wayInit());

        if (mspPostProcessFn) {
            *mspPostProcessFn = esc4wayProcess;
        }
        break;

#ifdef USE_ESCSERIAL
    case MSP_PASSTHROUGH_ESC_SIMONK:
    case MSP_PASSTHROUGH_ESC_BLHELI:
    case MSP_PASSTHROUGH_ESC_KISS:
    case MSP_PASSTHROUGH_ESC_KISSALL:
    case MSP_PASSTHROUGH_ESC_CASTLE:
        if (mspPassthroughArgument < getMotorCount() || (mspPassthroughMode == MSP_PASSTHROUGH_ESC_KISS && mspPassthroughArgument == ALL_MOTORS)) {
            sbufWriteU8(dst, 1);

            if (mspPostProcessFn) {
                *mspPostProcessFn = mspEscPassthroughFn;
            }

            break;
        }
        FALLTHROUGH;
#endif // USE_ESCSERIAL
#endif // USE_SERIAL_4WAY_BLHELI_INTERFACE
    default:
        sbufWriteU8(dst, 0);
    }
}

// TODO: Remove the pragma once this is called from unconditional code
#pragma GCC diagnostic ignored "-Wunused-function"
static void configRebootUpdateCheckU8(uint8_t *parm, uint8_t value)
{
    if (*parm != value) {
        setRebootRequired();
    }
    *parm = value;
}
#pragma GCC diagnostic pop

static void mspRebootFn(serialPort_t *serialPort)
{
    UNUSED(serialPort);

    motorShutdown();

    switch (rebootMode) {
    case MSP_REBOOT_FIRMWARE:
        systemReset();

        break;
    case MSP_REBOOT_BOOTLOADER_ROM:
        systemResetToBootloader(BOOTLOADER_REQUEST_ROM);

        break;
#if defined(USE_USB_MSC)
    case MSP_REBOOT_MSC:
    case MSP_REBOOT_MSC_UTC: {
#ifdef USE_RTC_TIME
        const int16_t timezoneOffsetMinutes = (rebootMode == MSP_REBOOT_MSC) ? timeConfig()->tz_offsetMinutes : 0;
        systemResetToMsc(timezoneOffsetMinutes);
#else
        systemResetToMsc(0);
#endif
        }
        break;
#endif
#if defined(USE_FLASH_BOOT_LOADER)
    case MSP_REBOOT_BOOTLOADER_FLASH:
        systemResetToBootloader(BOOTLOADER_REQUEST_FLASH);

        break;
#endif
    default:

        return;
    }

    // control should never return here.
    while (true) ;
}

#define MSP_DISPATCH_DELAY_US 1000000

void mspReboot(dispatchEntry_t* self)
{
    UNUSED(self);

    if (ARMING_FLAG(ARMED)) {
        return;
    }

    mspRebootFn(NULL);
}

dispatchEntry_t mspRebootEntry =
{
    mspReboot, 0, NULL, false
};

void writeReadEeprom(dispatchEntry_t* self)
{
    UNUSED(self);

    if (ARMING_FLAG(ARMED)) {
        return;
    }

    writeEEPROM();
    readEEPROM();

#ifdef USE_VTX_TABLE
    if (vtxTableNeedsInit) {
        vtxTableNeedsInit = false;
        vtxTableInit();  // Reinitialize and refresh the in-memory copies
    }
#endif
}

dispatchEntry_t writeReadEepromEntry =
{
    writeReadEeprom, 0, NULL, false
};

static void serializeSDCardSummaryReply(sbuf_t *dst)
{
    uint8_t flags = 0;
    uint8_t state = 0;
    uint8_t lastError = 0;
    uint32_t freeSpace = 0;
    uint32_t totalSpace = 0;

#if defined(USE_SDCARD)
    if (sdcardConfig()->mode != SDCARD_MODE_NONE) {
        flags = MSP_SDCARD_FLAG_SUPPORTED;

        // Merge the card and filesystem states together
        if (!sdcard_isInserted()) {
            state = MSP_SDCARD_STATE_NOT_PRESENT;
        } else if (!sdcard_isFunctional()) {
            state = MSP_SDCARD_STATE_FATAL;
        } else {
            switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                state = MSP_SDCARD_STATE_READY;
                break;

            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
             if (sdcard_isInitialized()) {
                 state = MSP_SDCARD_STATE_FS_INIT;
             } else {
                 state = MSP_SDCARD_STATE_CARD_INIT;
             }
             break;

            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
            default:
                state = MSP_SDCARD_STATE_FATAL;
                break;
            }
        }

        lastError = afatfs_getLastError();
        // Write free space and total space in kilobytes
        if (state == MSP_SDCARD_STATE_READY) {
            freeSpace = afatfs_getContiguousFreeSpace() / 1024;
            totalSpace = sdcard_getMetadata()->numBlocks / 2;
        }
    }
#endif

    sbufWriteU8(dst, flags);
    sbufWriteU8(dst, state);
    sbufWriteU8(dst, lastError);
    sbufWriteU32(dst, freeSpace);
    sbufWriteU32(dst, totalSpace);
}

static void serializeDataflashSummaryReply(sbuf_t *dst)
{
#ifdef USE_FLASHFS
    if (flashfsIsSupported()) {
        uint8_t flags = MSP_FLASHFS_FLAG_SUPPORTED;
        flags |= (flashfsIsReady() ? MSP_FLASHFS_FLAG_READY : 0);

        const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_FLASHFS);

        sbufWriteU8(dst, flags);
        sbufWriteU32(dst, FLASH_PARTITION_SECTOR_COUNT(flashPartition));
        sbufWriteU32(dst, flashfsGetSize());
        sbufWriteU32(dst, flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
    } else
#endif

    // FlashFS is not configured or valid device is not detected
    {
        sbufWriteU8(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 0);
    }
}

#ifdef USE_FLASHFS
enum compressionType_e {
    NO_COMPRESSION,
    HUFFMAN
};

static void serializeDataflashReadReply(sbuf_t *dst, uint32_t address, const uint16_t size, bool useLegacyFormat, bool allowCompression)
{
    STATIC_ASSERT(MSP_PORT_DATAFLASH_INFO_SIZE >= 16, MSP_PORT_DATAFLASH_INFO_SIZE_invalid);

    uint16_t readLen = size;
    const int bytesRemainingInBuf = sbufBytesRemaining(dst) - MSP_PORT_DATAFLASH_INFO_SIZE;
    if (readLen > bytesRemainingInBuf) {
        readLen = bytesRemainingInBuf;
    }
    // size will be lower than that requested if we reach end of volume
    const uint32_t flashfsSize = flashfsGetSize();
    if (readLen > flashfsSize - address) {
        // truncate the request
        readLen = flashfsSize - address;
    }
    sbufWriteU32(dst, address);

    // legacy format does not support compression
#ifdef USE_HUFFMAN
    const uint8_t compressionMethod = (!allowCompression || useLegacyFormat) ? NO_COMPRESSION : HUFFMAN;
#else
    const uint8_t compressionMethod = NO_COMPRESSION;
    UNUSED(allowCompression);
#endif

    if (compressionMethod == NO_COMPRESSION) {

        uint16_t *readLenPtr = (uint16_t *)sbufPtr(dst);
        if (!useLegacyFormat) {
            // new format supports variable read lengths
            sbufWriteU16(dst, readLen);
            sbufWriteU8(dst, 0); // placeholder for compression format
        }

        const int bytesRead = flashfsReadAbs(address, sbufPtr(dst), readLen);

        if (!useLegacyFormat) {
            // update the 'read length' with the actual amount read from flash.
            *readLenPtr = bytesRead;
        }

        sbufAdvance(dst, bytesRead);

        if (useLegacyFormat) {
            // pad the buffer with zeros
            for (int i = bytesRead; i < size; i++) {
                sbufWriteU8(dst, 0);
            }
        }
    } else {
#ifdef USE_HUFFMAN
        // compress in 256-byte chunks
        const uint16_t READ_BUFFER_SIZE = 256;
        // This may be DMAable, so make it cache aligned
        __attribute__ ((aligned(32))) uint8_t readBuffer[READ_BUFFER_SIZE];

        huffmanState_t state = {
            .bytesWritten = 0,
            .outByte = sbufPtr(dst) + sizeof(uint16_t) + sizeof(uint8_t) + HUFFMAN_INFO_SIZE,
            .outBufLen = readLen,
            .outBit = 0x80,
        };
        *state.outByte = 0;

        uint16_t bytesReadTotal = 0;
        // read until output buffer overflows or flash is exhausted
        while (state.bytesWritten < state.outBufLen && address + bytesReadTotal < flashfsSize) {
            const int bytesRead = flashfsReadAbs(address + bytesReadTotal, readBuffer,
                MIN(sizeof(readBuffer), flashfsSize - address - bytesReadTotal));

            const int status = huffmanEncodeBufStreaming(&state, readBuffer, bytesRead, huffmanTable);
            if (status == -1) {
                // overflow
                break;
            }

            bytesReadTotal += bytesRead;
        }

        if (state.outBit != 0x80) {
            ++state.bytesWritten;
        }

        // header
        sbufWriteU16(dst, HUFFMAN_INFO_SIZE + state.bytesWritten);
        sbufWriteU8(dst, compressionMethod);
        // payload
        sbufWriteU16(dst, bytesReadTotal);
        sbufAdvance(dst, state.bytesWritten);
#endif
    }
}
#endif // USE_FLASHFS

/*
 * Returns true if the command was processd, false otherwise.
 * May set mspPostProcessFunc to a function to be called once the command has been processed
 */
static bool mspCommonProcessOutCommand(int16_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
    UNUSED(mspPostProcessFn);

    switch (cmdMSP) {
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;

    case MSP_FC_VARIANT:
        sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;

    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO:
    {
        sbufWriteData(dst, systemConfig()->boardIdentifier, BOARD_IDENTIFIER_LENGTH);
#ifdef USE_HARDWARE_REVISION_DETECTION
        sbufWriteU16(dst, hardwareRevision);
#else
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection.
#endif
#if defined(USE_MAX7456)
        sbufWriteU8(dst, 2);  // 2 == FC with MAX7456
#else
        sbufWriteU8(dst, 0);  // 0 == FC
#endif

        // Target capabilities (uint8)
#define TARGET_HAS_VCP 0
#define TARGET_HAS_SOFTSERIAL 1
#define TARGET_HAS_FLASH_BOOTLOADER 3
#define TARGET_SUPPORTS_RX_BIND 6

        uint8_t targetCapabilities = 0;
#ifdef USE_VCP
        targetCapabilities |= BIT(TARGET_HAS_VCP);
#endif
#if defined(USE_SOFTSERIAL)
        targetCapabilities |= BIT(TARGET_HAS_SOFTSERIAL);
#endif
#if defined(USE_FLASH_BOOT_LOADER)
        targetCapabilities |= BIT(TARGET_HAS_FLASH_BOOTLOADER);
#endif
#if defined(USE_RX_BIND)
        if (getRxBindSupported()) {
            targetCapabilities |= BIT(TARGET_SUPPORTS_RX_BIND);
        }
#endif

        sbufWriteU8(dst, targetCapabilities);

        // Target name with explicit length
        sbufWriteU8(dst, strlen(targetName));
        sbufWriteData(dst, targetName, strlen(targetName));

#if defined(USE_BOARD_INFO)
        // Board name with explicit length
        char *value = getBoardName();
        sbufWriteU8(dst, strlen(value));
        sbufWriteString(dst, value);

        // Manufacturer id with explicit length
        value = getManufacturerId();
        sbufWriteU8(dst, strlen(value));
        sbufWriteString(dst, value);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif

#if defined(USE_SIGNATURE)
        // Signature
        sbufWriteData(dst, getSignature(), SIGNATURE_LENGTH);
#else
        uint8_t emptySignature[SIGNATURE_LENGTH];
        memset(emptySignature, 0, sizeof(emptySignature));
        sbufWriteData(dst, &emptySignature, sizeof(emptySignature));
#endif

        sbufWriteU8(dst, getMcuTypeId());

        // Added in API version 1.42
        sbufWriteU8(dst, systemConfig()->configurationState);

        // Added in API version 1.43
        sbufWriteU16(dst, gyro.sampleRateHz); // informational so the configurator can display the correct gyro/pid frequencies in the drop-down

        // Configuration warnings / problems (uint32_t)
#define PROBLEM_ACC_NEEDS_CALIBRATION 0
#define PROBLEM_MOTOR_PROTOCOL_DISABLED 1

        uint32_t configurationProblems = 0;

#if defined(USE_ACC)
        if (!accHasBeenCalibrated()) {
            configurationProblems |= BIT(PROBLEM_ACC_NEEDS_CALIBRATION);
        }
#endif

        if (!checkMotorProtocolEnabled(&motorConfig()->dev, NULL)) {
            configurationProblems |= BIT(PROBLEM_MOTOR_PROTOCOL_DISABLED);
        }

        sbufWriteU32(dst, configurationProblems);

        // Added in MSP API 1.44
#if defined(USE_SPI)
        sbufWriteU8(dst, spiGetRegisteredDeviceCount());
#else
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_I2C)
        sbufWriteU8(dst, i2cGetRegisteredDeviceCount());
#else
        sbufWriteU8(dst, 0);
#endif

        break;
    }

    case MSP_BUILD_INFO:
        sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
        sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
        sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        // Added in API version 1.46
        sbufWriteBuildInfoFlags(dst);
        break;

    case MSP_ANALOG:
        sbufWriteU8(dst, (uint8_t)constrain(getLegacyBatteryVoltage(), 0, 255));
        sbufWriteU16(dst, (uint16_t)constrain(getMAhDrawn(), 0, 0xFFFF)); // milliamp hours drawn from battery
        sbufWriteU16(dst, getRssi());
        sbufWriteU16(dst, (int16_t)constrain(getAmperage(), -0x8000, 0x7FFF)); // send current in 0.01 A steps, range is -320A to 320A
        sbufWriteU16(dst, getBatteryVoltage());
        break;

    case MSP_DEBUG:
        for (int i = 0; i < DEBUG16_VALUE_COUNT; i++) {
            sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
        }
        break;

    case MSP_UID:
        sbufWriteU32(dst, U_ID_0);
        sbufWriteU32(dst, U_ID_1);
        sbufWriteU32(dst, U_ID_2);
        break;

    case MSP_FEATURE_CONFIG:
        sbufWriteU32(dst, featureConfig()->enabledFeatures);
        break;

#ifdef USE_BEEPER
    case MSP_BEEPER_CONFIG:
        sbufWriteU32(dst, beeperConfig()->beeper_off_flags);
        sbufWriteU8(dst, beeperConfig()->dshotBeaconTone);
        sbufWriteU32(dst, beeperConfig()->dshotBeaconOffFlags);
        break;
#endif

    case MSP_BATTERY_STATE: {
        // battery characteristics
        sbufWriteU8(dst, (uint8_t)constrain(getBatteryCellCount(), 0, 255)); // 0 indicates battery not detected.
        sbufWriteU16(dst, batteryConfig()->batteryCapacity); // in mAh

        // battery state
        sbufWriteU8(dst, (uint8_t)constrain(getLegacyBatteryVoltage(), 0, 255)); // in 0.1V steps
        sbufWriteU16(dst, (uint16_t)constrain(getMAhDrawn(), 0, 0xFFFF)); // milliamp hours drawn from battery
        sbufWriteU16(dst, (int16_t)constrain(getAmperage(), -0x8000, 0x7FFF)); // send current in 0.01 A steps, range is -320A to 320A

        // battery alerts
        sbufWriteU8(dst, (uint8_t)getBatteryState());

        sbufWriteU16(dst, getBatteryVoltage()); // in 0.01V steps
        break;
    }

    case MSP_VOLTAGE_METERS: {
        // write out id and voltage meter values, once for each meter we support
        uint8_t count = supportedVoltageMeterCount;
#ifdef USE_ESC_SENSOR
        count -= VOLTAGE_METER_ID_ESC_COUNT - getMotorCount();
#endif

        for (int i = 0; i < count; i++) {

            voltageMeter_t meter;
            uint8_t id = (uint8_t)voltageMeterIds[i];
            voltageMeterRead(id, &meter);

            sbufWriteU8(dst, id);
            sbufWriteU8(dst, (uint8_t)constrain((meter.displayFiltered + 5) / 10, 0, 255));
        }
        break;
    }

    case MSP_CURRENT_METERS: {
        // write out id and current meter values, once for each meter we support
        uint8_t count = supportedCurrentMeterCount;
#ifdef USE_ESC_SENSOR
        count -= VOLTAGE_METER_ID_ESC_COUNT - getMotorCount();
#endif
        for (int i = 0; i < count; i++) {

            currentMeter_t meter;
            uint8_t id = (uint8_t)currentMeterIds[i];
            currentMeterRead(id, &meter);

            sbufWriteU8(dst, id);
            sbufWriteU16(dst, (uint16_t)constrain(meter.mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
            sbufWriteU16(dst, (uint16_t)constrain(meter.amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps (mA). Negative range is truncated to zero
        }
        break;
    }

    case MSP_VOLTAGE_METER_CONFIG:
        {
            // by using a sensor type and a sub-frame length it's possible to configure any type of voltage meter,
            // e.g. an i2c/spi/can sensor or any sensor not built directly into the FC such as ESC/RX/SPort/SBus that has
            // different configuration requirements.
            STATIC_ASSERT(VOLTAGE_SENSOR_ADC_VBAT == 0, VOLTAGE_SENSOR_ADC_VBAT_incorrect); // VOLTAGE_SENSOR_ADC_VBAT should be the first index
            sbufWriteU8(dst, MAX_VOLTAGE_SENSOR_ADC); // voltage meters in payload
            for (int i = VOLTAGE_SENSOR_ADC_VBAT; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
                const uint8_t adcSensorSubframeLength = 1 + 1 + 1 + 1 + 1; // length of id, type, vbatscale, vbatresdivval, vbatresdivmultipler, in bytes
                sbufWriteU8(dst, adcSensorSubframeLength); // ADC sensor sub-frame length

                sbufWriteU8(dst, voltageMeterADCtoIDMap[i]); // id of the sensor
                sbufWriteU8(dst, VOLTAGE_SENSOR_TYPE_ADC_RESISTOR_DIVIDER); // indicate the type of sensor that the next part of the payload is for

                sbufWriteU8(dst, voltageSensorADCConfig(i)->vbatscale);
                sbufWriteU8(dst, voltageSensorADCConfig(i)->vbatresdivval);
                sbufWriteU8(dst, voltageSensorADCConfig(i)->vbatresdivmultiplier);
            }
            // if we had any other voltage sensors, this is where we would output any needed configuration
        }

        break;
    case MSP_CURRENT_METER_CONFIG: {
        // the ADC and VIRTUAL sensors have the same configuration requirements, however this API reflects
        // that this situation may change and allows us to support configuration of any current sensor with
        // specialist configuration requirements.

        int currentMeterCount = 1;

#ifdef USE_VIRTUAL_CURRENT_METER
        currentMeterCount++;
#endif
        sbufWriteU8(dst, currentMeterCount);

        const uint8_t adcSensorSubframeLength = 1 + 1 + 2 + 2; // length of id, type, scale, offset, in bytes
        sbufWriteU8(dst, adcSensorSubframeLength);
        sbufWriteU8(dst, CURRENT_METER_ID_BATTERY_1); // the id of the meter
        sbufWriteU8(dst, CURRENT_SENSOR_ADC); // indicate the type of sensor that the next part of the payload is for
        sbufWriteU16(dst, currentSensorADCConfig()->scale);
        sbufWriteU16(dst, currentSensorADCConfig()->offset);

#ifdef USE_VIRTUAL_CURRENT_METER
        const int8_t virtualSensorSubframeLength = 1 + 1 + 2 + 2; // length of id, type, scale, offset, in bytes
        sbufWriteU8(dst, virtualSensorSubframeLength);
        sbufWriteU8(dst, CURRENT_METER_ID_VIRTUAL_1); // the id of the meter
        sbufWriteU8(dst, CURRENT_SENSOR_VIRTUAL); // indicate the type of sensor that the next part of the payload is for
        sbufWriteU16(dst, currentSensorVirtualConfig()->scale);
        sbufWriteU16(dst, currentSensorVirtualConfig()->offset);
#endif

        // if we had any other current sensors, this is where we would output any needed configuration
        break;
    }

    case MSP_BATTERY_CONFIG:
        sbufWriteU8(dst, (batteryConfig()->vbatmincellvoltage + 5) / 10);
        sbufWriteU8(dst, (batteryConfig()->vbatmaxcellvoltage + 5) / 10);
        sbufWriteU8(dst, (batteryConfig()->vbatwarningcellvoltage + 5) / 10);
        sbufWriteU16(dst, batteryConfig()->batteryCapacity);
        sbufWriteU8(dst, batteryConfig()->voltageMeterSource);
        sbufWriteU8(dst, batteryConfig()->currentMeterSource);
        sbufWriteU16(dst, batteryConfig()->vbatmincellvoltage);
        sbufWriteU16(dst, batteryConfig()->vbatmaxcellvoltage);
        sbufWriteU16(dst, batteryConfig()->vbatwarningcellvoltage);
        break;

    case MSP_TRANSPONDER_CONFIG: {
#ifdef USE_TRANSPONDER
        // Backward compatibility to BFC 3.1.1 is lost for this message type
        sbufWriteU8(dst, TRANSPONDER_PROVIDER_COUNT);
        for (unsigned int i = 0; i < TRANSPONDER_PROVIDER_COUNT; i++) {
            sbufWriteU8(dst, transponderRequirements[i].provider);
            sbufWriteU8(dst, transponderRequirements[i].dataLength);
        }

        uint8_t provider = transponderConfig()->provider;
        sbufWriteU8(dst, provider);

        if (provider) {
            uint8_t requirementIndex = provider - 1;
            uint8_t providerDataLength = transponderRequirements[requirementIndex].dataLength;

            for (unsigned int i = 0; i < providerDataLength; i++) {
                sbufWriteU8(dst, transponderConfig()->data[i]);
            }
        }
#else
        sbufWriteU8(dst, 0); // no providers
#endif
        break;
    }

#if defined(USE_OSD)
    case MSP_OSD_CONFIG: {
#define OSD_FLAGS_OSD_FEATURE           (1 << 0)
//#define OSD_FLAGS_OSD_SLAVE             (1 << 1)
#define OSD_FLAGS_RESERVED_1            (1 << 2)
#define OSD_FLAGS_OSD_HARDWARE_FRSKYOSD (1 << 3)
#define OSD_FLAGS_OSD_HARDWARE_MAX_7456 (1 << 4)
#define OSD_FLAGS_OSD_DEVICE_DETECTED   (1 << 5)
#define OSD_FLAGS_OSD_MSP_DEVICE        (1 << 6)

        uint8_t osdFlags = 0;

        osdFlags |= OSD_FLAGS_OSD_FEATURE;

        osdDisplayPortDevice_e deviceType;
        displayPort_t *osdDisplayPort = osdGetDisplayPort(&deviceType);
        bool displayIsReady = osdDisplayPort && displayCheckReady(osdDisplayPort, true);
        switch (deviceType) {
        case OSD_DISPLAYPORT_DEVICE_MAX7456:
            osdFlags |= OSD_FLAGS_OSD_HARDWARE_MAX_7456;
            if (displayIsReady) {
                osdFlags |= OSD_FLAGS_OSD_DEVICE_DETECTED;
            }

            break;
        case OSD_DISPLAYPORT_DEVICE_FRSKYOSD:
            osdFlags |= OSD_FLAGS_OSD_HARDWARE_FRSKYOSD;
            if (displayIsReady) {
                osdFlags |= OSD_FLAGS_OSD_DEVICE_DETECTED;
            }

            break;
        case OSD_DISPLAYPORT_DEVICE_MSP:
            osdFlags |= OSD_FLAGS_OSD_MSP_DEVICE;
            if (displayIsReady) {
                osdFlags |= OSD_FLAGS_OSD_DEVICE_DETECTED;
            }

            break;
        default:
            break;
        }

        sbufWriteU8(dst, osdFlags);

#ifdef USE_OSD_SD
        // send video system (AUTO/PAL/NTSC/HD)
        sbufWriteU8(dst, vcdProfile()->video_system);
#else
        sbufWriteU8(dst, VIDEO_SYSTEM_HD);
#endif // USE_OSD_SD

        // OSD specific, not applicable to OSD slaves.

        // Configuration
        sbufWriteU8(dst, osdConfig()->units);

        // Alarms
        sbufWriteU8(dst, osdConfig()->rssi_alarm);
        sbufWriteU16(dst, osdConfig()->cap_alarm);

        // Reuse old timer alarm (U16) as OSD_ITEM_COUNT
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, OSD_ITEM_COUNT);

        sbufWriteU16(dst, osdConfig()->alt_alarm);

        // Element position and visibility
        for (int i = 0; i < OSD_ITEM_COUNT; i++) {
            sbufWriteU16(dst, osdElementConfig()->item_pos[i]);
        }

        // Post flight statistics
        sbufWriteU8(dst, OSD_STAT_COUNT);
        for (int i = 0; i < OSD_STAT_COUNT; i++ ) {
            sbufWriteU8(dst, osdStatGetState(i));
        }

        // Timers
        sbufWriteU8(dst, OSD_TIMER_COUNT);
        for (int i = 0; i < OSD_TIMER_COUNT; i++) {
            sbufWriteU16(dst, osdConfig()->timers[i]);
        }

        // Enabled warnings
        // Send low word first for backwards compatibility (API < 1.41)
        sbufWriteU16(dst, (uint16_t)(osdConfig()->enabledWarnings & 0xFFFF));
        // API >= 1.41
        // Send the warnings count and 32bit enabled warnings flags.
        // Add currently active OSD profile (0 indicates OSD profiles not available).
        // Add OSD stick overlay mode (0 indicates OSD stick overlay not available).
        sbufWriteU8(dst, OSD_WARNING_COUNT);
        sbufWriteU32(dst, osdConfig()->enabledWarnings);

#ifdef USE_OSD_PROFILES
        sbufWriteU8(dst, OSD_PROFILE_COUNT);            // available profiles
        sbufWriteU8(dst, osdConfig()->osdProfileIndex); // selected profile
#else
        // If the feature is not available there is only 1 profile and it's always selected
        sbufWriteU8(dst, 1);
        sbufWriteU8(dst, 1);
#endif // USE_OSD_PROFILES

#ifdef USE_OSD_STICK_OVERLAY
        sbufWriteU8(dst, osdConfig()->overlay_radio_mode);
#else
        sbufWriteU8(dst, 0);
#endif // USE_OSD_STICK_OVERLAY

        // API >= 1.43
        // Add the camera frame element width/height
        sbufWriteU8(dst, osdConfig()->camera_frame_width);
        sbufWriteU8(dst, osdConfig()->camera_frame_height);

        // API >= 1.46
        sbufWriteU16(dst, osdConfig()->link_quality_alarm);

        // API >= 1.47
        sbufWriteU16(dst, osdConfig()->rssi_dbm_alarm);

        break;
    }
#endif // USE_OSD

    case MSP_OSD_CANVAS: {
#ifdef USE_OSD
        sbufWriteU8(dst, osdConfig()->canvas_cols);
        sbufWriteU8(dst, osdConfig()->canvas_rows);
#endif
        break;
    }

    default:
        return false;
    }
    return true;
}

static bool mspProcessOutCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *dst)
{
    bool unsupportedCommand = false;

#if !defined(USE_VTX_COMMON) || !defined(USE_VTX_MSP)
    UNUSED(srcDesc);
#endif

    switch (cmdMSP) {
    case MSP_STATUS_EX:
    case MSP_STATUS:
        {
            boxBitmask_t flightModeFlags;
            const int flagBits = packFlightModeFlags(&flightModeFlags);

            sbufWriteU16(dst, getTaskDeltaTimeUs(TASK_PID));
#ifdef USE_I2C
            sbufWriteU16(dst, i2cGetErrorCounter());
#else
            sbufWriteU16(dst, 0);
#endif
            sbufWriteU16(dst, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_RANGEFINDER) << 4 | sensors(SENSOR_GYRO) << 5);
            sbufWriteData(dst, &flightModeFlags, 4);        // unconditional part of flags, first 32 bits
            sbufWriteU8(dst, getCurrentPidProfileIndex());
            sbufWriteU16(dst, constrain(getAverageSystemLoadPercent(), 0, LOAD_PERCENTAGE_ONE));
            if (cmdMSP == MSP_STATUS_EX) {
                sbufWriteU8(dst, PID_PROFILE_COUNT);
                sbufWriteU8(dst, getCurrentControlRateProfileIndex());
            } else {  // MSP_STATUS
                sbufWriteU16(dst, 0); // gyro cycle time
            }

            // write flightModeFlags header. Lowest 4 bits contain number of bytes that follow
            // header is emited even when all bits fit into 32 bits to allow future extension
            int byteCount = (flagBits - 32 + 7) / 8;        // 32 already stored, round up
            byteCount = constrain(byteCount, 0, 15);        // limit to 16 bytes (128 bits)
            sbufWriteU8(dst, byteCount);
            sbufWriteData(dst, ((uint8_t*)&flightModeFlags) + 4, byteCount);

            // Write arming disable flags
            // 1 byte, flag count
            sbufWriteU8(dst, ARMING_DISABLE_FLAGS_COUNT);
            // 4 bytes, flags
            const uint32_t armingDisableFlags = getArmingDisableFlags();
            sbufWriteU32(dst, armingDisableFlags);

            // config state flags - bits to indicate the state of the configuration, reboot required, etc.
            // other flags can be added as needed
            sbufWriteU8(dst, (getRebootRequired() << 0));

            // Added in API version 1.46
            // Write CPU temp
#ifdef USE_ADC_INTERNAL
            sbufWriteU16(dst, getCoreTemperatureCelsius());
#else
            sbufWriteU16(dst, 0);
#endif
        }
        break;

    case MSP_RAW_IMU:
        {

            for (int i = 0; i < 3; i++) {
#if defined(USE_ACC)
                sbufWriteU16(dst, lrintf(acc.accADC.v[i]));
#else
                sbufWriteU16(dst, 0);
#endif
            }
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, gyroRateDps(i));
            }
            for (int i = 0; i < 3; i++) {
#if defined(USE_MAG)
                sbufWriteU16(dst, lrintf(mag.magADC.v[i]));
#else
                sbufWriteU16(dst, 0);
#endif
            }
        }
        break;

case MSP_NAME:
        {
            const int nameLen = strlen(pilotConfig()->craftName);
            for (int i = 0; i < nameLen; i++) {
                sbufWriteU8(dst, pilotConfig()->craftName[i]);
            }
        }
        break;

#ifdef USE_SERVOS
    case MSP_SERVO:
        sbufWriteData(dst, &servo, MAX_SUPPORTED_SERVOS * 2);
        break;
    case MSP_SERVO_CONFIGURATIONS:
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            sbufWriteU16(dst, servoParams(i)->min);
            sbufWriteU16(dst, servoParams(i)->max);
            sbufWriteU16(dst, servoParams(i)->middle);
            sbufWriteU8(dst, servoParams(i)->rate);
            sbufWriteU8(dst, servoParams(i)->forwardFromChannel);
            sbufWriteU32(dst, servoParams(i)->reversedSources);
        }
        break;

    case MSP_SERVO_MIX_RULES:
        for (int i = 0; i < MAX_SERVO_RULES; i++) {
            sbufWriteU8(dst, customServoMixers(i)->targetChannel);
            sbufWriteU8(dst, customServoMixers(i)->inputSource);
            sbufWriteU8(dst, customServoMixers(i)->rate);
            sbufWriteU8(dst, customServoMixers(i)->speed);
            sbufWriteU8(dst, customServoMixers(i)->min);
            sbufWriteU8(dst, customServoMixers(i)->max);
            sbufWriteU8(dst, customServoMixers(i)->box);
        }
        break;
#endif

    case MSP_MOTOR:
        for (unsigned i = 0; i < 8; i++) {
#ifdef USE_MOTOR
            if (!motorIsEnabled() || i >= MAX_SUPPORTED_MOTORS || !motorIsMotorEnabled(i)) {
                sbufWriteU16(dst, 0);
                continue;
            }

            sbufWriteU16(dst, motorConvertToExternal(motor[i]));
#else
            sbufWriteU16(dst, 0);
#endif
        }

        break;

    // Added in API version 1.42
    case MSP_MOTOR_TELEMETRY:
        sbufWriteU8(dst, getMotorCount());
        for (unsigned i = 0; i < getMotorCount(); i++) {
            int rpm = 0;
            uint16_t invalidPct = 0;
            uint8_t escTemperature = 0;  // degrees celcius
            uint16_t escVoltage = 0;     // 0.01V per unit
            uint16_t escCurrent = 0;     // 0.01A per unit
            uint16_t escConsumption = 0; // mAh

            bool rpmDataAvailable = false;

#ifdef USE_DSHOT_TELEMETRY
            if (useDshotTelemetry) {
                rpm = lrintf(getDshotRpm(i));
                rpmDataAvailable = true;
                invalidPct = 10000; // 100.00%


#ifdef USE_DSHOT_TELEMETRY_STATS
                if (isDshotMotorTelemetryActive(i)) {
                    invalidPct = getDshotTelemetryMotorInvalidPercent(i);
                }
#endif


                // Provide extended dshot telemetry
                if ((dshotTelemetryState.motorState[i].telemetryTypes & DSHOT_EXTENDED_TELEMETRY_MASK) != 0) {
                    // Temperature Celsius [0, 1, ..., 255] in degree Celsius, just like Blheli_32 and KISS
                    if ((dshotTelemetryState.motorState[i].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE)) != 0) {
                        escTemperature = dshotTelemetryState.motorState[i].telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE];
                    }

                    // Current -> 0-255A step 1A
                    if ((dshotTelemetryState.motorState[i].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_CURRENT)) != 0) {
                        escCurrent = dshotTelemetryState.motorState[i].telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT];
                    }

                    // Voltage -> 0-63,75V step 0,25V
                    if ((dshotTelemetryState.motorState[i].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_VOLTAGE)) != 0) {
                        escVoltage = dshotTelemetryState.motorState[i].telemetryData[DSHOT_TELEMETRY_TYPE_VOLTAGE] >> 2;
                    }
                }
            }
#endif

#ifdef USE_ESC_SENSOR
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                escSensorData_t *escData = getEscSensorData(i);
                if (!rpmDataAvailable) {  // We want DSHOT telemetry RPM data (if available) to have precedence
                    rpm = lrintf(erpmToRpm(escData->rpm));
                    rpmDataAvailable = true;
                }
                escTemperature = escData->temperature;
                escVoltage = escData->voltage;
                escCurrent = escData->current;
                escConsumption = escData->consumption;
            }
#endif

            sbufWriteU32(dst, (rpmDataAvailable ? rpm : 0));
            sbufWriteU16(dst, invalidPct);
            sbufWriteU8(dst, escTemperature);
            sbufWriteU16(dst, escVoltage);
            sbufWriteU16(dst, escCurrent);
            sbufWriteU16(dst, escConsumption);
        }
        break;

    case MSP2_MOTOR_OUTPUT_REORDERING:
        {
            sbufWriteU8(dst, MAX_SUPPORTED_MOTORS);

            for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
                sbufWriteU8(dst, motorConfig()->dev.motorOutputReordering[i]);
            }
        }
        break;

#ifdef USE_VTX_COMMON
    case MSP2_GET_VTX_DEVICE_STATUS:
        {
            const vtxDevice_t *vtxDevice = vtxCommonDevice();
            vtxCommonSerializeDeviceStatus(vtxDevice, dst);
        }
        break;
#endif

#ifdef USE_OSD
    case MSP2_GET_OSD_WARNINGS:
        {
            bool isBlinking;
            uint8_t displayAttr;
            char warningsBuffer[OSD_FORMAT_MESSAGE_BUFFER_SIZE];

            renderOsdWarning(warningsBuffer, &isBlinking, &displayAttr);
            const uint8_t warningsLen = strlen(warningsBuffer);

            if (isBlinking) {
                displayAttr |= DISPLAYPORT_BLINK;
            }
            sbufWriteU8(dst, displayAttr);  // see displayPortSeverity_e
            sbufWriteU8(dst, warningsLen);  // length byte followed by the actual characters
            for (unsigned i = 0; i < warningsLen; i++) {
                sbufWriteU8(dst, warningsBuffer[i]);
            }
            break;
        }
#endif

    case MSP_RC:
        for (int i = 0; i < rxRuntimeState.channelCount; i++) {
            sbufWriteU16(dst, rcData[i]);
        }
        break;

    case MSP_ATTITUDE:
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;

    case MSP_ALTITUDE:
        sbufWriteU32(dst, getEstimatedAltitudeCm());
#ifdef USE_VARIO
        sbufWriteU16(dst, getEstimatedVario());
#else
        sbufWriteU16(dst, 0);
#endif
        break;

    case MSP_SONAR_ALTITUDE:
#if defined(USE_RANGEFINDER)
        sbufWriteU32(dst, rangefinderGetLatestAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        break;

    case MSP_BOARD_ALIGNMENT_CONFIG:
        sbufWriteU16(dst, boardAlignment()->rollDegrees);
        sbufWriteU16(dst, boardAlignment()->pitchDegrees);
        sbufWriteU16(dst, boardAlignment()->yawDegrees);
        break;

    case MSP_ARMING_CONFIG:
        sbufWriteU8(dst, armingConfig()->auto_disarm_delay);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, imuConfig()->small_angle);
        sbufWriteU8(dst, armingConfig()->gyro_cal_on_first_arm);
        break;

    case MSP_RC_TUNING:
        sbufWriteU8(dst, currentControlRateProfile->rcRates[FD_ROLL]);
        sbufWriteU8(dst, currentControlRateProfile->rcExpo[FD_ROLL]);
        for (int i = 0 ; i < 3; i++) {
            sbufWriteU8(dst, currentControlRateProfile->rates[i]); // R,P,Y see flight_dynamics_index_t
        }
        sbufWriteU8(dst, 0);   // was currentControlRateProfile->tpa_rate
        sbufWriteU8(dst, currentControlRateProfile->thrMid8);
        sbufWriteU8(dst, currentControlRateProfile->thrExpo8);
        sbufWriteU16(dst, 0);   // was currentControlRateProfile->tpa_breakpoint
        sbufWriteU8(dst, currentControlRateProfile->rcExpo[FD_YAW]);
        sbufWriteU8(dst, currentControlRateProfile->rcRates[FD_YAW]);
        sbufWriteU8(dst, currentControlRateProfile->rcRates[FD_PITCH]);
        sbufWriteU8(dst, currentControlRateProfile->rcExpo[FD_PITCH]);

        // added in 1.41
        sbufWriteU8(dst, currentControlRateProfile->throttle_limit_type);
        sbufWriteU8(dst, currentControlRateProfile->throttle_limit_percent);

        // added in 1.42
        sbufWriteU16(dst, currentControlRateProfile->rate_limit[FD_ROLL]);
        sbufWriteU16(dst, currentControlRateProfile->rate_limit[FD_PITCH]);
        sbufWriteU16(dst, currentControlRateProfile->rate_limit[FD_YAW]);

        // added in 1.43
        sbufWriteU8(dst, currentControlRateProfile->rates_type);

        break;

    case MSP_PID:
        for (int i = 0; i < PID_ITEM_COUNT; i++) {
            sbufWriteU8(dst, currentPidProfile->pid[i].P);
            sbufWriteU8(dst, currentPidProfile->pid[i].I);
            sbufWriteU8(dst, currentPidProfile->pid[i].D);
        }
        break;

    case MSP_PIDNAMES:
        for (const char *c = pidNames; *c; c++) {
            sbufWriteU8(dst, *c);
        }
        break;

    case MSP_PID_CONTROLLER:
        sbufWriteU8(dst, PID_CONTROLLER_BETAFLIGHT);
        break;

    case MSP_MODE_RANGES:
        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *mac = modeActivationConditions(i);
            const box_t *box = findBoxByBoxId(mac->modeId);
            sbufWriteU8(dst, box->permanentId);
            sbufWriteU8(dst, mac->auxChannelIndex);
            sbufWriteU8(dst, mac->range.startStep);
            sbufWriteU8(dst, mac->range.endStep);
        }
        break;

    case MSP_MODE_RANGES_EXTRA:
        sbufWriteU8(dst, MAX_MODE_ACTIVATION_CONDITION_COUNT);          // prepend number of EXTRAs array elements

        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *mac = modeActivationConditions(i);
            const box_t *box = findBoxByBoxId(mac->modeId);
            const box_t *linkedBox = findBoxByBoxId(mac->linkedTo);
            sbufWriteU8(dst, box->permanentId);     // each element is aligned with MODE_RANGES by the permanentId
            sbufWriteU8(dst, mac->modeLogic);
            sbufWriteU8(dst, linkedBox->permanentId);
        }
        break;

    case MSP_ADJUSTMENT_RANGES:
        for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            const adjustmentRange_t *adjRange = adjustmentRanges(i);
            sbufWriteU8(dst, 0); // was adjRange->adjustmentIndex
            sbufWriteU8(dst, adjRange->auxChannelIndex);
            sbufWriteU8(dst, adjRange->range.startStep);
            sbufWriteU8(dst, adjRange->range.endStep);
            sbufWriteU8(dst, adjRange->adjustmentConfig);
            sbufWriteU8(dst, adjRange->auxSwitchChannelIndex);
        }
        break;

    case MSP_MOTOR_CONFIG:
        sbufWriteU16(dst, 0); // was minthrottle until after 4.5
        sbufWriteU16(dst, motorConfig()->maxthrottle);
        sbufWriteU16(dst, motorConfig()->mincommand);

        // API 1.42
        sbufWriteU8(dst, getMotorCount());
        sbufWriteU8(dst, motorConfig()->motorPoleCount);
#ifdef USE_DSHOT_TELEMETRY
        sbufWriteU8(dst, useDshotTelemetry);
#else
        sbufWriteU8(dst, 0);
#endif

#ifdef USE_ESC_SENSOR
        sbufWriteU8(dst, featureIsEnabled(FEATURE_ESC_SENSOR)); // ESC sensor available
#else
        sbufWriteU8(dst, 0);
#endif
        break;

#ifdef USE_MAG
    case MSP_COMPASS_CONFIG:
        sbufWriteU16(dst, imuConfig()->mag_declination);
        break;
#endif
    // Deprecated in favor of MSP_MOTOR_TELEMETY as of API version 1.42
    // Used by DJI FPV
    case MSP_ESC_SENSOR_DATA:
#if defined(USE_ESC_SENSOR)
        if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
            sbufWriteU8(dst, getMotorCount());
            for (int i = 0; i < getMotorCount(); i++) {
                const escSensorData_t *escData = getEscSensorData(i);
                sbufWriteU8(dst, escData->temperature);
                sbufWriteU16(dst, escData->rpm);
            }
        } else
#endif
#if defined(USE_DSHOT_TELEMETRY)
        if (useDshotTelemetry) {
            sbufWriteU8(dst, getMotorCount());
            for (int i = 0; i < getMotorCount(); i++) {
                sbufWriteU8(dst, dshotTelemetryState.motorState[i].telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE]);
                sbufWriteU16(dst, lrintf(getDshotRpm(i)));
            }
        }
        else
#endif
        {
            unsupportedCommand = true;
        }

        break;

#ifdef USE_GPS
    case MSP_GPS_CONFIG:
        sbufWriteU8(dst, gpsConfig()->provider);
        sbufWriteU8(dst, gpsConfig()->sbasMode);
        sbufWriteU8(dst, gpsConfig()->autoConfig);
        sbufWriteU8(dst, gpsConfig()->autoBaud);
        // Added in API version 1.43
        sbufWriteU8(dst, gpsConfig()->gps_set_home_point_once);
        sbufWriteU8(dst, gpsConfig()->gps_ublox_use_galileo);
        break;

    case MSP_RAW_GPS:
        sbufWriteU8(dst, STATE(GPS_FIX));
        sbufWriteU8(dst, gpsSol.numSat);
        sbufWriteU32(dst, gpsSol.llh.lat);
        sbufWriteU32(dst, gpsSol.llh.lon);
        sbufWriteU16(dst, (uint16_t)constrain(gpsSol.llh.altCm / 100, 0, UINT16_MAX)); // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. To maintain backwards compatibility compensate to 1m per lsb in MSP again.
        sbufWriteU16(dst, gpsSol.groundSpeed);
        sbufWriteU16(dst, gpsSol.groundCourse);
        // Added in API version 1.44
        sbufWriteU16(dst, gpsSol.dop.pdop);
        break;

    case MSP_COMP_GPS:
        sbufWriteU16(dst, GPS_distanceToHome);
        sbufWriteU16(dst, GPS_directionToHome / 10); // resolution increased in Betaflight 4.4 by factor of 10, this maintains backwards compatibility for DJI OSD
        sbufWriteU8(dst, GPS_update & 1);
        break;

    case MSP_GPSSVINFO:
        sbufWriteU8(dst, GPS_numCh);
        for (int i = 0; i < GPS_numCh; i++) {
           sbufWriteU8(dst, GPS_svinfo[i].chn);
           sbufWriteU8(dst, GPS_svinfo[i].svid);
           sbufWriteU8(dst, GPS_svinfo[i].quality);
           sbufWriteU8(dst, GPS_svinfo[i].cno);
        }
        break;

#ifdef USE_GPS_RESCUE
    case MSP_GPS_RESCUE:
        sbufWriteU16(dst, gpsRescueConfig()->maxRescueAngle);
        sbufWriteU16(dst, gpsRescueConfig()->returnAltitudeM);
        sbufWriteU16(dst, gpsRescueConfig()->descentDistanceM);
        sbufWriteU16(dst, gpsRescueConfig()->groundSpeedCmS);
        sbufWriteU16(dst, autopilotConfig()->throttle_min);
        sbufWriteU16(dst, autopilotConfig()->throttle_max);
        sbufWriteU16(dst, autopilotConfig()->hover_throttle);
        sbufWriteU8(dst,  gpsRescueConfig()->sanityChecks);
        sbufWriteU8(dst,  gpsRescueConfig()->minSats);

        // Added in API version 1.43
        sbufWriteU16(dst, gpsRescueConfig()->ascendRate);
        sbufWriteU16(dst, gpsRescueConfig()->descendRate);
        sbufWriteU8(dst, gpsRescueConfig()->allowArmingWithoutFix);
        sbufWriteU8(dst, gpsRescueConfig()->altitudeMode);
        // Added in API version 1.44
        sbufWriteU16(dst, gpsRescueConfig()->minStartDistM);
        // Added in API version 1.46
        sbufWriteU16(dst, gpsRescueConfig()->initialClimbM);
        break;

    case MSP_GPS_RESCUE_PIDS:
        sbufWriteU16(dst, autopilotConfig()->altitude_P);
        sbufWriteU16(dst, autopilotConfig()->altitude_I);
        sbufWriteU16(dst, autopilotConfig()->altitude_D);
        // altitude_F not implemented yet
        sbufWriteU16(dst, gpsRescueConfig()->velP);
        sbufWriteU16(dst, gpsRescueConfig()->velI);
        sbufWriteU16(dst, gpsRescueConfig()->velD);
        sbufWriteU16(dst, gpsRescueConfig()->yawP);
        break;
#endif
#endif

#if defined(USE_ACC)
    case MSP_ACC_TRIM:
        sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.pitch);
        sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.roll);

        break;
#endif
    case MSP_MIXER_CONFIG:
        sbufWriteU8(dst, mixerConfig()->mixerMode);
        sbufWriteU8(dst, mixerConfig()->yaw_motors_reversed);
        break;

    case MSP_RX_CONFIG:
        sbufWriteU8(dst, rxConfig()->serialrx_provider);
        sbufWriteU16(dst, rxConfig()->maxcheck);
        sbufWriteU16(dst, rxConfig()->midrc);
        sbufWriteU16(dst, rxConfig()->mincheck);
        sbufWriteU8(dst, rxConfig()->spektrum_sat_bind);
        sbufWriteU16(dst, rxConfig()->rx_min_usec);
        sbufWriteU16(dst, rxConfig()->rx_max_usec);
        sbufWriteU8(dst, 0); // not required in API 1.44, was rxConfig()->rcInterpolation
        sbufWriteU8(dst, 0); // not required in API 1.44, was rxConfig()->rcInterpolationInterval
        sbufWriteU16(dst, rxConfig()->airModeActivateThreshold * 10 + 1000);
#ifdef USE_RX_SPI
        sbufWriteU8(dst, rxSpiConfig()->rx_spi_protocol);
        sbufWriteU32(dst, rxSpiConfig()->rx_spi_id);
        sbufWriteU8(dst, rxSpiConfig()->rx_spi_rf_channel_count);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, rxConfig()->fpvCamAngleDegrees);
        sbufWriteU8(dst, 0); // not required in API 1.44, was rxConfig()->rcSmoothingChannels
#if defined(USE_RC_SMOOTHING_FILTER)
        sbufWriteU8(dst, 0); // not required in API 1.44, was rxConfig()->rc_smoothing_type
        sbufWriteU8(dst, rxConfig()->rc_smoothing_setpoint_cutoff);
        sbufWriteU8(dst, rxConfig()->rc_smoothing_feedforward_cutoff);
        sbufWriteU8(dst, 0); // not required in API 1.44, was rxConfig()->rc_smoothing_input_type
        sbufWriteU8(dst, 0); // not required in API 1.44, was rxConfig()->rc_smoothing_derivative_type
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_USB_CDC_HID)
        sbufWriteU8(dst, usbDevConfig()->type);
#else
        sbufWriteU8(dst, 0);
#endif
        // Added in MSP API 1.42
#if defined(USE_RC_SMOOTHING_FILTER)
        sbufWriteU8(dst, rxConfig()->rc_smoothing_auto_factor_rpy);
#else
        sbufWriteU8(dst, 0);
#endif
        // Added in MSP API 1.44
#if defined(USE_RC_SMOOTHING_FILTER)
        sbufWriteU8(dst, rxConfig()->rc_smoothing_mode);
#else
        sbufWriteU8(dst, 0);
#endif

        // Added in MSP API 1.45
#ifdef USE_RX_EXPRESSLRS
        sbufWriteData(dst, rxExpressLrsSpiConfig()->UID, sizeof(rxExpressLrsSpiConfig()->UID));
#else
        uint8_t emptyUid[6];
        memset(emptyUid, 0, sizeof(emptyUid));
        sbufWriteData(dst, &emptyUid, sizeof(emptyUid));
#endif
        // Added in MSP API 1.47
#ifdef USE_RX_EXPRESSLRS
        sbufWriteU8(dst, rxExpressLrsSpiConfig()->modelId);
#else
        sbufWriteU8(dst, 0);
#endif
        break;
    case MSP_FAILSAFE_CONFIG:
        sbufWriteU8(dst, failsafeConfig()->failsafe_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_landing_time);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);
        sbufWriteU8(dst, failsafeConfig()->failsafe_switch_mode);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle_low_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_procedure);
        break;

    case MSP_RXFAIL_CONFIG:
        for (int i = 0; i < rxRuntimeState.channelCount; i++) {
            sbufWriteU8(dst, rxFailsafeChannelConfigs(i)->mode);
            sbufWriteU16(dst, RXFAIL_STEP_TO_CHANNEL_VALUE(rxFailsafeChannelConfigs(i)->step));
        }
        break;

    case MSP_RSSI_CONFIG:
        sbufWriteU8(dst, rxConfig()->rssi_channel);
        break;

    case MSP_RX_MAP:
        sbufWriteData(dst, rxConfig()->rcmap, RX_MAPPABLE_CHANNEL_COUNT);
        break;

    case MSP_CF_SERIAL_CONFIG:
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                continue;
            };
            sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
            sbufWriteU16(dst, serialConfig()->portConfigs[i].functionMask);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].msp_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].gps_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].telemetry_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].blackbox_baudrateIndex);
        }
        break;
    case MSP2_COMMON_SERIAL_CONFIG: {
        uint8_t count = 0;
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                count++;
            }
        }
        sbufWriteU8(dst, count);
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                continue;
            };
            sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
            sbufWriteU32(dst, serialConfig()->portConfigs[i].functionMask);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].msp_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].gps_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].telemetry_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].blackbox_baudrateIndex);
        }
        break;
    }

#ifdef USE_LED_STRIP_STATUS_MODE
    case MSP_LED_COLORS:
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            const hsvColor_t *color = &ledStripStatusModeConfig()->colors[i];
            sbufWriteU16(dst, color->h);
            sbufWriteU8(dst, color->s);
            sbufWriteU8(dst, color->v);
        }
        break;
#endif

#ifdef USE_LED_STRIP
    case MSP_LED_STRIP_CONFIG:
        for (int i = 0; i < LED_STRIP_MAX_LENGTH; i++) {
#ifdef USE_LED_STRIP_STATUS_MODE
            const ledConfig_t *ledConfig = &ledStripStatusModeConfig()->ledConfigs[i];
            sbufWriteU32(dst, *ledConfig);
#else
            sbufWriteU32(dst, 0);
#endif
        }

        // API 1.41 - add indicator for advanced profile support and the current profile selection
        // 0 = basic ledstrip available
        // 1 = advanced ledstrip available
#ifdef USE_LED_STRIP_STATUS_MODE
        sbufWriteU8(dst, 1);   // advanced ledstrip available
#else
        sbufWriteU8(dst, 0);   // only simple ledstrip available
#endif
        sbufWriteU8(dst, ledStripConfig()->ledstrip_profile);
        break;
#endif

#ifdef USE_LED_STRIP_STATUS_MODE
    case MSP_LED_STRIP_MODECOLOR:
        for (int i = 0; i < LED_MODE_COUNT; i++) {
            for (int j = 0; j < LED_DIRECTION_COUNT; j++) {
                sbufWriteU8(dst, i);
                sbufWriteU8(dst, j);
                sbufWriteU8(dst, ledStripStatusModeConfig()->modeColors[i].color[j]);
            }
        }

        for (int j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
            sbufWriteU8(dst, LED_MODE_COUNT);
            sbufWriteU8(dst, j);
            sbufWriteU8(dst, ledStripStatusModeConfig()->specialColors.color[j]);
        }

        sbufWriteU8(dst, LED_AUX_CHANNEL);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, ledStripStatusModeConfig()->ledstrip_aux_channel);
        break;
#endif

    case MSP_DATAFLASH_SUMMARY:
        serializeDataflashSummaryReply(dst);
        break;

    case MSP_BLACKBOX_CONFIG:
#ifdef USE_BLACKBOX
        sbufWriteU8(dst, 1); //Blackbox supported
        sbufWriteU8(dst, blackboxConfig()->device);
        sbufWriteU8(dst, 1); // Rate numerator, not used anymore
        sbufWriteU8(dst, blackboxGetRateDenom());
        sbufWriteU16(dst, blackboxGetPRatio());
        sbufWriteU8(dst, blackboxConfig()->sample_rate);
        // Added in MSP API 1.45
        sbufWriteU32(dst, blackboxConfig()->fields_disabled_mask);
#else
        sbufWriteU8(dst, 0); // Blackbox not supported
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 0);
        // Added in MSP API 1.45
        sbufWriteU32(dst, 0);
#endif
        break;

    case MSP_SDCARD_SUMMARY:
        serializeSDCardSummaryReply(dst);
        break;

    case MSP_MOTOR_3D_CONFIG:
        sbufWriteU16(dst, flight3DConfig()->deadband3d_low);
        sbufWriteU16(dst, flight3DConfig()->deadband3d_high);
        sbufWriteU16(dst, flight3DConfig()->neutral3d);
        break;

    case MSP_RC_DEADBAND:
        sbufWriteU8(dst, rcControlsConfig()->deadband);
        sbufWriteU8(dst, rcControlsConfig()->yaw_deadband);
        sbufWriteU8(dst, rcControlsConfig()->alt_hold_deadband);
        sbufWriteU16(dst, flight3DConfig()->deadband3d_throttle);
        break;


    case MSP_SENSOR_ALIGNMENT: {
        uint8_t gyroAlignment;
#ifdef USE_MULTI_GYRO
        switch (gyroConfig()->gyro_to_use) {
        case GYRO_CONFIG_USE_GYRO_2:
            gyroAlignment = gyroDeviceConfig(1)->alignment;
            break;
        case GYRO_CONFIG_USE_GYRO_BOTH:
            // for dual-gyro in "BOTH" mode we only read/write gyro 0
        default:
            gyroAlignment = gyroDeviceConfig(0)->alignment;
            break;
        }
#else
        gyroAlignment = gyroDeviceConfig(0)->alignment;
#endif
        sbufWriteU8(dst, gyroAlignment);
        sbufWriteU8(dst, gyroAlignment);  // Starting with 4.0 gyro and acc alignment are the same
#if defined(USE_MAG)
        sbufWriteU8(dst, compassConfig()->mag_alignment);
#else
        sbufWriteU8(dst, 0);
#endif

        // API 1.41 - Add multi-gyro indicator, selected gyro, and support for separate gyro 1 & 2 alignment
        sbufWriteU8(dst, getGyroDetectionFlags());
#ifdef USE_MULTI_GYRO
        sbufWriteU8(dst, gyroConfig()->gyro_to_use);
        sbufWriteU8(dst, gyroDeviceConfig(0)->alignment);
        sbufWriteU8(dst, gyroDeviceConfig(1)->alignment);
#else
        sbufWriteU8(dst, GYRO_CONFIG_USE_GYRO_1);
        sbufWriteU8(dst, gyroDeviceConfig(0)->alignment);
        sbufWriteU8(dst, ALIGN_DEFAULT);
#endif

        break;
    }
    case MSP_ADVANCED_CONFIG:
        sbufWriteU8(dst, 1);  // was gyroConfig()->gyro_sync_denom - removed in API 1.43
        sbufWriteU8(dst, pidConfig()->pid_process_denom);
        sbufWriteU8(dst, motorConfig()->dev.useUnsyncedPwm);
        sbufWriteU8(dst, motorConfig()->dev.motorPwmProtocol);
        sbufWriteU16(dst, motorConfig()->dev.motorPwmRate);
        sbufWriteU16(dst, motorConfig()->motorIdle);
        sbufWriteU8(dst, 0); // DEPRECATED: gyro_use_32kHz
        sbufWriteU8(dst, motorConfig()->dev.motorPwmInversion);
        sbufWriteU8(dst, gyroConfig()->gyro_to_use);
        sbufWriteU8(dst, gyroConfig()->gyro_high_fsr);
        sbufWriteU8(dst, gyroConfig()->gyroMovementCalibrationThreshold);
        sbufWriteU16(dst, gyroConfig()->gyroCalibrationDuration);
        sbufWriteU16(dst, gyroConfig()->gyro_offset_yaw);
        sbufWriteU8(dst, gyroConfig()->checkOverflow);
        //Added in MSP API 1.42
        sbufWriteU8(dst, systemConfig()->debug_mode);
        sbufWriteU8(dst, DEBUG_COUNT);

        break;
    case MSP_FILTER_CONFIG :
        sbufWriteU8(dst, gyroConfig()->gyro_lpf1_static_hz);
        sbufWriteU16(dst, currentPidProfile->dterm_lpf1_static_hz);
        sbufWriteU16(dst, currentPidProfile->yaw_lowpass_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_1);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_1);
        sbufWriteU16(dst, currentPidProfile->dterm_notch_hz);
        sbufWriteU16(dst, currentPidProfile->dterm_notch_cutoff);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_2);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_2);
        sbufWriteU8(dst, currentPidProfile->dterm_lpf1_type);
        sbufWriteU8(dst, gyroConfig()->gyro_hardware_lpf);
        sbufWriteU8(dst, 0); // DEPRECATED: gyro_32khz_hardware_lpf
        sbufWriteU16(dst, gyroConfig()->gyro_lpf1_static_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_lpf2_static_hz);
        sbufWriteU8(dst, gyroConfig()->gyro_lpf1_type);
        sbufWriteU8(dst, gyroConfig()->gyro_lpf2_type);
        sbufWriteU16(dst, currentPidProfile->dterm_lpf2_static_hz);
        // Added in MSP API 1.41
        sbufWriteU8(dst, currentPidProfile->dterm_lpf2_type);
#if defined(USE_DYN_LPF)
        sbufWriteU16(dst, gyroConfig()->gyro_lpf1_dyn_min_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_lpf1_dyn_max_hz);
        sbufWriteU16(dst, currentPidProfile->dterm_lpf1_dyn_min_hz);
        sbufWriteU16(dst, currentPidProfile->dterm_lpf1_dyn_max_hz);
#else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif
        // Added in MSP API 1.42
#if defined(USE_DYN_NOTCH_FILTER)
        sbufWriteU8(dst, 0);  // DEPRECATED 1.43: dyn_notch_range
        sbufWriteU8(dst, 0);  // DEPRECATED 1.44: dyn_notch_width_percent
        sbufWriteU16(dst, dynNotchConfig()->dyn_notch_q);
        sbufWriteU16(dst, dynNotchConfig()->dyn_notch_min_hz);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif
#if defined(USE_RPM_FILTER)
        sbufWriteU8(dst, rpmFilterConfig()->rpm_filter_harmonics);
        sbufWriteU8(dst, rpmFilterConfig()->rpm_filter_min_hz);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_DYN_NOTCH_FILTER)
        // Added in MSP API 1.43
        sbufWriteU16(dst, dynNotchConfig()->dyn_notch_max_hz);
#else
        sbufWriteU16(dst, 0);
#endif
#if defined(USE_DYN_LPF)
        // Added in MSP API 1.44
        sbufWriteU8(dst, currentPidProfile->dterm_lpf1_dyn_expo);
#else
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_DYN_NOTCH_FILTER)
        sbufWriteU8(dst, dynNotchConfig()->dyn_notch_count);
#else
        sbufWriteU8(dst, 0);
#endif

        break;
    case MSP_PID_ADVANCED:
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0); // was pidProfile.yaw_p_limit
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // was vbatPidCompensation
#if defined(USE_FEEDFORWARD)
        sbufWriteU8(dst, currentPidProfile->feedforward_transition);
#else
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, 0); // was low byte of currentPidProfile->dtermSetpointWeight
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU16(dst, currentPidProfile->rateAccelLimit);
        sbufWriteU16(dst, currentPidProfile->yawRateAccelLimit);
        sbufWriteU8(dst, currentPidProfile->angle_limit);
        sbufWriteU8(dst, 0); // was pidProfile.levelSensitivity
        sbufWriteU16(dst, 0); // was currentPidProfile->itermThrottleThreshold
        sbufWriteU16(dst, currentPidProfile->anti_gravity_gain);
        sbufWriteU16(dst, 0); // was currentPidProfile->dtermSetpointWeight
        sbufWriteU8(dst, currentPidProfile->iterm_rotation);
        sbufWriteU8(dst, 0); // was currentPidProfile->smart_feedforward
#if defined(USE_ITERM_RELAX)
        sbufWriteU8(dst, currentPidProfile->iterm_relax);
        sbufWriteU8(dst, currentPidProfile->iterm_relax_type);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_ABSOLUTE_CONTROL)
        sbufWriteU8(dst, currentPidProfile->abs_control_gain);
#else
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_THROTTLE_BOOST)
        sbufWriteU8(dst, currentPidProfile->throttle_boost);
#else
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_ACRO_TRAINER)
        sbufWriteU8(dst, currentPidProfile->acro_trainer_angle_limit);
#else
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU16(dst, currentPidProfile->pid[PID_ROLL].F);
        sbufWriteU16(dst, currentPidProfile->pid[PID_PITCH].F);
        sbufWriteU16(dst, currentPidProfile->pid[PID_YAW].F);
        sbufWriteU8(dst, 0); // was currentPidProfile->antiGravityMode
#ifdef USE_D_MAX
        sbufWriteU8(dst, currentPidProfile->d_max[PID_ROLL]);
        sbufWriteU8(dst, currentPidProfile->d_max[PID_PITCH]);
        sbufWriteU8(dst, currentPidProfile->d_max[PID_YAW]);
        sbufWriteU8(dst, currentPidProfile->d_max_gain);
        sbufWriteU8(dst, currentPidProfile->d_max_advance);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_INTEGRATED_YAW_CONTROL)
        sbufWriteU8(dst, currentPidProfile->use_integrated_yaw);
        sbufWriteU8(dst, currentPidProfile->integrated_yaw_relax);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_ITERM_RELAX)
        // Added in MSP API 1.42
        sbufWriteU8(dst, currentPidProfile->iterm_relax_cutoff);
#else
        sbufWriteU8(dst, 0);
#endif
        // Added in MSP API 1.43
        sbufWriteU8(dst, currentPidProfile->motor_output_limit);
        sbufWriteU8(dst, currentPidProfile->auto_profile_cell_count);
#if defined(USE_DYN_IDLE)
        sbufWriteU8(dst, currentPidProfile->dyn_idle_min_rpm);
#else
        sbufWriteU8(dst, 0);
#endif
        // Added in MSP API 1.44
#if defined(USE_FEEDFORWARD)
        sbufWriteU8(dst, currentPidProfile->feedforward_averaging);
        sbufWriteU8(dst, currentPidProfile->feedforward_smooth_factor);
        sbufWriteU8(dst, currentPidProfile->feedforward_boost);
        sbufWriteU8(dst, currentPidProfile->feedforward_max_rate_limit);
        sbufWriteU8(dst, currentPidProfile->feedforward_jitter_factor);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
        sbufWriteU8(dst, currentPidProfile->vbat_sag_compensation);
#else
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_THRUST_LINEARIZATION)
        sbufWriteU8(dst, currentPidProfile->thrustLinearization);
#else
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, currentPidProfile->tpa_mode);
        sbufWriteU8(dst, currentPidProfile->tpa_rate);
        sbufWriteU16(dst, currentPidProfile->tpa_breakpoint);   // was currentControlRateProfile->tpa_breakpoint
        break;

    case MSP_SENSOR_CONFIG:
        // use sensorIndex_e index: 0:GyroHardware, 1:AccHardware, 2:BaroHardware, 3:MagHardware, 4:RangefinderHardware
#if defined(USE_ACC)
        sbufWriteU8(dst, accelerometerConfig()->acc_hardware);
#else
        sbufWriteU8(dst, ACC_NONE);
#endif
#ifdef USE_BARO
        sbufWriteU8(dst, barometerConfig()->baro_hardware);
#else
        sbufWriteU8(dst, BARO_NONE);
#endif
#ifdef USE_MAG
        sbufWriteU8(dst, compassConfig()->mag_hardware);
#else
        sbufWriteU8(dst, MAG_NONE);
#endif
        // Added in MSP API 1.46
#ifdef USE_RANGEFINDER
        sbufWriteU8(dst, rangefinderConfig()->rangefinder_hardware);    // no RANGEFINDER_DEFAULT value
#else
        sbufWriteU8(dst, RANGEFINDER_NONE);
#endif
        break;

    // Added in MSP API 1.46
    case MSP2_SENSOR_CONFIG_ACTIVE:

#define SENSOR_NOT_AVAILABLE 0xFF

#if defined(USE_GYRO)
        sbufWriteU8(dst, detectedSensors[SENSOR_INDEX_GYRO]);
#else
        sbufWriteU8(dst, SENSOR_NOT_AVAILABLE);
#endif
#if defined(USE_ACC)
        sbufWriteU8(dst, detectedSensors[SENSOR_INDEX_ACC]);
#else
        sbufWriteU8(dst, SENSOR_NOT_AVAILABLE);
#endif
#ifdef USE_BARO
        sbufWriteU8(dst, detectedSensors[SENSOR_INDEX_BARO]);
#else
        sbufWriteU8(dst, SENSOR_NOT_AVAILABLE);
#endif
#ifdef USE_MAG
        sbufWriteU8(dst, detectedSensors[SENSOR_INDEX_MAG]);
#else
        sbufWriteU8(dst, SENSOR_NOT_AVAILABLE);
#endif
#ifdef USE_RANGEFINDER
        sbufWriteU8(dst, detectedSensors[SENSOR_INDEX_RANGEFINDER]);
#else
        sbufWriteU8(dst, SENSOR_NOT_AVAILABLE);
#endif
        break;

#if defined(USE_VTX_COMMON)
    case MSP_VTX_CONFIG:
        {
            const vtxDevice_t *vtxDevice = vtxCommonDevice();
            unsigned vtxStatus = 0;
            vtxDevType_e vtxType = VTXDEV_UNKNOWN;
            uint8_t deviceIsReady = 0;
            if (vtxDevice) {
                vtxCommonGetStatus(vtxDevice, &vtxStatus);
                vtxType = vtxCommonGetDeviceType(vtxDevice);
                deviceIsReady = vtxCommonDeviceIsReady(vtxDevice) ? 1 : 0;
            }
            sbufWriteU8(dst, vtxType);
            sbufWriteU8(dst, vtxSettingsConfig()->band);
            sbufWriteU8(dst, vtxSettingsConfig()->channel);
            sbufWriteU8(dst, vtxSettingsConfig()->power);
            sbufWriteU8(dst, (vtxStatus & VTX_STATUS_PIT_MODE) ? 1 : 0);
            sbufWriteU16(dst, vtxSettingsConfig()->freq);
            sbufWriteU8(dst, deviceIsReady);
            sbufWriteU8(dst, vtxSettingsConfig()->lowPowerDisarm);

            // API version 1.42
            sbufWriteU16(dst, vtxSettingsConfig()->pitModeFreq);
#ifdef USE_VTX_TABLE
            sbufWriteU8(dst, 1);   // vtxtable is available
            sbufWriteU8(dst, vtxTableConfig()->bands);
            sbufWriteU8(dst, vtxTableConfig()->channels);
            sbufWriteU8(dst, vtxTableConfig()->powerLevels);
#else
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
#endif
#ifdef USE_VTX_MSP
            setMspVtxDeviceStatusReady(srcDesc);
#endif
        }
        break;
#endif

    case MSP_TX_INFO:
        sbufWriteU8(dst, rssiSource);
        uint8_t rtcDateTimeIsSet = 0;
#ifdef USE_RTC_TIME
        dateTime_t dt;
        if (rtcGetDateTime(&dt)) {
            rtcDateTimeIsSet = 1;
        }
#else
        rtcDateTimeIsSet = RTC_NOT_SUPPORTED;
#endif
        sbufWriteU8(dst, rtcDateTimeIsSet);

        break;
#ifdef USE_RTC_TIME
    case MSP_RTC:
        {
            dateTime_t dt;
            if (rtcGetDateTime(&dt)) {
                sbufWriteU16(dst, dt.year);
                sbufWriteU8(dst, dt.month);
                sbufWriteU8(dst, dt.day);
                sbufWriteU8(dst, dt.hours);
                sbufWriteU8(dst, dt.minutes);
                sbufWriteU8(dst, dt.seconds);
                sbufWriteU16(dst, dt.millis);
            }
        }

        break;
#endif
    default:
        unsupportedCommand = true;
    }
    return !unsupportedCommand;
}


#ifdef USE_SIMPLIFIED_TUNING
// Reads simplified PID tuning values from MSP buffer
static void readSimplifiedPids(pidProfile_t* pidProfile, sbuf_t *src)
{
    pidProfile->simplified_pids_mode = sbufReadU8(src);
    pidProfile->simplified_master_multiplier = sbufReadU8(src);
    pidProfile->simplified_roll_pitch_ratio = sbufReadU8(src);
    pidProfile->simplified_i_gain = sbufReadU8(src);
    pidProfile->simplified_d_gain = sbufReadU8(src);
    pidProfile->simplified_pi_gain = sbufReadU8(src);
#ifdef USE_D_MAX
    pidProfile->simplified_d_max_gain = sbufReadU8(src);
#else
    sbufReadU8(src);
#endif
    pidProfile->simplified_feedforward_gain = sbufReadU8(src);
    pidProfile->simplified_pitch_pi_gain = sbufReadU8(src);
    sbufReadU32(src); // reserved for future use
    sbufReadU32(src); // reserved for future use
}

// Writes simplified PID tuning values to MSP buffer
static void writeSimplifiedPids(const pidProfile_t *pidProfile, sbuf_t *dst)
{
    sbufWriteU8(dst, pidProfile->simplified_pids_mode);
    sbufWriteU8(dst, pidProfile->simplified_master_multiplier);
    sbufWriteU8(dst, pidProfile->simplified_roll_pitch_ratio);
    sbufWriteU8(dst, pidProfile->simplified_i_gain);
    sbufWriteU8(dst, pidProfile->simplified_d_gain);
    sbufWriteU8(dst, pidProfile->simplified_pi_gain);
#ifdef USE_D_MAX
    sbufWriteU8(dst, pidProfile->simplified_d_max_gain);
#else
    sbufWriteU8(dst, 0);
#endif
    sbufWriteU8(dst, pidProfile->simplified_feedforward_gain);
    sbufWriteU8(dst, pidProfile->simplified_pitch_pi_gain);
    sbufWriteU32(dst, 0); // reserved for future use
    sbufWriteU32(dst, 0); // reserved for future use
}

// Reads simplified Dterm Filter values from MSP buffer
static void readSimplifiedDtermFilters(pidProfile_t* pidProfile, sbuf_t *src)
{
    pidProfile->simplified_dterm_filter = sbufReadU8(src);
    pidProfile->simplified_dterm_filter_multiplier = sbufReadU8(src);
    pidProfile->dterm_lpf1_static_hz = sbufReadU16(src);
    pidProfile->dterm_lpf2_static_hz = sbufReadU16(src);
#if defined(USE_DYN_LPF)
    pidProfile->dterm_lpf1_dyn_min_hz = sbufReadU16(src);
    pidProfile->dterm_lpf1_dyn_max_hz = sbufReadU16(src);
#else
    sbufReadU16(src);
    sbufReadU16(src);
#endif
    sbufReadU32(src); // reserved for future use
    sbufReadU32(src); // reserved for future use
}

// Writes simplified Dterm Filter values into MSP buffer
static void writeSimplifiedDtermFilters(const pidProfile_t* pidProfile, sbuf_t *dst)
{
    sbufWriteU8(dst, pidProfile->simplified_dterm_filter);
    sbufWriteU8(dst, pidProfile->simplified_dterm_filter_multiplier);
    sbufWriteU16(dst, pidProfile->dterm_lpf1_static_hz);
    sbufWriteU16(dst, pidProfile->dterm_lpf2_static_hz);
#if defined(USE_DYN_LPF)
    sbufWriteU16(dst, pidProfile->dterm_lpf1_dyn_min_hz);
    sbufWriteU16(dst, pidProfile->dterm_lpf1_dyn_max_hz);
#else
    sbufWriteU16(dst, 0);
    sbufWriteU16(dst, 0);
#endif
    sbufWriteU32(dst, 0); // reserved for future use
    sbufWriteU32(dst, 0); // reserved for future use
}

// Writes simplified Gyro Filter values from MSP buffer
static void readSimplifiedGyroFilters(gyroConfig_t *gyroConfig, sbuf_t *src)
{
    gyroConfig->simplified_gyro_filter = sbufReadU8(src);
    gyroConfig->simplified_gyro_filter_multiplier = sbufReadU8(src);
    gyroConfig->gyro_lpf1_static_hz = sbufReadU16(src);
    gyroConfig->gyro_lpf2_static_hz = sbufReadU16(src);
#if defined(USE_DYN_LPF)
    gyroConfig->gyro_lpf1_dyn_min_hz = sbufReadU16(src);
    gyroConfig->gyro_lpf1_dyn_max_hz = sbufReadU16(src);
#else
    sbufReadU16(src);
    sbufReadU16(src);
#endif
    sbufReadU32(src); // reserved for future use
    sbufReadU32(src); // reserved for future use
}

// Writes simplified Gyro Filter values into MSP buffer
static void writeSimplifiedGyroFilters(const gyroConfig_t *gyroConfig, sbuf_t *dst)
{
    sbufWriteU8(dst, gyroConfig->simplified_gyro_filter);
    sbufWriteU8(dst, gyroConfig->simplified_gyro_filter_multiplier);
    sbufWriteU16(dst, gyroConfig->gyro_lpf1_static_hz);
    sbufWriteU16(dst, gyroConfig->gyro_lpf2_static_hz);
#if defined(USE_DYN_LPF)
    sbufWriteU16(dst, gyroConfig->gyro_lpf1_dyn_min_hz);
    sbufWriteU16(dst, gyroConfig->gyro_lpf1_dyn_max_hz);
#else
    sbufWriteU16(dst, 0);
    sbufWriteU16(dst, 0);
#endif
    sbufWriteU32(dst, 0); // reserved for future use
    sbufWriteU32(dst, 0); // reserved for future use
}

// writes results of simplified PID tuning values to MSP buffer
static void writePidfs(pidProfile_t* pidProfile, sbuf_t *dst)
{
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        sbufWriteU8(dst, pidProfile->pid[i].P);
        sbufWriteU8(dst, pidProfile->pid[i].I);
        sbufWriteU8(dst, pidProfile->pid[i].D);
        sbufWriteU8(dst, pidProfile->d_max[i]);
        sbufWriteU16(dst, pidProfile->pid[i].F);
    }
}
#endif // USE_SIMPLIFIED_TUNING

static mspResult_e mspFcProcessOutCommandWithArg(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{

    switch (cmdMSP) {
    case MSP_BOXNAMES:
        {
            const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            serializeBoxReply(dst, page, &serializeBoxNameFn);
        }
        break;
    case MSP_BOXIDS:
        {
            const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            serializeBoxReply(dst, page, &serializeBoxPermanentIdFn);
        }
        break;
    case MSP_REBOOT:
        if (sbufBytesRemaining(src)) {
            rebootMode = sbufReadU8(src);

            if (rebootMode >= MSP_REBOOT_COUNT
#if !defined(USE_USB_MSC)
                || rebootMode == MSP_REBOOT_MSC || rebootMode == MSP_REBOOT_MSC_UTC
#endif
                ) {
                return MSP_RESULT_ERROR;
            }
        } else {
            rebootMode = MSP_REBOOT_FIRMWARE;
        }

        sbufWriteU8(dst, rebootMode);

#if defined(USE_USB_MSC)
        if (rebootMode == MSP_REBOOT_MSC) {
            if (mscCheckFilesystemReady()) {
                sbufWriteU8(dst, 1);
            } else {
                sbufWriteU8(dst, 0);

                return MSP_RESULT_ACK;
            }
        }
#endif

#if defined(USE_MSP_OVER_TELEMETRY)
        if (featureIsEnabled(FEATURE_RX_SPI) && srcDesc == getMspTelemetryDescriptor()) {
            dispatchAdd(&mspRebootEntry, MSP_DISPATCH_DELAY_US);
        } else
#endif
        if (mspPostProcessFn) {
            *mspPostProcessFn = mspRebootFn;
        }

        break;
    case MSP_MULTIPLE_MSP:
        {
            uint8_t maxMSPs = 0;
            if (sbufBytesRemaining(src) == 0) {
                return MSP_RESULT_ERROR;
            }
            int bytesRemaining = sbufBytesRemaining(dst);
            mspPacket_t packetIn, packetOut;
            sbufInit(&packetIn.buf, src->end, src->end); // there is no paramater for MSP_MULTIPLE_MSP
            uint8_t* initialInputPtr = src->ptr;
            while (sbufBytesRemaining(src) && bytesRemaining > 0) {
                uint8_t newMSP = sbufReadU8(src);
                sbufInit(&packetOut.buf, dst->ptr + 1, dst->end); // reserve 1 byte for length
                packetIn.cmd = newMSP;
                mspFcProcessCommand(srcDesc, &packetIn, &packetOut, NULL);
                uint8_t mspSize = sbufPtr(&packetOut.buf) - dst->ptr; // length included
                bytesRemaining -= mspSize;
                if (bytesRemaining >= 0) {
                    maxMSPs++;
                }
            }
            src->ptr = initialInputPtr;
            sbufInit(&packetOut.buf, dst->ptr, dst->end);
            for (int i = 0; i < maxMSPs; i++) {
                uint8_t* sizePtr = sbufPtr(&packetOut.buf);
                sbufWriteU8(&packetOut.buf, 0); // placeholder for reply size
                packetIn.cmd = sbufReadU8(src);
                mspFcProcessCommand(srcDesc, &packetIn, &packetOut, NULL);
                *sizePtr = sbufPtr(&packetOut.buf) - (sizePtr + 1);
            }
            dst->ptr = packetOut.buf.ptr;
        }
        break;

#ifdef USE_VTX_TABLE
    case MSP_VTXTABLE_BAND:
        {
            const uint8_t band = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            if (band > 0 && band <= VTX_TABLE_MAX_BANDS) {
                sbufWriteU8(dst, band);  // band number (same as request)
                sbufWriteU8(dst, VTX_TABLE_BAND_NAME_LENGTH); // band name length
                for (int i = 0; i < VTX_TABLE_BAND_NAME_LENGTH; i++) { // band name bytes
                    sbufWriteU8(dst, vtxTableConfig()->bandNames[band - 1][i]);
                }
                sbufWriteU8(dst, vtxTableConfig()->bandLetters[band - 1]); // band letter
                sbufWriteU8(dst, vtxTableConfig()->isFactoryBand[band - 1]); // CUSTOM = 0; FACTORY = 1
                sbufWriteU8(dst, vtxTableConfig()->channels); // number of channel frequencies to follow
                for (int i = 0; i < vtxTableConfig()->channels; i++) { // the frequency for each channel
                    sbufWriteU16(dst, vtxTableConfig()->frequency[band - 1][i]);
                }
            } else {
                return MSP_RESULT_ERROR;
            }
#ifdef USE_VTX_MSP
            setMspVtxDeviceStatusReady(srcDesc);
#endif
        }
        break;

    case MSP_VTXTABLE_POWERLEVEL:
        {
            const uint8_t powerLevel = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            if (powerLevel > 0 && powerLevel <= VTX_TABLE_MAX_POWER_LEVELS) {
                sbufWriteU8(dst, powerLevel);  // powerLevel number (same as request)
                sbufWriteU16(dst, vtxTableConfig()->powerValues[powerLevel - 1]);
                sbufWriteU8(dst, VTX_TABLE_POWER_LABEL_LENGTH); // powerLevel label length
                for (int i = 0; i < VTX_TABLE_POWER_LABEL_LENGTH; i++) { // powerlevel label bytes
                    sbufWriteU8(dst, vtxTableConfig()->powerLabels[powerLevel - 1][i]);
                }
            } else {
                return MSP_RESULT_ERROR;
            }
#ifdef USE_VTX_MSP
            setMspVtxDeviceStatusReady(srcDesc);
#endif
        }
        break;
#endif // USE_VTX_TABLE

#ifdef USE_SIMPLIFIED_TUNING
    // Added in MSP API 1.44
    case MSP_SIMPLIFIED_TUNING:
        {
            writeSimplifiedPids(currentPidProfile, dst);
            writeSimplifiedDtermFilters(currentPidProfile, dst);
            writeSimplifiedGyroFilters(gyroConfig(), dst);
        }
        break;

    case MSP_CALCULATE_SIMPLIFIED_PID:
        {
            pidProfile_t tempPidProfile = *currentPidProfile;
            readSimplifiedPids(&tempPidProfile, src);
            applySimplifiedTuningPids(&tempPidProfile);
            writePidfs(&tempPidProfile, dst);
        }
        break;

    case MSP_CALCULATE_SIMPLIFIED_DTERM:
        {
            pidProfile_t tempPidProfile = *currentPidProfile;
            readSimplifiedDtermFilters(&tempPidProfile, src);
            applySimplifiedTuningDtermFilters(&tempPidProfile);
            writeSimplifiedDtermFilters(&tempPidProfile, dst);
        }
        break;

    case MSP_CALCULATE_SIMPLIFIED_GYRO:
        {
            gyroConfig_t tempGyroConfig = *gyroConfig();
            readSimplifiedGyroFilters(&tempGyroConfig, src);
            applySimplifiedTuningGyroFilters(&tempGyroConfig);
            writeSimplifiedGyroFilters(&tempGyroConfig, dst);
        }
        break;

    case MSP_VALIDATE_SIMPLIFIED_TUNING:
        {
            pidProfile_t tempPidProfile = *currentPidProfile;
            applySimplifiedTuningPids(&tempPidProfile);
            bool result = true;

            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                result = result &&
                    tempPidProfile.pid[i].P == currentPidProfile->pid[i].P &&
                    tempPidProfile.pid[i].I == currentPidProfile->pid[i].I &&
                    tempPidProfile.pid[i].D == currentPidProfile->pid[i].D &&
                    tempPidProfile.d_max[i] == currentPidProfile->d_max[i] &&
                    tempPidProfile.pid[i].F == currentPidProfile->pid[i].F;
            }

            sbufWriteU8(dst, result);

            gyroConfig_t tempGyroConfig = *gyroConfig();
            applySimplifiedTuningGyroFilters(&tempGyroConfig);
            result =
                tempGyroConfig.gyro_lpf1_static_hz == gyroConfig()->gyro_lpf1_static_hz &&
                tempGyroConfig.gyro_lpf2_static_hz == gyroConfig()->gyro_lpf2_static_hz;

#if defined(USE_DYN_LPF)
            result = result &&
                tempGyroConfig.gyro_lpf1_dyn_min_hz == gyroConfig()->gyro_lpf1_dyn_min_hz &&
                tempGyroConfig.gyro_lpf1_dyn_max_hz == gyroConfig()->gyro_lpf1_dyn_max_hz;
#endif

            sbufWriteU8(dst, result);

            applySimplifiedTuningDtermFilters(&tempPidProfile);
            result =
                tempPidProfile.dterm_lpf1_static_hz == currentPidProfile->dterm_lpf1_static_hz &&
                tempPidProfile.dterm_lpf2_static_hz == currentPidProfile->dterm_lpf2_static_hz;

#if defined(USE_DYN_LPF)
            result = result &&
                tempPidProfile.dterm_lpf1_dyn_min_hz == currentPidProfile->dterm_lpf1_dyn_min_hz &&
                tempPidProfile.dterm_lpf1_dyn_max_hz == currentPidProfile->dterm_lpf1_dyn_max_hz;
#endif

            sbufWriteU8(dst, result);
        }
        break;
#endif

    case MSP_RESET_CONF:
        {
            if (sbufBytesRemaining(src) >= 1) {
                // Added in MSP API 1.42
                sbufReadU8(src);
            }

            bool success = false;
            if (!ARMING_FLAG(ARMED)) {
                success = resetEEPROM();

                if (success && mspPostProcessFn) {
                    rebootMode = MSP_REBOOT_FIRMWARE;
                    *mspPostProcessFn = mspRebootFn;
                }
            }

            // Added in API version 1.42
            sbufWriteU8(dst, success);
        }

        break;

    case MSP2_GET_TEXT:
        {
            // type byte, then length byte followed by the actual characters
            const uint8_t textType = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;

            const char *textVar;

            switch (textType) {
                case MSP2TEXT_PILOT_NAME:
                    textVar = pilotConfigMutable()->pilotName;
                    break;

                case MSP2TEXT_CRAFT_NAME:
                    textVar = pilotConfigMutable()->craftName;
                    break;

                case MSP2TEXT_PID_PROFILE_NAME:
                    textVar = currentPidProfile->profileName;
                    break;

                case MSP2TEXT_RATE_PROFILE_NAME:
                    textVar = currentControlRateProfile->profileName;
                    break;

                case MSP2TEXT_BUILDKEY:
                    textVar = buildKey;
                    break;

                case MSP2TEXT_RELEASENAME:
                    textVar = releaseName;
                    break;

                default:
                    return MSP_RESULT_ERROR;
            }

            if (!textVar) return MSP_RESULT_ERROR;

            const uint8_t textLength = strlen(textVar);

            //  type byte, then length byte followed by the actual characters
            sbufWriteU8(dst, textType);
            sbufWriteU8(dst, textLength);
            for (unsigned int i = 0; i < textLength; i++) {
                sbufWriteU8(dst, textVar[i]);
            }
        }
        break;
#ifdef USE_LED_STRIP
    case MSP2_GET_LED_STRIP_CONFIG_VALUES:
        sbufWriteU8(dst, ledStripConfig()->ledstrip_brightness);
        sbufWriteU16(dst, ledStripConfig()->ledstrip_rainbow_delta);
        sbufWriteU16(dst, ledStripConfig()->ledstrip_rainbow_freq);
        break;
#endif

    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
    return MSP_RESULT_ACK;
}

#ifdef USE_FLASHFS
static void mspFcDataFlashReadCommand(sbuf_t *dst, sbuf_t *src)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    const uint32_t readAddress = sbufReadU32(src);
    uint16_t readLength;
    bool allowCompression = false;
    bool useLegacyFormat;
    if (dataSize >= sizeof(uint32_t) + sizeof(uint16_t)) {
        readLength = sbufReadU16(src);
        if (sbufBytesRemaining(src)) {
            allowCompression = sbufReadU8(src);
        }
        useLegacyFormat = false;
    } else {
        readLength = 128;
        useLegacyFormat = true;
    }

    serializeDataflashReadReply(dst, readAddress, readLength, useLegacyFormat, allowCompression);
}
#endif

static mspResult_e mspProcessInCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src)
{
    uint32_t i;
    uint8_t value;
    const unsigned int dataSize = sbufBytesRemaining(src);
    switch (cmdMSP) {
    case MSP_SELECT_SETTING:
        value = sbufReadU8(src);
        if ((value & RATEPROFILE_MASK) == 0) {
            if (!ARMING_FLAG(ARMED)) {
                if (value >= PID_PROFILE_COUNT) {
                    value = 0;
                }
                changePidProfile(value);
            }
        } else {
            value = value & ~RATEPROFILE_MASK;

            if (value >= CONTROL_RATE_PROFILE_COUNT) {
                value = 0;
            }
            changeControlRateProfile(value);
        }
        break;

    case MSP_COPY_PROFILE:
        value = sbufReadU8(src);        // 0 = pid profile, 1 = control rate profile
        uint8_t dstProfileIndex = sbufReadU8(src);
        uint8_t srcProfileIndex = sbufReadU8(src);
        if (value == 0) {
            pidCopyProfile(dstProfileIndex, srcProfileIndex);
        }
        else if (value == 1) {
            copyControlRateProfile(dstProfileIndex, srcProfileIndex);
        }
        break;

#if defined(USE_GPS) || defined(USE_MAG)
    case MSP_SET_HEADING:
        magHold = sbufReadU16(src);
        break;
#endif

    case MSP_SET_RAW_RC:
#ifdef USE_RX_MSP
        {
            uint8_t channelCount = dataSize / sizeof(uint16_t);
            if (channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT) {
                return MSP_RESULT_ERROR;
            } else {
                uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
                for (int i = 0; i < channelCount; i++) {
                    frame[i] = sbufReadU16(src);
                }
                rxMspFrameReceive(frame, channelCount);
            }
        }
#endif
        break;
#if defined(USE_ACC)
    case MSP_SET_ACC_TRIM:
        accelerometerConfigMutable()->accelerometerTrims.values.pitch = sbufReadU16(src);
        accelerometerConfigMutable()->accelerometerTrims.values.roll  = sbufReadU16(src);

        break;
#endif
    case MSP_SET_ARMING_CONFIG:
        armingConfigMutable()->auto_disarm_delay = sbufReadU8(src);
        sbufReadU8(src); // reserved. disarm_kill_switch was removed in #5073
        if (sbufBytesRemaining(src)) {
          imuConfigMutable()->small_angle = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src)) {
            armingConfigMutable()->gyro_cal_on_first_arm = sbufReadU8(src);
        }
        break;

    case MSP_SET_PID_CONTROLLER:
        break;

    case MSP_SET_PID:
        for (int i = 0; i < PID_ITEM_COUNT; i++) {
            currentPidProfile->pid[i].P = sbufReadU8(src);
            currentPidProfile->pid[i].I = sbufReadU8(src);
            currentPidProfile->pid[i].D = sbufReadU8(src);
        }
        pidInitConfig(currentPidProfile);
        break;

    case MSP_SET_MODE_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            i = sbufReadU8(src);
            const box_t *box = findBoxByPermanentId(i);
            if (box) {
                mac->modeId = box->boxId;
                mac->auxChannelIndex = sbufReadU8(src);
                mac->range.startStep = sbufReadU8(src);
                mac->range.endStep = sbufReadU8(src);
                if (sbufBytesRemaining(src) != 0) {
                    mac->modeLogic = sbufReadU8(src);

                    i = sbufReadU8(src);
                    mac->linkedTo = findBoxByPermanentId(i)->boxId;
                }
                rcControlsInit();
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_ADJUSTMENT_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *adjRange = adjustmentRangesMutable(i);
            sbufReadU8(src); // was adjRange->adjustmentIndex
            adjRange->auxChannelIndex = sbufReadU8(src);
            adjRange->range.startStep = sbufReadU8(src);
            adjRange->range.endStep = sbufReadU8(src);
            adjRange->adjustmentConfig = sbufReadU8(src);
            adjRange->auxSwitchChannelIndex = sbufReadU8(src);

            activeAdjustmentRangeReset();
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RC_TUNING:
        if (sbufBytesRemaining(src) >= 10) {
            value = sbufReadU8(src);
            if (currentControlRateProfile->rcRates[FD_PITCH] == currentControlRateProfile->rcRates[FD_ROLL]) {
                currentControlRateProfile->rcRates[FD_PITCH] = value;
            }
            currentControlRateProfile->rcRates[FD_ROLL] = value;

            value = sbufReadU8(src);
            if (currentControlRateProfile->rcExpo[FD_PITCH] == currentControlRateProfile->rcExpo[FD_ROLL]) {
                currentControlRateProfile->rcExpo[FD_PITCH] = value;
            }
            currentControlRateProfile->rcExpo[FD_ROLL] = value;

            for (int i = 0; i < 3; i++) {
                currentControlRateProfile->rates[i] = sbufReadU8(src);
            }

            sbufReadU8(src);    // tpa_rate is moved to PID profile
            currentControlRateProfile->thrMid8 = sbufReadU8(src);
            currentControlRateProfile->thrExpo8 = sbufReadU8(src);
            sbufReadU16(src);   // tpa_breakpoint is moved to PID profile

            if (sbufBytesRemaining(src) >= 1) {
                currentControlRateProfile->rcExpo[FD_YAW] = sbufReadU8(src);
            }

            if (sbufBytesRemaining(src) >= 1) {
                currentControlRateProfile->rcRates[FD_YAW] = sbufReadU8(src);
            }

            if (sbufBytesRemaining(src) >= 1) {
                currentControlRateProfile->rcRates[FD_PITCH] = sbufReadU8(src);
            }

            if (sbufBytesRemaining(src) >= 1) {
                currentControlRateProfile->rcExpo[FD_PITCH] = sbufReadU8(src);
            }

            // version 1.41
            if (sbufBytesRemaining(src) >= 2) {
                currentControlRateProfile->throttle_limit_type = sbufReadU8(src);
                currentControlRateProfile->throttle_limit_percent = sbufReadU8(src);
            }

            // version 1.42
            if (sbufBytesRemaining(src) >= 6) {
                currentControlRateProfile->rate_limit[FD_ROLL] = sbufReadU16(src);
                currentControlRateProfile->rate_limit[FD_PITCH] = sbufReadU16(src);
                currentControlRateProfile->rate_limit[FD_YAW] = sbufReadU16(src);
            }

            // version 1.43
            if (sbufBytesRemaining(src) >= 1) {
                currentControlRateProfile->rates_type = sbufReadU8(src);
            }

            initRcProcessing();
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_MOTOR_CONFIG:
        sbufReadU16(src);   // minthrottle deprecated in 4.6
        motorConfigMutable()->maxthrottle = sbufReadU16(src);
        motorConfigMutable()->mincommand = sbufReadU16(src);

        // version 1.42
        if (sbufBytesRemaining(src) >= 2) {
            motorConfigMutable()->motorPoleCount = sbufReadU8(src);
#if defined(USE_DSHOT_TELEMETRY)
            motorConfigMutable()->dev.useDshotTelemetry = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
        }
        break;

#ifdef USE_GPS
    case MSP_SET_GPS_CONFIG:
        gpsConfigMutable()->provider = sbufReadU8(src);
        gpsConfigMutable()->sbasMode = sbufReadU8(src);
        gpsConfigMutable()->autoConfig = sbufReadU8(src);
        gpsConfigMutable()->autoBaud = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 2) {
            // Added in API version 1.43
            gpsConfigMutable()->gps_set_home_point_once = sbufReadU8(src);
            gpsConfigMutable()->gps_ublox_use_galileo = sbufReadU8(src);
        }
        break;
#endif

#ifdef USE_MAG
    case MSP_SET_COMPASS_CONFIG:
        imuConfigMutable()->mag_declination = sbufReadU16(src);
        break;
#endif

#ifdef USE_GPS
#ifdef USE_GPS_RESCUE
    case MSP_SET_GPS_RESCUE:
        gpsRescueConfigMutable()->maxRescueAngle = sbufReadU16(src);
        gpsRescueConfigMutable()->returnAltitudeM = sbufReadU16(src);
        gpsRescueConfigMutable()->descentDistanceM = sbufReadU16(src);
        gpsRescueConfigMutable()->groundSpeedCmS = sbufReadU16(src);
        autopilotConfigMutable()->throttle_min = sbufReadU16(src);
        autopilotConfigMutable()->throttle_max = sbufReadU16(src);
        autopilotConfigMutable()->hover_throttle = sbufReadU16(src);
        gpsRescueConfigMutable()->sanityChecks = sbufReadU8(src);
        gpsRescueConfigMutable()->minSats = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 6) {
            // Added in API version 1.43
            gpsRescueConfigMutable()->ascendRate = sbufReadU16(src);
            gpsRescueConfigMutable()->descendRate = sbufReadU16(src);
            gpsRescueConfigMutable()->allowArmingWithoutFix = sbufReadU8(src);
            gpsRescueConfigMutable()->altitudeMode = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 2) {
            // Added in API version 1.44
            gpsRescueConfigMutable()->minStartDistM = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 2) {
            // Added in API version 1.46
            gpsRescueConfigMutable()->initialClimbM = sbufReadU16(src);
        }
        break;

    case MSP_SET_GPS_RESCUE_PIDS:
        autopilotConfigMutable()->altitude_P = sbufReadU16(src);
        autopilotConfigMutable()->altitude_I = sbufReadU16(src);
        autopilotConfigMutable()->altitude_D = sbufReadU16(src);
        // altitude_F not included in msp yet
        gpsRescueConfigMutable()->velP = sbufReadU16(src);
        gpsRescueConfigMutable()->velI = sbufReadU16(src);
        gpsRescueConfigMutable()->velD = sbufReadU16(src);
        gpsRescueConfigMutable()->yawP = sbufReadU16(src);
        break;
#endif
#endif

    case MSP_SET_MOTOR:
        for (int i = 0; i < getMotorCount(); i++) {
            motor_disarmed[i] = motorConvertFromExternal(sbufReadU16(src));
        }
        break;

    case MSP_SET_SERVO_CONFIGURATION:
#ifdef USE_SERVOS
        if (dataSize != 1 + 12) {
            return MSP_RESULT_ERROR;
        }
        i = sbufReadU8(src);
        if (i >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        } else {
            servoParamsMutable(i)->min = sbufReadU16(src);
            servoParamsMutable(i)->max = sbufReadU16(src);
            servoParamsMutable(i)->middle = sbufReadU16(src);
            servoParamsMutable(i)->rate = sbufReadU8(src);
            servoParamsMutable(i)->forwardFromChannel = sbufReadU8(src);
            servoParamsMutable(i)->reversedSources = sbufReadU32(src);
        }
#endif
        break;

    case MSP_SET_SERVO_MIX_RULE:
#ifdef USE_SERVOS
        i = sbufReadU8(src);
        if (i >= MAX_SERVO_RULES) {
            return MSP_RESULT_ERROR;
        } else {
            customServoMixersMutable(i)->targetChannel = sbufReadU8(src);
            customServoMixersMutable(i)->inputSource = sbufReadU8(src);
            customServoMixersMutable(i)->rate = sbufReadU8(src);
            customServoMixersMutable(i)->speed = sbufReadU8(src);
            customServoMixersMutable(i)->min = sbufReadU8(src);
            customServoMixersMutable(i)->max = sbufReadU8(src);
            customServoMixersMutable(i)->box = sbufReadU8(src);
            loadCustomServoMixer();
        }
#endif
        break;

    case MSP_SET_MOTOR_3D_CONFIG:
        flight3DConfigMutable()->deadband3d_low = sbufReadU16(src);
        flight3DConfigMutable()->deadband3d_high = sbufReadU16(src);
        flight3DConfigMutable()->neutral3d = sbufReadU16(src);
        break;

    case MSP_SET_RC_DEADBAND:
        rcControlsConfigMutable()->deadband = sbufReadU8(src);
        rcControlsConfigMutable()->yaw_deadband = sbufReadU8(src);
        rcControlsConfigMutable()->alt_hold_deadband = sbufReadU8(src);
        flight3DConfigMutable()->deadband3d_throttle = sbufReadU16(src);
        break;

    case MSP_SET_RESET_CURR_PID:
        resetPidProfile(currentPidProfile);
        break;

    case MSP_SET_SENSOR_ALIGNMENT: {
        // maintain backwards compatibility for API < 1.41
        const uint8_t gyroAlignment = sbufReadU8(src);
        sbufReadU8(src);  // discard deprecated acc_align
#if defined(USE_MAG)
        compassConfigMutable()->mag_alignment = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif

        if (sbufBytesRemaining(src) >= 3) {
            // API >= 1.41 - support the gyro_to_use and alignment for gyros 1 & 2
#ifdef USE_MULTI_GYRO
            gyroConfigMutable()->gyro_to_use = sbufReadU8(src);
            gyroDeviceConfigMutable(0)->alignment = sbufReadU8(src);
            gyroDeviceConfigMutable(1)->alignment = sbufReadU8(src);
#else
            sbufReadU8(src);  // unused gyro_to_use
            gyroDeviceConfigMutable(0)->alignment = sbufReadU8(src);
            sbufReadU8(src);  // unused gyro_2_sensor_align
#endif
        } else {
            // maintain backwards compatibility for API < 1.41
#ifdef USE_MULTI_GYRO
            switch (gyroConfig()->gyro_to_use) {
            case GYRO_CONFIG_USE_GYRO_2:
                gyroDeviceConfigMutable(1)->alignment = gyroAlignment;
                break;
            case GYRO_CONFIG_USE_GYRO_BOTH:
                // For dual-gyro in "BOTH" mode we'll only update gyro 0
            default:
                gyroDeviceConfigMutable(0)->alignment = gyroAlignment;
                break;
            }
#else
            gyroDeviceConfigMutable(0)->alignment = gyroAlignment;
#endif

        }
        break;
    }

    case MSP_SET_ADVANCED_CONFIG:
        sbufReadU8(src); // was gyroConfigMutable()->gyro_sync_denom - removed in API 1.43
        pidConfigMutable()->pid_process_denom = sbufReadU8(src);
        motorConfigMutable()->dev.useUnsyncedPwm = sbufReadU8(src);
        motorConfigMutable()->dev.motorPwmProtocol = sbufReadU8(src);
        motorConfigMutable()->dev.motorPwmRate = sbufReadU16(src);
        if (sbufBytesRemaining(src) >= 2) {
            motorConfigMutable()->motorIdle = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src)) {
            sbufReadU8(src); // DEPRECATED: gyro_use_32khz
        }
        if (sbufBytesRemaining(src)) {
            motorConfigMutable()->dev.motorPwmInversion = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 8) {
            gyroConfigMutable()->gyro_to_use = sbufReadU8(src);
            gyroConfigMutable()->gyro_high_fsr = sbufReadU8(src);
            gyroConfigMutable()->gyroMovementCalibrationThreshold = sbufReadU8(src);
            gyroConfigMutable()->gyroCalibrationDuration = sbufReadU16(src);
            gyroConfigMutable()->gyro_offset_yaw = sbufReadU16(src);
            gyroConfigMutable()->checkOverflow = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 1) {
            //Added in MSP API 1.42
            systemConfigMutable()->debug_mode = sbufReadU8(src);
        }

        validateAndFixGyroConfig();

        break;
    case MSP_SET_FILTER_CONFIG:
        gyroConfigMutable()->gyro_lpf1_static_hz = sbufReadU8(src);
        currentPidProfile->dterm_lpf1_static_hz = sbufReadU16(src);
        currentPidProfile->yaw_lowpass_hz = sbufReadU16(src);
        if (sbufBytesRemaining(src) >= 8) {
            gyroConfigMutable()->gyro_soft_notch_hz_1 = sbufReadU16(src);
            gyroConfigMutable()->gyro_soft_notch_cutoff_1 = sbufReadU16(src);
            currentPidProfile->dterm_notch_hz = sbufReadU16(src);
            currentPidProfile->dterm_notch_cutoff = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 4) {
            gyroConfigMutable()->gyro_soft_notch_hz_2 = sbufReadU16(src);
            gyroConfigMutable()->gyro_soft_notch_cutoff_2 = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 1) {
            currentPidProfile->dterm_lpf1_type = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 10) {
            gyroConfigMutable()->gyro_hardware_lpf = sbufReadU8(src);
            sbufReadU8(src); // DEPRECATED: gyro_32khz_hardware_lpf
            gyroConfigMutable()->gyro_lpf1_static_hz = sbufReadU16(src);
            gyroConfigMutable()->gyro_lpf2_static_hz = sbufReadU16(src);
            gyroConfigMutable()->gyro_lpf1_type = sbufReadU8(src);
            gyroConfigMutable()->gyro_lpf2_type = sbufReadU8(src);
            currentPidProfile->dterm_lpf2_static_hz = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 9) {
            // Added in MSP API 1.41
            currentPidProfile->dterm_lpf2_type = sbufReadU8(src);
#if defined(USE_DYN_LPF)
            gyroConfigMutable()->gyro_lpf1_dyn_min_hz = sbufReadU16(src);
            gyroConfigMutable()->gyro_lpf1_dyn_max_hz = sbufReadU16(src);
            currentPidProfile->dterm_lpf1_dyn_min_hz = sbufReadU16(src);
            currentPidProfile->dterm_lpf1_dyn_max_hz = sbufReadU16(src);
#else
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU16(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 8) {
            // Added in MSP API 1.42
#if defined(USE_DYN_NOTCH_FILTER)
            sbufReadU8(src); // DEPRECATED 1.43: dyn_notch_range
            sbufReadU8(src); // DEPRECATED 1.44: dyn_notch_width_percent
            dynNotchConfigMutable()->dyn_notch_q = sbufReadU16(src);
            dynNotchConfigMutable()->dyn_notch_min_hz = sbufReadU16(src);
#else
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU16(src);
            sbufReadU16(src);
#endif
#if defined(USE_RPM_FILTER)
            rpmFilterConfigMutable()->rpm_filter_harmonics = sbufReadU8(src);
            rpmFilterConfigMutable()->rpm_filter_min_hz = sbufReadU8(src);
#else
            sbufReadU8(src);
            sbufReadU8(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 2) {
#if defined(USE_DYN_NOTCH_FILTER)
            // Added in MSP API 1.43
            dynNotchConfigMutable()->dyn_notch_max_hz = sbufReadU16(src);
#else
            sbufReadU16(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 2) {
            // Added in MSP API 1.44
#if defined(USE_DYN_LPF)
            currentPidProfile->dterm_lpf1_dyn_expo = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
#if defined(USE_DYN_NOTCH_FILTER)
            dynNotchConfigMutable()->dyn_notch_count = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
        }

        // reinitialize the gyro filters with the new values
        validateAndFixGyroConfig();
        gyroInitFilters();
        // reinitialize the PID filters with the new values
        pidInitFilters(currentPidProfile);

        break;
    case MSP_SET_PID_ADVANCED:
        sbufReadU16(src);
        sbufReadU16(src);
        sbufReadU16(src); // was pidProfile.yaw_p_limit
        sbufReadU8(src); // reserved
        sbufReadU8(src); // was vbatPidCompensation
#if defined(USE_FEEDFORWARD)
        currentPidProfile->feedforward_transition = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
        sbufReadU8(src); // was low byte of currentPidProfile->dtermSetpointWeight
        sbufReadU8(src); // reserved
        sbufReadU8(src); // reserved
        sbufReadU8(src); // reserved
        currentPidProfile->rateAccelLimit = sbufReadU16(src);
        currentPidProfile->yawRateAccelLimit = sbufReadU16(src);
        if (sbufBytesRemaining(src) >= 2) {
            currentPidProfile->angle_limit = sbufReadU8(src);
            sbufReadU8(src); // was pidProfile.levelSensitivity
        }
        if (sbufBytesRemaining(src) >= 4) {
            sbufReadU16(src); // was currentPidProfile->itermThrottleThreshold
            currentPidProfile->anti_gravity_gain = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 2) {
            sbufReadU16(src); // was currentPidProfile->dtermSetpointWeight
        }
        if (sbufBytesRemaining(src) >= 14) {
            // Added in MSP API 1.40
            currentPidProfile->iterm_rotation = sbufReadU8(src);
            sbufReadU8(src); // was currentPidProfile->smart_feedforward
#if defined(USE_ITERM_RELAX)
            currentPidProfile->iterm_relax = sbufReadU8(src);
            currentPidProfile->iterm_relax_type = sbufReadU8(src);
#else
            sbufReadU8(src);
            sbufReadU8(src);
#endif
#if defined(USE_ABSOLUTE_CONTROL)
            currentPidProfile->abs_control_gain = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
#if defined(USE_THROTTLE_BOOST)
            currentPidProfile->throttle_boost = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
#if defined(USE_ACRO_TRAINER)
            currentPidProfile->acro_trainer_angle_limit = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
            // PID controller feedforward terms
            currentPidProfile->pid[PID_ROLL].F = sbufReadU16(src);
            currentPidProfile->pid[PID_PITCH].F = sbufReadU16(src);
            currentPidProfile->pid[PID_YAW].F = sbufReadU16(src);
            sbufReadU8(src); // was currentPidProfile->antiGravityMode
        }
        if (sbufBytesRemaining(src) >= 7) {
            // Added in MSP API 1.41
#ifdef USE_D_MAX
            currentPidProfile->d_max[PID_ROLL] = sbufReadU8(src);
            currentPidProfile->d_max[PID_PITCH] = sbufReadU8(src);
            currentPidProfile->d_max[PID_YAW] = sbufReadU8(src);
            currentPidProfile->d_max_gain = sbufReadU8(src);
            currentPidProfile->d_max_advance = sbufReadU8(src);
#else
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
#endif
#if defined(USE_INTEGRATED_YAW_CONTROL)
            currentPidProfile->use_integrated_yaw = sbufReadU8(src);
            currentPidProfile->integrated_yaw_relax = sbufReadU8(src);
#else
            sbufReadU8(src);
            sbufReadU8(src);
#endif
        }
        if(sbufBytesRemaining(src) >= 1) {
            // Added in MSP API 1.42
#if defined(USE_ITERM_RELAX)
            currentPidProfile->iterm_relax_cutoff = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 3) {
            // Added in MSP API 1.43
            currentPidProfile->motor_output_limit = sbufReadU8(src);
            currentPidProfile->auto_profile_cell_count = sbufReadU8(src);
#if defined(USE_DYN_IDLE)
            currentPidProfile->dyn_idle_min_rpm = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 7) {
            // Added in MSP API 1.44
#if defined(USE_FEEDFORWARD)
            currentPidProfile->feedforward_averaging = sbufReadU8(src);
            currentPidProfile->feedforward_smooth_factor = sbufReadU8(src);
            currentPidProfile->feedforward_boost = sbufReadU8(src);
            currentPidProfile->feedforward_max_rate_limit = sbufReadU8(src);
            currentPidProfile->feedforward_jitter_factor = sbufReadU8(src);
#else
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
#endif

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
            currentPidProfile->vbat_sag_compensation = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
#if defined(USE_THRUST_LINEARIZATION)
            currentPidProfile->thrustLinearization = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 4) {
            // Added in API 1.45
            currentPidProfile->tpa_mode = sbufReadU8(src);
            currentPidProfile->tpa_rate = MIN(sbufReadU8(src), TPA_MAX);
            currentPidProfile->tpa_breakpoint = sbufReadU16(src);
        }

        pidInitConfig(currentPidProfile);
        initEscEndpoints();
        mixerInitProfile();

        break;
    case MSP_SET_SENSOR_CONFIG:
#if defined(USE_ACC)
        accelerometerConfigMutable()->acc_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
#if defined(USE_BARO)
        barometerConfigMutable()->baro_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
#if defined(USE_MAG)
        compassConfigMutable()->mag_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
        break;

#ifdef USE_ACC
    case MSP_ACC_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            accStartCalibration();
        break;
#endif

#if defined(USE_MAG)
    case MSP_MAG_CALIBRATION:
        if (!ARMING_FLAG(ARMED)) {
            compassStartCalibration();
        }
#endif

        break;
    case MSP_EEPROM_WRITE:
        if (ARMING_FLAG(ARMED)) {
            return MSP_RESULT_ERROR;
        }

        // This is going to take some time and won't be done where real-time performance is needed so
        // ignore how long it takes to avoid confusing the scheduler
        schedulerIgnoreTaskStateTime();

#if defined(USE_MSP_OVER_TELEMETRY)
        if (featureIsEnabled(FEATURE_RX_SPI) && srcDesc == getMspTelemetryDescriptor()) {
            dispatchAdd(&writeReadEepromEntry, MSP_DISPATCH_DELAY_US);
        } else
#endif
        {
            writeReadEeprom(NULL);
        }

        break;

#ifdef USE_BLACKBOX
    case MSP_SET_BLACKBOX_CONFIG:
        // Don't allow config to be updated while Blackbox is logging
        if (blackboxMayEditConfig()) {
            blackboxConfigMutable()->device = sbufReadU8(src);
            const int rateNum = sbufReadU8(src); // was rate_num
            const int rateDenom = sbufReadU8(src); // was rate_denom
            uint16_t pRatio = 0;
            if (sbufBytesRemaining(src) >= 2) {
                // p_ratio specified, so use it directly
                pRatio = sbufReadU16(src);
            } else {
                // p_ratio not specified in MSP, so calculate it from old rateNum and rateDenom
                pRatio = blackboxCalculatePDenom(rateNum, rateDenom);
            }

            if (sbufBytesRemaining(src) >= 1) {
                // sample_rate specified, so use it directly
                blackboxConfigMutable()->sample_rate = sbufReadU8(src);
            } else {
                // sample_rate not specified in MSP, so calculate it from old p_ratio
                blackboxConfigMutable()->sample_rate = blackboxCalculateSampleRate(pRatio);
            }

            // Added in MSP API 1.45
            if (sbufBytesRemaining(src) >= 4) {
                blackboxConfigMutable()->fields_disabled_mask = sbufReadU32(src);
            }
        }
        break;
#endif

#ifdef USE_VTX_COMMON
    case MSP_SET_VTX_CONFIG:
        {
            vtxDevice_t *vtxDevice = vtxCommonDevice();
            vtxDevType_e vtxType = VTXDEV_UNKNOWN;
            if (vtxDevice) {
                vtxType = vtxCommonGetDeviceType(vtxDevice);
            }
            uint16_t newFrequency = sbufReadU16(src);
            if (newFrequency <= VTXCOMMON_MSP_BANDCHAN_CHKVAL) {  // Value is band and channel
                const uint8_t newBand = (newFrequency / 8) + 1;
                const uint8_t newChannel = (newFrequency % 8) + 1;
                vtxSettingsConfigMutable()->band = newBand;
                vtxSettingsConfigMutable()->channel = newChannel;
                vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxDevice, newBand, newChannel);
            } else if (newFrequency <= VTX_SETTINGS_MAX_FREQUENCY_MHZ) { // Value is frequency in MHz
                vtxSettingsConfigMutable()->band = 0;
                vtxSettingsConfigMutable()->freq = newFrequency;
            }

            if (sbufBytesRemaining(src) >= 2) {
                vtxSettingsConfigMutable()->power = sbufReadU8(src);
                const uint8_t newPitmode = sbufReadU8(src);
                if (vtxType != VTXDEV_UNKNOWN) {
                    // Delegate pitmode to vtx directly
                    unsigned vtxCurrentStatus;
                    vtxCommonGetStatus(vtxDevice, &vtxCurrentStatus);
                    if ((bool)(vtxCurrentStatus & VTX_STATUS_PIT_MODE) != (bool)newPitmode) {
                        vtxCommonSetPitMode(vtxDevice, newPitmode);
                    }
                }
            }

            if (sbufBytesRemaining(src)) {
                    vtxSettingsConfigMutable()->lowPowerDisarm = sbufReadU8(src);
            }

            // API version 1.42 - this parameter kept separate since clients may already be supplying
            if (sbufBytesRemaining(src) >= 2) {
                vtxSettingsConfigMutable()->pitModeFreq = sbufReadU16(src);
            }

            // API version 1.42 - extensions for non-encoded versions of the band, channel or frequency
            if (sbufBytesRemaining(src) >= 4) {
                // Added standalone values for band, channel and frequency to move
                // away from the flawed encoded combined method originally implemented.
                uint8_t newBand = sbufReadU8(src);
                const uint8_t newChannel = sbufReadU8(src);
                uint16_t newFreq = sbufReadU16(src);
                if (newBand) {
                    newFreq = vtxCommonLookupFrequency(vtxDevice, newBand, newChannel);
                }
                vtxSettingsConfigMutable()->band = newBand;
                vtxSettingsConfigMutable()->channel = newChannel;
                vtxSettingsConfigMutable()->freq = newFreq;
            }

            // API version 1.42 - extensions for vtxtable support
            if (sbufBytesRemaining(src) >= 4) {
#ifdef USE_VTX_TABLE
                const uint8_t newBandCount = sbufReadU8(src);
                const uint8_t newChannelCount = sbufReadU8(src);
                const uint8_t newPowerCount = sbufReadU8(src);

                if ((newBandCount > VTX_TABLE_MAX_BANDS) ||
                    (newChannelCount > VTX_TABLE_MAX_CHANNELS) ||
                    (newPowerCount > VTX_TABLE_MAX_POWER_LEVELS)) {
                    return MSP_RESULT_ERROR;
                }
                vtxTableConfigMutable()->bands = newBandCount;
                vtxTableConfigMutable()->channels = newChannelCount;
                vtxTableConfigMutable()->powerLevels = newPowerCount;

                // boolean to determine whether the vtxtable should be cleared in
                // expectation that the detailed band/channel and power level messages
                // will follow to repopulate the tables
                if (sbufReadU8(src)) {
                    for (int i = 0; i < VTX_TABLE_MAX_BANDS; i++) {
                        vtxTableConfigClearBand(vtxTableConfigMutable(), i);
                        vtxTableConfigClearChannels(vtxTableConfigMutable(), i, 0);
                    }
                    vtxTableConfigClearPowerLabels(vtxTableConfigMutable(), 0);
                    vtxTableConfigClearPowerValues(vtxTableConfigMutable(), 0);
                }
#else
                sbufReadU8(src);
                sbufReadU8(src);
                sbufReadU8(src);
                sbufReadU8(src);
#endif
            }
#ifdef USE_VTX_MSP
            setMspVtxDeviceStatusReady(srcDesc);
#endif
        }
        break;
#endif

#ifdef USE_VTX_TABLE
    case MSP_SET_VTXTABLE_BAND:
        {
            char bandName[VTX_TABLE_BAND_NAME_LENGTH + 1];
            memset(bandName, 0, VTX_TABLE_BAND_NAME_LENGTH + 1);
            uint16_t frequencies[VTX_TABLE_MAX_CHANNELS];
            const uint8_t band = sbufReadU8(src);
            const uint8_t bandNameLength = sbufReadU8(src);
            for (int i = 0; i < bandNameLength; i++) {
                const char nameChar = sbufReadU8(src);
                if (i < VTX_TABLE_BAND_NAME_LENGTH) {
                    bandName[i] = toupper(nameChar);
                }
            }
            const char bandLetter = toupper(sbufReadU8(src));
            const bool isFactoryBand = (bool)sbufReadU8(src);
            const uint8_t channelCount = sbufReadU8(src);
            for (int i = 0; i < channelCount; i++) {
                const uint16_t frequency = sbufReadU16(src);
                if (i < vtxTableConfig()->channels) {
                    frequencies[i] = frequency;
                }
            }

            if (band > 0 && band <= vtxTableConfig()->bands) {
                vtxTableStrncpyWithPad(vtxTableConfigMutable()->bandNames[band - 1], bandName, VTX_TABLE_BAND_NAME_LENGTH);
                vtxTableConfigMutable()->bandLetters[band - 1] = bandLetter;
                vtxTableConfigMutable()->isFactoryBand[band - 1] = isFactoryBand;
                for (int i = 0; i < vtxTableConfig()->channels; i++) {
                    vtxTableConfigMutable()->frequency[band - 1][i] = frequencies[i];
                }
                // If this is the currently selected band then reset the frequency
                if (band == vtxSettingsConfig()->band) {
                    uint16_t newFreq = 0;
                    if (vtxSettingsConfig()->channel > 0 && vtxSettingsConfig()->channel <= vtxTableConfig()->channels) {
                        newFreq = frequencies[vtxSettingsConfig()->channel - 1];
                    }
                    vtxSettingsConfigMutable()->freq = newFreq;
                }
                vtxTableNeedsInit = true;  // reinintialize vtxtable after eeprom write
            } else {
                return MSP_RESULT_ERROR;
            }
#ifdef USE_VTX_MSP
            setMspVtxDeviceStatusReady(srcDesc);
#endif
        }
        break;

    case MSP_SET_VTXTABLE_POWERLEVEL:
        {
            char powerLevelLabel[VTX_TABLE_POWER_LABEL_LENGTH + 1];
            memset(powerLevelLabel, 0, VTX_TABLE_POWER_LABEL_LENGTH + 1);
            const uint8_t powerLevel = sbufReadU8(src);
            const uint16_t powerValue = sbufReadU16(src);
            const uint8_t powerLevelLabelLength = sbufReadU8(src);
            for (int i = 0; i < powerLevelLabelLength; i++) {
                const char labelChar = sbufReadU8(src);
                if (i < VTX_TABLE_POWER_LABEL_LENGTH) {
                    powerLevelLabel[i] = toupper(labelChar);
                }
            }

            if (powerLevel > 0 && powerLevel <= vtxTableConfig()->powerLevels) {
                vtxTableConfigMutable()->powerValues[powerLevel - 1] = powerValue;
                vtxTableStrncpyWithPad(vtxTableConfigMutable()->powerLabels[powerLevel - 1], powerLevelLabel, VTX_TABLE_POWER_LABEL_LENGTH);
                vtxTableNeedsInit = true;  // reinintialize vtxtable after eeprom write
            } else {
                return MSP_RESULT_ERROR;
            }
#ifdef USE_VTX_MSP
            setMspVtxDeviceStatusReady(srcDesc);
#endif
        }
        break;
#endif

    case MSP2_SET_MOTOR_OUTPUT_REORDERING:
        {
            const uint8_t arraySize = sbufReadU8(src);

            for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
                uint8_t value = i;

                if (i < arraySize) {
                    value = sbufReadU8(src);
                }

                motorConfigMutable()->dev.motorOutputReordering[i] = value;
            }
        }
        break;

#ifdef USE_DSHOT
    case MSP2_SEND_DSHOT_COMMAND:
        {
            const bool armed = ARMING_FLAG(ARMED);

            if (!armed) {
                const uint8_t commandType = sbufReadU8(src);
                const uint8_t motorIndex = sbufReadU8(src);
                const uint8_t commandCount = sbufReadU8(src);

                if (DSHOT_CMD_TYPE_BLOCKING == commandType) {
                    motorDisable();
                }

                for (uint8_t i = 0; i < commandCount; i++) {
                    const uint8_t commandIndex = sbufReadU8(src);
                    dshotCommandWrite(motorIndex, getMotorCount(), commandIndex, commandType);
                }

                if (DSHOT_CMD_TYPE_BLOCKING == commandType) {
                    motorEnable();
                }
            }
        }
        break;
#endif

#ifdef USE_SIMPLIFIED_TUNING
    // Added in MSP API 1.44
    case MSP_SET_SIMPLIFIED_TUNING:
        {
            readSimplifiedPids(currentPidProfile, src);
            readSimplifiedDtermFilters(currentPidProfile, src);
            readSimplifiedGyroFilters(gyroConfigMutable(), src);
            applySimplifiedTuning(currentPidProfile, gyroConfigMutable());
        }
        break;
#endif

#ifdef USE_CAMERA_CONTROL
    case MSP_CAMERA_CONTROL:
        {
            if (ARMING_FLAG(ARMED)) {
                return MSP_RESULT_ERROR;
            }

            const uint8_t key = sbufReadU8(src);
            cameraControlKeyPress(key, 0);
        }
        break;
#endif

    case MSP_SET_ARMING_DISABLED:
        {
            const uint8_t command = sbufReadU8(src);
            uint8_t disableRunawayTakeoff = 0;
#ifndef USE_RUNAWAY_TAKEOFF
            UNUSED(disableRunawayTakeoff);
#endif
            if (sbufBytesRemaining(src)) {
                disableRunawayTakeoff = sbufReadU8(src);
            }
            if (command) {
#ifndef SIMULATOR_BUILD // In simulator mode we can safely arm with MSP link.
                mspArmingDisableByDescriptor(srcDesc);
                setArmingDisabled(ARMING_DISABLED_MSP);
                if (ARMING_FLAG(ARMED)) {
                    disarm(DISARM_REASON_ARMING_DISABLED);
                }
#endif
#ifdef USE_RUNAWAY_TAKEOFF
                runawayTakeoffTemporaryDisable(false);
#endif
            } else {
                mspArmingEnableByDescriptor(srcDesc);
                if (mspIsMspArmingEnabled()) {
                    unsetArmingDisabled(ARMING_DISABLED_MSP);
#ifdef USE_RUNAWAY_TAKEOFF
                    runawayTakeoffTemporaryDisable(disableRunawayTakeoff);
#endif
                }
            }
        }
        break;

#if defined(USE_FLASHFS) && defined(USE_BLACKBOX)
    case MSP_DATAFLASH_ERASE:
        blackboxEraseAll();

        break;
#endif

#ifdef USE_GPS
    case MSP2_SENSOR_GPS:
        (void)sbufReadU8(src);              // instance
        (void)sbufReadU16(src);             // gps_week
        gpsSol.time = sbufReadU32(src);     // ms_tow
        gpsSetFixState(sbufReadU8(src) != 0); // fix_type
        gpsSol.numSat = sbufReadU8(src);    // satellites_in_view
        gpsSol.acc.hAcc = sbufReadU16(src) * 10; // horizontal_pos_accuracy - convert cm to mm
        gpsSol.acc.vAcc = sbufReadU16(src) * 10; // vertical_pos_accuracy - convert cm to mm
        gpsSol.acc.sAcc = sbufReadU16(src) * 10; // horizontal_vel_accuracy - convert cm to mm
        gpsSol.dop.pdop = sbufReadU16(src); // hdop in 4.4 and earlier, pdop in 4.5 and above
        gpsSol.llh.lon = sbufReadU32(src);
        gpsSol.llh.lat = sbufReadU32(src);
        gpsSol.llh.altCm = sbufReadU32(src); // alt
        int32_t ned_vel_north = (int32_t)sbufReadU32(src);  // ned_vel_north
        int32_t ned_vel_east = (int32_t)sbufReadU32(src);   // ned_vel_east
        gpsSol.groundSpeed = (uint16_t)sqrtf((ned_vel_north * ned_vel_north) + (ned_vel_east * ned_vel_east));
        (void)sbufReadU32(src);             // ned_vel_down
        gpsSol.groundCourse = ((uint16_t)sbufReadU16(src) % 36000) / 10; // incoming value expected to be in centidegrees, output value in decidegrees
        (void)sbufReadU16(src);             // true_yaw
        (void)sbufReadU16(src);             // year
        (void)sbufReadU8(src);              // month
        (void)sbufReadU8(src);              // day
        (void)sbufReadU8(src);              // hour
        (void)sbufReadU8(src);              // min
        (void)sbufReadU8(src);              // sec
        GPS_update |= GPS_MSP_UPDATE;        // MSP data signalisation to GPS functions
        break;

    case MSP_SET_RAW_GPS:
        gpsSetFixState(sbufReadU8(src));
        gpsSol.numSat = sbufReadU8(src);
        gpsSol.llh.lat = sbufReadU32(src);
        gpsSol.llh.lon = sbufReadU32(src);
        gpsSol.llh.altCm = sbufReadU16(src) * 100; // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. Received MSP altitudes in 1m per lsb have to upscaled.
        gpsSol.groundSpeed = sbufReadU16(src);
        GPS_update |= GPS_MSP_UPDATE;        // MSP data signalisation to GPS functions
        break;
#endif // USE_GPS
    case MSP_SET_FEATURE_CONFIG:
        featureConfigReplace(sbufReadU32(src));
        break;

#ifdef USE_BEEPER
    case MSP_SET_BEEPER_CONFIG:
        beeperConfigMutable()->beeper_off_flags = sbufReadU32(src);
        if (sbufBytesRemaining(src) >= 1) {
            beeperConfigMutable()->dshotBeaconTone = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 4) {
            beeperConfigMutable()->dshotBeaconOffFlags = sbufReadU32(src);
        }
        break;
#endif

    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
        boardAlignmentMutable()->rollDegrees = sbufReadU16(src);
        boardAlignmentMutable()->pitchDegrees = sbufReadU16(src);
        boardAlignmentMutable()->yawDegrees = sbufReadU16(src);
        break;

    case MSP_SET_MIXER_CONFIG:
#ifndef USE_QUAD_MIXER_ONLY
        mixerConfigMutable()->mixerMode = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
        if (sbufBytesRemaining(src) >= 1) {
            mixerConfigMutable()->yaw_motors_reversed = sbufReadU8(src);
        }
        break;

    case MSP_SET_RX_CONFIG:
        rxConfigMutable()->serialrx_provider = sbufReadU8(src);
        rxConfigMutable()->maxcheck = sbufReadU16(src);
        rxConfigMutable()->midrc = sbufReadU16(src);
        rxConfigMutable()->mincheck = sbufReadU16(src);
        rxConfigMutable()->spektrum_sat_bind = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 4) {
            rxConfigMutable()->rx_min_usec = sbufReadU16(src);
            rxConfigMutable()->rx_max_usec = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 4) {
            sbufReadU8(src); // not required in API 1.44, was rxConfigMutable()->rcInterpolation
            sbufReadU8(src); // not required in API 1.44, was rxConfigMutable()->rcInterpolationInterval
            rxConfigMutable()->airModeActivateThreshold = (sbufReadU16(src) - 1000) / 10;
        }
        if (sbufBytesRemaining(src) >= 6) {
#ifdef USE_RX_SPI
            rxSpiConfigMutable()->rx_spi_protocol = sbufReadU8(src);
            rxSpiConfigMutable()->rx_spi_id = sbufReadU32(src);
            rxSpiConfigMutable()->rx_spi_rf_channel_count = sbufReadU8(src);
#else
            sbufReadU8(src);
            sbufReadU32(src);
            sbufReadU8(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 1) {
            rxConfigMutable()->fpvCamAngleDegrees = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 6) {
            // Added in MSP API 1.40
            sbufReadU8(src); // not required in API 1.44, was rxConfigMutable()->rcSmoothingChannels
#if defined(USE_RC_SMOOTHING_FILTER)
            sbufReadU8(src); // not required in API 1.44, was rc_smoothing_type
            configRebootUpdateCheckU8(&rxConfigMutable()->rc_smoothing_setpoint_cutoff, sbufReadU8(src));
            configRebootUpdateCheckU8(&rxConfigMutable()->rc_smoothing_feedforward_cutoff, sbufReadU8(src));
            sbufReadU8(src); // not required in API 1.44, was rc_smoothing_input_type
            sbufReadU8(src); // not required in API 1.44, was rc_smoothing_derivative_type
#else
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 1) {
            // Added in MSP API 1.40
            // Kept separate from the section above to work around missing Configurator support in version < 10.4.2
#if defined(USE_USB_CDC_HID)
            usbDevConfigMutable()->type = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 1) {
            // Added in MSP API 1.42
#if defined(USE_RC_SMOOTHING_FILTER)
            // Added extra validation/range constraint for rc_smoothing_auto_factor as a workaround for a bug in
            // the 10.6 configurator where it was possible to submit an invalid out-of-range value. We might be
            // able to remove the constraint at some point in the future once the affected versions are deprecated
            // enough that the risk is low.
            configRebootUpdateCheckU8(&rxConfigMutable()->rc_smoothing_auto_factor_rpy, constrain(sbufReadU8(src), RC_SMOOTHING_AUTO_FACTOR_MIN, RC_SMOOTHING_AUTO_FACTOR_MAX));
#else
            sbufReadU8(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 1) {
            // Added in MSP API 1.44
#if defined(USE_RC_SMOOTHING_FILTER)
            configRebootUpdateCheckU8(&rxConfigMutable()->rc_smoothing_mode, sbufReadU8(src));
#else
            sbufReadU8(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 6) {
            // Added in MSP API 1.45
#ifdef USE_RX_EXPRESSLRS
            sbufReadData(src, rxExpressLrsSpiConfigMutable()->UID, 6);
#else
            uint8_t emptyUid[6];
            sbufReadData(src, emptyUid, 6);
#endif
        }
        if (sbufBytesRemaining(src) >= 1) {
#ifdef USE_RX_EXPRESSLRS
            // Added in MSP API 1.47
            rxExpressLrsSpiConfigMutable()->modelId = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
        }
        break;
    case MSP_SET_FAILSAFE_CONFIG:
        failsafeConfigMutable()->failsafe_delay = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_landing_time = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_throttle = sbufReadU16(src);
        failsafeConfigMutable()->failsafe_switch_mode = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_throttle_low_delay = sbufReadU16(src);
        failsafeConfigMutable()->failsafe_procedure = sbufReadU8(src);
        break;

    case MSP_SET_RXFAIL_CONFIG:
        i = sbufReadU8(src);
        if (i < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
            rxFailsafeChannelConfigsMutable(i)->mode = sbufReadU8(src);
            rxFailsafeChannelConfigsMutable(i)->step = CHANNEL_VALUE_TO_RXFAIL_STEP(sbufReadU16(src));
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RSSI_CONFIG:
        rxConfigMutable()->rssi_channel = sbufReadU8(src);
        break;

    case MSP_SET_RX_MAP:
        for (int i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
            rxConfigMutable()->rcmap[i] = sbufReadU8(src);
        }
        break;

    case MSP_SET_CF_SERIAL_CONFIG:
        {
            uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (dataSize % portConfigSize != 0) {
                return MSP_RESULT_ERROR;
            }

            uint8_t remainingPortsInPacket = dataSize / portConfigSize;

            while (remainingPortsInPacket--) {
                uint8_t identifier = sbufReadU8(src);

                serialPortConfig_t *portConfig = serialFindPortConfigurationMutable(identifier);

                if (!portConfig) {
                    return MSP_RESULT_ERROR;
                }

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU16(src);
                portConfig->msp_baudrateIndex = sbufReadU8(src);
                portConfig->gps_baudrateIndex = sbufReadU8(src);
                portConfig->telemetry_baudrateIndex = sbufReadU8(src);
                portConfig->blackbox_baudrateIndex = sbufReadU8(src);
            }
        }
        break;
    case MSP2_COMMON_SET_SERIAL_CONFIG: {
        if (dataSize < 1) {
            return MSP_RESULT_ERROR;
        }
        unsigned count = sbufReadU8(src);
        unsigned portConfigSize = (dataSize - 1) / count;
        unsigned expectedPortSize = sizeof(uint8_t) + sizeof(uint32_t) + (sizeof(uint8_t) * 4);
        if (portConfigSize < expectedPortSize) {
            return MSP_RESULT_ERROR;
        }
        for (unsigned ii = 0; ii < count; ii++) {
            unsigned start = sbufBytesRemaining(src);
            uint8_t identifier = sbufReadU8(src);
            serialPortConfig_t *portConfig = serialFindPortConfigurationMutable(identifier);

            if (!portConfig) {
                return MSP_RESULT_ERROR;
            }

            portConfig->identifier = identifier;
            portConfig->functionMask = sbufReadU32(src);
            portConfig->msp_baudrateIndex = sbufReadU8(src);
            portConfig->gps_baudrateIndex = sbufReadU8(src);
            portConfig->telemetry_baudrateIndex = sbufReadU8(src);
            portConfig->blackbox_baudrateIndex = sbufReadU8(src);
            // Skip unknown bytes
            while (start - sbufBytesRemaining(src) < portConfigSize && sbufBytesRemaining(src)) {
                sbufReadU8(src);
            }
        }
        break;
    }

#ifdef USE_LED_STRIP_STATUS_MODE
    case MSP_SET_LED_COLORS:
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            hsvColor_t *color = &ledStripStatusModeConfigMutable()->colors[i];
            color->h = sbufReadU16(src);
            color->s = sbufReadU8(src);
            color->v = sbufReadU8(src);
        }
        break;
#endif

#ifdef USE_LED_STRIP
    case MSP_SET_LED_STRIP_CONFIG:
        {
            i = sbufReadU8(src);
            if (i >= LED_STRIP_MAX_LENGTH || dataSize != (1 + 4)) {
                return MSP_RESULT_ERROR;
            }
#ifdef USE_LED_STRIP_STATUS_MODE
            ledConfig_t *ledConfig = &ledStripStatusModeConfigMutable()->ledConfigs[i];
            *ledConfig = sbufReadU32(src);
            reevaluateLedConfig();
#else
            sbufReadU32(src);
#endif
            // API 1.41 - selected ledstrip_profile
            if (sbufBytesRemaining(src) >= 1) {
                ledStripConfigMutable()->ledstrip_profile = sbufReadU8(src);
            }
        }
        break;
#endif

#ifdef USE_LED_STRIP_STATUS_MODE
    case MSP_SET_LED_STRIP_MODECOLOR:
        {
            ledModeIndex_e modeIdx = sbufReadU8(src);
            int funIdx = sbufReadU8(src);
            int color = sbufReadU8(src);

            if (!setModeColor(modeIdx, funIdx, color)) {
                return MSP_RESULT_ERROR;
            }
        }
        break;
#endif

    case MSP_SET_NAME:
        memset(pilotConfigMutable()->craftName, 0, ARRAYLEN(pilotConfig()->craftName));
        for (unsigned int i = 0; i < MIN(MAX_NAME_LENGTH, dataSize); i++) {
            pilotConfigMutable()->craftName[i] = sbufReadU8(src);
        }
#ifdef USE_OSD
        osdAnalyzeActiveElements();
#endif
        break;

#ifdef USE_RTC_TIME
    case MSP_SET_RTC:
        {
            // Use seconds and milliseconds to make senders
            // easier to implement. Generating a 64 bit value
            // might not be trivial in some platforms.
            int32_t secs = (int32_t)sbufReadU32(src);
            uint16_t millis = sbufReadU16(src);
            rtcTime_t t = rtcTimeMake(secs, millis);
            rtcSet(&t);
        }

        break;
#endif

    case MSP_SET_TX_INFO:
        setRssiMsp(sbufReadU8(src));

        break;

#if defined(USE_BOARD_INFO)
    case MSP_SET_BOARD_INFO:
        if (!boardInformationIsSet()) {
            uint8_t length = sbufReadU8(src);
            char boardName[MAX_BOARD_NAME_LENGTH + 1];
            sbufReadData(src, boardName, MIN(length, MAX_BOARD_NAME_LENGTH));
            if (length > MAX_BOARD_NAME_LENGTH) {
                sbufAdvance(src, length - MAX_BOARD_NAME_LENGTH);
                length = MAX_BOARD_NAME_LENGTH;
            }
            boardName[length] = '\0';
            length = sbufReadU8(src);
            char manufacturerId[MAX_MANUFACTURER_ID_LENGTH + 1];
            sbufReadData(src, manufacturerId, MIN(length, MAX_MANUFACTURER_ID_LENGTH));
            if (length > MAX_MANUFACTURER_ID_LENGTH) {
                sbufAdvance(src, length - MAX_MANUFACTURER_ID_LENGTH);
                length = MAX_MANUFACTURER_ID_LENGTH;
            }
            manufacturerId[length] = '\0';

            setBoardName(boardName);
            setManufacturerId(manufacturerId);
            persistBoardInformation();
        } else {
            return MSP_RESULT_ERROR;
        }

        break;
#if defined(USE_SIGNATURE)
    case MSP_SET_SIGNATURE:
        if (!signatureIsSet()) {
            uint8_t signature[SIGNATURE_LENGTH];
            sbufReadData(src, signature, SIGNATURE_LENGTH);
            setSignature(signature);
            persistSignature();
        } else {
            return MSP_RESULT_ERROR;
        }

        break;
#endif
#endif // USE_BOARD_INFO
#if defined(USE_RX_BIND)
    case MSP2_BETAFLIGHT_BIND:
        if (!startRxBind()) {
            return MSP_RESULT_ERROR;
        }

        break;
#endif

    case MSP2_SET_TEXT:
        {
            // type byte, then length byte followed by the actual characters
            const uint8_t textType = sbufReadU8(src);

            char* textVar;
            const uint8_t textLength = MIN(MAX_NAME_LENGTH, sbufReadU8(src));
            switch (textType) {
                case MSP2TEXT_PILOT_NAME:
                    textVar = pilotConfigMutable()->pilotName;
                    break;

                case MSP2TEXT_CRAFT_NAME:
                    textVar = pilotConfigMutable()->craftName;
                    break;

                case MSP2TEXT_PID_PROFILE_NAME:
                    textVar = currentPidProfile->profileName;
                    break;

                case MSP2TEXT_RATE_PROFILE_NAME:
                    textVar = currentControlRateProfile->profileName;
                    break;

                default:
                    return MSP_RESULT_ERROR;
            }

            memset(textVar, 0, strlen(textVar));
            for (unsigned int i = 0; i < textLength; i++) {
                textVar[i] = sbufReadU8(src);
            }

#ifdef USE_OSD
            if (textType == MSP2TEXT_PILOT_NAME || textType == MSP2TEXT_CRAFT_NAME) {
                osdAnalyzeActiveElements();
            }
#endif
        }
        break;

#ifdef USE_LED_STRIP
    case MSP2_SET_LED_STRIP_CONFIG_VALUES:
        ledStripConfigMutable()->ledstrip_brightness = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_rainbow_delta = sbufReadU16(src);
        ledStripConfigMutable()->ledstrip_rainbow_freq = sbufReadU16(src);
        break;
#endif

    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

static mspResult_e mspCommonProcessInCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    UNUSED(mspPostProcessFn);
    const unsigned int dataSize = sbufBytesRemaining(src);
    UNUSED(dataSize); // maybe unused due to compiler options

    switch (cmdMSP) {
#ifdef USE_TRANSPONDER
    case MSP_SET_TRANSPONDER_CONFIG: {
        // Backward compatibility to BFC 3.1.1 is lost for this message type

        uint8_t provider = sbufReadU8(src);
        uint8_t bytesRemaining = dataSize - 1;

        if (provider > TRANSPONDER_PROVIDER_COUNT) {
            return MSP_RESULT_ERROR;
        }

        const uint8_t requirementIndex = provider - 1;
        const uint8_t transponderDataSize = transponderRequirements[requirementIndex].dataLength;

        transponderConfigMutable()->provider = provider;

        if (provider == TRANSPONDER_NONE) {
            break;
        }

        if (bytesRemaining != transponderDataSize) {
            return MSP_RESULT_ERROR;
        }

        if (provider != transponderConfig()->provider) {
            transponderStopRepeating();
        }

        memset(transponderConfigMutable()->data, 0, sizeof(transponderConfig()->data));

        for (unsigned int i = 0; i < transponderDataSize; i++) {
            transponderConfigMutable()->data[i] = sbufReadU8(src);
        }
        transponderUpdateData();
        break;
    }
#endif

    case MSP_SET_VOLTAGE_METER_CONFIG: {
        int8_t id = sbufReadU8(src);

        //
        // find and configure an ADC voltage sensor
        //
        int8_t voltageSensorADCIndex;
        for (voltageSensorADCIndex = 0; voltageSensorADCIndex < MAX_VOLTAGE_SENSOR_ADC; voltageSensorADCIndex++) {
            if (id == voltageMeterADCtoIDMap[voltageSensorADCIndex]) {
                break;
            }
        }

        if (voltageSensorADCIndex < MAX_VOLTAGE_SENSOR_ADC) {
            voltageSensorADCConfigMutable(voltageSensorADCIndex)->vbatscale = sbufReadU8(src);
            voltageSensorADCConfigMutable(voltageSensorADCIndex)->vbatresdivval = sbufReadU8(src);
            voltageSensorADCConfigMutable(voltageSensorADCIndex)->vbatresdivmultiplier = sbufReadU8(src);
        } else {
            // if we had any other types of voltage sensor to configure, this is where we'd do it.
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
        }
        break;
    }

    case MSP_SET_CURRENT_METER_CONFIG: {
        int id = sbufReadU8(src);

        switch (id) {
            case CURRENT_METER_ID_BATTERY_1:
                currentSensorADCConfigMutable()->scale = sbufReadU16(src);
                currentSensorADCConfigMutable()->offset = sbufReadU16(src);
                break;
#ifdef USE_VIRTUAL_CURRENT_METER
            case CURRENT_METER_ID_VIRTUAL_1:
                currentSensorVirtualConfigMutable()->scale = sbufReadU16(src);
                currentSensorVirtualConfigMutable()->offset = sbufReadU16(src);
                break;
#endif
            default:
                sbufReadU16(src);
                sbufReadU16(src);
                break;
        }
        break;
    }

    case MSP_SET_BATTERY_CONFIG:
        batteryConfigMutable()->vbatmincellvoltage = sbufReadU8(src) * 10;      // vbatlevel_warn1 in MWC2.3 GUI
        batteryConfigMutable()->vbatmaxcellvoltage = sbufReadU8(src) * 10;      // vbatlevel_warn2 in MWC2.3 GUI
        batteryConfigMutable()->vbatwarningcellvoltage = sbufReadU8(src) * 10;  // vbatlevel when buzzer starts to alert
        batteryConfigMutable()->batteryCapacity = sbufReadU16(src);
        batteryConfigMutable()->voltageMeterSource = sbufReadU8(src);
        batteryConfigMutable()->currentMeterSource = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 6) {
            batteryConfigMutable()->vbatmincellvoltage = sbufReadU16(src);
            batteryConfigMutable()->vbatmaxcellvoltage = sbufReadU16(src);
            batteryConfigMutable()->vbatwarningcellvoltage = sbufReadU16(src);
        }
        break;

#if defined(USE_OSD)
    case MSP_SET_OSD_CONFIG:
        {
            const uint8_t addr = sbufReadU8(src);

            if ((int8_t)addr == -1) {
                /* Set general OSD settings */
                videoSystem_e video_system = sbufReadU8(src);

                if ((video_system == VIDEO_SYSTEM_HD) && (vcdProfile()->video_system != VIDEO_SYSTEM_HD)) {
                    // If switching to HD, don't wait for the VTX to communicate the correct resolution, just
#ifdef USE_OSD_HD
                    // If an HD build, increase the canvas size to the HD default as that is what the user will expect
                    osdConfigMutable()->canvas_cols = OSD_HD_COLS;
                    osdConfigMutable()->canvas_rows = OSD_HD_ROWS;
                    // Also force use of MSP displayport
                    osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_MSP;
#else
                    // must have an SD build option, keep existing SD video_system, do not change canvas size
                    video_system = vcdProfile()->video_system;
#endif
                } else if ((video_system != VIDEO_SYSTEM_HD) && (vcdProfile()->video_system == VIDEO_SYSTEM_HD)) {
                    // Switching away from HD to SD
#ifdef USE_OSD_SD
                    // SD is in the build; set canvas size to SD and displayport device to auto
                    osdConfigMutable()->canvas_cols = OSD_SD_COLS;
                    osdConfigMutable()->canvas_rows = (video_system == VIDEO_SYSTEM_NTSC) ? VIDEO_LINES_NTSC : VIDEO_LINES_PAL;
                    osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_AUTO;
#else
                    // must have an HD build option, keep existing HD video_system, do not change canvas size
                    video_system = VIDEO_SYSTEM_HD;
#endif
                }

                vcdProfileMutable()->video_system = video_system;

                osdConfigMutable()->units = sbufReadU8(src);

                // Alarms
                osdConfigMutable()->rssi_alarm = sbufReadU8(src);
                osdConfigMutable()->cap_alarm = sbufReadU16(src);
                sbufReadU16(src); // Skip unused (previously fly timer)
                osdConfigMutable()->alt_alarm = sbufReadU16(src);

                if (sbufBytesRemaining(src) >= 2) {
                    /* Enabled warnings */
                    // API < 1.41 supports only the low 16 bits
                    osdConfigMutable()->enabledWarnings = sbufReadU16(src);
                }

                if (sbufBytesRemaining(src) >= 4) {
                    // 32bit version of enabled warnings (API >= 1.41)
                    osdConfigMutable()->enabledWarnings = sbufReadU32(src);
                }

                if (sbufBytesRemaining(src) >= 1) {
                    // API >= 1.41
                    // selected OSD profile
#ifdef USE_OSD_PROFILES
                    changeOsdProfileIndex(sbufReadU8(src));
#else
                    sbufReadU8(src);
#endif // USE_OSD_PROFILES
                }

                if (sbufBytesRemaining(src) >= 1) {
                    // API >= 1.41
                    // OSD stick overlay mode

#ifdef USE_OSD_STICK_OVERLAY
                    osdConfigMutable()->overlay_radio_mode = sbufReadU8(src);
#else
                    sbufReadU8(src);
#endif // USE_OSD_STICK_OVERLAY

                }

                if (sbufBytesRemaining(src) >= 2) {
                    // API >= 1.43
                    // OSD camera frame element width/height
                    osdConfigMutable()->camera_frame_width = sbufReadU8(src);
                    osdConfigMutable()->camera_frame_height = sbufReadU8(src);
                }

                if (sbufBytesRemaining(src) >= 2) {
                    // API >= 1.46
                    osdConfigMutable()->link_quality_alarm = sbufReadU16(src);
                }

                if (sbufBytesRemaining(src) >= 2) {
                    // API >= 1.47
                    osdConfigMutable()->rssi_dbm_alarm = sbufReadU16(src);
                }

            } else if ((int8_t)addr == -2) {
                // Timers
                uint8_t index = sbufReadU8(src);
                if (index > OSD_TIMER_COUNT) {
                    return MSP_RESULT_ERROR;
                }
                osdConfigMutable()->timers[index] = sbufReadU16(src);
            } else {
                const uint16_t value = sbufReadU16(src);

                /* Get screen index, 0 is post flight statistics, 1 and above are in flight OSD screens */
                const uint8_t screen = (sbufBytesRemaining(src) >= 1) ? sbufReadU8(src) : 1;

                if (screen == 0 && addr < OSD_STAT_COUNT) {
                    /* Set statistic item enable */
                    osdStatSetState(addr, (value != 0));
                } else if (addr < OSD_ITEM_COUNT) {
                    /* Set element positions */
                    osdElementConfigMutable()->item_pos[addr] = value;
                    osdAnalyzeActiveElements();
                } else {
                    return MSP_RESULT_ERROR;
                }
            }
        }
        break;

    case MSP_OSD_CHAR_WRITE:
        {
            osdCharacter_t chr;
            size_t osdCharacterBytes;
            uint16_t addr;
            if (dataSize >= OSD_CHAR_VISIBLE_BYTES + 2) {
                if (dataSize >= OSD_CHAR_BYTES + 2) {
                    // 16 bit address, full char with metadata
                    addr = sbufReadU16(src);
                    osdCharacterBytes = OSD_CHAR_BYTES;
                } else if (dataSize >= OSD_CHAR_BYTES + 1) {
                    // 8 bit address, full char with metadata
                    addr = sbufReadU8(src);
                    osdCharacterBytes = OSD_CHAR_BYTES;
                } else {
                    // 16 bit character address, only visible char bytes
                    addr = sbufReadU16(src);
                    osdCharacterBytes = OSD_CHAR_VISIBLE_BYTES;
                }
            } else {
                // 8 bit character address, only visible char bytes
                addr = sbufReadU8(src);
                osdCharacterBytes = OSD_CHAR_VISIBLE_BYTES;
            }
            for (unsigned ii = 0; ii < MIN(osdCharacterBytes, sizeof(chr.data)); ii++) {
                chr.data[ii] = sbufReadU8(src);
            }
            displayPort_t *osdDisplayPort = osdGetDisplayPort(NULL);
            if (!osdDisplayPort) {
                return MSP_RESULT_ERROR;
            }

            if (!displayWriteFontCharacter(osdDisplayPort, addr, &chr)) {
                return MSP_RESULT_ERROR;
            }
        }
        break;

#ifdef USE_OSD_HD
    case MSP_SET_OSD_CANVAS:
        {
            osdConfigMutable()->canvas_cols = sbufReadU8(src);
            osdConfigMutable()->canvas_rows = sbufReadU8(src);

            if ((vcdProfile()->video_system != VIDEO_SYSTEM_HD) ||
                (osdConfig()->displayPortDevice != OSD_DISPLAYPORT_DEVICE_MSP)) {
                // An HD VTX has communicated it's canvas size, so we must be in HD mode
                vcdProfileMutable()->video_system = VIDEO_SYSTEM_HD;
                // And using MSP displayport
                osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_MSP;

                // Save settings and reboot or the user won't see the effect and will have to manually save
                writeEEPROM();
                systemReset();
            }
        }
        break;
#endif //USE_OSD_HD
#endif // OSD

    default:
        return mspProcessInCommand(srcDesc, cmdMSP, src);
    }
    return MSP_RESULT_ACK;
}

/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
mspResult_e mspFcProcessCommand(mspDescriptor_t srcDesc, mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const int16_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspCommonProcessOutCommand(cmdMSP, dst, mspPostProcessFn)) {
        ret = MSP_RESULT_ACK;
    } else if (mspProcessOutCommand(srcDesc, cmdMSP, dst)) {
        ret = MSP_RESULT_ACK;
    } else if ((ret = mspFcProcessOutCommandWithArg(srcDesc, cmdMSP, src, dst, mspPostProcessFn)) != MSP_RESULT_CMD_UNKNOWN) {
        /* ret */;
    } else if (cmdMSP == MSP_SET_PASSTHROUGH) {
        mspFcSetPassthroughCommand(dst, src, mspPostProcessFn);
        ret = MSP_RESULT_ACK;
#ifdef USE_FLASHFS
    } else if (cmdMSP == MSP_DATAFLASH_READ) {
        mspFcDataFlashReadCommand(dst, src);
        ret = MSP_RESULT_ACK;
#endif
    } else {
        ret = mspCommonProcessInCommand(srcDesc, cmdMSP, src, mspPostProcessFn);
    }
    reply->result = ret;
    return ret;
}

void mspFcProcessReply(mspPacket_t *reply)
{
    sbuf_t *src = &reply->buf;
    UNUSED(src); // potentially unused depending on compile options.

    switch (reply->cmd) {
    case MSP_ANALOG:
        {
            uint8_t batteryVoltage = sbufReadU8(src);
            uint16_t mAhDrawn = sbufReadU16(src);
            uint16_t rssi = sbufReadU16(src);
            uint16_t amperage = sbufReadU16(src);

            UNUSED(rssi);
            UNUSED(batteryVoltage);
            UNUSED(amperage);
            UNUSED(mAhDrawn);

#ifdef USE_MSP_CURRENT_METER
            currentMeterMSPSet(amperage, mAhDrawn);
#endif
        }
        break;
    }
}

void mspInit(void)
{
    initActiveBoxIds();
}
