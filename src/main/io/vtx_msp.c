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

/* Created by phobos- */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#if defined(USE_VTX_MSP) && defined(USE_VTX_CONTROL) && defined(USE_VTX_COMMON)

#include "build/debug.h"

#include "cms/cms_menu_vtx_msp.h"
#include "common/crc.h"
#include "config/feature.h"

#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "fc/runtime_config.h"
#include "flight/failsafe.h"

#include "io/serial.h"
#include "io/vtx_msp.h"
#include "io/vtx_control.h"
#include "io/vtx.h"

#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "pg/vtx_table.h"

#include "rx/crsf.h"
#include "rx/crsf_protocol.h"
#include "rx/rx.h"

#include "telemetry/msp_shared.h"

static uint16_t mspConfFreq = 0;
static uint8_t mspConfBand = 0;
static uint8_t mspConfChannel = 0;
static uint16_t mspConfPower = 0;
static uint8_t mspConfPitMode = 0;
static bool mspVtxConfigChanged = false;
static timeUs_t mspVtxLastTimeUs = 0;
static bool prevLowPowerDisarmedState = false;

static const vtxVTable_t mspVTable; // forward
static vtxDevice_t vtxMsp = {
    .vTable = &mspVTable,
};

STATIC_UNIT_TESTED mspVtxStatus_e mspVtxStatus = MSP_VTX_STATUS_OFFLINE;
static uint8_t mspVtxPortIdentifier = 255;

#define MSP_VTX_REQUEST_PERIOD_US (200 * 1000) // 200ms

static bool isCrsfPortConfig(const serialPortConfig_t *portConfig)
{
    return portConfig->functionMask & FUNCTION_RX_SERIAL && portConfig->functionMask & FUNCTION_VTX_MSP && rxRuntimeState.serialrxProvider == SERIALRX_CRSF;
}

static bool isLowPowerDisarmed(void)
{
    return (!ARMING_FLAG(ARMED) && !failsafeIsActive() &&
        (vtxSettingsConfig()->lowPowerDisarm == VTX_LOW_POWER_DISARM_ALWAYS ||
        (vtxSettingsConfig()->lowPowerDisarm == VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM && !ARMING_FLAG(WAS_EVER_ARMED))));
}

void setMspVtxDeviceStatusReady(const int descriptor)
{
    if (mspVtxStatus != MSP_VTX_STATUS_READY && vtxTableConfig()->bands && vtxTableConfig()->channels && vtxTableConfig()->powerLevels) {
#if defined(USE_MSP_OVER_TELEMETRY)
        if (getMspSerialPortDescriptor(mspVtxPortIdentifier) == descriptor || getMspTelemetryDescriptor() == descriptor) {
#else
        if (getMspSerialPortDescriptor(mspVtxPortIdentifier) == descriptor) {
#endif
            mspVtxStatus = MSP_VTX_STATUS_READY;
        }
    }
}

void prepareMspFrame(uint8_t *mspFrame)
{
    mspFrame[0] = VTXDEV_MSP;
    mspFrame[1] = vtxSettingsConfig()->band;
    mspFrame[2] = vtxSettingsConfig()->channel;
    mspFrame[3] = isLowPowerDisarmed() ? 1 : vtxSettingsConfig()->power; // index based
    mspFrame[4] = mspConfPitMode;
    mspFrame[5] = vtxSettingsConfig()->freq & 0xFF;
    mspFrame[6] = (vtxSettingsConfig()->freq >> 8) & 0xFF;
    mspFrame[7] = (mspVtxStatus == MSP_VTX_STATUS_READY) ? 1 : 0;
    mspFrame[8] = vtxSettingsConfig()->lowPowerDisarm;
    mspFrame[9] = vtxSettingsConfig()->pitModeFreq & 0xFF;
    mspFrame[10] = (vtxSettingsConfig()->pitModeFreq >> 8) & 0xFF;
#ifdef USE_VTX_TABLE
    mspFrame[11] = 1;
    mspFrame[12] = vtxTableConfig()->bands;
    mspFrame[13] = vtxTableConfig()->channels;
    mspFrame[14] = vtxTableConfig()->powerLevels;
#else
    mspFrame[11] = 0;
    mspFrame[12] = 0;
    mspFrame[13] = 0;
    mspFrame[14] = 0;
#endif
}

static void mspCrsfPush(const uint8_t mspCommand, const uint8_t *mspFrame, const uint8_t mspFrameSize)
{
#ifndef USE_TELEMETRY_CRSF
    UNUSED(mspCommand);
    UNUSED(mspFrame);
    UNUSED(mspFrameSize);
#else
    sbuf_t crsfPayloadBuf;
    sbuf_t *dst = &crsfPayloadBuf;

    uint8_t mspHeader[6] = {0x50, 0, mspCommand & 0xFF, (mspCommand >> 8) & 0xFF, mspFrameSize & 0xFF, (mspFrameSize >> 8) & 0xFF }; // MSP V2 over CRSF header
    uint8_t mspChecksum;

    mspChecksum = crc8_dvb_s2_update(0, &mspHeader[1], 5); // first character is not checksummable
    mspChecksum = crc8_dvb_s2_update(mspChecksum, mspFrame, mspFrameSize);

    uint8_t fullMspFrameSize = mspFrameSize + sizeof(mspHeader) + 1;  // add 1 for msp checksum
    uint8_t crsfFrameSize = CRSF_FRAME_LENGTH_EXT_TYPE_CRC + CRSF_FRAME_LENGTH_TYPE_CRC + fullMspFrameSize;

    uint8_t crsfFrame[crsfFrameSize];

    dst->ptr = crsfFrame;
    dst->end = ARRAYEND(crsfFrame);

    sbufWriteU8(dst, CRSF_SYNC_BYTE);
    sbufWriteU8(dst, fullMspFrameSize + CRSF_FRAME_LENGTH_EXT_TYPE_CRC); // size of CRSF frame (everything except sync and size itself)
    sbufWriteU8(dst, CRSF_FRAMETYPE_MSP_RESP); // CRSF type
    sbufWriteU8(dst, CRSF_ADDRESS_CRSF_RECEIVER); // response destination is the receiver the vtx connection
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER); // origin is always this device
    sbufWriteData(dst, mspHeader, sizeof(mspHeader));
    sbufWriteData(dst, mspFrame, mspFrameSize);
    sbufWriteU8(dst, mspChecksum);
    crc8_dvb_s2_sbuf_append(dst, &crsfFrame[2]); // start at byte 2, since CRC does not include device address and frame length
    sbufSwitchToReader(dst, crsfFrame);

    crsfRxSendTelemetryData(); //give the FC a chance to send outstanding telemetry
    crsfRxWriteTelemetryData(sbufPtr(dst), sbufBytesRemaining(dst));
    crsfRxSendTelemetryData();
#endif
}

static uint16_t packetCounter = 0;

static void vtxMspProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs)
{
    UNUSED(vtxDevice);

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_MSP);
    uint8_t frame[15];

    switch (mspVtxStatus) {
    case MSP_VTX_STATUS_OFFLINE:
        // wait for MSP communication from the VTX
#ifdef USE_CMS
        mspCmsUpdateStatusString();
#endif
        break;
    case MSP_VTX_STATUS_READY:
        if (isLowPowerDisarmed() != prevLowPowerDisarmedState) {
            mspVtxConfigChanged = true;
            prevLowPowerDisarmedState = isLowPowerDisarmed();
        }

        // send an update if stuff has changed with 200ms period
        if (mspVtxConfigChanged && cmp32(currentTimeUs, mspVtxLastTimeUs) >= MSP_VTX_REQUEST_PERIOD_US) {

            prepareMspFrame(frame);

            if (isCrsfPortConfig(portConfig)) {
                mspCrsfPush(MSP_VTX_CONFIG, frame, sizeof(frame));
            } else {
                mspSerialPush((serialPortIdentifier_e) portConfig->identifier, MSP_VTX_CONFIG, frame, sizeof(frame), MSP_DIRECTION_REPLY, MSP_V2_NATIVE);
            }
            packetCounter++;
            mspVtxLastTimeUs = currentTimeUs;
            mspVtxConfigChanged = false;

#ifdef USE_CMS
            mspCmsUpdateStatusString();
#endif
        }
        break;
    default:
        mspVtxStatus = MSP_VTX_STATUS_OFFLINE;
        break;
    }

    DEBUG_SET(DEBUG_VTX_MSP, 0, packetCounter);
    DEBUG_SET(DEBUG_VTX_MSP, 1, isCrsfPortConfig(portConfig));
    DEBUG_SET(DEBUG_VTX_MSP, 2, isLowPowerDisarmed());
#if defined(USE_MSP_OVER_TELEMETRY)
    DEBUG_SET(DEBUG_VTX_MSP, 3, isCrsfPortConfig(portConfig) ? getMspTelemetryDescriptor() : getMspSerialPortDescriptor(mspVtxPortIdentifier));
#else
    DEBUG_SET(DEBUG_VTX_MSP, 3, getMspSerialPortDescriptor(mspVtxPortIdentifier));
#endif
}

static vtxDevType_e vtxMspGetDeviceType(const vtxDevice_t *vtxDevice)
{
    UNUSED(vtxDevice);
    return VTXDEV_MSP;
}

static bool vtxMspIsReady(const vtxDevice_t *vtxDevice)
{
    return vtxDevice != NULL && mspVtxStatus == MSP_VTX_STATUS_READY;
}

static void vtxMspSetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    UNUSED(vtxDevice);
    if (band != mspConfBand || channel != mspConfChannel) {
        mspVtxConfigChanged = true;
    }
    mspConfBand = band;
    mspConfChannel = channel;
}

static void vtxMspSetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    uint16_t powerValue;

    if (vtxCommonLookupPowerValue(vtxDevice, index, &powerValue)) {
        if (powerValue != mspConfPower) {
            mspVtxConfigChanged = true;
        }
        mspConfPower = powerValue;
    }
}

static void vtxMspSetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    UNUSED(vtxDevice);
    if (onoff != mspConfPitMode) {
        mspVtxConfigChanged = true;
    }
    mspConfPitMode = onoff;
}

static void vtxMspSetFreq(vtxDevice_t *vtxDevice, uint16_t freq)
{
    UNUSED(vtxDevice);
    if (freq != mspConfFreq) {
        mspVtxConfigChanged = true;
    }
    mspConfFreq = freq;
}

static bool vtxMspGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    if (!vtxMspIsReady(vtxDevice)) {
        return false;
    }

    *pBand = mspConfBand;
    *pChannel = mspConfChannel;
    return true;
}

static bool vtxMspGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    if (!vtxMspIsReady(vtxDevice)) {
        return false;
    }

    // Special case, power not set
    if (mspConfPower == 0) {
        *pIndex = 0;
        return true;
    }

    // Lookup value in table
    for (uint8_t i = 0; i < vtxTablePowerLevels; i++) {
        // Find value that matches current configured power level
        if (mspConfPower == vtxTablePowerValues[i]) {
            // Value found, return index
            *pIndex = i + 1;
            return true;
        }
    }

    // Value not found in table
    return false;
}

static bool vtxMspGetFreq(const vtxDevice_t *vtxDevice, uint16_t *pFreq)
{
    if (!vtxMspIsReady(vtxDevice)) {
        return false;
    }

    *pFreq = vtxCommonLookupFrequency(vtxDevice, mspConfBand, mspConfChannel);
    return true;
}

static bool vtxMspGetStatus(const vtxDevice_t *vtxDevice, unsigned *status)
{
    if (!vtxMspIsReady(vtxDevice)) {
        return false;
    }

    // Mirror configued pit mode state rather than use current pitmode as we
    // should, otherwise the logic in vtxProcessPitMode may not get us to the
    // correct state if pitmode is toggled quickly
    *status = (mspConfPitMode ? VTX_STATUS_PIT_MODE : 0);

    return true;
}

static uint8_t vtxMspGetPowerLevels(const vtxDevice_t *vtxDevice, uint16_t *levels, uint16_t *powers)
{
    if (!vtxMspIsReady(vtxDevice)) {
        return 0;
    }

    for (uint8_t i = 0; i < vtxTablePowerLevels; i++) {
        levels[i] = vtxTablePowerValues[i];
        uint16_t power = (uint16_t)powf(10.0f, levels[i] / 10.0f);
        powers[i] = power;
    }

    return vtxTablePowerLevels;
}

static const vtxVTable_t mspVTable = {
    .process = vtxMspProcess,
    .getDeviceType = vtxMspGetDeviceType,
    .isReady = vtxMspIsReady,
    .setBandAndChannel = vtxMspSetBandAndChannel,
    .setPowerByIndex = vtxMspSetPowerByIndex,
    .setPitMode = vtxMspSetPitMode,
    .setFrequency = vtxMspSetFreq,
    .getBandAndChannel = vtxMspGetBandAndChannel,
    .getPowerIndex = vtxMspGetPowerIndex,
    .getFrequency = vtxMspGetFreq,
    .getStatus = vtxMspGetStatus,
    .getPowerLevels = vtxMspGetPowerLevels,
    .serializeCustomDeviceStatus = NULL,
};

bool vtxMspInit(void)
{
    // don't bother setting up this device if we don't have MSP vtx enabled
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_MSP);
    if (!portConfig) {
        return false;
    }

    mspVtxPortIdentifier = portConfig->identifier;

    // XXX Effect of USE_VTX_COMMON should be reviewed, as following call to vtxInit will do nothing if vtxCommonSetDevice is not called.
#if defined(USE_VTX_COMMON)
    vtxCommonSetDevice(&vtxMsp);
#endif

    vtxInit();

    return true;
}

#endif
