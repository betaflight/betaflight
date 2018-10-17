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

/* Created by jflyper */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#if defined(USE_VTX_BEESIGN) && defined(USE_VTX_CONTROL)

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "cms/cms_menu_vtx_tramp.h"

#include "drivers/vtx_common.h"

#include "io/serial.h"
#include "io/vtx_tramp.h"
#include "io/vtx_control.h"
#include "io/vtx.h"
#include "io/vtx_string.h"
#include "io/vtx_beesign.h"

static serialPort_t *beesignSerialPort = NULL;

#if defined(USE_CMS) || defined(USE_VTX_COMMON)

const uint16_t bsPowerTable[VTX_BEESIGN_POWER_COUNT - VTX_BEESIGN_DEFAULT_POWER] = {
    25, 100, 200, 400, 600
};

const char * const bsPowerNames[] = {
    "OFF",  "25", "100", "200", "400", "600", "PIT MODE"
};

const char * const bsModeNames[] = {
    "RACE", "MANUAL", "POR"
};
#endif

#ifdef USE_VTX_COMMON
static const vtxVTable_t bsVTable;    // Forward
static vtxDevice_t vtxBeesign = {
    .vTable = &bsVTable,
    .capability.bandCount = VTX_BEESIGN_BAND_COUNT,
    .capability.channelCount = VTX_BEESIGN_CHANNEL_COUNT,
    .capability.powerCount = VTX_BEESIGN_POWER_COUNT,
    .bandNames = (char **)vtx58BandNames,
    .channelNames = (char **)vtx58ChannelNames,
    .powerNames = (char **)bsPowerNames,
};
#endif

#define WAIT_TX_COMPLETE()      while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)

#define BS_DEVICE_CHVAL_TO_BAND(val) ((val) / (uint8_t)8)
#define BS_DEVICE_CHVAL_TO_CHANNEL(val) ((val) % (uint8_t)8)
#define BS_BANDCHAN_TO_DEVICE_CHVAL(band, channel) ((band) * (uint8_t)8 + (channel))

typedef enum {
    BS_STATE_HDR = 0,
    BS_STATE_TYPE,
    BS_STATE_LEN,
    BS_STATE_PAYLOAD,
    BS_STATE_CRC
} bsState_e;


beeSignDevice_t bsDevice = {
    .version = 1,
    .channel = -1,
    .power = -1,
    .mode = 0,
    .freq = 0,
    .porfreq = 0,
};

static uint8_t receiveBuffer[BEESIGN_FM_MAX_LEN];
static uint8_t receiveFrame[BEESIGN_FM_MAX_LEN];
static uint8_t receiveFrameValid = 0;

#define POLY                    (0xB2)
#define CALC_CRC(crc, data)     do {                                                        \
                                    crc ^= (data);                                          \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                } while (0)

static uint8_t beesignCRC(const beesign_frame_t *pPackage) {
    uint8_t i;
    uint8_t crc = 0;

    CALC_CRC(crc, pPackage->hdr);
    CALC_CRC(crc, pPackage->type);
    CALC_CRC(crc, pPackage->len);
    for (i = 0; i < pPackage->len; i++) {
        CALC_CRC(crc, *(pPackage->payload + i));
    }
    return crc;
}

static uint8_t beesignChkID(uint8_t id) {
    UNUSED(id);
    return BEESIGN_OK;
}

uint8_t beesignReceive(uint8_t **pRcvFrame) {
    if (!receiveFrameValid) {
        (void)pRcvFrame;
        return BEESIGN_ERROR;
    }
    *pRcvFrame = receiveFrame;
    receiveFrameValid = 0;
    return BEESIGN_OK;
}

static uint8_t BSProcessResponse(uint8_t type ,uint8_t *pRcvFrame)
{
    if (beesignReceive(&pRcvFrame) != BEESIGN_ERROR) {
        if (BEESIGN_PKT_TYPE(pRcvFrame) == (type | BEESIGN_TYPE_MASK_FM)) {
            switch (BEESIGN_PKT_TYPE(pRcvFrame)) {
                case BEESIGN_V_GET_STATUS | BEESIGN_TYPE_MASK_FM:
                    bsDevice.channel = BEESIGN_PKT_DATA(pRcvFrame, 0);
                    bsDevice.freq = (uint16_t)BEESIGN_PKT_DATA(pRcvFrame, 2) + (BEESIGN_PKT_DATA(pRcvFrame, 1) << 8);
                    bsDevice.power = BEESIGN_PKT_DATA(pRcvFrame, 0);
                    bsDevice.mode = BEESIGN_PKT_DATA(pRcvFrame, 0);
                    break;
                case BEESIGN_O_GET_STATUS | BEESIGN_TYPE_MASK_FM:

                    break;
                case BEESIGN_A_GET_STATUS | BEESIGN_TYPE_MASK_FM:

                    break;
                case BEESIGN_S_GET_STATUS | BEESIGN_TYPE_MASK_FM:

                    break;
                default :
                    break;
            }
            return 0;
        }
    }
    return 1;
}

static void bsReceiveFramer(uint8_t ch) {
    static bsState_e state = BS_STATE_HDR;
    static uint8_t idx;
    static uint8_t len;
    switch (state) {
        case BS_STATE_HDR:
            if (ch == BEESIGN_HDR) {
                state = BS_STATE_TYPE;
                idx = 0;
                receiveBuffer[idx++] = ch;
            }
            break;
        case BS_STATE_TYPE:
            state = BS_STATE_LEN;
            receiveBuffer[idx++] = ch;
            break;
        case BS_STATE_LEN:
            if (ch <= BEESIGN_PL_MAX_LEN) {
                state = BS_STATE_PAYLOAD;
                len = ch;
                receiveBuffer[idx++] = ch;
            } else {
                state = BS_STATE_HDR;
            }
            break;
        case BS_STATE_PAYLOAD:
            receiveBuffer[idx++] = ch;
            if (--len == 0) {
                state = BS_STATE_CRC;
            }
            break;
        case BS_STATE_CRC:
            do {
                beesign_frame_t package = { .hdr = receiveBuffer[0],
                                            .type = receiveBuffer[1],
                                            .len = receiveBuffer[2],
                                            .payload = &receiveBuffer[3] };
                package.crc = beesignCRC(&package);
                if (package.crc == ch) {
                    receiveFrameValid = 1;
                    memcpy(receiveFrame, receiveBuffer, receiveBuffer[2]+3);
                    BSProcessResponse(package.type, receiveFrame);
                }
            } while (0);
            state = BS_STATE_HDR;
            break;
        default:
            state = BS_STATE_HDR;
            break;
    }
}

uint8_t beesignSend(uint8_t id, uint8_t len, uint8_t *pData) {
    beesign_frame_t package = { .hdr = BEESIGN_HDR,
                                .type = id,
                                .len = len,
                                .payload = pData };

    if ((len >= BEESIGN_PL_MAX_LEN) ||
        (pData == 0) ||
        (beesignChkID(id) != BEESIGN_OK)) {
        return BEESIGN_ERROR;
    }

    package.crc = beesignCRC(&package);

    serialWrite(beesignSerialPort, package.hdr);
    serialWrite(beesignSerialPort, package.type);
    serialWrite(beesignSerialPort, package.len);
    for (uint8_t i = 0; i < len; i++) {
        serialWrite(beesignSerialPort, *(package.payload + i));
    }
    serialWrite(beesignSerialPort, package.crc);

    return BEESIGN_OK;
}

void bsSetVTxUnlock(void) {
    uint16_t unlock = BEESIGN_VTX_UNLOCK;
    uint8_t unlockData[2] = {unlock >> 8, unlock};
    beesignSend(BEESIGN_V_UNLOCK, 2, unlockData);
    beesignSend(BEESIGN_M_SAVE_SETTING, 1, 0);
}

void bsSetVTxLock(void) {
    uint16_t lock = BEESIGN_VTX_LOCK;
    uint8_t lockData[2] = {lock >> 8, lock};
    beesignSend(BEESIGN_V_LOCK, 2, lockData);
    beesignSend(BEESIGN_M_SAVE_SETTING, 1, 0);
}

static bool bsValidateBandAndChannel(uint8_t band, uint8_t channel)
{
    return (band >= VTX_BEESIGN_MIN_BAND && band <= VTX_BEESIGN_MAX_BAND &&
             channel >= VTX_BEESIGN_MIN_CHANNEL && channel <= VTX_BEESIGN_MAX_CHANNEL);
}

void bsSetBandAndChannel(uint8_t band, uint8_t channel)
{
    uint8_t deviceChannel = BS_BANDCHAN_TO_DEVICE_CHVAL(band, channel);
    bsDevice.channel = deviceChannel;
    bsDevice.freq = vtx58frequencyTable[band][channel];
    beesignSend(BEESIGN_V_SET_CHAN, 1, &deviceChannel);
}

void bsSetPower(uint8_t index)
{

    if (index > VTX_BEESIGN_POWER_COUNT) {
        return;
    }
    bsDevice.power = index;
    index -= 1;
    beesignSend(BEESIGN_V_SET_PWR, 1, &index);
}

void bsSetMode(uint8_t mode)
{
    if (mode > 2) return;
    bsDevice.mode = mode;
    beesignSend(BEESIGN_V_SET_MODE, 1, &mode);
}

static bool bsValidateFreq(uint16_t freq)
{
    return (freq >= VTX_BEESIGN_MIN_FREQUENCY_MHZ && freq <= VTX_BEESIGN_MAX_FREQUENCY_MHZ);
}

void bsSetFreq(uint16_t freq)
{
    uint8_t buf[2];

    buf[0] = (freq >> 8) & 0xff;
    buf[1] = freq & 0xff;
    bsDevice.freq = freq;
    bsDevice.channel = VTX_BEESIGN_ERROR_CHANNEL;
    beesignSend(BEESIGN_V_SET_FREQ, 2, buf);
}

void bsGetVtxState(void) {
    uint8_t buf = '0';
    beesignSend(BEESIGN_V_GET_STATUS, 1, &buf);
}

bool beesignInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_BEESIGN);
    if (portConfig) {
        beesignSerialPort = openSerialPort(portConfig->identifier, FUNCTION_NONE, NULL, NULL, 115200, MODE_RXTX, SERIAL_BIDIR);
    }

    if (!beesignSerialPort) {
        return false;
    }

    vtxCommonSetDevice(&vtxBeesign);
    // bsSetVTxUnlock();
    return true;
}

void vtxBSProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs)
{
    UNUSED(vtxDevice);
    UNUSED(currentTimeUs);

    while (serialRxBytesWaiting(beesignSerialPort) > 0) {
        const uint8_t ch = serialRead(beesignSerialPort);
        bsReceiveFramer(ch);
    }
}

#ifdef USE_VTX_COMMON
// Interface to common VTX API

vtxDevType_e vtxBSGetDeviceType(const vtxDevice_t *vtxDevice)
{
    UNUSED(vtxDevice);
    return VTXDEV_BEESIGN;
}

static bool vtxBSIsReady(const vtxDevice_t *vtxDevice)
{
    return vtxDevice!=NULL && !(bsDevice.version == 0);
}

static void vtxBSSetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    UNUSED(vtxDevice);
    if (bsValidateBandAndChannel(band, channel)) {
        bsSetBandAndChannel(band - 1, channel - 1);
    }
}

static void vtxBSSetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    UNUSED(vtxDevice);
    if (onoff) {
        bsDevice.lastPower = bsDevice.power;
        vtxSettingsConfigMutable()->power = VTX_PWR_PIT + VTX_BEESIGN_MIN_POWER;  // pit mode
    } else {
        vtxSettingsConfigMutable()->power = bsDevice.lastPower;
    }
}


static void vtxBSSetPowerIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    UNUSED(vtxDevice);
    if (index == 0) {
        // beesign doesn't support power off.
        return;
    }
    bsSetPower(index);

}

static void vtxBSSetFreq(vtxDevice_t *vtxDevice, uint16_t freq)
{
    UNUSED(vtxDevice);
    if (bsValidateFreq(freq)) {
        bsSetFreq(freq);
    }
}

static bool vtxBSGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    if (!vtxBSIsReady(vtxDevice)) {
        return false;
    }

    // if in user-freq mode then report band as zero
    *pBand = (bsDevice.mode == BEESIGN_VTX_MODE_RACE) ?
        (BS_DEVICE_CHVAL_TO_BAND(bsDevice.channel) + 1) : 0;
    *pChannel = BS_DEVICE_CHVAL_TO_CHANNEL(bsDevice.channel) + 1;
    return true;
}

static bool vtxBSGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    if (!vtxBSIsReady(vtxDevice)) {
        return false;
    }

    *pIndex = bsDevice.power;
    return true;
}

static bool vtxBSGetPitMode(const vtxDevice_t *vtxDevice, uint8_t *pOnOff)
{
    if (!vtxBSIsReady(vtxDevice)) {
        return false;
    }
    if (bsDevice.power == VTX_PWR_PIT + VTX_BEESIGN_MIN_POWER) {
        *pOnOff = 1;
    } else {
        *pOnOff = 0;
    }
    return true;
}

static bool vtxBSGetFreq(const vtxDevice_t *vtxDevice, uint16_t *pFreq)
{
    if (!vtxBSIsReady(vtxDevice)) {
        return false;
    }

    // // if not in user-freq mode then convert band/chan to frequency
    *pFreq = bsDevice.freq;
    return true;
}

static const vtxVTable_t bsVTable = {
    .process = vtxBSProcess,
    .getDeviceType = vtxBSGetDeviceType,
    .isReady = vtxBSIsReady,
    .setBandAndChannel = vtxBSSetBandAndChannel,
    .setPowerByIndex = vtxBSSetPowerIndex,
    .setPitMode = vtxBSSetPitMode,
    .setFrequency = vtxBSSetFreq,
    .getBandAndChannel = vtxBSGetBandAndChannel,
    .getPowerIndex = vtxBSGetPowerIndex,
    .getPitMode = vtxBSGetPitMode,
    .getFrequency = vtxBSGetFreq,
};
#endif // VTX_COMMON

#endif // USE_VTX_BEESIGN
