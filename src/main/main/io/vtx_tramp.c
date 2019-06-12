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

#if defined(USE_VTX_TRAMP) && defined(USE_VTX_CONTROL)

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "cms/cms_menu_vtx_tramp.h"

#include "drivers/vtx_common.h"

#ifdef USE_VTX_TABLE
#include "drivers/vtx_table.h"
#endif

#include "io/serial.h"
#include "io/vtx_tramp.h"
#include "io/vtx_control.h"
#include "io/vtx.h"
#include "io/vtx_string.h"

#if defined(USE_CMS) || defined(USE_VTX_COMMON)
const uint16_t trampPowerTable[VTX_TRAMP_POWER_COUNT] = {
    25, 100, 200, 400, 600
};

const char * trampPowerNames[VTX_TRAMP_POWER_COUNT+1] = {
    "---", "25 ", "100", "200", "400", "600"
};
#endif

#if defined(USE_VTX_COMMON)
static const vtxVTable_t trampVTable; // forward
static vtxDevice_t vtxTramp = {
    .vTable = &trampVTable,
};
#endif

static serialPort_t *trampSerialPort = NULL;

static uint8_t trampReqBuffer[16];
static uint8_t trampRespBuffer[16];

typedef enum {
    TRAMP_STATUS_BAD_DEVICE = -1,
    TRAMP_STATUS_OFFLINE = 0,
    TRAMP_STATUS_ONLINE,
    TRAMP_STATUS_SET_FREQ_PW,
    TRAMP_STATUS_CHECK_FREQ_PW
} trampStatus_e;

trampStatus_e trampStatus = TRAMP_STATUS_OFFLINE;

uint32_t trampRFFreqMin;
uint32_t trampRFFreqMax;
uint32_t trampRFPowerMax;

bool trampSetByFreqFlag = false;  //false = set via band/channel
uint32_t trampCurFreq = 0;
uint8_t trampBand = 0;
uint8_t trampChannel = 0;
uint16_t trampPower = 0;       // Actual transmitting power
uint16_t trampConfiguredPower = 0; // Configured transmitting power
int16_t trampTemperature = 0;
uint8_t trampPitMode = 0;

// Maximum number of requests sent to try a config change
#define TRAMP_MAX_RETRIES 2

uint32_t trampConfFreq = 0;
uint8_t  trampFreqRetries = 0;

uint16_t trampConfPower = 0;
uint8_t  trampPowerRetries = 0;

static void trampWriteBuf(uint8_t *buf)
{
    serialWriteBuf(trampSerialPort, buf, 16);
}

static uint8_t trampChecksum(uint8_t *trampBuf)
{
    uint8_t cksum = 0;

    for (int i = 1 ; i < 14 ; i++) {
        cksum += trampBuf[i];
    }

    return cksum;
}

static void trampCmdU16(uint8_t cmd, uint16_t param)
{
    if (!trampSerialPort || IS_RC_MODE_ACTIVE(BOXVTXCONTROLDISABLE)) {
        return;
    }

    memset(trampReqBuffer, 0, ARRAYLEN(trampReqBuffer));
    trampReqBuffer[0] = 15;
    trampReqBuffer[1] = cmd;
    trampReqBuffer[2] = param & 0xff;
    trampReqBuffer[3] = (param >> 8) & 0xff;
    trampReqBuffer[14] = trampChecksum(trampReqBuffer);
    trampWriteBuf(trampReqBuffer);
}

static bool trampValidateFreq(uint16_t freq)
{
    return (freq >= VTX_TRAMP_MIN_FREQUENCY_MHZ && freq <= VTX_TRAMP_MAX_FREQUENCY_MHZ);
}

static void trampDevSetFreq(uint16_t freq)
{
    trampConfFreq = freq;
    if (trampConfFreq != trampCurFreq) {
        trampFreqRetries = TRAMP_MAX_RETRIES;
    }
}

void trampSetFreq(uint16_t freq)
{
    trampSetByFreqFlag = true;         //set freq via MHz value
    trampDevSetFreq(freq);
}

void trampSendFreq(uint16_t freq)
{
    trampCmdU16('F', freq);
}

static bool trampValidateBandAndChannel(uint8_t band, uint8_t channel)
{
    return (band >= VTX_TRAMP_MIN_BAND && band <= VTX_TRAMP_MAX_BAND &&
            channel >= VTX_TRAMP_MIN_CHANNEL && channel <= VTX_TRAMP_MAX_CHANNEL);
}

static void trampDevSetBandAndChannel(uint8_t band, uint8_t channel)
{
    trampDevSetFreq(vtxCommonLookupFrequency(&vtxTramp, band, channel));
}

void trampSetBandAndChannel(uint8_t band, uint8_t channel)
{
    trampSetByFreqFlag = false;        //set freq via band/channel
    trampDevSetBandAndChannel(band, channel);
}

void trampSetRFPower(uint16_t level)
{
    trampConfPower = level;
    if (trampConfPower != trampPower) {
        trampPowerRetries = TRAMP_MAX_RETRIES;
    }
}

void trampSendRFPower(uint16_t level)
{
    trampCmdU16('P', level);
}

// return false if error
bool trampCommitChanges(void)
{
    if (trampStatus != TRAMP_STATUS_ONLINE) {
        return false;
    }

    trampStatus = TRAMP_STATUS_SET_FREQ_PW;
    return true;
}

// return false if index out of range
static bool trampDevSetPowerByIndex(uint8_t index)
{
    if (index > 0 && index <= vtxTramp.capability.powerCount) {
        trampSetRFPower(vtxTramp.powerValues[index - 1]);
        trampCommitChanges();
        return true;
    }
    return false;
}

void trampSetPitMode(uint8_t onoff)
{
    trampCmdU16('I', onoff ? 0 : 1);
}

// returns completed response code
static char trampHandleResponse(void)
{
    const uint8_t respCode = trampRespBuffer[1];

    switch (respCode) {
    case 'r':
        {
            const uint16_t min_freq = trampRespBuffer[2]|(trampRespBuffer[3] << 8);
            if (min_freq != 0) {
                trampRFFreqMin = min_freq;
                trampRFFreqMax = trampRespBuffer[4]|(trampRespBuffer[5] << 8);
                trampRFPowerMax = trampRespBuffer[6]|(trampRespBuffer[7] << 8);
                return 'r';
            }

            // throw bytes echoed from tx to rx in bidirectional mode away
        }
        break;

    case 'v':
        {
            const uint16_t freq = trampRespBuffer[2]|(trampRespBuffer[3] << 8);
            if (freq != 0) {
                trampCurFreq = freq;
                trampConfiguredPower = trampRespBuffer[4]|(trampRespBuffer[5] << 8);
                trampPitMode = trampRespBuffer[7];
                trampPower = trampRespBuffer[8]|(trampRespBuffer[9] << 8);

                // if no band/chan match then make sure set-by-freq mode is flagged
                if (!vtxCommonLookupBandChan(&vtxTramp, trampCurFreq, &trampBand, &trampChannel)) {
                    trampSetByFreqFlag = true;
                }

                if (trampConfFreq == 0)  trampConfFreq  = trampCurFreq;
                if (trampConfPower == 0) trampConfPower = trampPower;
                return 'v';
            }

            // throw bytes echoed from tx to rx in bidirectional mode away
        }
        break;

    case 's':
        {
            const uint16_t temp = (int16_t)(trampRespBuffer[6]|(trampRespBuffer[7] << 8));
            if (temp != 0) {
                trampTemperature = temp;
                return 's';
            }
        }
        break;
    }

    return 0;
}

typedef enum {
    S_WAIT_LEN = 0,   // Waiting for a packet len
    S_WAIT_CODE,      // Waiting for a response code
    S_DATA,           // Waiting for rest of the packet.
} trampReceiveState_e;

static trampReceiveState_e trampReceiveState = S_WAIT_LEN;
static int trampReceivePos = 0;

static void trampResetReceiver(void)
{
    trampReceiveState = S_WAIT_LEN;
    trampReceivePos = 0;
}

static bool trampIsValidResponseCode(uint8_t code)
{
    if (code == 'r' || code == 'v' || code == 's') {
        return true;
    } else {
        return false;
    }
}

// returns completed response code or 0
static char trampReceive(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!trampSerialPort) {
        return 0;
    }

    while (serialRxBytesWaiting(trampSerialPort)) {
        const uint8_t c = serialRead(trampSerialPort);
        trampRespBuffer[trampReceivePos++] = c;

        switch (trampReceiveState) {
        case S_WAIT_LEN:
            if (c == 0x0F) {
                trampReceiveState = S_WAIT_CODE;
            } else {
                trampReceivePos = 0;
            }
            break;

        case S_WAIT_CODE:
            if (trampIsValidResponseCode(c)) {
                trampReceiveState = S_DATA;
            } else {
                trampResetReceiver();
            }
            break;

        case S_DATA:
            if (trampReceivePos == 16) {
                uint8_t cksum = trampChecksum(trampRespBuffer);

                trampResetReceiver();

                if ((trampRespBuffer[14] == cksum) && (trampRespBuffer[15] == 0) && !IS_RC_MODE_ACTIVE(BOXVTXCONTROLDISABLE)) {
                    return trampHandleResponse();
                }
            }
            break;

        default:
            trampResetReceiver();
            break;
        }
    }

    return 0;
}

void trampQuery(uint8_t cmd)
{
    trampResetReceiver();
    trampCmdU16(cmd, 0);
}

void trampQueryR(void)
{
    trampQuery('r');
}

void trampQueryV(void)
{
    trampQuery('v');
}

void trampQueryS(void)
{
    trampQuery('s');
}

static void vtxTrampProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs)
{
    UNUSED(vtxDevice);

    static timeUs_t lastQueryTimeUs = 0;
    static bool initSettingsDoneFlag = false;

#ifdef TRAMP_DEBUG
    static uint16_t debugFreqReqCounter = 0;
    static uint16_t debugPowReqCounter = 0;
#endif

    if (trampStatus == TRAMP_STATUS_BAD_DEVICE) {
        return;
    }

    const char replyCode = trampReceive(currentTimeUs);

#ifdef TRAMP_DEBUG
    debug[0] = trampStatus;
#endif

    switch (replyCode) {
    case 'r':
        if (trampStatus <= TRAMP_STATUS_OFFLINE) {
            trampStatus = TRAMP_STATUS_ONLINE;

            // once device is ready enter vtx settings
            if (!initSettingsDoneFlag) {
                initSettingsDoneFlag = true;
                // if vtx_band!=0 then enter 'vtx_band/chan' values (and power)
            }
        }
        break;

    case 'v':
         if (trampStatus == TRAMP_STATUS_CHECK_FREQ_PW) {
             trampStatus = TRAMP_STATUS_SET_FREQ_PW;
         }
         break;
    }

    switch (trampStatus) {

    case TRAMP_STATUS_OFFLINE:
    case TRAMP_STATUS_ONLINE:
        if (cmp32(currentTimeUs, lastQueryTimeUs) > 1000 * 1000) { // 1s

            if (trampStatus == TRAMP_STATUS_OFFLINE) {
                trampQueryR();
            } else {
                static unsigned int cnt = 0;
                if (((cnt++) & 1) == 0) {
                    trampQueryV();
                } else {
                    trampQueryS();
                }
            }

            lastQueryTimeUs = currentTimeUs;
        }
        break;

    case TRAMP_STATUS_SET_FREQ_PW:
        {
            bool done = true;
            if (trampConfFreq && trampFreqRetries && (trampConfFreq != trampCurFreq)) {
                trampSendFreq(trampConfFreq);
                trampFreqRetries--;
#ifdef TRAMP_DEBUG
                debugFreqReqCounter++;
#endif
                done = false;
            } else if (trampConfPower && trampPowerRetries && (trampConfPower != trampConfiguredPower)) {
                trampSendRFPower(trampConfPower);
                trampPowerRetries--;
#ifdef TRAMP_DEBUG
                debugPowReqCounter++;
#endif
                done = false;
            }

            if (!done) {
                trampStatus = TRAMP_STATUS_CHECK_FREQ_PW;

                // delay next status query by 300ms
                lastQueryTimeUs = currentTimeUs + 300 * 1000;
            } else {
                // everything has been done, let's return to original state
                trampStatus = TRAMP_STATUS_ONLINE;
                // reset configuration value in case it failed (no more retries)
                trampConfFreq  = trampCurFreq;
                trampConfPower = trampPower;
                trampFreqRetries = trampPowerRetries = 0;
            }
        }
        break;

    case TRAMP_STATUS_CHECK_FREQ_PW:
        if (cmp32(currentTimeUs, lastQueryTimeUs) > 200 * 1000) {
            trampQueryV();
            lastQueryTimeUs = currentTimeUs;
        }
        break;

    default:
        break;
    }

#ifdef TRAMP_DEBUG
    debug[1] = debugFreqReqCounter;
    debug[2] = debugPowReqCounter;
    debug[3] = 0;
#endif

#ifdef USE_CMS
    trampCmsUpdateStatusString();
#endif
}


#ifdef USE_VTX_COMMON

// Interface to common VTX API

static vtxDevType_e vtxTrampGetDeviceType(const vtxDevice_t *vtxDevice)
{
    UNUSED(vtxDevice);
    return VTXDEV_TRAMP;
}

static bool vtxTrampIsReady(const vtxDevice_t *vtxDevice)
{
    return vtxDevice!=NULL && trampStatus > TRAMP_STATUS_OFFLINE;
}

static void vtxTrampSetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    UNUSED(vtxDevice);
    if (trampValidateBandAndChannel(band, channel)) {
        trampSetBandAndChannel(band, channel);
        trampCommitChanges();
    }
}

static void vtxTrampSetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    UNUSED(vtxDevice);
    trampDevSetPowerByIndex(index);
}

static void vtxTrampSetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    UNUSED(vtxDevice);
    trampSetPitMode(onoff);
}

static void vtxTrampSetFreq(vtxDevice_t *vtxDevice, uint16_t freq)
{
    UNUSED(vtxDevice);
    if (trampValidateFreq(freq)) {
        trampSetFreq(freq);
        trampCommitChanges();
    }
}

static bool vtxTrampGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    if (!vtxTrampIsReady(vtxDevice)) {
        return false;
    }

    // if in user-freq mode then report band as zero
    *pBand = trampSetByFreqFlag ? 0 : trampBand;
    *pChannel = trampChannel;
    return true;
}

static bool vtxTrampGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    if (!vtxTrampIsReady(vtxDevice)) {
        return false;
    }

    if (trampConfiguredPower > 0) {
        for (uint8_t i = 0; i < vtxTramp.capability.powerCount; i++) {
            if (trampConfiguredPower <= vtxTramp.powerValues[i]) {
                *pIndex = i + 1;
                break;
            }
        }
    }

    return true;
}

static bool vtxTrampGetPitMode(const vtxDevice_t *vtxDevice, uint8_t *pOnOff)
{
    if (!vtxTrampIsReady(vtxDevice)) {
        return false;
    }

    *pOnOff = trampPitMode;
    return true;
}

static bool vtxTrampGetFreq(const vtxDevice_t *vtxDevice, uint16_t *pFreq)
{
    if (!vtxTrampIsReady(vtxDevice)) {
        return false;
    }

    *pFreq = trampCurFreq;
    return true;
}

static const vtxVTable_t trampVTable = {
    .process = vtxTrampProcess,
    .getDeviceType = vtxTrampGetDeviceType,
    .isReady = vtxTrampIsReady,
    .setBandAndChannel = vtxTrampSetBandAndChannel,
    .setPowerByIndex = vtxTrampSetPowerByIndex,
    .setPitMode = vtxTrampSetPitMode,
    .setFrequency = vtxTrampSetFreq,
    .getBandAndChannel = vtxTrampGetBandAndChannel,
    .getPowerIndex = vtxTrampGetPowerIndex,
    .getPitMode = vtxTrampGetPitMode,
    .getFrequency = vtxTrampGetFreq,
};

#endif

bool vtxTrampInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_TRAMP);

    if (portConfig) {
        portOptions_e portOptions = 0;
#if defined(USE_VTX_COMMON)
        portOptions = portOptions | (vtxConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR);
#else
        portOptions = SERIAL_BIDIR;
#endif

        trampSerialPort = openSerialPort(portConfig->identifier, FUNCTION_VTX_TRAMP, NULL, NULL, 9600, MODE_RXTX, portOptions);
    }

    if (!trampSerialPort) {
        return false;
    }

    // XXX Effect of USE_VTX_COMMON should be reviewed, as following call to vtxInit will do nothing if vtxCommonSetDevice is not called.
#if defined(USE_VTX_COMMON)
#if defined(USE_VTX_TABLE)
    vtxTramp.capability.bandCount = vtxTableBandCount;
    vtxTramp.capability.channelCount = vtxTableChannelCount;
    vtxTramp.capability.powerCount = vtxTablePowerLevels;
    vtxTramp.frequencyTable = (uint16_t *)vtxTableFrequency;
    vtxTramp.bandNames = vtxTableBandNames;
    vtxTramp.bandLetters = vtxTableBandLetters;
    vtxTramp.channelNames = vtxTableChannelNames;
    vtxTramp.powerNames = vtxTablePowerLabels;
    vtxTramp.powerValues = vtxTablePowerValues;
#else
    vtxTramp.capability.bandCount = VTX_TRAMP_BAND_COUNT;
    vtxTramp.capability.channelCount = VTX_TRAMP_CHANNEL_COUNT;
    vtxTramp.capability.powerCount = sizeof(trampPowerTable),
    vtxTramp.frequencyTable = vtxStringFrequencyTable();
    vtxTramp.bandNames = vtxStringBandNames();
    vtxTramp.bandLetters = vtxStringBandLetters();
    vtxTramp.channelNames = vtxStringChannelNames();
    vtxTramp.powerNames = trampPowerNames;
    vtxTramp.powerValues = trampPowerTable;
#endif

    vtxCommonSetDevice(&vtxTramp);

#endif

    vtxInit();

    return true;
}

#endif // VTX_TRAMP
