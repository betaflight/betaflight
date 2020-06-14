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
#include "drivers/vtx_table.h"

#include "io/serial.h"
#include "io/vtx_tramp.h"
#include "io/vtx_control.h"
#include "io/vtx.h"

#if (defined(USE_CMS) || defined(USE_VTX_COMMON)) && !defined(USE_VTX_TABLE)
const uint16_t trampPowerTable[VTX_TRAMP_POWER_COUNT] = {
    25, 100, 200, 400, 600
};

const char *trampPowerNames[VTX_TRAMP_POWER_COUNT + 1] = {
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

static uint32_t trampRFFreqMin = 0;
static uint32_t trampRFFreqMax = 0;
static uint32_t trampRFPowerMax;

uint32_t trampCurFreq = 0;
static uint32_t trampLastFreq = 0;
uint16_t trampPower = 0;       // Actual transmitting power
uint16_t trampConfiguredPower = 0; // Configured transmitting power
static uint16_t trampLastPower = 0;
int16_t trampTemperature = 0;
uint8_t trampPitMode = 0;
static uint8_t trampControlMode = 0;

#define TRAMP_CONTROL_RACE_LOCK 0x01

// Maximum number of requests sent to try a config change
// Some VTX fail to respond to every request (like Matek FCHUB-VTX) so
// we sometimes need multiple retries to get the VTX to respond.
#define TRAMP_MAX_RETRIES 20

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

static bool trampVtxControlEnabled(void)
{
    return !IS_RC_MODE_ACTIVE(BOXVTXCONTROLDISABLE);
}

static bool trampVtxRaceLockEnabled(void)
{
    return trampControlMode & TRAMP_CONTROL_RACE_LOCK;
}

static void trampSendU16(uint8_t cmd, uint16_t param)
{
    if (!trampSerialPort) {
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

static void trampSendCommand(uint8_t cmd, uint16_t param)
{
    if (trampVtxControlEnabled()) {
        trampSendU16(cmd, param);
    }
}

static bool trampValidateFreq(uint16_t freq)
{
    if (trampRFFreqMin != 0 && trampRFFreqMax != 0) {
        return (freq >= trampRFFreqMin && freq <= trampRFFreqMax);
    } else {
        return (freq >= VTX_TRAMP_MIN_FREQUENCY_MHZ && freq <= VTX_TRAMP_MAX_FREQUENCY_MHZ);
    }
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
    trampDevSetFreq(freq);
}

void trampSendFreq(uint16_t freq)
{
    if (!trampVtxRaceLockEnabled()) {
        trampSendCommand('F', freq);
    }
}

void trampSetRFPower(uint16_t level)
{
    trampConfPower = level;
    if (trampConfPower != trampConfiguredPower) {
        trampPowerRetries = TRAMP_MAX_RETRIES;
    }
}

void trampSendRFPower(uint16_t level)
{
    if (!trampVtxRaceLockEnabled()) {
        trampSendCommand('P', level);
    }
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

void trampSetPitMode(uint8_t onoff)
{
    trampSendCommand('I', onoff ? 0 : 1);
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
                if (trampLastFreq != trampCurFreq) {
                    trampFreqRetries = TRAMP_MAX_RETRIES;
                }
                trampLastFreq = trampCurFreq;

                trampConfiguredPower = trampRespBuffer[4]|(trampRespBuffer[5] << 8);
                if (trampConfiguredPower != trampLastPower) {
                    trampPowerRetries = TRAMP_MAX_RETRIES;
                }
                trampLastPower = trampConfiguredPower;

                trampControlMode = trampRespBuffer[6]; // Currently only used for race lock

                trampPitMode = trampRespBuffer[7];
                trampPower = trampRespBuffer[8]|(trampRespBuffer[9] << 8);

                if (trampConfFreq == 0) {
                    trampConfFreq  = trampCurFreq;
                }
                if (trampConfPower == 0) {
                    trampConfPower = trampConfiguredPower;
                }

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

                if ((trampRespBuffer[14] == cksum) && (trampRespBuffer[15] == 0)) {
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
    trampSendU16(cmd, 0);
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
        trampStatus = TRAMP_STATUS_SET_FREQ_PW;

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
                trampConfPower = trampConfiguredPower;
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
    return vtxDevice != NULL && trampStatus > TRAMP_STATUS_OFFLINE;
}

static void vtxTrampSetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    UNUSED(vtxDevice);
    UNUSED(band);
    UNUSED(channel);
    //tramp does not support band/channel mode, only frequency
}

static void vtxTrampSetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    uint16_t powerValue = 0;
    if (vtxCommonLookupPowerValue(vtxDevice, index, &powerValue)) {
        trampSetRFPower(powerValue);
        trampCommitChanges();
    }
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

    // tramp does not support band and channel
    *pBand = 0;
    *pChannel = 0;
    return true;
}

static bool vtxTrampGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    if (!vtxTrampIsReady(vtxDevice)) {
        return false;
    }

    if (trampConfiguredPower > 0) {
        for (uint8_t i = 0; i < vtxTablePowerLevels; i++) {
            if (trampConfiguredPower <= vtxTablePowerValues[i]) {
                *pIndex = i + 1;
                break;
            }
        }
    }

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

static bool vtxTrampGetStatus(const vtxDevice_t *vtxDevice, unsigned *status)
{
    if (!vtxTrampIsReady(vtxDevice)) {
        return false;
    }

    *status = (trampPitMode ? VTX_STATUS_PIT_MODE : 0)
        | ((trampControlMode & TRAMP_CONTROL_RACE_LOCK) ? VTX_STATUS_LOCKED : 0);
    return true;
}

static uint8_t vtxTrampGetPowerLevels(const vtxDevice_t *vtxDevice, uint16_t *levels, uint16_t *powers)
{
    UNUSED(vtxDevice);
    UNUSED(levels);
    UNUSED(powers);

    return 0;
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
    .getFrequency = vtxTrampGetFreq,
    .getStatus = vtxTrampGetStatus,
    .getPowerLevels = vtxTrampGetPowerLevels,
};
#endif

bool vtxTrampInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_TRAMP);

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

    vtxCommonSetDevice(&vtxTramp);
#ifndef USE_VTX_TABLE
    //without USE_VTX_TABLE, fill vtxTable variables with default settings (instead of loading them from PG)
    vtxTablePowerLevels = VTX_TRAMP_POWER_COUNT;
    for (int i = 0; i < VTX_TRAMP_POWER_COUNT + 1; i++) {
        vtxTablePowerLabels[i] = trampPowerNames[i];
    }
    for (int i = 0; i < VTX_TRAMP_POWER_COUNT; i++) {
        vtxTablePowerValues[i] = trampPowerTable[i];
    }

#endif

#endif

    vtxInit();

    return true;
}

#endif // VTX_TRAMP
