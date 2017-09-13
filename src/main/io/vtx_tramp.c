/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Created by jflyper */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#if defined(VTX_TRAMP) && defined(VTX_CONTROL)

#include "build/debug.h"

#include "common/utils.h"

#include "cms/cms_menu_vtx_tramp.h"

#include "drivers/vtx_common.h"

#include "io/serial.h"
#include "io/vtx_control.h"
#include "io/vtx_string.h"
#include "io/vtx_tramp.h"

#if defined(CMS) || defined(VTX_COMMON)
const uint16_t trampPowerTable[VTX_TRAMP_POWER_COUNT] = {
    25, 100, 200, 400, 600
};

const char * const trampPowerNames[VTX_TRAMP_POWER_COUNT+1] = {
    "---", "25 ", "100", "200", "400", "600"
};
#endif

#if defined(VTX_COMMON)
static const vtxVTable_t trampVTable; // forward
static vtxDevice_t vtxTramp = {
    .vTable = &trampVTable,
    .capability.bandCount = 5,
    .capability.channelCount = 8,
    .capability.powerCount = sizeof(trampPowerTable),
    .bandNames = (char **)vtx58BandNames,
    .channelNames = (char **)vtx58ChannelNames,
    .powerNames = (char **)trampPowerNames,
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

    for (int i = 1 ; i < 14 ; i++)
        cksum += trampBuf[i];

    return cksum;
}

void trampCmdU16(uint8_t cmd, uint16_t param)
{
    if (!trampSerialPort)
        return;

    memset(trampReqBuffer, 0, ARRAYLEN(trampReqBuffer));
    trampReqBuffer[0] = 15;
    trampReqBuffer[1] = cmd;
    trampReqBuffer[2] = param & 0xff;
    trampReqBuffer[3] = (param >> 8) & 0xff;
    trampReqBuffer[14] = trampChecksum(trampReqBuffer);
    trampWriteBuf(trampReqBuffer);
}

void trampSetFreq(uint16_t freq)
{
    trampConfFreq = freq;
    if (trampConfFreq != trampCurFreq)
        trampFreqRetries = TRAMP_MAX_RETRIES;
}

void trampSendFreq(uint16_t freq)
{
    trampCmdU16('F', freq);
}

void trampSetBandAndChannel(uint8_t band, uint8_t channel)
{
    trampSetFreq(vtx58frequencyTable[band - 1][channel - 1]);
}

void trampSetRFPower(uint16_t level)
{
    trampConfPower = level;
    if (trampConfPower != trampPower)
        trampPowerRetries = TRAMP_MAX_RETRIES;
}

void trampSendRFPower(uint16_t level)
{
    trampCmdU16('P', level);
}

// return false if error
bool trampCommitChanges()
{
    if (trampStatus != TRAMP_STATUS_ONLINE)
        return false;

    trampStatus = TRAMP_STATUS_SET_FREQ_PW;
    return true;
}

void trampSetPitMode(uint8_t onoff)
{
    trampCmdU16('I', onoff ? 0 : 1);
}

// returns completed response code
char trampHandleResponse(void)
{
    uint8_t respCode = trampRespBuffer[1];

    switch (respCode) {
    case 'r':
        {
            uint16_t min_freq = trampRespBuffer[2]|(trampRespBuffer[3] << 8);
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
            uint16_t freq = trampRespBuffer[2]|(trampRespBuffer[3] << 8);
            if (freq != 0) {
                trampCurFreq = freq;
                trampConfiguredPower = trampRespBuffer[4]|(trampRespBuffer[5] << 8);
                trampPitMode = trampRespBuffer[7];
                trampPower = trampRespBuffer[8]|(trampRespBuffer[9] << 8);
                vtx58_Freq2Bandchan(trampCurFreq, &trampBand, &trampChannel);

                if (trampConfFreq == 0)  trampConfFreq  = trampCurFreq;
                if (trampConfPower == 0) trampConfPower = trampPower;
                return 'v';
            }

            // throw bytes echoed from tx to rx in bidirectional mode away
        }
        break;

    case 's':
        {
            uint16_t temp = (int16_t)(trampRespBuffer[6]|(trampRespBuffer[7] << 8));
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

static void trampResetReceiver()
{
    trampReceiveState = S_WAIT_LEN;
    trampReceivePos = 0;
}

static bool trampIsValidResponseCode(uint8_t code)
{
    if (code == 'r' || code == 'v' || code == 's')
        return true;
    else
        return false;
}

// returns completed response code or 0
static char trampReceive(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!trampSerialPort)
        return 0;

    while (serialRxBytesWaiting(trampSerialPort)) {
        uint8_t c = serialRead(trampSerialPort);
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

void vtxTrampProcess(uint32_t currentTimeUs)
{
    static uint32_t lastQueryTimeUs = 0;

#ifdef TRAMP_DEBUG
    static uint16_t debugFreqReqCounter = 0;
    static uint16_t debugPowReqCounter = 0;
#endif

    if (trampStatus == TRAMP_STATUS_BAD_DEVICE)
        return;

    char replyCode = trampReceive(currentTimeUs);

#ifdef TRAMP_DEBUG
    debug[0] = trampStatus;
#endif

    switch (replyCode) {
    case 'r':
        if (trampStatus <= TRAMP_STATUS_OFFLINE)
            trampStatus = TRAMP_STATUS_ONLINE;
        break;

    case 'v':
         if (trampStatus == TRAMP_STATUS_CHECK_FREQ_PW)
             trampStatus = TRAMP_STATUS_SET_FREQ_PW;
         break;
    }

    switch (trampStatus) {

    case TRAMP_STATUS_OFFLINE:
    case TRAMP_STATUS_ONLINE:
        if (cmp32(currentTimeUs, lastQueryTimeUs) > 1000 * 1000) { // 1s

            if (trampStatus == TRAMP_STATUS_OFFLINE)
                trampQueryR();
            else {
                static unsigned int cnt = 0;
                if (((cnt++) & 1) == 0)
                    trampQueryV();
                else
                    trampQueryS();
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
            }
            else if (trampConfPower && trampPowerRetries && (trampConfPower != trampConfiguredPower)) {
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
            }
            else {
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

#ifdef CMS
    trampCmsUpdateStatusString();
#endif
}


#ifdef VTX_COMMON

// Interface to common VTX API

vtxDevType_e vtxTrampGetDeviceType(void)
{
    return VTXDEV_TRAMP;
}

bool vtxTrampIsReady(void)
{
    return trampStatus > TRAMP_STATUS_OFFLINE;
}

void vtxTrampSetBandAndChannel(uint8_t band, uint8_t channel)
{
    if (band && channel) {
        trampSetBandAndChannel(band, channel);
        trampCommitChanges();
    }
}

void vtxTrampSetPowerByIndex(uint8_t index)
{
    if (index) {
        trampSetRFPower(trampPowerTable[index - 1]);
        trampCommitChanges();
    }
}

void vtxTrampSetPitMode(uint8_t onoff)
{
    trampSetPitMode(onoff);
}

bool vtxTrampGetBandAndChannel(uint8_t *pBand, uint8_t *pChannel)
{
    if (!vtxTrampIsReady())
        return false;

    *pBand = trampBand;
    *pChannel = trampChannel;
    return true;
}

bool vtxTrampGetPowerIndex(uint8_t *pIndex)
{
    if (!vtxTrampIsReady())
        return false;

    if (trampConfiguredPower > 0) {
        for (uint8_t i = 0; i < sizeof(trampPowerTable); i++) {
            if (trampConfiguredPower <= trampPowerTable[i]) {
                *pIndex = i + 1;
                break;
            }
        }
    }

    return true;
}

bool vtxTrampGetPitMode(uint8_t *pOnOff)
{
    if (!vtxTrampIsReady())
        return false;

    *pOnOff = trampPitMode;
    return true;
}

static const vtxVTable_t trampVTable = {
    .process = vtxTrampProcess,
    .getDeviceType = vtxTrampGetDeviceType,
    .isReady = vtxTrampIsReady,
    .setBandAndChannel = vtxTrampSetBandAndChannel,
    .setPowerByIndex = vtxTrampSetPowerByIndex,
    .setPitMode = vtxTrampSetPitMode,
    .getBandAndChannel = vtxTrampGetBandAndChannel,
    .getPowerIndex = vtxTrampGetPowerIndex,
    .getPitMode = vtxTrampGetPitMode,
};

#endif

bool vtxTrampInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_TRAMP);

    if (portConfig) {
        portOptions_e portOptions = 0; 
#if defined(VTX_COMMON)
        portOptions = portOptions | (vtxConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR);
#else
        portOptions = SERIAL_BIDIR;
#endif

        trampSerialPort = openSerialPort(portConfig->identifier, FUNCTION_VTX_TRAMP, NULL, 9600, MODE_RXTX, portOptions);
    }

    if (!trampSerialPort) {
        return false;
    }

#if defined(VTX_COMMON)
    vtxCommonRegisterDevice(&vtxTramp);
#endif

    return true;
}

#endif // VTX_TRAMP
