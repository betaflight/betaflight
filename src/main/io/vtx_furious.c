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

#if defined(VTX_FURIOUS) && defined(VTX_CONTROL)

#include "build/debug.h"

#include "common/utils.h"

#include "cms/cms_menu_vtx_furious.h"

#include "drivers/vtx_common.h"

#include "io/serial.h"
#include "io/vtx_string.h"
#include "io/vtx_furious.h"

#define FURIOUS_SERIAL_OPTIONS (SERIAL_BIDIR)

#if defined(CMS) || defined(VTX_COMMON)
const uint16_t furiousPowerTable[VTX_FURIOUS_POWER_COUNT] = {
    25, 200, 400, 600
};

const char * const furiousPowerNames[VTX_FURIOUS_POWER_COUNT+1] = {
    "---", "25 ", "200", "400", "600"
};
#endif

#if defined(VTX_COMMON)
static const vtxVTable_t furiousVTable; // forward
static vtxDevice_t vtxFurious = {
    .vTable = &furiousVTable,
    .capability.bandCount = VTX58_BAND_COUNT,
    .capability.channelCount = VTX58_CHANNEL_COUNT,
    .capability.powerCount = sizeof(furiousPowerTable),
    .bandNames = (char **)vtx58BandNames,
    .channelNames = (char **)vtx58ChannelNames,
    .powerNames = (char **)furiousPowerNames,
};
#endif

static serialPort_t *furiousSerialPort = NULL;

static uint8_t furiousReqBuffer[16];
static uint8_t furiousRespBuffer[16];

typedef enum {
    FURIOUS_STATUS_BAD_DEVICE = -1,
    FURIOUS_STATUS_OFFLINE = 0,
    FURIOUS_STATUS_ONLINE,
    FURIOUS_STATUS_SET_FREQ_PW,
    FURIOUS_STATUS_CHECK_FREQ_PW
} furiousStatus_e;

furiousStatus_e furiousStatus = FURIOUS_STATUS_OFFLINE;

uint32_t furiousRFFreqMin;
uint32_t furiousRFFreqMax;
uint32_t furiousRFPowerMax;

uint32_t furiousCurFreq = 0;
uint8_t furiousBand = 0;
uint8_t furiousChannel = 0;
uint16_t furiousPower = 0;       // Actual transmitting power
uint16_t furiousConfiguredPower = 0; // Configured transmitting power
int16_t furiousTemperature = 0;
uint8_t furiousPitMode = 0;
uint8_t furiousHardwareVersion = 0;
uint8_t furiousFirmwareVersion = 0;

// Maximum number of requests sent to try a config change
#define FURIOUS_MAX_RETRIES 2

uint32_t furiousConfFreq = 0;
uint8_t  furiousConfBand = 0;
uint8_t  furiousFreqRetries = 0;

uint16_t furiousConfPower = 0;
uint8_t  furiousPowerRetries = 0;

#ifdef CMS
void furiousCmsUpdateStatusString(void); // Forward
#endif

static void furiousWriteBuf(uint8_t *buf)
{
    serialWriteBuf(furiousSerialPort, buf, 16);
}

static uint8_t furiousChecksum(uint8_t *furiousBuf)
{
    uint8_t cksum = 0;

    for (int i = 1 ; i < 14 ; i++)
        cksum ^= furiousBuf[i];

    return cksum;
}

void furiousCmdU16(uint8_t cmd, uint16_t param1, uint8_t param2)
{
    if (!furiousSerialPort)
        return;

    memset(furiousReqBuffer, 0, ARRAYLEN(furiousReqBuffer));
    furiousReqBuffer[0] = 15;
    furiousReqBuffer[1] = cmd;
    furiousReqBuffer[2] = param1 & 0xff;
    furiousReqBuffer[3] = (param1 >> 8) & 0xff;
    furiousReqBuffer[4] = param2;
    furiousReqBuffer[14] = furiousChecksum(furiousReqBuffer);
    furiousWriteBuf(furiousReqBuffer);
}

void furiousSetFreq(uint16_t freq)
{
    furiousConfFreq = freq;
    if ((furiousConfFreq != furiousCurFreq) || (furiousConfBand != furiousBand))
        furiousFreqRetries = FURIOUS_MAX_RETRIES;
}

void furiousSendFreq(uint16_t freq, uint8_t band)
{
    furiousCmdU16('F', freq, band);
}

void furiousSetBandAndChannel(uint8_t band, uint8_t channel)
{
    furiousSetFreq(vtx58frequencyTable[band - 1][channel - 1]);
    furiousConfBand = band;
}

void furiousSetRFPower(uint16_t level)
{
    furiousConfPower = level;
    if (furiousConfPower != furiousPower)
        furiousPowerRetries = FURIOUS_MAX_RETRIES;
}

void furiousSendRFPower(uint16_t level)
{
    furiousCmdU16('P', level, 0);
}

// return false if error
bool furiousCommitChanges()
{
    if (furiousStatus != FURIOUS_STATUS_ONLINE)
        return false;

    furiousStatus = FURIOUS_STATUS_SET_FREQ_PW;
    return true;
}

void furiousSetPitMode(uint8_t onoff)
{
    furiousCmdU16('I', onoff ? 0 : 1, 0);
}

// returns completed response code
char furiousHandleResponse(void)
{
    uint8_t respCode = furiousRespBuffer[1];

    switch (respCode) {
    case 'f':
        {
            uint16_t min_freq = furiousRespBuffer[2]|(furiousRespBuffer[3] << 8);
            if (min_freq != 0) {
                furiousRFFreqMin = min_freq;
                furiousRFFreqMax = furiousRespBuffer[4]|(furiousRespBuffer[5] << 8);
                furiousRFPowerMax = furiousRespBuffer[6]|(furiousRespBuffer[7] << 8);
                furiousHardwareVersion = furiousRespBuffer[8];
                furiousFirmwareVersion = furiousRespBuffer[9];
                return 'f';
            }

            // throw bytes echoed from tx to rx in bidirectional mode away
        }
        break;

    case 'v':
        {
            uint16_t freq = furiousRespBuffer[2]|(furiousRespBuffer[3] << 8);
            if (freq != 0) {
                furiousCurFreq = freq;
                furiousConfiguredPower = furiousRespBuffer[4]|(furiousRespBuffer[5] << 8);
                furiousPitMode = furiousRespBuffer[7];
                furiousPower = furiousRespBuffer[8]|(furiousRespBuffer[9] << 8);
                furiousBand = furiousRespBuffer[10] - 1;
                vtx58_Freq2Bandchan(furiousCurFreq, &furiousBand, &furiousChannel);

                if (furiousConfFreq == 0)  furiousConfFreq  = furiousCurFreq;
                if (furiousConfPower == 0) furiousConfPower = furiousPower;
                return 'v';
            }

            // throw bytes echoed from tx to rx in bidirectional mode away
        }
        break;

    case 's':
        {
            uint16_t temp = (int16_t)(furiousRespBuffer[6]|(furiousRespBuffer[7] << 8));
            if (temp != 0) {
                furiousTemperature = temp;
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
} furiousReceiveState_e;

static furiousReceiveState_e furiousReceiveState = S_WAIT_LEN;
static int furiousReceivePos = 0;

static void furiousResetReceiver()
{
    furiousReceiveState = S_WAIT_LEN;
    furiousReceivePos = 0;
}

static bool furiousIsValidResponseCode(uint8_t code)
{
    if (code == 'f' || code == 'v' || code == 's')
        return true;
    else
        return false;
}

// returns completed response code or 0
static char furiousReceive(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!furiousSerialPort)
        return 0;

    while (serialRxBytesWaiting(furiousSerialPort)) {
        uint8_t c = serialRead(furiousSerialPort);
        furiousRespBuffer[furiousReceivePos++] = c;

        switch (furiousReceiveState) {
        case S_WAIT_LEN:
            if (c == 0x0F) {
                furiousReceiveState = S_WAIT_CODE;
            } else {
                furiousReceivePos = 0;
            }
            break;

        case S_WAIT_CODE:
            if (furiousIsValidResponseCode(c)) {
                furiousReceiveState = S_DATA;
            } else {
                furiousResetReceiver();
            }
            break;

        case S_DATA:
            if (furiousReceivePos == 16) {
                uint8_t cksum = furiousChecksum(furiousRespBuffer);

                furiousResetReceiver();

                if ((furiousRespBuffer[14] == cksum) && (furiousRespBuffer[15] == 0)) {
                    return furiousHandleResponse();
                }
            }
            break;

        default:
            furiousResetReceiver();
        }
    }

    return 0;
}

void furiousQuery(uint8_t cmd)
{
    furiousResetReceiver();
    furiousCmdU16(cmd, 0, 0);
}

void furiousQueryR(void)
{
    furiousQuery('f');
}

void furiousQueryV(void)
{
    furiousQuery('v');
}

void furiousQueryS(void)
{
    furiousQuery('s');
}

void vtxFuriousProcess(uint32_t currentTimeUs)
{
    static uint32_t lastQueryTimeUs = 0;

#ifdef FURIOUS_DEBUG
    static uint16_t debugFreqReqCounter = 0;
    static uint16_t debugPowReqCounter = 0;
#endif

    if (furiousStatus == FURIOUS_STATUS_BAD_DEVICE)
        return;

    char replyCode = furiousReceive(currentTimeUs);

#ifdef FURIOUS_DEBUG
    debug[0] = furiousStatus;
#endif

    switch (replyCode) {
    case 'f':
        if (furiousStatus <= FURIOUS_STATUS_OFFLINE)
            furiousStatus = FURIOUS_STATUS_ONLINE;
        break;

    case 'v':
         if (furiousStatus == FURIOUS_STATUS_CHECK_FREQ_PW)
             furiousStatus = FURIOUS_STATUS_SET_FREQ_PW;
         break;
    }

    switch (furiousStatus) {

    case FURIOUS_STATUS_OFFLINE:
    case FURIOUS_STATUS_ONLINE:
        if (cmp32(currentTimeUs, lastQueryTimeUs) > 1000 * 1000) { // 1s

            if (furiousStatus == FURIOUS_STATUS_OFFLINE)
                furiousQueryR();
            else {
                static unsigned int cnt = 0;
                if (((cnt++) & 1) == 0)
                    furiousQueryV();
                else
                    furiousQueryS();
            }

            lastQueryTimeUs = currentTimeUs;
        }
        break;

    case FURIOUS_STATUS_SET_FREQ_PW:
        {
            bool done = true;
            if  ((furiousConfFreq != furiousCurFreq) || (furiousConfBand != furiousBand)) {
                furiousSendFreq(furiousConfFreq, furiousConfBand - 1);
                furiousFreqRetries--;
#ifdef FURIOUS_DEBUG
                debugFreqReqCounter++;
#endif
                done = false;
            }
            else if (furiousConfPower && furiousPowerRetries && (furiousConfPower != furiousConfiguredPower)) {
                furiousSendRFPower(furiousConfPower);
                furiousPowerRetries--;
#ifdef FURIOUS_DEBUG
                debugPowReqCounter++;
#endif
                done = false;
            }

            if (!done) {
                furiousStatus = FURIOUS_STATUS_CHECK_FREQ_PW;

                // delay next status query by 300ms
                lastQueryTimeUs = currentTimeUs + 300 * 1000;
            }
            else {
                // everything has been done, let's return to original state
                furiousStatus = FURIOUS_STATUS_ONLINE;
                // reset configuration value in case it failed (no more retries)
                furiousConfFreq  = furiousCurFreq;
                furiousConfPower = furiousPower;
                furiousConfBand = furiousBand;
                furiousFreqRetries = furiousPowerRetries = 0;
            }
        }
        break;

    case FURIOUS_STATUS_CHECK_FREQ_PW:
        if (cmp32(currentTimeUs, lastQueryTimeUs) > 200 * 1000) {
            furiousQueryV();
            lastQueryTimeUs = currentTimeUs;
        }
        break;

    default:
        break;
    }

#ifdef FURIOUS_DEBUG
    debug[1] = debugFreqReqCounter;
    debug[2] = debugPowReqCounter;
    debug[3] = 0;
#endif

#ifdef CMS
    furiousCmsUpdateStatusString();
#endif
}


#ifdef VTX_COMMON

// Interface to common VTX API

vtxDevType_e vtxFuriousGetDeviceType(void)
{
    return VTXDEV_FURIOUS;
}

bool vtxFuriousIsReady(void)
{
    return furiousStatus > FURIOUS_STATUS_OFFLINE;
}

void vtxFuriousSetBandAndChannel(uint8_t band, uint8_t channel)
{
    if (band && channel) {
        furiousSetBandAndChannel(band, channel);
        furiousCommitChanges();
    }
}

void vtxFuriousSetPowerByIndex(uint8_t index)
{
    if (index) {
        furiousSetRFPower(furiousPowerTable[index - 1]);
        furiousCommitChanges();
    }
}

void vtxFuriousSetPitMode(uint8_t onoff)
{
    furiousSetPitMode(onoff);
}

bool vtxFuriousGetBandAndChannel(uint8_t *pBand, uint8_t *pChannel)
{
    if (!vtxFuriousIsReady())
        return false;

    *pBand = furiousBand;
    *pChannel = furiousChannel;
    return true;
}

bool vtxFuriousGetPowerIndex(uint8_t *pIndex)
{
    if (!vtxFuriousIsReady())
        return false;

    if (furiousConfiguredPower > 0) {
        for (uint8_t i = 0; i < sizeof(furiousPowerTable); i++) {
            if (furiousConfiguredPower <= furiousPowerTable[i]) {
                *pIndex = i + 1;
                break;
            }
        }
    }

    return true;
}

bool vtxFuriousGetPitMode(uint8_t *pOnOff)
{
    if (!vtxFuriousIsReady())
        return false;

    *pOnOff = furiousPitMode;
    return true;
}

static const vtxVTable_t furiousVTable = {
    .process = vtxFuriousProcess,
    .getDeviceType = vtxFuriousGetDeviceType,
    .isReady = vtxFuriousIsReady,
    .setBandAndChannel = vtxFuriousSetBandAndChannel,
    .setPowerByIndex = vtxFuriousSetPowerByIndex,
    .setPitMode = vtxFuriousSetPitMode,
    .getBandAndChannel = vtxFuriousGetBandAndChannel,
    .getPowerIndex = vtxFuriousGetPowerIndex,
    .getPitMode = vtxFuriousGetPitMode,
};

#endif

bool vtxFuriousInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_FURIOUS);

    if (portConfig) {
        furiousSerialPort = openSerialPort(portConfig->identifier, FUNCTION_VTX_FURIOUS, NULL, 9600, MODE_RXTX, FURIOUS_SERIAL_OPTIONS);
    }

    if (!furiousSerialPort) {
        return false;
    }

#if defined(VTX_COMMON)
    vtxCommonRegisterDevice(&vtxFurious);
#endif

    return true;
}

#endif // VTX_FURIOUS
