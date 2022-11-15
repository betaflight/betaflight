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

// Maximum number of requests sent to try a config change
// Some VTX fail to respond to every request (like Matek FCHUB-VTX) so
// we sometimes need multiple retries to get the VTX to respond.
#define TRAMP_MAX_RETRIES (20)

// Race lock - settings can't be changed
#define TRAMP_CONTROL_RACE_LOCK (0x01)

// Define periods between requests
#define TRAMP_MIN_REQUEST_PERIOD_US (200 * 1000) // 200ms
#define TRAMP_STATUS_REQUEST_PERIOD_US (1000 * 1000) // 1s

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

// Device serial port instance
static serialPort_t *trampSerialPort = NULL;

// Serial transmit and receive buffers
static uint8_t trampReqBuffer[16];
static uint8_t trampRespBuffer[16];

// Module state machine
typedef enum {
    // Offline - device hasn't responded yet
    TRAMP_STATUS_OFFLINE = 0,
    // Init - fetching current settings from device
    TRAMP_STATUS_INIT,
    // Online - device is ready and being monitored - freq/power/pitmode
    TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT,
    // Online - device is ready and being monitored - temperature
    TRAMP_STATUS_ONLINE_MONITOR_TEMP,
    // Online - device is ready and config has just been updated
    TRAMP_STATUS_ONLINE_CONFIG
} trampStatus_e;

// Module state
static trampStatus_e trampStatus = TRAMP_STATUS_OFFLINE;

// Device limits, read from device during init
static uint32_t trampRFFreqMin = 0;
static uint32_t trampRFFreqMax = 0;
static uint32_t trampRFPowerMax;

// Device status, read from device periodically
static uint32_t trampCurFreq = 0;
static uint16_t trampCurConfPower = 0; // Configured power
static uint16_t trampCurActPower = 0; // Actual power
static uint8_t trampCurPitMode = 0; // Expect to startup out of pitmode
static int16_t trampCurTemp = 0;
static uint8_t trampCurControlMode = 0;

// Device configuration, desired state of device
static uint32_t trampConfFreq = 0;
static uint16_t trampConfPower = 0;
static uint8_t trampConfPitMode = 0; // Initially configured out of pitmode

// Last device configuration, last desired state of device - used to reset
// retry count
static uint32_t trampLastConfFreq = 0;
static uint16_t trampLastConfPower = 0;
static uint8_t trampLastConfPitMode = 0; // Mirror trampConfPitMode

// Retry count
static uint8_t trampRetryCount = TRAMP_MAX_RETRIES;

// Receive state machine
typedef enum {
    S_WAIT_LEN = 0,   // Waiting for a packet len
    S_WAIT_CODE,      // Waiting for a response code
    S_DATA,           // Waiting for rest of the packet.
} trampReceiveState_e;

static trampReceiveState_e trampReceiveState = S_WAIT_LEN;

// Receive buffer index
static int trampReceivePos = 0;

// Last action time
static timeUs_t trampLastTimeUs = 0;

// Calculate tramp protocol checksum of provided buffer
static uint8_t trampChecksum(uint8_t *trampBuf)
{
    uint8_t cksum = 0;

    for (int i = 1 ; i < 14 ; i++) {
        cksum += trampBuf[i];
    }

    return cksum;
}

// Check if race lock is enabled
static bool trampVtxRaceLockEnabled(void)
{
    return trampCurControlMode & TRAMP_CONTROL_RACE_LOCK;
}

// Send tramp protocol frame to device
static void trampSendU16(uint8_t cmd, uint16_t param)
{
    if (!trampSerialPort) {
        return;
    }

    memset(trampReqBuffer, 0, ARRAYLEN(trampReqBuffer));
    trampReqBuffer[0] = 0x0F;
    trampReqBuffer[1] = cmd;
    trampReqBuffer[2] = param & 0xff;
    trampReqBuffer[3] = (param >> 8) & 0xff;
    trampReqBuffer[14] = trampChecksum(trampReqBuffer);
    serialWriteBuf(trampSerialPort, trampReqBuffer, 16);
}

// Send command to device
static void trampSendCommand(uint8_t cmd, uint16_t param)
{
    // Is VTX control enabled?
    if (!IS_RC_MODE_ACTIVE(BOXVTXCONTROLDISABLE)) {
        // Yes, send command
        trampSendU16(cmd, param);
    }
}

// Process response and return code if valid else 0
static char trampHandleResponse(void)
{
    const uint8_t respCode = trampRespBuffer[1];

    switch (respCode) {
    case 'r':
        {
            const uint16_t min_freq = trampRespBuffer[2]|(trampRespBuffer[3] << 8);
            // Check we're not reading the request (indicated by freq zero)
            if (min_freq != 0) {
                // Got response, update device limits
                trampRFFreqMin = min_freq;
                trampRFFreqMax = trampRespBuffer[4]|(trampRespBuffer[5] << 8);
                trampRFPowerMax = trampRespBuffer[6]|(trampRespBuffer[7] << 8);
                return 'r';
            }
            break;
        }
    case 'v':
        {
            const uint16_t freq = trampRespBuffer[2]|(trampRespBuffer[3] << 8);
            // Check we're not reading the request (indicated by freq zero)
            if (freq != 0) {
                // Got response, update device status
                trampCurFreq = freq;
                trampCurConfPower = trampRespBuffer[4]|(trampRespBuffer[5] << 8);
                trampCurControlMode = trampRespBuffer[6]; // Currently only used for race lock
                trampCurPitMode = trampRespBuffer[7];
                trampCurActPower = trampRespBuffer[8]|(trampRespBuffer[9] << 8);

                // Init config with current status if not set
                if (trampConfFreq == 0) {
                    trampConfFreq  = trampCurFreq;
                }
                if (trampConfPower == 0) {
                    trampConfPower = trampCurConfPower;
                }
                return 'v';
            }
            break;
        }
    case 's':
        {
            const uint16_t temp = (int16_t)(trampRespBuffer[6]|(trampRespBuffer[7] << 8));
            // Check we're not reading the request (indicated by temp zero)
            if (temp != 0) {
                // Got response, update device status
                trampCurTemp = temp;
                return 's';
            }
            break;
        }
    }

    // Likely reading a request, return zero to indicate not accepted
    return 0;
}

// Reset receiver state machine
static void trampResetReceiver(void)
{
    trampReceiveState = S_WAIT_LEN;
    trampReceivePos = 0;
}

// returns completed response code or 0
static char trampReceive(void)
{
    if (!trampSerialPort) {
        return 0;
    }

    while (serialRxBytesWaiting(trampSerialPort)) {
        const uint8_t c = serialRead(trampSerialPort);
        trampRespBuffer[trampReceivePos++] = c;

        switch (trampReceiveState) {
        case S_WAIT_LEN:
            {
                if (c == 0x0F) {
                    // Found header byte, advance to wait for code
                    trampReceiveState = S_WAIT_CODE;
                } else {
                    // Unexpected header, reset state machine
                    trampResetReceiver();
                }
                break;
            }
        case S_WAIT_CODE:
            {
                if (c == 'r' || c == 'v' || c == 's') {
                    // Code is for response is one we're interested in, advance to data
                    trampReceiveState = S_DATA;
                } else {
                    // Unexpected code, reset state machine
                    trampResetReceiver();
                }
                break;
            }
        case S_DATA:
            {
                if (trampReceivePos == 16) {
                    // Buffer is full, calculate checksum
                    uint8_t cksum = trampChecksum(trampRespBuffer);

                    // Reset state machine ready for next response
                    trampResetReceiver();

                    if ((trampRespBuffer[14] == cksum) && (trampRespBuffer[15] == 0)) {
                        // Checksum is correct, process response
                        char r = trampHandleResponse();

                        // Check response valid else keep on reading
                        if (r != 0) {
                            return r;
                        }
                    }
                }
                break;
            }
        default:
            {
                // Invalid state, reset state machine
                trampResetReceiver();
                break;
            }
        }
    }

    return 0;
}

static void trampQuery(uint8_t cmd)
{
    // Reset receive buffer and issue command
    trampResetReceiver();
    trampSendU16(cmd, 0);
}

static void vtxTrampProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs)
{
    UNUSED(vtxDevice);
    bool configUpdateRequired = false;

    // Read response from device
    const char replyCode = trampReceive();

    // Act on state
    switch(trampStatus) {
    case TRAMP_STATUS_OFFLINE:
        {
            // Offline, check for response
            if (replyCode == 'r') {
                // Device replied to reset? request, enter init
                trampStatus = TRAMP_STATUS_INIT;
            } else if (cmp32(currentTimeUs, trampLastTimeUs) >= TRAMP_MIN_REQUEST_PERIOD_US) {
                // Min request period exceeded, issue another reset?
                trampQuery('r');

                // Update last time
                trampLastTimeUs = currentTimeUs;
            }
            break;
        }
    case TRAMP_STATUS_INIT:
        {
            // Initializing, check for response
            if (replyCode == 'v') {
                // Device replied to freq / power / pit query, enter online
                trampStatus = TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT;
            } else if (cmp32(currentTimeUs, trampLastTimeUs) >= TRAMP_MIN_REQUEST_PERIOD_US) {
                // Min request period exceeded, issue another query
                trampQuery('v');

                // Update last time
                trampLastTimeUs = currentTimeUs;
            }
            break;
        }
    case TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT:
        {
            // Note after config a status update request is made, a new status
            // request is made, this request is handled above and should prevent
            // subsiquent config updates if the config is now correct
            if (trampRetryCount > 0 && (cmp32(currentTimeUs, trampLastTimeUs) >= TRAMP_MIN_REQUEST_PERIOD_US)) {
                // Config retries remain and min request period exceeded, check freq
                if (!trampVtxRaceLockEnabled() && (trampConfFreq != trampCurFreq)) {
                    // Freq can be and needs to be updated, issue request
                    trampSendCommand('F', trampConfFreq);

                    // Set flag
                    configUpdateRequired = true;
                } else if (!trampVtxRaceLockEnabled() && (trampConfPower != trampCurConfPower)) {
                    // Power can be and needs to be updated, issue request
                    trampSendCommand('P', trampConfPower);

                    // Set flag
                    configUpdateRequired = true;
                } else if (trampConfPitMode != trampCurPitMode) {
                    // Pit mode needs to be updated, issue request
                    trampSendCommand('I', trampConfPitMode ? 0 : 1);

                    // Set flag
                    configUpdateRequired = true;
                }

                if (configUpdateRequired) {
                    // Update required, decrement retry count
                    trampRetryCount--;

                    // Update last time
                    trampLastTimeUs = currentTimeUs;

                    // Advance state
                    trampStatus = TRAMP_STATUS_ONLINE_CONFIG;
                } else {
                    // No update required, reset retry count
                    trampRetryCount = TRAMP_MAX_RETRIES;
                }
            }

            /* Was a config update made? */
            if (!configUpdateRequired) {
                /* No, look to continue monitoring */
                if (cmp32(currentTimeUs, trampLastTimeUs) >= TRAMP_STATUS_REQUEST_PERIOD_US) {
                    // Request period exceeded, issue freq/power/pit query
                    trampQuery('v');

                    // Update last time
                    trampLastTimeUs = currentTimeUs;
                } else if (replyCode == 'v') {
                    // Got reply, issue temp query
                    trampQuery('s');

                    // Wait for reply
                    trampStatus = TRAMP_STATUS_ONLINE_MONITOR_TEMP;

                    // Update last time
                    trampLastTimeUs = currentTimeUs;
                }
            }

            break;
        }
    case TRAMP_STATUS_ONLINE_MONITOR_TEMP:
        {
            // Check request time
            if (replyCode == 's') {
                // Got reply, return to request freq/power/pit
                trampStatus = TRAMP_STATUS_ONLINE_MONITOR_TEMP;
            } else if (cmp32(currentTimeUs, trampLastTimeUs) >= TRAMP_MIN_REQUEST_PERIOD_US) {
                // Timed out after min request period, return to request freq/power/pit query
                trampStatus = TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT;
            }
            break;
        }
    case TRAMP_STATUS_ONLINE_CONFIG:
        {
            // Param should now be set, check time
            if (cmp32(currentTimeUs, trampLastTimeUs) >= TRAMP_MIN_REQUEST_PERIOD_US) {
                // Min request period exceeded, re-query
                trampQuery('v');

                // Advance state
                trampStatus = TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT;

                // Update last time
                trampLastTimeUs = currentTimeUs;
            }
            break;
        }
    default:
        {
            // Invalid state, reset
            trampStatus = TRAMP_STATUS_OFFLINE;
            break;
        }
    }

    DEBUG_SET(DEBUG_VTX_TRAMP, 0, trampStatus);
    DEBUG_SET(DEBUG_VTX_TRAMP, 1, replyCode);
    DEBUG_SET(DEBUG_VTX_TRAMP, 2, ((trampConfPitMode << 14) &              0xC000) |
                                  ((trampCurPitMode << 12) &               0x3000) |
                                  ((trampConfPower << 8) &                 0x0F00) |
                                  ((trampCurConfPower << 4) &              0x00F0) |
                                  ((trampConfFreq != trampCurFreq) ?       0x0008 : 0x0000) |
                                  ((trampConfPower != trampCurConfPower) ? 0x0004 : 0x0000) |
                                  ((trampConfPitMode != trampCurPitMode) ? 0x0002 : 0x0000) |
                                  (configUpdateRequired ?                  0x0001 : 0x0000));
    DEBUG_SET(DEBUG_VTX_TRAMP, 3, trampRetryCount);

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
    return vtxDevice != NULL && trampStatus >= TRAMP_STATUS_ONLINE_MONITOR_FREQPWRPIT;
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
    uint16_t powerValue;

    // Lookup power level value
    if (vtxCommonLookupPowerValue(vtxDevice, index, &powerValue)) {
        // Value found, apply
        trampConfPower = powerValue;
        if (trampConfPower != trampLastConfPower) {
            // Requested power changed, reset retry count
            trampRetryCount = TRAMP_MAX_RETRIES;
            trampLastConfPower = trampConfPower;
        }
    }
}

static void vtxTrampSetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    UNUSED(vtxDevice);

    trampConfPitMode = onoff;
    if (trampConfPitMode != trampLastConfPitMode) {
        // Requested pitmode changed, reset retry count
        trampRetryCount = TRAMP_MAX_RETRIES;
        trampLastConfPitMode = trampConfPitMode;
    }
}

static void vtxTrampSetFreq(vtxDevice_t *vtxDevice, uint16_t freq)
{
    UNUSED(vtxDevice);

    uint8_t freqValid;

    // Check frequency valid
    if (trampRFFreqMin != 0 && trampRFFreqMax != 0) {
        freqValid = (freq >= trampRFFreqMin && freq <= trampRFFreqMax);
    } else {
        freqValid = (freq >= VTX_TRAMP_MIN_FREQUENCY_MHZ && freq <= VTX_TRAMP_MAX_FREQUENCY_MHZ);
    }

    // Is frequency valid?
    if (freqValid) {
        // Yes, set freq
        trampConfFreq = freq;
        if (trampConfFreq != trampLastConfFreq) {
            // Requested freq changed, reset retry count
            trampRetryCount = TRAMP_MAX_RETRIES;
            trampLastConfFreq = trampConfFreq;
        }
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

    // Special case, power not set
    if (trampConfPower == 0) {
        *pIndex = 0;
        return true;
    }

    // Lookup value in table
    for (uint8_t i = 0; i < vtxTablePowerLevels; i++) {
        // Find value that matches current configured power level
        if (trampConfPower == vtxTablePowerValues[i]) {
            // Value found, return index
            *pIndex = i + 1;
            return true;
        }
    }

    // Value not found in table
    return false;
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

    // Mirror configued pit mode state rather than use current pitmode as we
    // should, otherwise the logic in vtxProcessPitMode may not get us to the
    // correct state if pitmode is toggled quickly
    *status = (trampConfPitMode ? VTX_STATUS_PIT_MODE : 0);

    // Check VTX is not locked
    *status |= ((trampCurControlMode & TRAMP_CONTROL_RACE_LOCK) ? VTX_STATUS_LOCKED : 0);

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
    .serializeCustomDeviceStatus = NULL,
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

uint16_t vtxTrampGetCurrentActualPower(void)
{
    return trampCurActPower;
}

uint16_t vtxTrampGetCurrentTemp(void)
{
    return trampCurTemp;
}

#endif // VTX_TRAMP
