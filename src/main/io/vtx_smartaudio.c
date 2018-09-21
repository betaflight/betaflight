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
#include <string.h>
#include <ctype.h>

#include "platform.h"

#if defined(USE_VTX_SMARTAUDIO) && defined(USE_VTX_CONTROL)

#include "build/debug.h"

#include "cms/cms.h"
#include "cms/cms_menu_vtx_smartaudio.h"

#include "common/maths.h"
#include "common/printf.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "io/serial.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_smartaudio.h"
#include "io/vtx_string.h"


// Timing parameters
// Note that vtxSAProcess() is normally called at 200ms interval
#define SMARTAUDIO_CMD_TIMEOUT       120    // Time until the command is considered lost
#define SMARTAUDIO_POLLING_INTERVAL  150    // Minimum time between state polling
#define SMARTAUDIO_POLLING_WINDOW   1000    // Time window after command polling for state change

//#define USE_SMARTAUDIO_DPRINTF
//#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART1

#ifdef USE_SMARTAUDIO_DPRINTF
serialPort_t *debugSerialPort = NULL;
#endif // USE_SMARTAUDIO_DPRINTF

static serialPort_t *smartAudioSerialPort = NULL;

#if defined(USE_CMS) || defined(USE_VTX_COMMON)
const char * const saPowerNames[VTX_SMARTAUDIO_POWER_COUNT+1] = {
    "---", "25 ", "200", "500", "800",
};
#endif

#ifdef USE_VTX_COMMON
static const vtxVTable_t saVTable;    // Forward
static vtxDevice_t vtxSmartAudio = {
    .vTable = &saVTable,
    .capability.bandCount = VTX_SMARTAUDIO_BAND_COUNT,
    .capability.channelCount = VTX_SMARTAUDIO_CHANNEL_COUNT,
    .capability.powerCount = VTX_SMARTAUDIO_POWER_COUNT,
    .bandNames = (char **)vtx58BandNames,
    .channelNames = (char **)vtx58ChannelNames,
    .powerNames = (char **)saPowerNames,
};
#endif

// SmartAudio command and response codes
enum {
    SA_CMD_NONE = 0x00,
    SA_CMD_GET_SETTINGS = 0x01,
    SA_CMD_SET_POWER,
    SA_CMD_SET_CHAN,
    SA_CMD_SET_FREQ,
    SA_CMD_SET_MODE,
    SA_CMD_GET_SETTINGS_V2 = 0x09        // Response only
} smartAudioCommand_e;

// This is not a good design; can't distinguish command from response this way.
#define SACMD(cmd) (((cmd) << 1) | 1)


#define SA_IS_PITMODE(n) ((n) & SA_MODE_GET_PITMODE)
#define SA_IS_PIRMODE(n) (((n) & SA_MODE_GET_PITMODE) && ((n) & SA_MODE_GET_IN_RANGE_PITMODE))
#define SA_IS_PORMODE(n) (((n) & SA_MODE_GET_PITMODE) && ((n) & SA_MODE_GET_OUT_RANGE_PITMODE))


// convert between 'saDevice.channel' and band/channel values
#define SA_DEVICE_CHVAL_TO_BAND(val) ((val) / (uint8_t)8)
#define SA_DEVICE_CHVAL_TO_CHANNEL(val) ((val) % (uint8_t)8)
#define SA_BANDCHAN_TO_DEVICE_CHVAL(band, channel) ((band) * (uint8_t)8 + (channel))


// Statistical counters, for user side trouble shooting.

smartAudioStat_t saStat = {
    .pktsent = 0,
    .pktrcvd = 0,
    .badpre = 0,
    .badlen = 0,
    .crc = 0,
    .ooopresp = 0,
    .badcode = 0,
};

saPowerTable_t saPowerTable[VTX_SMARTAUDIO_POWER_COUNT] = {
    {  25,   7,   0 },
    { 200,  16,   1 },
    { 500,  25,   2 },
    { 800,  40,   3 },
};

// Last received device ('hard') states

smartAudioDevice_t saDevice = {
    .version = 0,
    .channel = -1,
    .power = -1,
    .mode = 0,
    .freq = 0,
    .orfreq = 0,
};

static smartAudioDevice_t saDevicePrev = {
    .version = 0,
};

// XXX Possible compliance problem here. Need LOCK/UNLOCK menu?
static uint8_t saLockMode = SA_MODE_SET_UNLOCK; // saCms variable?

// XXX Should be configurable by user?
bool saDeferred = true; // saCms variable?

// Receive frame reassembly buffer
#define SA_MAX_RCVLEN 11
static uint8_t sa_rbuf[SA_MAX_RCVLEN+4]; // XXX delete 4 byte guard

//
// CRC8 computations
//

#define POLYGEN 0xd5

static uint8_t CRC8(const uint8_t *data, const int8_t len)
{
    uint8_t crc = 0; /* start with 0 so first byte can be 'xored' in */
    uint8_t currByte;

    for (int i = 0 ; i < len ; i++) {
        currByte = data[i];

        crc ^= currByte; /* XOR-in the next input byte */

        for (int i = 0; i < 8; i++) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t)((crc << 1) ^ POLYGEN);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


#ifdef USE_SMARTAUDIO_DPRINTF
static void saPrintSettings(void)
{
    dprintf(("Current status: version: %d\r\n", saDevice.version));
    dprintf(("  mode(0x%x): fmode=%s", saDevice.mode,  (saDevice.mode & 1) ? "freq" : "chan"));
    dprintf((" pit=%s ", (saDevice.mode & 2) ? "on " : "off"));
    dprintf((" inb=%s", (saDevice.mode & 4) ? "on " : "off"));
    dprintf((" outb=%s", (saDevice.mode & 8) ? "on " : "off"));
    dprintf((" lock=%s", (saDevice.mode & 16) ? "unlocked" : "locked"));
    dprintf((" deferred=%s\r\n", (saDevice.mode & 32) ? "on" : "off"));
    dprintf(("  channel: %d ", saDevice.channel));
    dprintf(("freq: %d ", saDevice.freq));
    dprintf(("power: %d ", saDevice.power));
    dprintf(("pitfreq: %d ", saDevice.orfreq));
    dprintf(("\r\n"));
}
#endif

int saDacToPowerIndex(int dac)
{
    for (int idx = 3 ; idx >= 0 ; idx--) {
        if (saPowerTable[idx].valueV1 <= dac) {
            return idx;
        }
    }
    return 0;
}

//
// Autobauding
//

#define SMARTBAUD_MIN 4800
#define SMARTBAUD_MAX 4950
uint16_t sa_smartbaud = SMARTBAUD_MIN;
static int sa_adjdir = 1; // -1=going down, 1=going up
static int sa_baudstep = 50;

static void saAutobaud(void)
{
    if (saStat.pktsent < 10) {
        // Not enough samples collected
        return;
    }

#if 0
    dprintf(("autobaud: %d rcvd %d/%d (%d)\r\n",
        sa_smartbaud, saStat.pktrcvd, saStat.pktsent, ((saStat.pktrcvd * 100) / saStat.pktsent)));
#endif

    if (((saStat.pktrcvd * 100) / saStat.pktsent) >= 70) {
        // This is okay
        saStat.pktsent = 0; // Should be more moderate?
        saStat.pktrcvd = 0;
        return;
    }

    dprintf(("autobaud: adjusting\r\n"));

    if ((sa_adjdir == 1) && (sa_smartbaud == SMARTBAUD_MAX)) {
       sa_adjdir = -1;
       dprintf(("autobaud: now going down\r\n"));
    } else if ((sa_adjdir == -1 && sa_smartbaud == SMARTBAUD_MIN)) {
       sa_adjdir = 1;
       dprintf(("autobaud: now going up\r\n"));
    }

    sa_smartbaud += sa_baudstep * sa_adjdir;

    dprintf(("autobaud: %d\r\n", sa_smartbaud));

    smartAudioSerialPort->vTable->serialSetBaudRate(smartAudioSerialPort, sa_smartbaud);

    saStat.pktsent = 0;
    saStat.pktrcvd = 0;
}

// Transport level variables

static timeUs_t sa_lastTransmissionMs = 0;
static uint8_t sa_outstanding = SA_CMD_NONE; // Outstanding command
static uint8_t sa_osbuf[32]; // Outstanding comamnd frame for retransmission
static int sa_oslen;         // And associate length

static void saProcessResponse(uint8_t *buf, int len)
{
    uint8_t resp = buf[0];

    if (resp == sa_outstanding) {
        sa_outstanding = SA_CMD_NONE;
    } else if ((resp == SA_CMD_GET_SETTINGS_V2) && (sa_outstanding == SA_CMD_GET_SETTINGS)) {
        sa_outstanding = SA_CMD_NONE;
    } else {
        saStat.ooopresp++;
        dprintf(("processResponse: outstanding %d got %d\r\n", sa_outstanding, resp));
    }

    switch (resp) {
    case SA_CMD_GET_SETTINGS_V2: // Version 2 Get Settings
    case SA_CMD_GET_SETTINGS:    // Version 1 Get Settings
        if (len < 7) {
            break;
        }

        saDevice.version = (buf[0] == SA_CMD_GET_SETTINGS) ? 1 : 2;
        saDevice.channel = buf[2];
        saDevice.power = buf[3];
        saDevice.mode = buf[4];
        saDevice.freq = (buf[5] << 8)|buf[6];

        DEBUG_SET(DEBUG_SMARTAUDIO, 0, saDevice.version * 100 + saDevice.mode);
        DEBUG_SET(DEBUG_SMARTAUDIO, 1, saDevice.channel);
        DEBUG_SET(DEBUG_SMARTAUDIO, 2, saDevice.freq);
        DEBUG_SET(DEBUG_SMARTAUDIO, 3, saDevice.power);
        break;

    case SA_CMD_SET_POWER: // Set Power
        break;

    case SA_CMD_SET_CHAN: // Set Channel
        break;

    case SA_CMD_SET_FREQ: // Set Frequency
        if (len < 5) {
            break;
        }

        const uint16_t freq = (buf[2] << 8)|buf[3];

        if (freq & SA_FREQ_GETPIT) {
            saDevice.orfreq = freq & SA_FREQ_MASK;
            dprintf(("saProcessResponse: GETPIT freq %d\r\n", saDevice.orfreq));
        } else if (freq & SA_FREQ_SETPIT) {
            saDevice.orfreq = freq & SA_FREQ_MASK;
            dprintf(("saProcessResponse: SETPIT freq %d\r\n", saDevice.orfreq));
        } else {
            saDevice.freq = freq;
            dprintf(("saProcessResponse: SETFREQ freq %d\r\n", freq));
        }
        break;

    case SA_CMD_SET_MODE: // Set Mode
        dprintf(("saProcessResponse: SET_MODE 0x%x\r\n", buf[2]));
        break;

    default:
        saStat.badcode++;
        return;
    }

    if (memcmp(&saDevice, &saDevicePrev, sizeof(smartAudioDevice_t))) {
#ifdef USE_CMS    //if changes then trigger saCms update
        saCmsResetOpmodel();
#endif
#ifdef USE_SMARTAUDIO_DPRINTF    // Debug
        saPrintSettings();
#endif
    }
    saDevicePrev = saDevice;

#ifdef USE_VTX_COMMON
    // Todo: Update states in saVtxDevice?
#endif

#ifdef USE_CMS
    // Export current device status for CMS
    saCmsUpdate();
    saUpdateStatusString();
#endif
}

//
// Datalink
//

static void saReceiveFramer(uint8_t c)
{

    static enum saFramerState_e {
        S_WAITPRE1, // Waiting for preamble 1 (0xAA)
        S_WAITPRE2, // Waiting for preamble 2 (0x55)
        S_WAITRESP, // Waiting for response code
        S_WAITLEN,  // Waiting for length
        S_DATA,     // Receiving data
        S_WAITCRC,  // Waiting for CRC
    } state = S_WAITPRE1;

    static int len;
    static int dlen;

    switch (state) {
    case S_WAITPRE1:
        if (c == 0xAA) {
            state = S_WAITPRE2;
        } else {
            state = S_WAITPRE1; // Don't need this (no change)
        }
        break;

    case S_WAITPRE2:
        if (c == 0x55) {
            state = S_WAITRESP;
        } else {
            saStat.badpre++;
            state = S_WAITPRE1;
        }
        break;

    case S_WAITRESP:
        sa_rbuf[0] = c;
        state = S_WAITLEN;
        break;

    case S_WAITLEN:
        sa_rbuf[1] = c;
        len = c;

        if (len > SA_MAX_RCVLEN - 2) {
            saStat.badlen++;
            state = S_WAITPRE1;
        } else if (len == 0) {
            state = S_WAITCRC;
        } else {
            dlen = 0;
            state = S_DATA;
        }
        break;

    case S_DATA:
        // XXX Should check buffer overflow (-> saerr_overflow)
        sa_rbuf[2 + dlen] = c;
        if (++dlen == len) {
            state = S_WAITCRC;
        }
        break;

    case S_WAITCRC:
        if (CRC8(sa_rbuf, 2 + len) == c) {
            // Got a response
            saProcessResponse(sa_rbuf, len + 2);
            saStat.pktrcvd++;
        } else if (sa_rbuf[0] & 1) {
            // Command echo
            // XXX There is an exceptional case (V2 response)
            // XXX Should check crc in the command format?
        } else {
            saStat.crc++;
        }
        state = S_WAITPRE1;
        break;
    }
}

static void saSendFrame(uint8_t *buf, int len)
{
    switch (smartAudioSerialPort->identifier) {
        case SERIAL_PORT_SOFTSERIAL1:
        case SERIAL_PORT_SOFTSERIAL2:
            break;
        default:
            serialWrite(smartAudioSerialPort, 0x00); // Generate 1st start bit
            break;
    }

    for (int i = 0 ; i < len ; i++) {
        serialWrite(smartAudioSerialPort, buf[i]);
    }

    serialWrite(smartAudioSerialPort, 0x00);

    sa_lastTransmissionMs = millis();
    saStat.pktsent++;
}

/*
 * Retransmission and command queuing
 *
 *   The transport level support includes retransmission on response timeout
 * and command queueing.
 *
 * Resend buffer:
 *   The smartaudio returns response for valid command frames in no less
 * than 60msec, which we can't wait. So there's a need for a resend buffer.
 *
 * Command queueing:
 *   The driver autonomously sends GetSettings command for auto-bauding,
 * asynchronous to user initiated commands; commands issued while another
 * command is outstanding must be queued for later processing.
 *   The queueing also handles the case in which multiple commands are
 * required to implement a user level command.
 */

// Retransmission

static void saResendCmd(void)
{
    saSendFrame(sa_osbuf, sa_oslen);
}

static void saSendCmd(uint8_t *buf, int len)
{
    for (int i = 0 ; i < len ; i++) {
        sa_osbuf[i] = buf[i];
    }

    sa_oslen = len;
    sa_outstanding = (buf[2] >> 1);

    saSendFrame(sa_osbuf, sa_oslen);
}

// Command queue management

typedef struct saCmdQueue_s {
    uint8_t *buf;
    int len;
} saCmdQueue_t;

#define SA_QSIZE 6     // 1 heartbeat (GetSettings) + 2 commands + 1 slack
static saCmdQueue_t sa_queue[SA_QSIZE];
static uint8_t sa_qhead = 0;
static uint8_t sa_qtail = 0;

static bool saQueueEmpty(void)
{
    return sa_qhead == sa_qtail;
}

static bool saQueueFull(void)
{
    return ((sa_qhead + 1) % SA_QSIZE) == sa_qtail;
}

static void saQueueCmd(uint8_t *buf, int len)
{
    if (saQueueFull()) {
         return;
    }

    sa_queue[sa_qhead].buf = buf;
    sa_queue[sa_qhead].len = len;
    sa_qhead = (sa_qhead + 1) % SA_QSIZE;
}

static void saSendQueue(void)
{
    if (saQueueEmpty()) {
         return;
    }

    saSendCmd(sa_queue[sa_qtail].buf, sa_queue[sa_qtail].len);
    sa_qtail = (sa_qtail + 1) % SA_QSIZE;
}

// Individual commands

static void saGetSettings(void)
{
    static uint8_t bufGetSettings[5] = {0xAA, 0x55, SACMD(SA_CMD_GET_SETTINGS), 0x00, 0x9F};

    saQueueCmd(bufGetSettings, 5);
}

static bool saValidateFreq(uint16_t freq)
{
    return (freq >= VTX_SMARTAUDIO_MIN_FREQUENCY_MHZ && freq <= VTX_SMARTAUDIO_MAX_FREQUENCY_MHZ);
}

static void saDoDevSetFreq(uint16_t freq)
{
    static uint8_t buf[7] = { 0xAA, 0x55, SACMD(SA_CMD_SET_FREQ), 2 };
    static uint8_t switchBuf[7];

    if (freq & SA_FREQ_GETPIT) {
        dprintf(("smartAudioSetFreq: GETPIT\r\n"));
    } else if (freq & SA_FREQ_SETPIT) {
        dprintf(("smartAudioSetFreq: SETPIT %d\r\n", freq & SA_FREQ_MASK));
    } else {
        dprintf(("smartAudioSetFreq: SET %d\r\n", freq));
    }

    buf[4] = (freq >> 8) & 0xff;
    buf[5] = freq & 0xff;
    buf[6] = CRC8(buf, 6);

    // Need to work around apparent SmartAudio bug when going from 'channel'
    // to 'user-freq' mode, where the set-freq command will fail if the freq
    // value is unchanged from the previous 'user-freq' mode
    if ((saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) == 0 && freq == saDevice.freq) {
        memcpy(&switchBuf, &buf, sizeof(buf));
        const uint16_t switchFreq = freq + ((freq == VTX_SMARTAUDIO_MAX_FREQUENCY_MHZ) ? -1 : 1);
        switchBuf[4] = (switchFreq >> 8);
        switchBuf[5] = switchFreq & 0xff;
        switchBuf[6] = CRC8(switchBuf, 6);

        saQueueCmd(switchBuf, 7);
    }

    saQueueCmd(buf, 7);
}

void saSetFreq(uint16_t freq)
{
    saDoDevSetFreq(freq);
}

void saSetPitFreq(uint16_t freq)
{
    saDoDevSetFreq(freq | SA_FREQ_SETPIT);
}

#if 0
static void saGetPitFreq(void)
{
    saDoDevSetFreq(SA_FREQ_GETPIT);
}
#endif

static bool saValidateBandAndChannel(uint8_t band, uint8_t channel)
{
    return (band >= VTX_SMARTAUDIO_MIN_BAND && band <= VTX_SMARTAUDIO_MAX_BAND &&
             channel >= VTX_SMARTAUDIO_MIN_CHANNEL && channel <= VTX_SMARTAUDIO_MAX_CHANNEL);
}

static void saDevSetBandAndChannel(uint8_t band, uint8_t channel)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_CHAN), 1 };

    buf[4] = SA_BANDCHAN_TO_DEVICE_CHVAL(band, channel);
    buf[5] = CRC8(buf, 5);

    saQueueCmd(buf, 6);
}

void saSetBandAndChannel(uint8_t band, uint8_t channel)
{
    saDevSetBandAndChannel(band, channel);
}

void saSetMode(int mode)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_MODE), 1 };

    buf[4] = (mode & 0x3f)|saLockMode;
    buf[5] = CRC8(buf, 5);

    saQueueCmd(buf, 6);
}

static void saDevSetPowerByIndex(uint8_t index)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_POWER), 1 };

    dprintf(("saSetPowerByIndex: index %d\r\n", index));

    if (saDevice.version == 0) {
        // Unknown or yet unknown version.
        return;
    }

    if (index >= VTX_SMARTAUDIO_POWER_COUNT) {
        return;
    }

    buf[4] = (saDevice.version == 1) ? saPowerTable[index].valueV1 : saPowerTable[index].valueV2;
    buf[5] = CRC8(buf, 5);
    saQueueCmd(buf, 6);
}

void saSetPowerByIndex(uint8_t index)
{
    saDevSetPowerByIndex(index);
}

bool vtxSmartAudioInit(void)
{
#ifdef USE_SMARTAUDIO_DPRINTF
    // Setup debugSerialPort

    debugSerialPort = openSerialPort(DPRINTF_SERIAL_PORT, FUNCTION_NONE, NULL, NULL, 115200, MODE_RXTX, 0);
    if (debugSerialPort) {
        setPrintfSerialPort(debugSerialPort);
        dprintf(("smartAudioInit: OK\r\n"));
    }
#endif

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_SMARTAUDIO);
    if (portConfig) {
        portOptions_e portOptions = SERIAL_STOPBITS_2 | SERIAL_BIDIR_NOPULL;
#if defined(USE_VTX_COMMON)
        portOptions = portOptions | (vtxConfig()->halfDuplex ? SERIAL_BIDIR | SERIAL_BIDIR_PP : SERIAL_UNIDIR);
#else
        portOptions = SERIAL_BIDIR;
#endif

        smartAudioSerialPort = openSerialPort(portConfig->identifier, FUNCTION_VTX_SMARTAUDIO, NULL, NULL, 4800, MODE_RXTX, portOptions);
    }

    if (!smartAudioSerialPort) {
        return false;
    }

    vtxCommonSetDevice(&vtxSmartAudio);

    return true;
}

#define SA_INITPHASE_START         0
#define SA_INITPHASE_WAIT_SETTINGS 1 // SA_CMD_GET_SETTINGS was sent and waiting for reply.
#define SA_INITPHASE_WAIT_PITFREQ  2 // SA_FREQ_GETPIT sent and waiting for reply.
#define SA_INITPHASE_DONE          3

static void vtxSAProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs)
{
    UNUSED(vtxDevice);
    UNUSED(currentTimeUs);

    static char initPhase = SA_INITPHASE_START;

    if (smartAudioSerialPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(smartAudioSerialPort) > 0) {
        uint8_t c = serialRead(smartAudioSerialPort);
        saReceiveFramer((uint16_t)c);
    }

    // Re-evaluate baudrate after each frame reception
    saAutobaud();

    switch (initPhase) {
    case SA_INITPHASE_START:
        saGetSettings();
        //saSendQueue();
        initPhase = SA_INITPHASE_WAIT_SETTINGS;
        break;

    case SA_INITPHASE_WAIT_SETTINGS:
        // Don't send SA_FREQ_GETPIT to V1 device; it act as plain SA_CMD_SET_FREQ,
        // and put the device into user frequency mode with uninitialized freq.
        if (saDevice.version) {
            if (saDevice.version == 2) {
                saDoDevSetFreq(SA_FREQ_GETPIT);
                initPhase = SA_INITPHASE_WAIT_PITFREQ;
            } else {
                initPhase = SA_INITPHASE_DONE;
            }
        }
        break;

    case SA_INITPHASE_WAIT_PITFREQ:
        if (saDevice.orfreq) {
            initPhase = SA_INITPHASE_DONE;
        }
        break;

    case SA_INITPHASE_DONE:
        break;
    }

    // Command queue control

    timeMs_t nowMs = millis();             // Don't substitute with "currentTimeUs / 1000"; sa_lastTransmissionMs is based on millis().
    static timeMs_t lastCommandSentMs = 0; // Last non-GET_SETTINGS sent

    if ((sa_outstanding != SA_CMD_NONE) && (nowMs - sa_lastTransmissionMs > SMARTAUDIO_CMD_TIMEOUT)) {
        // Last command timed out
        // dprintf(("process: resending 0x%x\r\n", sa_outstanding));
        // XXX Todo: Resend termination and possible offline transition
        saResendCmd();
    lastCommandSentMs = nowMs;
    } else if (!saQueueEmpty()) {
        // Command pending. Send it.
        // dprintf(("process: sending queue\r\n"));
        saSendQueue();
    lastCommandSentMs = nowMs;
    } else if ((nowMs - lastCommandSentMs < SMARTAUDIO_POLLING_WINDOW) && (nowMs - sa_lastTransmissionMs >= SMARTAUDIO_POLLING_INTERVAL)) {
    //dprintf(("process: sending status change polling\r\n"));
    saGetSettings();
    saSendQueue();
    }
}

#ifdef USE_VTX_COMMON
// Interface to common VTX API

vtxDevType_e vtxSAGetDeviceType(const vtxDevice_t *vtxDevice)
{
    UNUSED(vtxDevice);
    return VTXDEV_SMARTAUDIO;
}

static bool vtxSAIsReady(const vtxDevice_t *vtxDevice)
{
    return vtxDevice!=NULL && !(saDevice.version == 0);
}

static void vtxSASetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    UNUSED(vtxDevice);
    if (saValidateBandAndChannel(band, channel)) {
        saSetBandAndChannel(band - 1, channel - 1);
    }
}

static void vtxSASetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    UNUSED(vtxDevice);
    if (index == 0) {
        // SmartAudio doesn't support power off.
        return;
    }

    saSetPowerByIndex(index - 1);
}

static void vtxSASetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    if (!(vtxSAIsReady(vtxDevice) && (saDevice.version == 2))) {
        return;
    }

    if (onoff) {
        // SmartAudio can not turn pit mode on by software.
        return;
    }

    uint8_t newmode = SA_MODE_CLR_PITMODE;

    if (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) {
        newmode |= SA_MODE_SET_IN_RANGE_PITMODE;
    }

    if (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE) {
        newmode |= SA_MODE_SET_OUT_RANGE_PITMODE;
    }

    saSetMode(newmode);

    return;
}

static void vtxSASetFreq(vtxDevice_t *vtxDevice, uint16_t freq)
{
    UNUSED(vtxDevice);
    if (saValidateFreq(freq)) {
        saSetMode(0);        //need to be in FREE mode to set freq
        saSetFreq(freq);
    }
}

static bool vtxSAGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return false;
    }

    // if in user-freq mode then report band as zero
    *pBand = (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) ? 0 :
        (SA_DEVICE_CHVAL_TO_BAND(saDevice.channel) + 1);
    *pChannel = SA_DEVICE_CHVAL_TO_CHANNEL(saDevice.channel) + 1;
    return true;
}

static bool vtxSAGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return false;
    }

    *pIndex = ((saDevice.version == 1) ? saDacToPowerIndex(saDevice.power) : saDevice.power) + 1;
    return true;
}

static bool vtxSAGetPitMode(const vtxDevice_t *vtxDevice, uint8_t *pOnOff)
{
    if (!(vtxSAIsReady(vtxDevice) && (saDevice.version == 2))) {
        return false;
    }

    *pOnOff = (saDevice.mode & SA_MODE_GET_PITMODE) ? 1 : 0;
    return true;
}

static bool vtxSAGetFreq(const vtxDevice_t *vtxDevice, uint16_t *pFreq)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return false;
    }

    // if not in user-freq mode then convert band/chan to frequency
    *pFreq = (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) ? saDevice.freq :
        vtx58_Bandchan2Freq(SA_DEVICE_CHVAL_TO_BAND(saDevice.channel) + 1,
        SA_DEVICE_CHVAL_TO_CHANNEL(saDevice.channel) + 1);
    return true;
}

static const vtxVTable_t saVTable = {
    .process = vtxSAProcess,
    .getDeviceType = vtxSAGetDeviceType,
    .isReady = vtxSAIsReady,
    .setBandAndChannel = vtxSASetBandAndChannel,
    .setPowerByIndex = vtxSASetPowerByIndex,
    .setPitMode = vtxSASetPitMode,
    .setFrequency = vtxSASetFreq,
    .getBandAndChannel = vtxSAGetBandAndChannel,
    .getPowerIndex = vtxSAGetPowerIndex,
    .getPitMode = vtxSAGetPitMode,
    .getFrequency = vtxSAGetFreq,
};
#endif // VTX_COMMON


#endif // VTX_SMARTAUDIO
