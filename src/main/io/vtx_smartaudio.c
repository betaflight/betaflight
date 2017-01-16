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

#include "platform.h"

#if defined(VTX_SMARTAUDIO) && defined(VTX_CONTROL)

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "string.h"
#include "common/printf.h"
#include "common/utils.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/vtx_common.h"
#include "io/serial.h"
#include "io/vtx_smartaudio.h"
#include "io/vtx_string.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "config/config_master.h"

#include "build/build_config.h"

//#define SMARTAUDIO_DPRINTF
//#define SMARTAUDIO_DEBUG_MONITOR

#ifdef SMARTAUDIO_DPRINTF

#ifdef OMNIBUSF4
#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART3
#else
#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART1
#endif

serialPort_t *debugSerialPort = NULL;
#define dprintf(x) if (debugSerialPort) printf x
#else
#define dprintf(x)
#endif

#include "build/debug.h"

#ifdef CMS
static void saUpdateStatusString(void); // Forward
#endif

static serialPort_t *smartAudioSerialPort = NULL;

#if defined(CMS) || defined(VTX_COMMON)
static const char * const saPowerNames[] = {
    "---", "25 ", "200", "500", "800",
};
#endif

#ifdef VTX_COMMON
static vtxVTable_t saVTable;    // Forward
static vtxDevice_t vtxSmartAudio = {
    .vTable = &saVTable,
    .numBand = 5,
    .numChan = 8,
    .numPower = 4,
    .bandNames = (char **)vtx58BandNames,
    .chanNames = (char **)vtx58ChannelNames,
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

// opmode flags, GET side
#define SA_MODE_GET_FREQ_BY_FREQ            1
#define SA_MODE_GET_PITMODE                 2
#define SA_MODE_GET_IN_RANGE_PITMODE        4
#define SA_MODE_GET_OUT_RANGE_PITMODE       8
#define SA_MODE_GET_UNLOCK                 16
#define SA_MODE_GET_DEFERRED_FREQ          32

#define SA_IS_PITMODE(n) ((n) & SA_MODE_GET_PITMODE)
#define SA_IS_PIRMODE(n) (((n) & SA_MODE_GET_PITMODE) && ((n) & SA_MODE_GET_IN_RANGE_PITMODE))
#define SA_IS_PORMODE(n) (((n) & SA_MODE_GET_PITMODE) && ((n) & SA_MODE_GET_OUT_RANGE_PITMODE))

// opmode flags, SET side
#define SA_MODE_SET_IN_RANGE_PITMODE        1 // Immediate
#define SA_MODE_SET_OUT_RANGE_PITMODE	    2 // Immediate
#define SA_MODE_CLR_PITMODE                 4 // Immediate
#define SA_MODE_SET_UNLOCK                  8
#define SA_MODE_SET_LOCK                    0 // ~UNLOCK
#define SA_MODE_SET_DEFERRED_FREQ          16

// SetFrequency flags, for pit mode frequency manipulation
#define SA_FREQ_GETPIT                      (1 << 14)
#define SA_FREQ_SETPIT                      (1 << 15)
#define SA_FREQ_MASK                        (~(SA_FREQ_GETPIT|SA_FREQ_SETPIT))

// Statistical counters, for user side trouble shooting.

typedef struct smartAudioStat_s {
    uint16_t pktsent;
    uint16_t pktrcvd;
    uint16_t badpre;
    uint16_t badlen;
    uint16_t crc;
    uint16_t ooopresp;
    uint16_t badcode;
} smartAudioStat_t;

static smartAudioStat_t saStat = {
    .pktsent = 0,
    .pktrcvd = 0,
    .badpre = 0,
    .badlen = 0,
    .crc = 0,
    .ooopresp = 0,
    .badcode = 0,
};

typedef struct saPowerTable_s {
    int rfpower;
    int16_t valueV1;
    int16_t valueV2;
} saPowerTable_t;

static saPowerTable_t saPowerTable[] = {
    {  25,   7,   0 },
    { 200,  16,   1 },
    { 500,  25,   2 },
    { 800,  40,   3 },
};

// Last received device ('hard') states

typedef struct smartAudioDevice_s {
    int8_t version;
    int8_t chan;
    int8_t power;
    int8_t mode;
    uint16_t freq;
    uint16_t orfreq;
} smartAudioDevice_t;

static smartAudioDevice_t saDevice = {
    .version = 0,
    .chan = -1,
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
static bool saDeferred = true; // saCms variable?

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


static void saPrintSettings(void)
{
#ifdef SMARTAUDIO_DPRINTF
    dprintf(("Current status: version: %d\r\n", saDevice.version));
    dprintf(("  mode(0x%x): fmode=%s", saDevice.mode,  (saDevice.mode & 1) ? "freq" : "chan"));
    dprintf((" pit=%s ", (saDevice.mode & 2) ? "on " : "off"));
    dprintf((" inb=%s", (saDevice.mode & 4) ? "on " : "off"));
    dprintf((" outb=%s", (saDevice.mode & 8) ? "on " : "off"));
    dprintf((" lock=%s", (saDevice.mode & 16) ? "unlocked" : "locked"));
    dprintf((" deferred=%s\r\n", (saDevice.mode & 32) ? "on" : "off"));
    dprintf(("  chan: %d ", saDevice.chan));
    dprintf(("freq: %d ", saDevice.freq));
    dprintf(("power: %d ", saDevice.power));
    dprintf(("pitfreq: %d ", saDevice.orfreq));
    dprintf(("\r\n"));
#endif
}

static int saDacToPowerIndex(int dac)
{
    int idx;

    for (idx = 3 ; idx >= 0 ; idx--) {
        if (saPowerTable[idx].valueV1 <= dac)
            return(idx);
    }
    return(0);
}

//
// Autobauding
//

#define SMARTBAUD_MIN 4800
#define SMARTBAUD_MAX 4950
uint16_t sa_smartbaud = SMARTBAUD_MIN;
static int sa_adjdir = 1; // -1=going down, 1=going up
static int sa_baudstep = 50;

#define SMARTAUDIO_CMD_TIMEOUT    120

static void saAutobaud(void)
{
    if (saStat.pktsent < 10)
        // Not enough samples collected
        return;

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

static uint32_t sa_lastTransmission = 0;
static uint8_t sa_outstanding = SA_CMD_NONE; // Outstanding command
static uint8_t sa_osbuf[32]; // Outstanding comamnd frame for retransmission
static int sa_oslen;         // And associate length

#ifdef CMS
void saCmsUpdate(void);
#endif

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

    switch(resp) {
    case SA_CMD_GET_SETTINGS_V2: // Version 2 Get Settings
    case SA_CMD_GET_SETTINGS:    // Version 1 Get Settings
        if (len < 7)
            break;

        saDevice.version = (buf[0] == SA_CMD_GET_SETTINGS) ? 1 : 2;
        saDevice.chan = buf[2];
        saDevice.power = buf[3];
        saDevice.mode = buf[4];
        saDevice.freq = (buf[5] << 8)|buf[6];

#ifdef SMARTAUDIO_DEBUG_MONITOR
        debug[0] = saDevice.version * 100 + saDevice.mode;
        debug[1] = saDevice.chan;
        debug[2] = saDevice.freq;
        debug[3] = saDevice.power;
#endif
        break;

    case SA_CMD_SET_POWER: // Set Power
        break;

    case SA_CMD_SET_CHAN: // Set Channel
        break;

    case SA_CMD_SET_FREQ: // Set Frequency
        if (len < 5)
            break;

        uint16_t freq = (buf[2] << 8)|buf[3];

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

    // Debug
    if (memcmp(&saDevice, &saDevicePrev, sizeof(smartAudioDevice_t)))
        saPrintSettings();
    saDevicePrev = saDevice;

#ifdef VTX_COMMON
    // Todo: Update states in saVtxDevice?
#endif

#ifdef CMS
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

    switch(state) {
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
    int i;

    serialWrite(smartAudioSerialPort, 0x00); // Generate 1st start bit

    for (i = 0 ; i < len ; i++)
        serialWrite(smartAudioSerialPort, buf[i]);

    serialWrite(smartAudioSerialPort, 0x00); // XXX Probably don't need this

    sa_lastTransmission = millis();
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
    int i;

    for (i = 0 ; i < len ; i++)
        sa_osbuf[i] = buf[i];

    sa_oslen = len;
    sa_outstanding = (buf[2] >> 1);

    saSendFrame(sa_osbuf, sa_oslen);
}

// Command queue management

typedef struct saCmdQueue_s {
    uint8_t *buf;
    int len;
} saCmdQueue_t;

#define SA_QSIZE 4     // 1 heartbeat (GetSettings) + 2 commands + 1 slack
static saCmdQueue_t sa_queue[SA_QSIZE];
static uint8_t sa_qhead = 0;
static uint8_t sa_qtail = 0;

#ifdef DPRINTF_SMARTAUDIO
static int saQueueLength()
{
    if (sa_qhead >= sa_qtail) {
        return sa_qhead - sa_qtail;
    } else {
        return SA_QSIZE + sa_qhead - sa_qtail;
    }
}
#endif

static bool saQueueEmpty()
{
    return sa_qhead == sa_qtail;
}

static bool saQueueFull()
{
    return ((sa_qhead + 1) % SA_QSIZE) == sa_qtail;
}

static void saQueueCmd(uint8_t *buf, int len)
{
    if (saQueueFull())
         return;

    sa_queue[sa_qhead].buf = buf;
    sa_queue[sa_qhead].len = len;
    sa_qhead = (sa_qhead + 1) % SA_QSIZE;
}

static void saSendQueue(void)
{
    if (saQueueEmpty())
         return;

    saSendCmd(sa_queue[sa_qtail].buf, sa_queue[sa_qtail].len);
    sa_qtail = (sa_qtail + 1) % SA_QSIZE;
}

// Individual commands

static void saGetSettings(void)
{
    static uint8_t bufGetSettings[5] = {0xAA, 0x55, SACMD(SA_CMD_GET_SETTINGS), 0x00, 0x9F};

    saQueueCmd(bufGetSettings, 5);
}

static void saSetFreq(uint16_t freq)
{
    static uint8_t buf[7] = { 0xAA, 0x55, SACMD(SA_CMD_SET_FREQ), 2 };

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

    saQueueCmd(buf, 7);
}

#if 0
static void saSetPitFreq(uint16_t freq)
{
    saSetFreq(freq | SA_FREQ_SETPIT);
}

static void saGetPitFreq(void)
{
    saSetFreq(SA_FREQ_GETPIT);
}
#endif

void saSetBandChan(uint8_t band, uint8_t chan)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_CHAN), 1 };

    buf[4] = band * 8 + chan;
    buf[5] = CRC8(buf, 5);

    saQueueCmd(buf, 6);
}

static void saSetMode(int mode)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_MODE), 1 };

    buf[4] = (mode & 0x3f)|saLockMode;
    buf[5] = CRC8(buf, 5);

    saQueueCmd(buf, 6);
}

void saSetPowerByIndex(uint8_t index)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_POWER), 1 };

    dprintf(("saSetPowerByIndex: index %d\r\n", index));

    if (saDevice.version == 0) {
        // Unknown or yet unknown version.
        return;
    }

    if (index > 3)
        return;

    buf[4] = (saDevice.version == 1) ? saPowerTable[index].valueV1 : saPowerTable[index].valueV2;
    buf[5] = CRC8(buf, 5);
    saQueueCmd(buf, 6);
}

bool smartAudioInit()
{
#ifdef SMARTAUDIO_DPRINTF
    // Setup debugSerialPort

    debugSerialPort = openSerialPort(DPRINTF_SERIAL_PORT, FUNCTION_NONE, NULL, 115200, MODE_RXTX, 0);
    if (debugSerialPort) {
        setPrintfSerialPort(debugSerialPort);
        dprintf(("smartAudioInit: OK\r\n"));
    }
#endif

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_SMARTAUDIO);
    if (portConfig) {
        smartAudioSerialPort = openSerialPort(portConfig->identifier, FUNCTION_VTX_SMARTAUDIO, NULL, 4800, MODE_RXTX, SERIAL_BIDIR|SERIAL_BIDIR_PP);
    }

    if (!smartAudioSerialPort) {
        return false;
    }

    vtxSmartAudio.vTable = &saVTable;
    vtxCommonRegisterDevice(&vtxSmartAudio);

    return true;
}

void vtxSAProcess(uint32_t now)
{
    static bool initialSent = false;

    if (smartAudioSerialPort == NULL)
        return;

    while (serialRxBytesWaiting(smartAudioSerialPort) > 0) {
        uint8_t c = serialRead(smartAudioSerialPort);
        saReceiveFramer((uint16_t)c);
    }

    // Re-evaluate baudrate after each frame reception
    saAutobaud();

    if (!initialSent) {
        saGetSettings();
        saSetFreq(SA_FREQ_GETPIT);
        saSendQueue();
        initialSent = true;
        return;
    }

    if ((sa_outstanding != SA_CMD_NONE)
            && (now - sa_lastTransmission > SMARTAUDIO_CMD_TIMEOUT)) {
        // Last command timed out
        // dprintf(("process: resending 0x%x\r\n", sa_outstanding));
        // XXX Todo: Resend termination and possible offline transition
        saResendCmd();
    } else if (!saQueueEmpty()) {
        // Command pending. Send it.
        // dprintf(("process: sending queue\r\n"));
        saSendQueue();
    } else if (now - sa_lastTransmission >= 1000) {
        // Heart beat for autobauding
        //dprintf(("process: sending heartbeat\r\n"));
        saGetSettings();
        saSendQueue();
    }

#ifdef SMARTAUDIO_TEST_VTX_COMMON
    // Testing VTX_COMMON API
    {
        static uint32_t lastMonitorUs = 0;
        if (cmp32(now, lastMonitorUs) < 5 * 1000 * 1000)
            return;

        static uint8_t monBand;
        static uint8_t monChan;
        static uint8_t monPower;

        vtxCommonGetBandChan(&monBand, &monChan);
        vtxCommonGetPowerIndex(&monPower);
        debug[0] = monBand;
        debug[1] = monChan;
        debug[2] = monPower;
    }
#endif
}

#ifdef VTX_COMMON
// Interface to common VTX API

vtxDevType_e vtxSAGetDeviceType(void)
{
    return VTXDEV_SMARTAUDIO;
}

bool vtxSAIsReady(void)
{
    return !(saDevice.version == 0);
}

void vtxSASetBandChan(uint8_t band, uint8_t chan)
{
    if (band && chan)
        saSetBandChan(band - 1, chan - 1);
}

void vtxSASetPowerByIndex(uint8_t index)
{
    if (index == 0) {
        // SmartAudio doesn't support power off.
        return;
    }

    saSetPowerByIndex(index - 1);
}

void vtxSASetPitmode(uint8_t onoff)
{
    if (!(vtxSAIsReady() && (saDevice.version == 2)))
        return;

    if (onoff) {
        // SmartAudio can not turn pit mode on by software.
        return;
    }

    uint8_t newmode = SA_MODE_CLR_PITMODE;

    if (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE)
        newmode |= SA_MODE_SET_IN_RANGE_PITMODE;

    if (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE)
        newmode |= SA_MODE_SET_OUT_RANGE_PITMODE;

    saSetMode(newmode);

    return true;
}

bool vtxSAGetBandChan(uint8_t *pBand, uint8_t *pChan)
{
    if (!vtxSAIsReady())
        return false;

    *pBand = (saDevice.chan / 8) + 1;
    *pChan = (saDevice.chan % 8) + 1;
    return true;
}

bool vtxSAGetPowerIndex(uint8_t *pIndex)
{
    if (!vtxSAIsReady())
        return false;

    *pIndex = ((saDevice.version == 1) ? saDacToPowerIndex(saDevice.power) : saDevice.power) + 1;
    return true;
}

bool vtxSAGetPitmode(uint8_t *pOnoff)
{
    if (!(vtxSAIsReady() && (saDevice.version == 2)))
        return false;

    *pOnoff = (saDevice.mode & SA_MODE_GET_PITMODE) ? 1 : 0;
    return true;
}

static vtxVTable_t saVTable = {
    .process = vtxSAProcess,
    .getDeviceType = vtxSAGetDeviceType,
    .isReady = vtxSAIsReady,
    .setBandChan = vtxSASetBandChan,
    .setPowerByIndex = vtxSASetPowerByIndex,
    .setPitmode = vtxSASetPitmode,
    .getBandChan = vtxSAGetBandChan,
    .getPowerIndex = vtxSAGetPowerIndex,
    .getPitmode = vtxSAGetPitmode,
};
#endif // VTX_COMMON

#ifdef CMS

// Interface to CMS

// Operational Model and RF modes (CMS)

#define SACMS_OPMODEL_UNDEF        0 // Not known yet
#define SACMS_OPMODEL_FREE         1 // Freestyle model: Power up transmitting
#define SACMS_OPMODEL_RACE         2 // Race model: Power up in pit mode

uint8_t  saCmsOpmodel = SACMS_OPMODEL_UNDEF;

#define SACMS_TXMODE_NODEF         0
#define SACMS_TXMODE_PIT_OUTRANGE  1
#define SACMS_TXMODE_PIT_INRANGE   2
#define SACMS_TXMODE_ACTIVE        3

uint8_t  saCmsRFState;          // RF state; ACTIVE, PIR, POR XXX Not currently used

uint8_t  saCmsBand = 0;
uint8_t  saCmsChan = 0;
uint8_t  saCmsPower = 0;

// Frequency derived from channel table (used for reference in band/chan mode)
uint16_t saCmsFreqRef = 0;

uint16_t saCmsDeviceFreq = 0;

uint8_t  saCmsDeviceStatus = 0;
uint8_t  saCmsPower;
uint8_t  saCmsPitFMode;         // In-Range or Out-Range
uint8_t  saCmsFselMode;          // Channel(0) or User defined(1)

uint16_t saCmsORFreq = 0;       // POR frequency
uint16_t saCmsORFreqNew;        // POR frequency

uint16_t saCmsUserFreq = 0;     // User defined frequency
uint16_t saCmsUserFreqNew;      // User defined frequency

void saCmsUpdate(void)
{
// XXX Take care of pit mode update somewhere???

    if (saCmsOpmodel == SACMS_OPMODEL_UNDEF) {
        // This is a first valid response to GET_SETTINGS.
        saCmsOpmodel = (saDevice.mode & SA_MODE_GET_PITMODE) ? SACMS_OPMODEL_RACE : SACMS_OPMODEL_FREE;

        saCmsFselMode = (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) ? 1 : 0;

        saCmsBand = (saDevice.chan / 8) + 1;
        saCmsChan = (saDevice.chan % 8) + 1;
        saCmsFreqRef = vtx58FreqTable[saDevice.chan / 8][saDevice.chan % 8];

        saCmsDeviceFreq = vtx58FreqTable[saDevice.chan / 8][saDevice.chan % 8];

        if ((saDevice.mode & SA_MODE_GET_PITMODE) == 0) {
            saCmsRFState = SACMS_TXMODE_ACTIVE;
        } else if (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) {
            saCmsRFState = SACMS_TXMODE_PIT_INRANGE;
        } else {
            saCmsRFState = SACMS_TXMODE_PIT_OUTRANGE;
        }

        if (saDevice.version == 2) {
            saCmsPower = saDevice.power + 1; // XXX Take care V1
        } else {
            saCmsPower = saDacToPowerIndex(saDevice.power) + 1;
        }
    }

    saUpdateStatusString();
}

char saCmsStatusString[31] = "- -- ---- ---";
//                            m bc ffff ppp
//                            0123456789012

static long saCmsConfigOpmodelByGvar(displayPort_t *, const void *self);
static long saCmsConfigPitFModeByGvar(displayPort_t *, const void *self);
static long saCmsConfigBandByGvar(displayPort_t *, const void *self);
static long saCmsConfigChanByGvar(displayPort_t *, const void *self);
static long saCmsConfigPowerByGvar(displayPort_t *, const void *self);

static void saUpdateStatusString(void)
{
    if (saDevice.version == 0)
        return;

// XXX These should be done somewhere else
if (saCmsDeviceStatus == 0 && saDevice.version != 0)
    saCmsDeviceStatus = saDevice.version;
if (saCmsORFreq == 0 && saDevice.orfreq != 0)
    saCmsORFreq = saDevice.orfreq;
if (saCmsUserFreq == 0 && saDevice.freq != 0)
    saCmsUserFreq = saDevice.freq;

if (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE)
    saCmsPitFMode = 1;
else
    saCmsPitFMode = 0;

    saCmsStatusString[0] = "-FR"[saCmsOpmodel];
    saCmsStatusString[2] = "ABEFR"[saDevice.chan / 8];
    saCmsStatusString[3] = '1' + (saDevice.chan % 8);

    if ((saDevice.mode & SA_MODE_GET_PITMODE)
       && (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE))
        tfp_sprintf(&saCmsStatusString[5], "%4d", saDevice.orfreq);
    else if (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ)
        tfp_sprintf(&saCmsStatusString[5], "%4d", saDevice.freq);
    else
        tfp_sprintf(&saCmsStatusString[5], "%4d",
            vtx58FreqTable[saDevice.chan / 8][saDevice.chan % 8]);

    saCmsStatusString[9] = ' ';

    if (saDevice.mode & SA_MODE_GET_PITMODE) {
        saCmsStatusString[10] = 'P';
        if (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) {
            saCmsStatusString[11] = 'I';
        } else {
            saCmsStatusString[11] = 'O';
        }
        saCmsStatusString[12] = 'R';
        saCmsStatusString[13] = 0;
    } else {
        tfp_sprintf(&saCmsStatusString[10], "%3d", (saDevice.version == 2) ?  saPowerTable[saDevice.power].rfpower : saPowerTable[saDacToPowerIndex(saDevice.power)].rfpower);
    }
}

static long saCmsConfigBandByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        saCmsBand = 0;
        return 0;
    }

    if (saCmsBand == 0) {
        // Bouce back, no going back to undef state
        saCmsBand = 1;
        return 0;
    }

    if ((saCmsOpmodel == SACMS_OPMODEL_FREE) && !saDeferred)
        saSetBandChan(saCmsBand - 1, saCmsChan - 1);

    saCmsFreqRef = vtx58FreqTable[saCmsBand - 1][saCmsChan - 1];

    return 0;
}

static long saCmsConfigChanByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        saCmsChan = 0;
        return 0;
    }

    if (saCmsChan == 0) {
        // Bounce back; no going back to undef state
        saCmsChan = 1;
        return 0;
    }

    if ((saCmsOpmodel == SACMS_OPMODEL_FREE) && !saDeferred)
        saSetBandChan(saCmsBand - 1, saCmsChan - 1);

    saCmsFreqRef = vtx58FreqTable[saCmsBand - 1][saCmsChan - 1];

    return 0;
}

static long saCmsConfigPowerByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        saCmsPower = 0;
        return 0;
    }

    if (saCmsPower == 0) {
        // Bouce back; no going back to undef state
        saCmsPower = 1;
        return 0;
    }

    if (saCmsOpmodel == SACMS_OPMODEL_FREE)
        saSetPowerByIndex(saCmsPower - 1);

    return 0;
}

static long saCmsConfigPitFModeByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    dprintf(("saCmsConfigPitFmodeByGbar: saCmsPitFMode %d\r\n", saCmsPitFMode));

    if (saCmsPitFMode == 0) {
        saSetMode(SA_MODE_SET_IN_RANGE_PITMODE);
    } else {
        saSetMode(SA_MODE_SET_OUT_RANGE_PITMODE);
    }

    return 0;
}

static long saCmsConfigOpmodelByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    uint8_t opmodel = saCmsOpmodel;

    dprintf(("saCmsConfigOpmodelByGvar: opmodel %d\r\n", opmodel));

    if (opmodel == SACMS_OPMODEL_FREE) {
        // VTX should power up transmitting.
        // Turn off In-Range and Out-Range bits
        saSetMode(0);
    } else if (opmodel == SACMS_OPMODEL_RACE) {
        // VTX should power up in pit mode.
        // Default PitFMode is in-range to prevent users without
        // out-range receivers from getting blinded.
        saCmsPitFMode = 0;
        saCmsConfigPitFModeByGvar(pDisp, self);
    } else {
        // Trying to go back to unknown state; bounce back
        saCmsOpmodel = SACMS_OPMODEL_UNDEF + 1;
    }

    return 0;
}

static const char * const saCmsDeviceStatusNames[] = {
    "OFFL",
    "ONL V1",
    "ONL V2",
};

static OSD_TAB_t saCmsEntOnline = { &saCmsDeviceStatus, 2, saCmsDeviceStatusNames };

static OSD_Entry saCmsMenuStatsEntries[] = {
    { "- SA STATS -", OME_Label, NULL, NULL, 0 },
    { "STATUS",   OME_TAB,    NULL, &saCmsEntOnline,                              DYNAMIC },
    { "BAUDRATE", OME_UINT16, NULL, &(OSD_UINT16_t){ &sa_smartbaud, 0, 0, 0 },    DYNAMIC },
    { "SENT",     OME_UINT16, NULL, &(OSD_UINT16_t){ &saStat.pktsent, 0, 0, 0 },  DYNAMIC },
    { "RCVD",     OME_UINT16, NULL, &(OSD_UINT16_t){ &saStat.pktrcvd, 0, 0, 0 },  DYNAMIC },
    { "BADPRE",   OME_UINT16, NULL, &(OSD_UINT16_t){ &saStat.badpre, 0, 0, 0 },   DYNAMIC },
    { "BADLEN",   OME_UINT16, NULL, &(OSD_UINT16_t){ &saStat.badlen, 0, 0, 0 },   DYNAMIC },
    { "CRCERR",   OME_UINT16, NULL, &(OSD_UINT16_t){ &saStat.crc, 0, 0, 0 },      DYNAMIC },
    { "OOOERR",   OME_UINT16, NULL, &(OSD_UINT16_t){ &saStat.ooopresp, 0, 0, 0 }, DYNAMIC },
    { "BACK",     OME_Back,   NULL, NULL, 0 },
    { NULL,       OME_END,    NULL, NULL, 0 }
};

static CMS_Menu saCmsMenuStats = {
    .GUARD_text = "XSAST",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = saCmsMenuStatsEntries
};

static OSD_TAB_t saCmsEntBand = { &saCmsBand, 5, vtx58BandNames, NULL };

static OSD_TAB_t saCmsEntChan = { &saCmsChan, 8, vtx58ChannelNames, NULL };

static const char * const saCmsPowerNames[] = {
    "---",
    "25 ",
    "200",
    "500",
    "800",
};

static OSD_TAB_t saCmsEntPower = { &saCmsPower, 4, saCmsPowerNames};

static OSD_UINT16_t saCmsEntFreqRef = { &saCmsFreqRef, 5600, 5900, 0 };

static const char * const saCmsOpmodelNames[] = {
    "----",
    "FREE",
    "RACE",
};

static const char * const saCmsFselModeNames[] = {
    "CHAN",
    "USER"
};

static const char * const saCmsPitFModeNames[] = {
    "PIR",
    "POR"
};

static OSD_TAB_t saCmsEntPitFMode = { &saCmsPitFMode, 1, saCmsPitFModeNames };

static long sacms_SetupTopMenu(void); // Forward

static long saCmsConfigFreqModeByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saCmsFselMode == 0) {
        // CHAN
        saSetBandChan(saCmsBand - 1, saCmsChan - 1);
    } else {
        // USER: User frequency mode is only available in FREE opmodel.
        if (saCmsOpmodel == SACMS_OPMODEL_FREE) {
            saSetFreq(saCmsUserFreq);
        } else {
            // Bounce back
            saCmsFselMode = 0;
        }
    }

    sacms_SetupTopMenu();

    return 0;
}

static long saCmsCommence(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saCmsOpmodel == SACMS_OPMODEL_RACE) {
        // Race model
        // Setup band, freq and power.

        saSetBandChan(saCmsBand - 1, saCmsChan - 1);
        saSetPowerByIndex(saCmsPower - 1);

        // If in pit mode, cancel it.

        if (saCmsPitFMode == 0)
            saSetMode(SA_MODE_CLR_PITMODE|SA_MODE_SET_IN_RANGE_PITMODE);
        else
            saSetMode(SA_MODE_CLR_PITMODE|SA_MODE_SET_OUT_RANGE_PITMODE);
    } else {
        // Freestyle model
        // Setup band and freq / user freq
        if (saCmsFselMode == 0)
            saSetBandChan(saCmsBand - 1, saCmsChan - 1);
        else
            saSetFreq(saCmsUserFreq);
    }

    return MENU_CHAIN_BACK;
}

static long saCmsSetPORFreqOnEnter(void)
{
    saCmsORFreqNew = saCmsORFreq;

    return 0;
}

static long saCmsSetPORFreq(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    saSetFreq(saCmsORFreqNew|SA_FREQ_SETPIT);

    return 0;
}

static char *saCmsORFreqGetString(void)
{
    static char pbuf[5];

    tfp_sprintf(pbuf, "%4d", saCmsORFreq);

    return pbuf;
}

static char *saCmsUserFreqGetString(void)
{
    static char pbuf[5];

    tfp_sprintf(pbuf, "%4d", saCmsUserFreq);

    return pbuf;
}

static long saCmsSetUserFreqOnEnter(void)
{
    saCmsUserFreqNew = saCmsUserFreq;

    return 0;
}

static long saCmsSetUserFreq(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    saCmsUserFreq = saCmsUserFreqNew;
    saSetFreq(saCmsUserFreq);

    return 0;
}

static OSD_Entry saCmsMenuPORFreqEntries[] = {
    { "- POR FREQ -", OME_Label,   NULL,             NULL,                                                 0 },

    { "CUR FREQ",     OME_UINT16,  NULL,             &(OSD_UINT16_t){ &saCmsORFreq, 5000, 5900, 0 },       DYNAMIC },
    { "NEW FREQ",     OME_UINT16,  NULL,             &(OSD_UINT16_t){ &saCmsORFreqNew, 5000, 5900, 1 },    0 },
    { "SET",          OME_Funcall, saCmsSetPORFreq,  NULL,                                                 0 },

    { "BACK",         OME_Back,    NULL,             NULL,                                                 0 },
    { NULL,           OME_END,     NULL,             NULL,                                                 0 }
};

static CMS_Menu saCmsMenuPORFreq =
{
    .GUARD_text = "XSAPOR",
    .GUARD_type = OME_MENU,
    .onEnter = saCmsSetPORFreqOnEnter,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = saCmsMenuPORFreqEntries,
};

static OSD_Entry saCmsMenuUserFreqEntries[] = {
    { "- USER FREQ -", OME_Label,   NULL,             NULL,                                                0 },

    { "CUR FREQ",      OME_UINT16,  NULL,             &(OSD_UINT16_t){ &saCmsUserFreq, 5000, 5900, 0 },    DYNAMIC },
    { "NEW FREQ",      OME_UINT16,  NULL,             &(OSD_UINT16_t){ &saCmsUserFreqNew, 5000, 5900, 1 }, 0 },
    { "SET",           OME_Funcall, saCmsSetUserFreq, NULL,                                                0 },

    { "BACK",          OME_Back,    NULL,             NULL,                                                0 },
    { NULL,            OME_END,     NULL,             NULL,                                                0 }
};

static CMS_Menu saCmsMenuUserFreq =
{
    .GUARD_text = "XSAUFQ",
    .GUARD_type = OME_MENU,
    .onEnter = saCmsSetUserFreqOnEnter,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = saCmsMenuUserFreqEntries,
};

static OSD_TAB_t saCmsEntFselMode = { &saCmsFselMode, 1, saCmsFselModeNames };

static OSD_Entry saCmsMenuConfigEntries[] = {
    { "- SA CONFIG -", OME_Label, NULL, NULL, 0 },

    { "OP MODEL",  OME_TAB,     saCmsConfigOpmodelByGvar,              &(OSD_TAB_t){ &saCmsOpmodel, 2, saCmsOpmodelNames }, 0 },
    { "FSEL MODE", OME_TAB,     saCmsConfigFreqModeByGvar,             &saCmsEntFselMode,                                   0 },
    { "PIT FMODE", OME_TAB,     saCmsConfigPitFModeByGvar,             &saCmsEntPitFMode,                                   0 },
    { "POR FREQ",  OME_Submenu, (CMSEntryFuncPtr)saCmsORFreqGetString, &saCmsMenuPORFreq,                                   OPTSTRING },
    { "STATX",     OME_Submenu, cmsMenuChange,                         &saCmsMenuStats,                                     0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu saCmsMenuConfig = {
    .GUARD_text = "XSACFG",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = saCmsMenuConfigEntries
};

static OSD_Entry saCmsMenuCommenceEntries[] = {
    { "CONFIRM", OME_Label,   NULL,          NULL, 0 },

    { "YES",     OME_Funcall, saCmsCommence, NULL, 0 },

    { "BACK",    OME_Back, NULL, NULL, 0 },
    { NULL,      OME_END, NULL, NULL, 0 }
};

static CMS_Menu saCmsMenuCommence = {
    .GUARD_text = "XVTXCOM",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = saCmsMenuCommenceEntries,
};

static OSD_Entry saCmsMenuFreqModeEntries[] = {
    { "- SMARTAUDIO -", OME_Label, NULL, NULL, 0 },

    { "",       OME_Label,   NULL,                                     saCmsStatusString,  DYNAMIC },
    { "FREQ",   OME_Submenu, (CMSEntryFuncPtr)saCmsUserFreqGetString,  &saCmsMenuUserFreq, OPTSTRING },
    { "POWER",  OME_TAB,     saCmsConfigPowerByGvar,                   &saCmsEntPower,     0 },
    { "SET",    OME_Submenu, cmsMenuChange,                            &saCmsMenuCommence, 0 },
    { "CONFIG", OME_Submenu, cmsMenuChange,                            &saCmsMenuConfig,   0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static OSD_Entry saCmsMenuChanModeEntries[] =
{
    { "- SMARTAUDIO -", OME_Label, NULL, NULL, 0 },

    { "",       OME_Label,   NULL,                   saCmsStatusString,  DYNAMIC },
    { "BAND",   OME_TAB,     saCmsConfigBandByGvar,  &saCmsEntBand,      0 },
    { "CHAN",   OME_TAB,     saCmsConfigChanByGvar,  &saCmsEntChan,      0 },
    { "(FREQ)", OME_UINT16,  NULL,                   &saCmsEntFreqRef,   DYNAMIC },
    { "POWER",  OME_TAB,     saCmsConfigPowerByGvar, &saCmsEntPower,     0 },
    { "SET",    OME_Submenu, cmsMenuChange,          &saCmsMenuCommence, 0 },
    { "CONFIG", OME_Submenu, cmsMenuChange,          &saCmsMenuConfig,   0 },

    { "BACK",   OME_Back, NULL, NULL, 0 },
    { NULL,     OME_END, NULL, NULL, 0 }
};

static OSD_Entry saCmsMenuOfflineEntries[] =
{
    { "- VTX SMARTAUDIO -", OME_Label, NULL, NULL, 0 },

    { "",      OME_Label,   NULL,          saCmsStatusString, DYNAMIC },
    { "STATX", OME_Submenu, cmsMenuChange, &saCmsMenuStats,   0 },

    { "BACK",  OME_Back, NULL, NULL, 0 },
    { NULL,    OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuVtxSmartAudio; // Forward

static long sacms_SetupTopMenu(void)
{
    if (saCmsDeviceStatus) {
        if (saCmsFselMode == 0)
            cmsx_menuVtxSmartAudio.entries = saCmsMenuChanModeEntries;
        else
            cmsx_menuVtxSmartAudio.entries = saCmsMenuFreqModeEntries;
    } else {
        cmsx_menuVtxSmartAudio.entries = saCmsMenuOfflineEntries;
    }

    return 0;
}

CMS_Menu cmsx_menuVtxSmartAudio = {
    .GUARD_text = "XVTXSA",
    .GUARD_type = OME_MENU,
    .onEnter = sacms_SetupTopMenu,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = saCmsMenuOfflineEntries,
};

#endif // CMS

#endif // VTX_SMARTAUDIO
