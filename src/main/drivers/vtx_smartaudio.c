#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "platform.h"

#ifdef VTX_SMARTAUDIO

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/vtx_smartaudio.h"
#include "io/serial.h"
#include "fc/runtime_config.h"
#include "config/config_master.h"


//#define SMARTAUDIO_DPRINTF
//#define SMARTAUDIO_DEBUG_MONITOR

#ifdef SMARTAUDIO_DPRINTF
#include "common/printf.h"
#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART3
serialPort_t *debugSerialPort = NULL;
#define dprintf(x) if (debugSerialPort) printf x;
#else
#define dprintf(x)
#endif

#ifdef SMARTAUDIO_DEBUG_MONITOR
#include "build/debug.h"
#endif

static serialPort_t *smartAudioSerialPort = NULL;

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

#define SACMD(cmd) (((cmd) << 1) | 1)

// opmode flags, GET side
#define SA_MODE_GET_FREQ_BY_FREQ            1
#define SA_MODE_GET_PITMODE                 2
#define SA_MODE_GET_IN_RANGE_PITMODE        4
#define SA_MODE_GET_OUT_RANGE_PITMODE       8
#define SA_MODE_GET_UNLOCK                 16

// opmode flags, SET side
#define SA_MODE_SET_IN_RANGE_PITMODE        1
#define SA_MODE_SET_OUT_RANGE_PITMODE	    2
#define SA_MODE_SET_PITMODE                 4
#define SA_MODE_CLR_PITMODE                 4
#define SA_MODE_SET_UNLOCK                  8
#define SA_MODE_SET_LOCK                    0 // ~UNLOCK

// SetFrequency flags, for pit mode frequency manipulation
#define SA_FREQ_GETPIT                      (1 << 14)
#define SA_FREQ_SETPIT                      (1 << 15)

// Error counters, may be good for post production debugging.
static uint16_t saerr_badpre = 0;
static uint16_t saerr_badlen = 0;
static uint16_t saerr_crc = 0;
static uint16_t saerr_oooresp = 0;

// Receive frame reassembly buffer
#define SMARTAUDIO_MAXLEN 32
static uint8_t sa_rbuf[SMARTAUDIO_MAXLEN];

// CRC8 computations

#define POLYGEN 0xd5

static uint8_t CRC8(uint8_t *data, int8_t len)
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

// Last received device states

static int8_t sa_vers = 0; // Will be set to 1 or 2
static int8_t sa_chan = -1;
static int8_t sa_power = -1;
static int8_t sa_opmode = -1;
static uint16_t sa_freq = 0;
static uint16_t sa_pitfreq = 0;
static bool sa_pitfreqpending = false;

// A measure for osd.c that resets on exit:
// masterConfig.{vtx_channel,vtx_power} can not be set at boot time,
// but after a communication with the smartaudio device is established.
// We remember here if channel and power is in sync with masterConfig.

static bool sa_configSynced = false;

static void smartAudioPrintSettings(void)
{
#ifdef SMARTAUDIO_DPRINTF
    static int osa_vers;
    static int osa_chan;
    static int osa_power;
    static int osa_opmode;
    static uint32_t osa_freq;

    if ((osa_vers == sa_vers)
            && (osa_chan == sa_chan)
            && (osa_power == sa_power)
            && (osa_opmode == sa_opmode)
            && (osa_freq == sa_freq))
        return;

    dprintf(("Settings:\r\n"));
    dprintf(("  version: %d\r\n", sa_vers));
    dprintf(("     mode(0x%x): vtx=%s", sa_opmode,  (sa_opmode & 1) ? "freq" : "chan"));
    dprintf((" pit=%s ", (sa_opmode & 2) ? "on " : "off"));
    dprintf((" inb=%s", (sa_opmode & 4) ? "on " : "off"));
    dprintf((" outb=%s", (sa_opmode & 8) ? "on " : "off"));
    dprintf((" lock=%s\r\n", (sa_opmode & 16) ? "unlocked" : "locked"));
    dprintf(("     chan: %d\r\n", sa_chan));
    dprintf(("     freq: %d\r\n", sa_freq));
    dprintf(("    power: %d\r\n", sa_power));
    dprintf(("\r\n"));

    osa_vers = sa_vers;
    osa_chan = sa_chan;
    osa_power = sa_power;
    osa_opmode = sa_opmode;
    osa_freq = sa_freq;
#endif
}

// Autobauding

#define SMARTBAUD_MIN 4800
#define SMARTBAUD_MAX 4950
static int smartbaud = SMARTBAUD_MIN;
static int adjdir = 1; // -1=going down, 1=going up
static int baudstep = 50;

#define SMARTAUDIO_CMD_TIMEOUT    120

// Statistics for autobauding

static int sa_pktsent = 0;
static int sa_pktrcvd = 0;

static void saAutobaud(void)
{
    if (sa_pktsent < 10)
        // Not enough samples collected
        return;

    dprintf(("autobaud: %d rcvd %d/%d (%d)\r\n",
        smartbaud, sa_pktrcvd, sa_pktsent, ((sa_pktrcvd * 100) / sa_pktsent)));

    if (((sa_pktrcvd * 100) / sa_pktsent) >= 70) {
        // This is okay
        sa_pktsent = 0; // Should be more moderate?
        sa_pktrcvd = 0;
        return;
    }

    dprintf(("autobaud: adjusting\r\n"));

    if ((adjdir == 1) && (smartbaud == SMARTBAUD_MAX)) {
       adjdir = -1;
       dprintf(("autobaud: now going down\r\n"));
    } else if ((adjdir == -1 && smartbaud == SMARTBAUD_MIN)) {
       adjdir = 1;
       dprintf(("autobaud: now going up\r\n"));
    }

    smartbaud += baudstep * adjdir;

    dprintf(("autobaud: %d\r\n", smartbaud));

    smartAudioSerialPort->vTable->serialSetBaudRate(smartAudioSerialPort, smartbaud);

    sa_pktsent = 0;
    sa_pktrcvd = 0;
}

// Transport level protocol variables

static uint32_t sa_lastTransmission = 0;
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
        saerr_oooresp++;
        dprintf(("processResponse: outstanding %d got %d\r\n", sa_outstanding, resp));
    }

    switch(resp) {
    case SA_CMD_GET_SETTINGS_V2: // Version 2 Get Settings
    case SA_CMD_GET_SETTINGS: // Version 1 Get Settings
        if (len < 7)
            break;

        sa_vers = (buf[0] == SA_CMD_GET_SETTINGS) ? 1 : 2;
        sa_chan = buf[2];
        sa_power = buf[3];
        sa_opmode = buf[4];
        sa_freq = (buf[5] << 8)|buf[6];

        smartAudioPrintSettings();

#ifdef SMARTAUDIO_DEBUG_MONITOR
        debug[0] = sa_vers * 100 + sa_opmode;
        debug[1] = sa_chan;
        debug[2] = sa_freq;
        debug[3] = sa_power;
#endif

        break;

    case SA_CMD_SET_POWER: // Set Power
        break;

    case SA_CMD_SET_CHAN: // Set Channel
        break;

    case SA_CMD_SET_FREQ: // Set Frequency
        if (len < 5)
            break;

        if (sa_pitfreqpending)
            sa_pitfreq = ((buf[2] << 8)|buf[3]) & ~SA_FREQ_GETPIT;
        break;

    case SA_CMD_SET_MODE: // Set Mode
        dprintf(("resp SET_MODE 0x%x\r\n", buf[2]));
        break;
    }
}

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
            state = S_WAITPRE1; // Don't need this
        }
        break;

    case S_WAITPRE2:
        if (c == 0x55) {
            state = S_WAITRESP;
        } else {
            saerr_badpre++;
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

        if (len > SMARTAUDIO_MAXLEN - 2) {
            saerr_badlen++;
            state = S_WAITPRE1;
        } else if (len == 0) {
            state = S_WAITCRC;
        } else {
            dlen = 0;
            state = S_DATA;
        }
        break;

    case S_DATA:
        sa_rbuf[2 + dlen] = c;
        if (++dlen == len) {
            state = S_WAITCRC;
        }
        break;

    case S_WAITCRC:
        if (CRC8(sa_rbuf, 2 + len) == c) {
            // Response
            saProcessResponse(sa_rbuf, len + 2);
            sa_pktrcvd++;
        } else if (sa_rbuf[0] & 1) {
            // Command echo (may be)
        } else {
            saerr_crc++;
        }
        state = S_WAITPRE1;
        break;
    }
}

// Output framer

static void saSendFrame(uint8_t *buf, int len)
{
    int i;

    serialWrite(smartAudioSerialPort, 0x00);

    for (i = 0 ; i < len ; i++)
        serialWrite(smartAudioSerialPort, buf[i]);

    serialWrite(smartAudioSerialPort, 0x00);

    sa_lastTransmission = millis();
    sa_pktsent++;
}

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

// Command transmission queue and management

typedef struct saCmdQueue_s {
    uint8_t *buf;
    int len;
} saCmdQueue_t;

#define SA_QSIZE 4     // 2 should be enough
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

vtxPowerTable_t saPowerTableV1[] = {
    { " 25",  25,   7 },
    { "200", 200,  16 },
    { "500", 500,  25 },
    { "800", 800,  40 },
    { NULL }
};

vtxPowerTable_t saPowerTableV2[] = {
    { "PIT",   0,  -1 },
    { " 25",  25,   0 },
    { "200", 200,   1 },
    { "500", 500,   2 },
    { "800", 800,   3 },
    { NULL }
};

void smartAudioGetSettings(void)
{
    static uint8_t bufGetSettings[5] = {0xAA, 0x55, 0x03, 0x00, 0x9F};
    saQueueCmd(bufGetSettings, 5);
}

void smartAudioSetFreq(uint16_t freq)
{
    static uint8_t buf[7] = { 0xAA, 0x55, SACMD(SA_CMD_SET_FREQ), 2 };

    dprintf(("smartAudioSetFreq: freq %d\r\n", freq));

    buf[4] = (freq >> 8) & 0xff;
    buf[5] = freq & 0xff;
    buf[6] = CRC8(buf, 6);

    saQueueCmd(buf, 7);

    sa_freq = 0; // Will be read by a following heartbeat
}

#ifdef SMARTAUDIO_EXTENDED_API
void smartAudioSetFreqSetPit(uint16_t freq)
{
    smartAudioSetFreq(freq | SA_FREQ_SETPIT);

    sa_pitfreq = 0; // Will be read by a following heartbeat
}

void smartAudioSetFreqGetPit(void)
{
    smartAudioSetFreq(SA_FREQ_GETPIT);

    sa_pitfreqpending = true;
}

uint16_t smartAudioGetFreq(void)
{
    return sa_freq;
}

uint16_t smartAudioGetPitFreq(void)
{
    return sa_pitfreq;
}
#endif

void smartAudioSetBandChan(int band, int chan)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_CHAN), 1 };

    buf[4] = band * 8 + chan;
    buf[5] = CRC8(buf, 5);

    saQueueCmd(buf, 6);
}

void smartAudioSetPowerByIndex(uint8_t index)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_POWER), 1 };

    dprintf(("smartAudioSetPowerByIndex: index %d\r\n", index));

    if (sa_vers != 1 && sa_vers != 2) {
        // Unknown or yet unknown version.
        return;
    }

    if (index > 4)
        return;

    if (sa_vers == 1) {
        dprintf(("smartAudioSetPowerByIndex: V1 value %d\r\n", 
            saPowerTableV1[index].value));
        buf[4] = saPowerTableV1[index].value;
        buf[5] = CRC8(buf, 5);
        saQueueCmd(buf, 6);
    } else {
        index++; // XXX Skip pit mode for v3.0.0 BFOSD API

        if (index > 0) {
            if (sa_opmode & SA_MODE_GET_PITMODE) {
                // Currently in pit mode; have to deactivate and set power.
                // XXX Have to implement chained request...
            } else {
                dprintf(("smartAudioSetPowerByIndex: V2 value %d\r\n", 
                    saPowerTableV2[index].value));
                buf[4] = saPowerTableV2[index].value;
                buf[5] = CRC8(buf, 5);
                saQueueCmd(buf, 6);
            }
        } else {
            // Pit mode requested.
            // Not implemented yet.
        }
    }
}

#ifdef SMARTAUDIO_EXTENDED_API
void smartAudioSetMode(int mode)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_MODE), 1 };

    buf[4] = mode & 0x0f;
    buf[5] = CRC8(buf, 5);

    saQueueCmd(buf, 6);
}
#endif

void smartAudioInit(void)
{
    portOptions_t portOptions;

#ifdef SMARTAUDIO_DPRINTF
    // Setup debugSerialPort

    debugSerialPort = openSerialPort(DPRINTF_SERIAL_PORT, FUNCTION_NONE, NULL, 115200, MODE_RXTX, 0);
    if (!debugSerialPort) {
        return;
    }
    setPrintfSerialPort(debugSerialPort);
    dprintf(("smartAudioInit: OK\r\n"));
#endif

    portOptions = SERIAL_BIDIR|SERIAL_BIDIR_PP;

    // XXX Fixed at UART2 fow the first cut
    smartAudioSerialPort = openSerialPort(SERIAL_PORT_USART2, FUNCTION_NONE, NULL, 4800, MODE_RXTX, portOptions);

    if (!smartAudioSerialPort) {
        return;
    }
}

#ifdef SMARTAUDIO_EXTENDED_API
bool smartAudioIsReady(void)
{
    return (sa_vers != 0);
}

int smartAudioGetPowerTable(int *pTableSize, vtxPowerTable_t **pTable)
{
    switch (sa_vers) {
    case 0:
        return -1;

    case 1:
        if (pTableSize)
            *pTableSize = 4;

        if (pTable)
            *pTable = saPowerTableV1;

        break;

    case 2:
        if (pTableSize)
            *pTableSize = 5;

        if (pTable)
            *pTable = saPowerTableV2;
    }

    return 0;
}
#endif

#ifdef SMARTAUDIO_PITMODE_DEBUG
void smartAudioPitMode(void)
{
    static int turn = 0;

    if ((turn++ % 2) == 0) {
        smartAudioSetMode(SA_MODE_SET_UNLOCK|SA_MODE_SET_PITMODE|SA_MODE_SET_IN_RANGE_PITMODE);
    } else {
        smartAudioSetMode(SA_MODE_SET_UNLOCK|SA_MODE_CLR_PITMODE);
    }
}
#endif

void smartAudioProcess(uint32_t now)
{
    bool armedState = ARMING_FLAG(ARMED) ? true : false;

    if (armedState)
        return;

    if (smartAudioSerialPort == NULL)
        return;

    while (serialRxBytesWaiting(smartAudioSerialPort) > 0) {
        uint8_t c = serialRead(smartAudioSerialPort);
        saReceiveFramer((uint16_t)c);
    }

    // Evaluate baudrate after each frame reception

    saAutobaud();

    // If we haven't talked to the device, keep trying.

    if (sa_vers == 0) {
        smartAudioGetSettings();
        saSendQueue();
        return;
    } else if (!sa_configSynced) {
        // XXX Should take care of pit mode on boot case.
        // Note that vtx_power = 1 means LOW POWER (25mW).
        smartAudioSetPowerByIndex(masterConfig.vtx_power ? 0 : 1);
        smartAudioSetBandChan(masterConfig.vtx_channel / 8, masterConfig.vtx_channel % 8);
        saSendQueue();
        sa_configSynced = true;
    }

    // 

    if ((sa_outstanding != SA_CMD_NONE)
            && (now - sa_lastTransmission > SMARTAUDIO_CMD_TIMEOUT)) {
        // Last command timed out
        saResendCmd();
    } else if (!saQueueEmpty()) {
        // Command pending. Send it.
        saSendQueue();
    } else if (now - sa_lastTransmission >= 1000) {
        // Heart beat for autobauding

#ifdef SMARTAUDIO_PITMODE_DEBUG
        static int turn = 0;
        if ((turn++ % 2) == 0) {
            smartAudioGetSettings();
        } else {
            smartAudioPitMode();
        }
#else
        smartAudioGetSettings();
#endif
        saSendQueue();
    }
}

// Things to make it work for the first cut on v3.0.0

// This doesn't belong here, really.
uint16_t current_vtx_channel;

// A table that's repeated over and over in every vtx code.
const uint16_t vtx_freq[] =
{
    5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Boacam A
    5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Boscam B
    5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Boscam E
    5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // FatShark
    5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, // RaceBand
};

#endif
