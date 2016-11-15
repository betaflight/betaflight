#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "platform.h"

#ifdef VTX_SMARTAUDIO

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "string.h"
#include "common/printf.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "io/serial.h"
#include "io/vtx_smartaudio.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#if 0
#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/config_profile.h"
#endif
#include "config/config_master.h"

#include "build/build_config.h"

#define SMARTAUDIO_DPRINTF
//#define SMARTAUDIO_DEBUG_MONITOR

#ifdef SMARTAUDIO_DPRINTF
#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART1
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

// This is not a good design
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
    uint16_t badpre;
    uint16_t badlen;
    uint16_t crc;
    uint16_t ooopresp;
    uint16_t badcode;
} smartAudioStat_t;

static smartAudioStat_t saStat = {
    .badpre = 0,
    .badlen = 0,
    .crc = 0,
    .ooopresp = 0,
    .badcode = 0,
};

// The band/chan to frequency table
// XXX Should really be consolidated among different vtx drivers
static const uint16_t saFreqTable[5][8] =
{
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boacam A
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
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

static uint8_t saLockMode = SA_MODE_SET_UNLOCK;

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


static void smartAudioPrintSettings(void)
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

    for (idx = 0 ; idx < 4 ; idx++) {
        if (saPowerTable[idx].valueV1 <= dac)
            return(idx);
    }
    return(3);
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

// Statistics for autobauding

static int sa_pktsent = 0;
static int sa_pktrcvd = 0;

static void saAutobaud(void)
{
    if (sa_pktsent < 10)
        // Not enough samples collected
        return;

#if 0
    dprintf(("autobaud: %d rcvd %d/%d (%d)\r\n",
        sa_smartbaud, sa_pktrcvd, sa_pktsent, ((sa_pktrcvd * 100) / sa_pktsent)));
#endif

    if (((sa_pktrcvd * 100) / sa_pktsent) >= 70) {
        // This is okay
        sa_pktsent = 0; // Should be more moderate?
        sa_pktrcvd = 0;
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

    sa_pktsent = 0;
    sa_pktrcvd = 0;
}

// Transport level variables

static uint32_t sa_lastTransmission = 0;
static uint8_t sa_outstanding = SA_CMD_NONE; // Outstanding command
static uint8_t sa_osbuf[32]; // Outstanding comamnd frame for retransmission
static int sa_oslen;         // And associate length

#ifdef CMS
void smartAudioCmsUpdate(void);
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
    case SA_CMD_GET_SETTINGS: // Version 1 Get Settings
        if (len < 7)
            break;

        saDevice.version = (buf[0] == SA_CMD_GET_SETTINGS) ? 1 : 2;
        saDevice.chan = buf[2];
        saDevice.power = buf[3];
        saDevice.mode = buf[4];
        saDevice.freq = (buf[5] << 8)|buf[6];

#ifdef CMS
        // Export current device status for CMS
        smartAudioCmsUpdate();
#endif
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
        dprintf(("resp SET_MODE 0x%x\r\n", buf[2]));
        break;

    default:
        saStat.badcode++;
        return;
    }

    // Debug
    if (memcmp(&saDevice, &saDevicePrev, sizeof(smartAudioDevice_t)))
        smartAudioPrintSettings();

    saDevicePrev = saDevice;

#ifdef CMS
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
            state = S_WAITPRE1; // Don't need this
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
            sa_pktrcvd++;
        } else if (sa_rbuf[0] & 1) {
            // Command echo (may be)
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
    sa_pktsent++;
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

static void saSetPitFreq(uint16_t freq)
{
    saSetFreq(freq | SA_FREQ_SETPIT);
}

static void saGetPitFreq(void)
{
    saSetFreq(SA_FREQ_GETPIT);
}

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

    buf[4] = (mode & 0x1f)|saLockMode;
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

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_CONTROL);
    if (portConfig) {
        smartAudioSerialPort = openSerialPort(portConfig->identifier, FUNCTION_VTX_CONTROL, NULL, 4800, MODE_RXTX, SERIAL_BIDIR|SERIAL_BIDIR_PP);
    }

    if (!smartAudioSerialPort) {
        return false;
    }

    return true;
}

void smartAudioProcess(uint32_t now)
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
        saGetPitFreq();
        saSendQueue();
    saSetPitFreq(5705);
        initialSent = true;
        return;
    }

    if ((sa_outstanding != SA_CMD_NONE)
            && (now - sa_lastTransmission > SMARTAUDIO_CMD_TIMEOUT)) {
        // Last command timed out
        // dprintf(("process: resending 0x%x\r\n", sa_outstanding));
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
}

#ifdef CMS

// CMS menu variables

// Operational Model and RF modes (CMS)

#define SA_OPMODEL_UNDEF        0 // Not known yet
#define SA_OPMODEL_FREE         1 // Power up transmitting
#define SA_OPMODEL_PIT          2 // Power up in pit mode

#define SA_TXMODE_NODEF         0
#define SA_TXMODE_PIT_OUTRANGE  1
#define SA_TXMODE_PIT_INRANGE   2
#define SA_TXMODE_ACTIVE        3

uint8_t saCmsOpmodel = SA_OPMODEL_UNDEF;
uint8_t smartAudioBand = 0;
uint8_t smartAudioChan = 0;
uint8_t smartAudioPower = 0;
uint16_t smartAudioFreq = 0;

uint8_t smartAudioOpModel = 0;
uint8_t smartAudioStatus = 0;
uint8_t smartAudioPower;
uint8_t smartAudioTxMode;      // RF state; ACTIVE, PIR, POR
uint8_t smartAudioPitFMode;    // In-Range or Out-Range
uint16_t smartAudioORFreq = 0;     // POR frequency
uint8_t smartAudioFreqMode;    // Channel or User defined
uint16_t smartAudioUserFreq = 0;   // User defined frequency


void smartAudioCmsUpdate(void)
{
    if (saCmsOpmodel == SA_OPMODEL_UNDEF) {
        // This is a first valid response to GET_SETTINGS.
        saCmsOpmodel = (saDevice.mode & SA_MODE_GET_PITMODE) ? SA_OPMODEL_PIT : SA_OPMODEL_FREE;
    }

    smartAudioBand = (saDevice.chan / 8) + 1;
    smartAudioChan = (saDevice.chan % 8) + 1;
    smartAudioFreq = saFreqTable[saDevice.chan / 8][saDevice.chan % 8];

    if ((saDevice.mode & SA_MODE_GET_PITMODE) == 0) {
        smartAudioTxMode = SA_TXMODE_ACTIVE;
    } else if (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) {
        smartAudioTxMode = SA_TXMODE_PIT_INRANGE;
    } else {
        smartAudioTxMode = SA_TXMODE_PIT_OUTRANGE;
    }

    if (saDevice.version == 2) {
        smartAudioPower = saDevice.power + 1; // XXX Take care V1
    } else {
        smartAudioPower = saDacToPowerIndex(saDevice.power) + 1;
    }

    saUpdateStatusString();
}

char saCmsStatusString[31] = "- -- ---- ---";
//                            m bc ffff ppp
//                            0123456789012

long saConfigureOpModelByGvar(displayPort_t *, const void *self);
long saConfigurePitFModeByGvar(displayPort_t *, const void *self);
long saConfigureBandByGvar(displayPort_t *, const void *self);
long saConfigureChanByGvar(displayPort_t *, const void *self);
long saConfigurePowerByGvar(displayPort_t *, const void *self);

static void saUpdateStatusString(void)
{
    if (saDevice.version == 0)
        return;

// XXX These should be done somewhere else
if (smartAudioStatus == 0 && saDevice.version != 0)
    smartAudioStatus = saDevice.version;
if (smartAudioORFreq == 0 && saDevice.orfreq != 0)
    smartAudioORFreq = saDevice.orfreq;
if (smartAudioUserFreq == 0 && saDevice.freq != 0)
    smartAudioUserFreq = saDevice.freq;
if (smartAudioOpModel == 0 && saCmsOpmodel != 0)
    smartAudioOpModel = saCmsOpmodel + 1;

    saCmsStatusString[0] = "-FP"[saCmsOpmodel];
    saCmsStatusString[2] = "ABEFR"[saDevice.chan / 8];
    saCmsStatusString[3] = '1' + (saDevice.chan % 8);

    if ((saDevice.mode & SA_MODE_GET_PITMODE)
       && (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE))
        tfp_sprintf(&saCmsStatusString[5], "%4d", saDevice.orfreq);
    else if (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ)
        tfp_sprintf(&saCmsStatusString[5], "%4d", saDevice.freq);
    else
        tfp_sprintf(&saCmsStatusString[5], "%4d",
            saFreqTable[saDevice.chan / 8][saDevice.chan % 8]);

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

static long sacms_SetupTopMenu(void);

long saConfigureBandByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        smartAudioBand = 0;
        return 0;
    }

    if (smartAudioBand == 0) {
        // Bouce back, no going back to undef state
        smartAudioBand = 1;
        return 0;
    }

    saSetBandChan(smartAudioBand - 1, smartAudioChan - 1);

    return 0;
}

long saConfigureChanByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

dprintf(("saConfigureBandByGvar: called\r\n"));

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        smartAudioChan = 0;
        return 0;
    }

    if (smartAudioChan == 0) {
        // Bounce back; no going back to undef state
        smartAudioChan = 1;
        return 0;
    }

dprintf(("saConfigureBandByGvar: calling saSetBandChan\r\n"));

    saSetBandChan(smartAudioBand - 1, smartAudioChan - 1);

    return 0;
}

long saConfigurePowerByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        smartAudioPower = 0;
        return 0;
    }

    if (smartAudioPower == 0) {
        // Bouce back; no going back to undef state
        smartAudioPower = 1;
        return 0;
    }

    saSetPowerByIndex(smartAudioPower - 1);

    return 0;
}

long saConfigurePitFModeByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    dprintf(("saConfigurePitFmodeByGbar: smartAudioPitFMode %d\r\n", smartAudioPitFMode));

    if (smartAudioPitFMode == 0) {
        saSetMode(SA_MODE_SET_IN_RANGE_PITMODE);
    } else {
        saSetMode(SA_MODE_SET_OUT_RANGE_PITMODE);
    }

    return 0;
}

long saConfigureOpModelByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    uint8_t opmodel = smartAudioOpModel;

    dprintf(("saConfigureOpModelByGvar: opmodel %d\r\n", opmodel));

    if (opmodel == SA_OPMODEL_FREE) {
        // VTX should power up transmitting.
        // Turn off In-Range and Out-Range bits
        saSetMode(0);
    } else if (opmodel == SA_OPMODEL_PIT) {
        // VTX should power up in pit mode.
        // Setup In-Range or Out-Range bits
        saConfigurePitFModeByGvar(pDisp, self);
    }

    return 0;
}

static const char * const smartAudioStatusNames[] = {
    "OFFL",
    "ONL V1",
    "ONL V2",
};

OSD_TAB_t cmsEntOnline = { &smartAudioStatus, 2, smartAudioStatusNames };
OSD_UINT16_t cmsEntBaudrate = { &sa_smartbaud, 0, 0, 0 };
OSD_UINT16_t cmsEntStatBadpre = { &saStat.badpre, 0, 0, 0 };
OSD_UINT16_t cmsEntStatBadlen = { &saStat.badlen, 0, 0, 0 };
OSD_UINT16_t cmsEntStatCrcerr = { &saStat.crc, 0, 0, 0 };
OSD_UINT16_t cmsEntStatOooerr = { &saStat.ooopresp, 0, 0, 0 };

OSD_Entry menu_smartAudioStatsEntries[] = {
    { "- SA STATS -", OME_Label, NULL, NULL, 0 },
    { "STATUS", OME_TAB, NULL, &cmsEntOnline, DYNAMIC },
    { "BAUDRATE", OME_UINT16, NULL, &cmsEntBaudrate, DYNAMIC },
    { "BADPRE", OME_UINT16, NULL, &cmsEntStatBadpre, DYNAMIC },
    { "BADLEN", OME_UINT16, NULL, &cmsEntStatBadlen, DYNAMIC },
    { "CRCERR", OME_UINT16, NULL, &cmsEntStatCrcerr, DYNAMIC },
    { "OOOERR", OME_UINT16, NULL, &cmsEntStatOooerr, DYNAMIC },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu menu_smartAudioStats = {
    .GUARD_text = "XSAST",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menu_smartAudioStatsEntries
};

static const char * const smartAudioBandNames[] = {
    "--------",
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
};

OSD_TAB_t cmsEntBand = { &smartAudioBand, 5, &smartAudioBandNames[0], NULL };

static const char * const smartAudioChanNames[] = {
    "-", "1", "2", "3", "4", "5", "6", "7", "8",
};

OSD_TAB_t cmsEntChan = { &smartAudioChan, 8, &smartAudioChanNames[0], NULL };

static const char * const smartAudioPowerNames[] = {
    "---",
    "25 ",
    "200",
    "500",
    "800",
};

OSD_TAB_t cmsEntPower = { &smartAudioPower, 4, &smartAudioPowerNames[0]};

static const char * const smartAudioTxModeNames[] = {
    "------",
    "PIT-OR",
    "PIT-IR",
    "ACTIVE",
};

OSD_TAB_t cmsEntTxMode = { &smartAudioTxMode, 3, &smartAudioTxModeNames[0]};

OSD_UINT16_t cmsEntFreq = { &smartAudioFreq, 5600, 5900, 0 };

static const char * const smartAudioOpModelNames[] = {
    "----",
    "FREE",
    "PIT ",
};

OSD_TAB_t cmsEntOpModel = { &smartAudioOpModel, 2, &smartAudioOpModelNames[0] };

static const char * const smartAudioPitFModeNames[] = {
    "IN-R ",
    "OUT-R"
};

OSD_TAB_t cmsEntPitFMode = { &smartAudioPitFMode, 1, &smartAudioPitFModeNames[0] };

OSD_UINT16_t cmsEntORFreq = { &smartAudioORFreq, 5000, 5900, 1 };

static const char * const smartAudioFreqModeNames[] = {
    "CHAN",
    "USER"
};

OSD_TAB_t cmsEntFreqMode = { &smartAudioFreqMode, 1, smartAudioFreqModeNames };

OSD_UINT16_t cmsEntUserFreq = { &smartAudioUserFreq, 5000, 5900, 1 };

long saConfigureUserFreqByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    saSetFreq(smartAudioFreq);

    return 0;
}

long saConfigureFreqModeByGvar(displayPort_t *pDisp, const void *self)
{
    if (smartAudioFreqMode == 0) {
        // CHAN
        saSetBandChan(smartAudioBand - 1, smartAudioChan - 1);
    } else {
        // USER
        saConfigureUserFreqByGvar(pDisp, self);
    }

    sacms_SetupTopMenu();

    return 0;
}

long saClearPitMode(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (smartAudioPitFMode == 0)
        saSetMode(SA_MODE_CLR_PITMODE|SA_MODE_SET_IN_RANGE_PITMODE);
    else
        saSetMode(SA_MODE_CLR_PITMODE|SA_MODE_SET_OUT_RANGE_PITMODE);

    return 0;
}

OSD_Entry menu_smartAudioConfigEntries[] = {
    { "- SA CONFIG -", OME_Label, NULL, NULL, 0 },
    { "OP MODEL", OME_TAB, saConfigureOpModelByGvar, &cmsEntOpModel, 0 },
    { "PIT FMODE", OME_TAB, saConfigurePitFModeByGvar, &cmsEntPitFMode, 0 },
    { "POR FREQ", OME_UINT16, NULL, &cmsEntORFreq, 0 },
    { "FREQ MODE", OME_TAB, saConfigureFreqModeByGvar, &cmsEntFreqMode, 0 },
    { "STATX", OME_Submenu, cmsMenuChange, &menu_smartAudioStats, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu menu_smartAudioConfig = {
    .GUARD_text = "XSACFG",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menu_smartAudioConfigEntries
};

OSD_Entry saMenuFreqModeEntries[] = {
    { "- SMARTAUDIO -", OME_Label, NULL, NULL, 0 },
    { "", OME_Label, NULL, saCmsStatusString, DYNAMIC },
    { "FREQ", OME_UINT16, NULL, &cmsEntUserFreq, 0 },
    { "POWER", OME_TAB, saConfigurePowerByGvar, &cmsEntPower, 0 },
    { "START", OME_Funcall, saClearPitMode, NULL, 0 },
    { "CONFIG", OME_Submenu, cmsMenuChange, &menu_smartAudioConfig, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

OSD_Entry saMenuChanModeEntries[] =
{
    { "- SMARTAUDIO -", OME_Label, NULL, NULL, 0 },
    { "", OME_Label, NULL, saCmsStatusString, DYNAMIC },
    { "BAND", OME_TAB, saConfigureBandByGvar, &cmsEntBand, 0 },
    { "CHAN", OME_TAB, saConfigureChanByGvar, &cmsEntChan, 0 },
    { "FREQ", OME_UINT16, NULL, &cmsEntFreq, DYNAMIC },
    { "POWER", OME_TAB, saConfigurePowerByGvar, &cmsEntPower, 0 },
    { "START", OME_Funcall, saClearPitMode, NULL, 0 },
    { "CONFIG", OME_Submenu, cmsMenuChange, &menu_smartAudioConfig, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

OSD_Entry saMenuOfflineEntries[] =
{
    { "- VTX SMARTAUDIO -", OME_Label, NULL, NULL, 0 },
    { "", OME_Label, NULL, saCmsStatusString, DYNAMIC },
    { "STATX", OME_Submenu, cmsMenuChange, &menu_smartAudioStats, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuVtxSmartAudio;

static long sacms_SetupTopMenu(void)
{
    if (smartAudioStatus) {
        if (smartAudioFreqMode == 0)
            cmsx_menuVtxSmartAudio.entries = saMenuChanModeEntries;
        else
            cmsx_menuVtxSmartAudio.entries = saMenuFreqModeEntries;
    } else {
        cmsx_menuVtxSmartAudio.entries = saMenuOfflineEntries;
    }

    return 0;
}

CMS_Menu cmsx_menuVtxSmartAudio = {
    .GUARD_text = "XVTXSA",
    .GUARD_type = OME_MENU,
    .onEnter = sacms_SetupTopMenu,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = saMenuOfflineEntries,
};

#endif // CMS

#endif // VTX_SMARTAUDIO
