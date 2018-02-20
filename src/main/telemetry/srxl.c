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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_TELEMETRY

#include "build/version.h"

#include "cms/cms.h"
#include "io/displayport_srxl.h"

#include "common/crc.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/feature.h"

#include "io/gps.h"
#include "io/serial.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "io/gps.h"

#include "rx/rx.h"
#include "rx/spektrum.h"
#include "io/spektrum_vtx_control.h"

#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/srxl.h"

#include "drivers/vtx_common.h"
#include "io/vtx_tramp.h"
#include "io/vtx_smartaudio.h"

#define SRXL_CYCLETIME_US           33000 // 33ms, 30 Hz

#define SRXL_ADDRESS_FIRST          0xA5
#define SRXL_ADDRESS_SECOND         0x80
#define SRXL_PACKET_LENGTH          0x15

#define SRXL_FRAMETYPE_TELE_QOS     0x7F
#define SRXL_FRAMETYPE_TELE_RPM     0x7E
#define SRXL_FRAMETYPE_POWERBOX     0x0A
#define SRXL_FRAMETYPE_TELE_FP_MAH  0x34
#define TELE_DEVICE_VTX             0x0D   // Video Transmitter Status
#define SRXL_FRAMETYPE_SID          0x00

static bool srxlTelemetryEnabled;
static uint8_t srxlFrame[SRXL_FRAME_SIZE_MAX];

static void srxlInitializeFrame(sbuf_t *dst)
{
    dst->ptr = srxlFrame;
    dst->end = ARRAYEND(srxlFrame);

    sbufWriteU8(dst, SRXL_ADDRESS_FIRST);
    sbufWriteU8(dst, SRXL_ADDRESS_SECOND);
    sbufWriteU8(dst, SRXL_PACKET_LENGTH);
}

static void srxlFinalize(sbuf_t *dst)
{
    crc16_ccitt_sbuf_append(dst, &srxlFrame[3]); // start at byte 3, since CRC does not include device address and packet length
    sbufSwitchToReader(dst, srxlFrame);
    // write the telemetry frame to the receiver.
    srxlRxWriteTelemetryData(sbufPtr(dst), sbufBytesRemaining(dst));
}

/*
SRXL frame has the structure:
<0xA5><0x80><Length><16-byte telemetry packet><2 Byte CRC of payload>
The <Length> shall be 0x15 (length of the 16-byte telemetry packet + overhead).
*/

/*
typedef struct
{
    UINT8 identifier; // Source device = 0x7F
    UINT8 sID; // Secondary ID
    UINT16 A;
    UINT16 B;
    UINT16 L;
    UINT16 R;
    UINT16 F;
    UINT16 H;
    UINT16 rxVoltage; // Volts, 0.01V increments
} STRU_TELE_QOS;
*/

#define STRU_TELE_QOS_EMPTY_REPORT_COUNT 14

bool srxlFrameQos(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    sbufWriteU8(dst, SRXL_FRAMETYPE_TELE_QOS);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);

    sbufFill(dst, 0xFF, STRU_TELE_QOS_EMPTY_REPORT_COUNT); // Clear remainder

    return true;
}

/*
typedef struct
{
    UINT8 identifier; // Source device = 0x7E
    UINT8 sID; // Secondary ID
    UINT16 microseconds; // microseconds between pulse leading edges
    UINT16 volts; // 0.01V increments
    INT16 temperature; // degrees F
    INT8 dBm_A, // Average signal for A antenna in dBm
    INT8 dBm_B; // Average signal for B antenna in dBm.
    // If only 1 antenna, set B = A
} STRU_TELE_RPM;
*/

#define STRU_TELE_RPM_EMPTY_FIELDS_COUNT 8

bool srxlFrameRpm(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    sbufWriteU8(dst, SRXL_FRAMETYPE_TELE_RPM);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
    sbufWriteU16BigEndian(dst, 0xFFFF);                     // pulse leading edges
    sbufWriteU16BigEndian(dst, getBatteryVoltage() * 10);   // vbat is in units of 0.1V
    sbufWriteU16BigEndian(dst, 0x7FFF);                     // temperature

    sbufFill(dst, 0xFF, STRU_TELE_RPM_EMPTY_FIELDS_COUNT);
    return true;
}

/*
typedef struct
{
    UINT8   identifier;     // Source device = 0x34
    UINT8   sID;            // Secondary ID
    INT16   current_A;      // Instantaneous current, 0.1A (0-3276.8A)
    INT16   chargeUsed_A;   // Integrated mAh used, 1mAh (0-32.766Ah)
    UINT16  temp_A;         // Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
    INT16   current_B;      // Instantaneous current, 0.1A (0-3276.8A)
    INT16   chargeUsed_B;   // Integrated mAh used, 1mAh (0-32.766Ah)
    UINT16  temp_B;         // Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
    UINT16  spare;          // Not used
} STRU_TELE_FP_MAH;
*/

#define FP_MAH_KEEPALIVE_TIME_OUT 2000000 // 2s

bool srxlFrameFlightPackCurrent(sbuf_t *dst, timeUs_t currentTimeUs)
{
    uint16_t amps = getAmperage() / 10;
    uint16_t mah  = getMAhDrawn();
    static uint16_t sentAmps;
    static uint16_t sentMah;
    static timeUs_t lastTimeSentFPmAh = 0;

    timeUs_t keepAlive = currentTimeUs - lastTimeSentFPmAh;

    if ( (amps != sentAmps) || (mah != sentMah) ||
         keepAlive > FP_MAH_KEEPALIVE_TIME_OUT ) {
        sbufWriteU8(dst, SRXL_FRAMETYPE_TELE_FP_MAH);
        sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
        sbufWriteU16(dst, amps);
        sbufWriteU16(dst, mah);
        sbufWriteU16(dst, 0x7fff);            // temp A
        sbufWriteU16(dst, 0xffff);            // Amps B
        sbufWriteU16(dst, 0xffff);            // mAH B
        sbufWriteU16(dst, 0x7fff);            // temp B
        sbufWriteU16(dst, 0xffff);

        sentAmps = amps;
        sentMah = mah;
        lastTimeSentFPmAh = currentTimeUs;
        return true;
    }
    return false;
}

#if defined (USE_SPEKTRUM_CMS_TELEMETRY) && defined (USE_CMS)

// Betaflight CMS using Spektrum Tx telemetry TEXT_GEN sensor as display.

#define SPEKTRUM_SRXL_DEVICE_TEXTGEN (0x0C)     // Text Generator
#define SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS (9)   // Text Generator ROWS
#define SPEKTRUM_SRXL_DEVICE_TEXTGEN_COLS (13)  // Text Generator COLS

/*
typedef struct
{
    UINT8       identifier;
    UINT8       sID;               // Secondary ID
    UINT8       lineNumber;        // Line number to display (0 = title, 1-8 for general, 254 = Refresh backlight, 255 = Erase all text on screen)
    char        text[13];          // 0-terminated text when < 13 chars
} STRU_SPEKTRUM_SRXL_TEXTGEN;
*/

#if ( SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS > SPEKTRUM_SRXL_DEVICE_TEXTGEN_COLS )
static char srxlTextBuff[SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS][SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS];
static bool lineSent[SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS];
#else
static char srxlTextBuff[SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS][SPEKTRUM_SRXL_DEVICE_TEXTGEN_COLS];
static bool lineSent[SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS];
#endif

//**************************************************************************
// API Running in external client task context. E.g. in the CMS task
int spektrumTmTextGenPutChar(uint8_t col, uint8_t row, char c)
{
    if (row < SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS && col < SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS) {
      // Only update and force a tm transmision if something has actually changed.
        if (srxlTextBuff[row][col] != c) {
          srxlTextBuff[row][col] = c;
          lineSent[row] = false;
        }
    }
    return 0;
}
//**************************************************************************

bool srxlFrameText(sbuf_t *dst, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    static uint8_t lineNo = 0;
    int lineCount = 0;

    // Skip already sent lines...
    while (lineSent[lineNo] &&
           lineCount < SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS) {
        lineNo = (lineNo + 1) % SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS;
        lineCount++;
    }

    sbufWriteU8(dst, SPEKTRUM_SRXL_DEVICE_TEXTGEN);
    sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
    sbufWriteU8(dst, lineNo);
    sbufWriteData(dst, srxlTextBuff[lineNo], SPEKTRUM_SRXL_DEVICE_TEXTGEN_COLS);

    lineSent[lineNo] = true;
    lineNo = (lineNo + 1) % SPEKTRUM_SRXL_DEVICE_TEXTGEN_ROWS;

    // Always send something, Always one user frame after the two mandatory frames
    // I.e. All of the three frame prep routines QOS, RPM, TEXT should always return true
    // too keep the "Waltz" sequence intact.
    return true;
}
#endif

#if defined(USE_SPEKTRUM_VTX_TELEMETRY) && defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)

static uint8_t vtxDeviceType;

static void collectVtxTmData(spektrumVtx_t * vtx)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    vtxDeviceType = vtxCommonGetDeviceType(vtxDevice);

    // Collect all data from VTX, if VTX is ready
    if (vtxDevice == NULL || !(vtxCommonGetBandAndChannel(vtxDevice, &vtx->band, &vtx->channel) &&
           vtxCommonGetPitMode(vtxDevice, &vtx->pitMode) &&
           vtxCommonGetPowerIndex(vtxDevice, &vtx->power)) )
        {
            vtx->band    = 0;
            vtx->channel = 0;
            vtx->power   = 0;
            vtx->pitMode = 0;
        }

    vtx->powerValue = 0;
#ifdef USE_SPEKTRUM_REGION_CODES
    vtx->region = SpektrumRegion;
#else
    vtx->region = SPEKTRUM_VTX_REGION_NONE;
#endif
}

// Reverse lookup, device power index to Spektrum power range index.
static void convertVtxPower(spektrumVtx_t * vtx)
    {
        uint8_t const * powerIndexTable = NULL;

        switch (vtxDeviceType) {

#if defined(USE_VTX_TRAMP)
        case VTXDEV_TRAMP:
            powerIndexTable = vtxTrampPi;
            vtx->powerValue = trampPowerTable[vtx->power -1];      // Lookup the device power value, 0-based table vs 1-based index. Doh.
            break;
#endif
#if defined(USE_VTX_SMARTAUDIO)
        case VTXDEV_SMARTAUDIO:
            powerIndexTable = vtxSaPi;
            vtx->powerValue = saPowerTable[vtx->power -1].rfpower;
            break;
#endif
#if defined(USE_VTX_RTC6705)
        case VTXDEV_RTC6705:
            powerIndexTable = vtxRTC6705Pi;
            // No power value table available.Hard code some "knowledge" here. Doh.
            vtx->powerValue = vtx->power == VTX_6705_POWER_200 ? 200 : 25;
            break;
#endif

        case VTXDEV_UNKNOWN:
        case VTXDEV_UNSUPPORTED:
        default:
          break;

        }

        if (powerIndexTable != NULL) {
            for (int i = 0; i < SPEKTRUM_VTX_POWER_COUNT; i++)
                if (powerIndexTable[i] >= vtx->power) {
                    vtx->power = i;                                    // Translate device power index to Spektrum power index.
                    break;
                }
        }
    }

static void convertVtxTmData(spektrumVtx_t * vtx)
{
    // Convert from internal band indexes to Spektrum indexes
    for (int i = 0; i < SPEKTRUM_VTX_BAND_COUNT; i++) {
        if (spek2commonBand[i] == vtx->band) {
            vtx->band = i;
            break;
        }
    }

    // De-bump channel no 1 based interally, 0-based in Spektrum.
    vtx->channel--;

    // Convert Power index to Spektrum ranges, different per brand.
    convertVtxPower(vtx);
}

/*
typedef struct
{
    UINT8		identifier;
    UINT8		sID;	  // Secondary ID
    UINT8		band;	  // VTX Band (0 = Fatshark, 1 = Raceband, 2 = E, 3 = B, 4 = A, 5-7 = Reserved)
    UINT8		channel;  // VTX Channel (0-7)
    UINT8		pit;	  // Pit/Race mode (0 = Race, 1 = Pit). Race = (normal operating) mode. Pit = (reduced power) mode. When PIT is set, it overrides all other power settings
    UINT8		power;	  // VTX Power (0 = Off, 1 = 1mw to 14mW, 2 = 15mW to 25mW, 3 = 26mW to 99mW, 4 = 100mW to 299mW, 5 = 300mW to 600mW, 6 = 601mW+, 7 = manual control)
    UINT16		powerDec; // VTX Power as a decimal 1mw/unit
    UINT8		region;	  // Region (0 = USA, 1 = EU, 0xFF = N/A)
    UINT8		rfu[7];	  // reserved
} STRU_TELE_VTX;
*/

#define STRU_TELE_VTX_RESERVE_COUNT 7
#define VTX_KEEPALIVE_TIME_OUT 2000000 // uS

static bool srxlFrameVTX(sbuf_t *dst, timeUs_t currentTimeUs)
{
    static timeUs_t lastTimeSentVtx = 0;
    static spektrumVtx_t vtxSent;

    spektrumVtx_t vtx;
    collectVtxTmData(&vtx);

    if ((vtxDeviceType != VTXDEV_UNKNOWN) && vtxDeviceType != VTXDEV_UNSUPPORTED) {
        convertVtxTmData(&vtx);

        if ((memcmp(&vtxSent, &vtx, sizeof(spektrumVtx_t)) != 0) ||
            ((currentTimeUs - lastTimeSentVtx) > VTX_KEEPALIVE_TIME_OUT) ) {
            // Fill in the VTX tm structure
            sbufWriteU8(dst, TELE_DEVICE_VTX);
            sbufWriteU8(dst, SRXL_FRAMETYPE_SID);
            sbufWriteU8(dst,  vtx.band);
            sbufWriteU8(dst,  vtx.channel);
            sbufWriteU8(dst,  vtx.pitMode);
            sbufWriteU8(dst,  vtx.power);
            sbufWriteU16(dst, vtx.powerValue);
            sbufWriteU8(dst,  vtx.region);

            sbufFill(dst, 0xFF, STRU_TELE_VTX_RESERVE_COUNT);

            memcpy(&vtxSent, &vtx, sizeof(spektrumVtx_t));
            lastTimeSentVtx = currentTimeUs;
            return true;
        }
    }
    return false;
}
#endif // USE_SPEKTRUM_VTX_TELEMETRY && USE_SPEKTRUM_VTX_CONTROL && USE_VTX_COMMON


// Schedule array to decide how often each type of frame is sent
// The frames are scheduled in sets of 3 frames, 2 mandatory and 1 user frame.
// The user frame type is cycled for each set.
// Example. QOS, RPM,.CURRENT, QOS, RPM, TEXT. QOS, RPM, CURRENT, etc etc

#define SRXL_SCHEDULE_MANDATORY_COUNT  2 // Mandatory QOS and RPM sensors

#define SRXL_FP_MAH_COUNT    1

#if defined (USE_SPEKTRUM_CMS_TELEMETRY) && defined (USE_CMS)
#define SRXL_SCHEDULE_CMS_COUNT  1
#else
#define SRXL_SCHEDULE_CMS_COUNT  0
#endif

#if defined(USE_SPEKTRUM_VTX_TELEMETRY) && defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)
#define SRXL_VTX_TM_COUNT        1
#else
#define SRXL_VTX_TM_COUNT        0
#endif

#define SRXL_SCHEDULE_USER_COUNT (SRXL_FP_MAH_COUNT + SRXL_SCHEDULE_CMS_COUNT + SRXL_VTX_TM_COUNT)
#define SRXL_SCHEDULE_COUNT_MAX  (SRXL_SCHEDULE_MANDATORY_COUNT + 1)
#define SRXL_TOTAL_COUNT         (SRXL_SCHEDULE_MANDATORY_COUNT + SRXL_SCHEDULE_USER_COUNT)

typedef bool (*srxlScheduleFnPtr)(sbuf_t *dst, timeUs_t currentTimeUs);

const srxlScheduleFnPtr srxlScheduleFuncs[SRXL_TOTAL_COUNT] = {
    /* must send srxlFrameQos, Rpm and then alternating items of our own */
    srxlFrameQos,
    srxlFrameRpm,
    srxlFrameFlightPackCurrent,
#if defined(USE_SPEKTRUM_VTX_TELEMETRY) && defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)
    srxlFrameVTX,
#endif
#if defined (USE_SPEKTRUM_CMS_TELEMETRY) && defined (USE_CMS)
    srxlFrameText,
#endif
};


static void processSrxl(timeUs_t currentTimeUs)
{
    static uint8_t srxlScheduleIndex = 0;
    static uint8_t srxlScheduleUserIndex = 0;

    sbuf_t srxlPayloadBuf;
    sbuf_t *dst = &srxlPayloadBuf;
    srxlScheduleFnPtr srxlFnPtr;

    if (srxlScheduleIndex < SRXL_SCHEDULE_MANDATORY_COUNT) {
        srxlFnPtr = srxlScheduleFuncs[srxlScheduleIndex];
    } else {
        srxlFnPtr = srxlScheduleFuncs[srxlScheduleIndex + srxlScheduleUserIndex];
        srxlScheduleUserIndex = (srxlScheduleUserIndex + 1) % SRXL_SCHEDULE_USER_COUNT;

#if defined (USE_SPEKTRUM_CMS_TELEMETRY) && defined (USE_CMS)
        // Boost CMS performance by sending nothing else but CMS Text frames when in a CMS menu.
        // Sideeffect, all other reports are still not sent if user leaves CMS without a proper EXIT.
        if (cmsInMenu &&
            (pCurrentDisplay == &srxlDisplayPort)) {
            srxlFnPtr = srxlFrameText;
        }
#endif

    }

    if (srxlFnPtr) {
        srxlInitializeFrame(dst);
        if (srxlFnPtr(dst, currentTimeUs)) {
            srxlFinalize(dst);
        }
    }
    srxlScheduleIndex = (srxlScheduleIndex + 1) % SRXL_SCHEDULE_COUNT_MAX;
}

void initSrxlTelemetry(void)
{
    // check if there is a serial port open for SRXL telemetry (ie opened by the SRXL RX)
    // and feature is enabled, if so, set SRXL telemetry enabled
    srxlTelemetryEnabled = srxlRxIsActive();
 }

bool checkSrxlTelemetryState(void)
{
    return srxlTelemetryEnabled;
}

/*
 * Called periodically by the scheduler
 */
void handleSrxlTelemetry(timeUs_t currentTimeUs)
{
    static uint32_t srxlLastCycleTime;

    if (!srxlTelemetryEnabled) {
        return;
    }

    // Actual telemetry data only needs to be sent at a low frequency, ie 10Hz
    if (currentTimeUs >= srxlLastCycleTime + SRXL_CYCLETIME_US) {
        srxlLastCycleTime = currentTimeUs;
        processSrxl(currentTimeUs);
    }
}
#endif
