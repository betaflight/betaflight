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

#ifdef TELEMETRY

#include "config/feature.h"
#include "build/version.h"

#include "common/streambuf.h"
#include "common/utils.h"

#include "sensors/battery.h"

#include "io/gps.h"
#include "io/serial.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/gps.h"

#include "flight/imu.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "telemetry/telemetry.h"
#include "telemetry/srxl.h"

#include "fc/config.h"

#define SRXL_CYCLETIME_US           100000 // 100ms, 10 Hz

#define SRXL_ADDRESS_FIRST          0xA5
#define SRXL_ADDRESS_SECOND         0x80
#define SRXL_PACKET_LENGTH          0x15

#define SRXL_FRAMETYPE_TELE_QOS     0x7F
#define SRXL_FRAMETYPE_TELE_RPM     0x7E
#define SRXL_FRAMETYPE_POWERBOX     0x0A
#define SRXL_FRAMETYPE_SID          0x00

static bool srxlTelemetryEnabled;
static uint16_t srxlCrc;
static uint8_t srxlFrame[SRXL_FRAME_SIZE_MAX];

#define SRXL_POLY 0x1021
static uint16_t srxlCrc16(uint16_t crc, uint8_t data)
{
    crc = crc ^ data << 8;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = crc << 1 ^ SRXL_POLY;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

static void srxlInitializeFrame(sbuf_t *dst)
{
    srxlCrc = 0;
    dst->ptr = srxlFrame;
    dst->end = ARRAYEND(srxlFrame);

    sbufWriteU8(dst, SRXL_ADDRESS_FIRST);
    sbufWriteU8(dst, SRXL_ADDRESS_SECOND);
    sbufWriteU8(dst, SRXL_PACKET_LENGTH);
}

static void srxlSerialize8(sbuf_t *dst, uint8_t v)
{
    sbufWriteU8(dst, v);
    srxlCrc = srxlCrc16(srxlCrc, v);
}

static void srxlSerialize16(sbuf_t *dst, uint16_t v)
{
    // Use BigEndian format
    srxlSerialize8(dst,  (v >> 8));
    srxlSerialize8(dst, (uint8_t)v);
}

static void srxlFinalize(sbuf_t *dst)
{
    sbufWriteU16(dst, srxlCrc);
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
void srxlFrameQos(sbuf_t *dst)
{
    srxlSerialize8(dst, SRXL_FRAMETYPE_TELE_QOS);
    srxlSerialize8(dst, SRXL_FRAMETYPE_SID);
    srxlSerialize16(dst, 0xFFFF); // A
    srxlSerialize16(dst, 0xFFFF); // B
    srxlSerialize16(dst, 0xFFFF); // L
    srxlSerialize16(dst, 0xFFFF); // R
    srxlSerialize16(dst, 0xFFFF); // F
    srxlSerialize16(dst, 0xFFFF); // H
    srxlSerialize16(dst, 0xFFFF); // rxVoltage
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
    dBm_B; // Average signal for B antenna in dBm.
    // If only 1 antenna, set B = A
} STRU_TELE_RPM;
*/
void srxlFrameRpm(sbuf_t *dst)
{
    srxlSerialize8(dst, SRXL_FRAMETYPE_TELE_RPM);
    srxlSerialize8(dst, SRXL_FRAMETYPE_SID);
    srxlSerialize16(dst, 0xFFFF); // pulse leading edges
    srxlSerialize16(dst, getVbat() * 10);   // vbat is in units of 0.1V
    srxlSerialize16(dst, 0x7FFF); // temperature
    srxlSerialize8(dst, 0xFF);    // dbmA
    srxlSerialize8(dst, 0xFF);    // dbmB

    /* unused */
    srxlSerialize16(dst, 0xFFFF);
    srxlSerialize16(dst, 0xFFFF);
    srxlSerialize16(dst, 0xFFFF);
}

/*
typedef struct
{
    UINT8 identifier; // Source device = 0x0A
    UINT8 sID; // Secondary ID
    UINT16 volt1; // Volts, 0.01v
    UINT16 volt2; // Volts, 0.01v
    UINT16 capacity1; // mAh, 1mAh
    UINT16 capacity2; // mAh, 1mAh
    UINT16 spare16_1;
    UINT16 spare16_2;
    UINT8 spare;
    UINT8 alarms; // Alarm bitmask (see below)
} STRU_TELE_POWERBOX;
*/
#define TELE_PBOX_ALARM_VOLTAGE_1 (0x01)
#define TELE_PBOX_ALARM_VOLTAGE_2 (0x02)
#define TELE_PBOX_ALARM_CAPACITY_1 (0x04)
#define TELE_PBOX_ALARM_CAPACITY_2 (0x08)
//#define TELE_PBOX_ALARM_RPM (0x10)
//#define TELE_PBOX_ALARM_TEMPERATURE (0x20)
#define TELE_PBOX_ALARM_RESERVED_1 (0x40)
#define TELE_PBOX_ALARM_RESERVED_2 (0x80)

void srxlFramePowerBox(sbuf_t *dst)
{
    srxlSerialize8(dst, SRXL_FRAMETYPE_POWERBOX);
    srxlSerialize8(dst, SRXL_FRAMETYPE_SID);
    srxlSerialize16(dst, getVbat() * 10); // vbat is in units of 0.1V - vbat1
    srxlSerialize16(dst, getVbat() * 10); // vbat is in units of 0.1V - vbat2
    srxlSerialize16(dst, amperage / 10);
    srxlSerialize16(dst, 0xFFFF);

    srxlSerialize16(dst, 0xFFFF); // spare
    srxlSerialize16(dst, 0xFFFF); // spare
    srxlSerialize8(dst, 0xFF); // spare
    srxlSerialize8(dst, 0x00); // ALARMS
}

// schedule array to decide how often each type of frame is sent
#define SRXL_SCHEDULE_COUNT_MAX     3

typedef void (*srxlSchedulePtr)(sbuf_t *dst);
const srxlSchedulePtr srxlScheduleFuncs[SRXL_SCHEDULE_COUNT_MAX] = {
    /* must send srxlFrameQos, Rpm and then alternating items of our own */
    srxlFrameQos,
    srxlFrameRpm,
    srxlFramePowerBox
};

static void processSrxl(void)
{
    static uint8_t srxlScheduleIndex = 0;

    sbuf_t srxlPayloadBuf;
    sbuf_t *dst = &srxlPayloadBuf;

    srxlSchedulePtr srxlPtr = srxlScheduleFuncs[srxlScheduleIndex];
    if (srxlPtr) {
        srxlInitializeFrame(dst);
        srxlPtr(dst);
        srxlFinalize(dst);
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
        processSrxl();
    }
}
#endif
