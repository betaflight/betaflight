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

/*
 * LightTelemetry from KipK
 *
 * Minimal one way telemetry protocol for really low bitrates (1200/2400 bauds).
 * Effective for ground OSD, groundstation HUD and Antenna tracker
 * http://www.wedontneednasa.com/2014/02/lighttelemetry-v2-en-route-to-ground-osd.html
 *
 * This implementation is for LTM v2 > 2400 baud rates
 *
 * Cleanflight implementation by Jonathan Hudson
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"


#if defined(TELEMETRY_LTM)

#include "build/build_config.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/diagnostics.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/pitotmeter.h"

#include "telemetry/ltm.h"
#include "telemetry/telemetry.h"


#define TELEMETRY_LTM_INITIAL_PORT_MODE MODE_TX
#define LTM_CYCLETIME   100
#define LTM_SCHEDULE_SIZE (1000/LTM_CYCLETIME)

extern uint16_t rssi;           // FIXME dependency on mw.c
static serialPort_t *ltmPort;
static serialPortConfig_t *portConfig;
static bool ltmEnabled;
static portSharing_e ltmPortSharing;
static uint8_t ltmFrame[LTM_MAX_MESSAGE_SIZE];
static uint8_t ltm_x_counter;

static void ltm_initialise_packet(sbuf_t *dst)
{
    dst->ptr = ltmFrame;
    dst->end = ARRAYEND(ltmFrame);

    sbufWriteU8(dst, '$');
    sbufWriteU8(dst, 'T');
}

static void ltm_finalise(sbuf_t *dst)
{
    uint8_t crc = 0;
    for (const uint8_t *ptr = &ltmFrame[3]; ptr < dst->ptr; ++ptr) {
        crc ^= *ptr;
    }
    sbufWriteU8(dst, crc);
    sbufSwitchToReader(dst, ltmFrame);
    serialWriteBuf(ltmPort, sbufPtr(dst), sbufBytesRemaining(dst));
}

#if defined(GPS)
/*
 * GPS G-frame 5Hhz at > 2400 baud
 * LAT LON SPD ALT SAT/FIX
 */
void ltm_gframe(sbuf_t *dst)
{
    uint8_t gps_fix_type = 0;
    int32_t ltm_lat = 0, ltm_lon = 0, ltm_alt = 0, ltm_gs = 0;

    if (sensors(SENSOR_GPS)) {
        if (gpsSol.fixType == GPS_NO_FIX)
            gps_fix_type = 1;
        else if (gpsSol.fixType == GPS_FIX_2D)
            gps_fix_type = 2;
        else if (gpsSol.fixType == GPS_FIX_3D)
            gps_fix_type = 3;

        ltm_lat = gpsSol.llh.lat;
        ltm_lon = gpsSol.llh.lon;
        ltm_gs = gpsSol.groundSpeed / 100;
    }

#if defined(NAV)
    ltm_alt = getEstimatedActualPosition(Z); // cm
#else
    ltm_alt = sensors(SENSOR_GPS) ? gpsSol.llh.alt : 0; // cm
#endif

    sbufWriteU8(dst, 'G');
    sbufWriteU32(dst, ltm_lat);
    sbufWriteU32(dst, ltm_lon);
    sbufWriteU8(dst, (uint8_t)ltm_gs);
    sbufWriteU32(dst, ltm_alt);
    sbufWriteU8(dst, (gpsSol.numSat << 2) | gps_fix_type);
}
#endif

/*
 * Sensors S-frame 5Hhz at > 2400 baud
 * VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD
 * Flight mode(0-19):
 *     0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon,
 *     4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
 *     8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints,
 *     11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe,
 *     15: LAND, 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
 */

void ltm_sframe(sbuf_t *dst)
{
    uint8_t lt_flightmode;

    if (FLIGHT_MODE(PASSTHRU_MODE))
        lt_flightmode = 0;
    else if (FLIGHT_MODE(NAV_WP_MODE))
        lt_flightmode = 10;
    else if (FLIGHT_MODE(NAV_RTH_MODE))
        lt_flightmode = 13;
    else if (FLIGHT_MODE(NAV_POSHOLD_MODE))
        lt_flightmode = 9;
    else if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
        lt_flightmode = 8;
    else if (FLIGHT_MODE(HEADFREE_MODE) || FLIGHT_MODE(HEADING_MODE))
        lt_flightmode = 11;
    else if (FLIGHT_MODE(ANGLE_MODE))
        lt_flightmode = 2;
    else if (FLIGHT_MODE(HORIZON_MODE))
        lt_flightmode = 3;
    else
        lt_flightmode = 1;      // Rate mode

    uint8_t lt_statemode = (ARMING_FLAG(ARMED)) ? 1 : 0;
    if (failsafeIsActive())
        lt_statemode |= 2;
    sbufWriteU8(dst, 'S');
    sbufWriteU16(dst, vbat * 100);    //vbat converted to mv
    sbufWriteU16(dst, (uint16_t)constrain(mAhDrawn, 0, 0xFFFF));    // current mAh (65535 mAh max)
    sbufWriteU8(dst, (uint8_t)((rssi * 254) / 1023));        // scaled RSSI (uchar)
#if defined(PITOT)
    sbufWriteU8(dst, sensors(SENSOR_PITOT) ? pitot.airSpeed / 100.0f : 0);  // in m/s
#else
    sbufWriteU8(dst, 0);
#endif
    sbufWriteU8(dst, (lt_flightmode << 2) | lt_statemode);
}

/*
 * Attitude A-frame - 10 Hz at > 2400 baud
 *  PITCH ROLL HEADING
 */
void ltm_aframe(sbuf_t *dst)
{
    sbufWriteU8(dst, 'A');
    sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.pitch));
    sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.roll));
    sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
}

#if defined(GPS)
/*
 * OSD additional data frame, 1 Hz rate
 *  This frame will be ignored by Ghettostation, but processed by GhettOSD if it is used as standalone onboard OSD
 *  home pos, home alt, direction to home
 */
void ltm_oframe(sbuf_t *dst)
{
    sbufWriteU8(dst, 'O');
    sbufWriteU32(dst, GPS_home.lat);
    sbufWriteU32(dst, GPS_home.lon);
    sbufWriteU32(dst, GPS_home.alt);
    sbufWriteU8(dst, 1);                 // OSD always ON
    sbufWriteU8(dst, STATE(GPS_FIX_HOME) ? 1 : 0);
}
#endif

/*
 * Extended information data frame, 1 Hz rate
 *  This frame is intended to report extended GPS and NAV data, however at the moment it contains only HDOP value
 */
void ltm_xframe(sbuf_t *dst)
{
    uint8_t sensorStatus =
        (isHardwareHealthy() ? 0 : 1) << 0;     // bit 0 - hardware failure indication (1 - something is wrong with the hardware sensors)

    sbufWriteU8(dst, 'X');
#if defined(GPS)
    sbufWriteU16(dst, gpsSol.hdop);
#else
    sbufWriteU16(dst, 9999);
#endif
    sbufWriteU8(dst, sensorStatus);
    sbufWriteU8(dst, ltm_x_counter);
    sbufWriteU8(dst, getDisarmReason());
    sbufWriteU8(dst, 0);
    ltm_x_counter++; // overflow is OK
}

#if defined(NAV)
/** OSD additional data frame, ~4 Hz rate, navigation system status
 */
void ltm_nframe(sbuf_t *dst)
{
    sbufWriteU8(dst, 'N');
    sbufWriteU8(dst, NAV_Status.mode);
    sbufWriteU8(dst, NAV_Status.state);
    sbufWriteU8(dst, NAV_Status.activeWpAction);
    sbufWriteU8(dst, NAV_Status.activeWpNumber);
    sbufWriteU8(dst, NAV_Status.error);
    sbufWriteU8(dst, NAV_Status.flags);
}
#endif

#define LTM_BIT_AFRAME  (1 << 0)
#define LTM_BIT_GFRAME  (1 << 1)
#define LTM_BIT_SFRAME  (1 << 2)
#define LTM_BIT_OFRAME  (1 << 3)
#define LTM_BIT_NFRAME  (1 << 4)
#define LTM_BIT_XFRAME  (1 << 5)

/*
 * This is the normal (default) scheduler, needs c. 4800 baud or faster
 * Equates to c. 303 bytes / second
 */
static uint8_t ltm_normal_schedule[LTM_SCHEDULE_SIZE] = {
    LTM_BIT_AFRAME | LTM_BIT_GFRAME,
    LTM_BIT_AFRAME | LTM_BIT_SFRAME | LTM_BIT_OFRAME,
    LTM_BIT_AFRAME | LTM_BIT_GFRAME,
    LTM_BIT_AFRAME | LTM_BIT_SFRAME | LTM_BIT_NFRAME,
    LTM_BIT_AFRAME | LTM_BIT_GFRAME,
    LTM_BIT_AFRAME | LTM_BIT_SFRAME | LTM_BIT_XFRAME,
    LTM_BIT_AFRAME | LTM_BIT_GFRAME,
    LTM_BIT_AFRAME | LTM_BIT_SFRAME | LTM_BIT_NFRAME,
    LTM_BIT_AFRAME | LTM_BIT_GFRAME,
    LTM_BIT_AFRAME | LTM_BIT_SFRAME | LTM_BIT_NFRAME
};

/*
 * This is the medium scheduler, needs c. 2400 baud or faster
 * Equates to c. 164 bytes / second
 */
static uint8_t ltm_medium_schedule[LTM_SCHEDULE_SIZE] = {
    LTM_BIT_AFRAME,
    LTM_BIT_GFRAME,
    LTM_BIT_AFRAME | LTM_BIT_SFRAME,
    LTM_BIT_OFRAME,
    LTM_BIT_AFRAME | LTM_BIT_XFRAME,
    LTM_BIT_OFRAME,
    LTM_BIT_AFRAME | LTM_BIT_SFRAME,
    LTM_BIT_GFRAME,
    LTM_BIT_AFRAME,
    LTM_BIT_NFRAME
};

/*
 * This is the slow scheduler, needs c. 1200 baud or faster
 * Equates to c. 105 bytes / second (91 b/s if the second GFRAME is zeroed)
 */
static uint8_t ltm_slow_schedule[LTM_SCHEDULE_SIZE] = {
    LTM_BIT_GFRAME,
    LTM_BIT_SFRAME,
    LTM_BIT_AFRAME,
    0,
    LTM_BIT_OFRAME,
    LTM_BIT_XFRAME,
    LTM_BIT_GFRAME, // consider zeroing this for even lower bytes/sec
    0,
    LTM_BIT_AFRAME,
    LTM_BIT_NFRAME,
};

/* Set by initialisation */
static uint8_t *ltm_schedule;

static void process_ltm(void)
{
    static uint8_t ltm_scheduler = 0;
    uint8_t current_schedule = ltm_schedule[ltm_scheduler];

    sbuf_t ltmFrameBuf;
    sbuf_t *dst = &ltmFrameBuf;

    if (current_schedule & LTM_BIT_AFRAME) {
        ltm_initialise_packet(dst);
        ltm_aframe(dst);
        ltm_finalise(dst);
    }

#if defined(GPS)
    if (current_schedule & LTM_BIT_GFRAME) {
        ltm_initialise_packet(dst);
        ltm_gframe(dst);
        ltm_finalise(dst);
    }

    if (current_schedule & LTM_BIT_OFRAME) {
        ltm_initialise_packet(dst);
        ltm_oframe(dst);
        ltm_finalise(dst);
    }

    if (current_schedule & LTM_BIT_XFRAME) {
        ltm_initialise_packet(dst);
        ltm_xframe(dst);
        ltm_finalise(dst);
    }
#endif

    if (current_schedule & LTM_BIT_SFRAME) {
        ltm_initialise_packet(dst);
        ltm_sframe(dst);
        ltm_finalise(dst);
    }

#if defined(NAV)
    if (current_schedule & LTM_BIT_NFRAME) {
        ltm_initialise_packet(dst);
        ltm_nframe(dst);
        ltm_finalise(dst);
    }
#endif

    ltm_scheduler = (ltm_scheduler + 1) % 10;
}

void handleLtmTelemetry(void)
{
    static uint32_t ltm_lastCycleTime;
    if (!ltmEnabled)
        return;
    if (!ltmPort)
        return;
    const uint32_t now = millis();
    if ((now - ltm_lastCycleTime) >= LTM_CYCLETIME) {
        process_ltm();
        ltm_lastCycleTime = now;
    }
}

void freeLtmTelemetryPort(void)
{
    closeSerialPort(ltmPort);
    ltmPort = NULL;
    ltmEnabled = false;
}

void initLtmTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_LTM);
    ltmPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_LTM);
}

void configureLtmTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }
    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_19200;
    }

    /* setup scheduler, default to 'normal' */
    if (telemetryConfig()->ltmUpdateRate == LTM_RATE_MEDIUM)
        ltm_schedule = ltm_medium_schedule;
    else if (telemetryConfig()->ltmUpdateRate == LTM_RATE_SLOW)
        ltm_schedule = ltm_slow_schedule;
    else
        ltm_schedule = ltm_normal_schedule;

    /* Sanity check that we can support the scheduler */
    if (baudRateIndex == BAUD_2400 && telemetryConfig()->ltmUpdateRate == LTM_RATE_NORMAL)
         ltm_schedule = ltm_medium_schedule;
    if (baudRateIndex == BAUD_1200)
         ltm_schedule = ltm_slow_schedule;

    ltmPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_LTM, NULL, baudRates[baudRateIndex], TELEMETRY_LTM_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);
    if (!ltmPort)
        return;
    ltm_x_counter = 0;
    ltmEnabled = true;
}

void checkLtmTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig)) {
        if (!ltmEnabled && telemetrySharedPort != NULL) {
            ltmPort = telemetrySharedPort;
            ltmEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(ltmPortSharing);
        if (newTelemetryEnabledValue == ltmEnabled)
            return;
        if (newTelemetryEnabledValue)
            configureLtmTelemetryPort();
        else
            freeLtmTelemetryPort();
    }
}

int getLtmFrame(uint8_t *frame, ltm_frame_e ltmFrameType)
{
    static uint8_t ltmFrame[LTM_MAX_MESSAGE_SIZE];

    sbuf_t ltmFrameBuf = { .ptr = ltmFrame, .end =ARRAYEND(ltmFrame) };
    sbuf_t * const sbuf = &ltmFrameBuf;

    switch (ltmFrameType) {
    default:
    case LTM_AFRAME:
        ltm_aframe(sbuf);
        break;
    case LTM_SFRAME:
        ltm_sframe(sbuf);
        break;
#if defined(GPS)
    case LTM_GFRAME:
        ltm_gframe(sbuf);
        break;
    case LTM_OFRAME:
        ltm_oframe(sbuf);
        break;
#endif
    case LTM_XFRAME:
        ltm_xframe(sbuf);
        break;
#if defined(NAV)
    case LTM_NFRAME:
        ltm_nframe(sbuf);
        break;
#endif
    }
    sbufSwitchToReader(sbuf, ltmFrame);
    const int frameSize = sbufBytesRemaining(sbuf);
    for (int ii = 0; sbufBytesRemaining(sbuf); ++ii) {
        frame[ii] = sbufReadU8(sbuf);
    }
    return frameSize;
}
#endif
