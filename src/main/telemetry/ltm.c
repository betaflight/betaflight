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
 * Minimal one way telemetry protocol for really bitrates (1200/2400 bauds).
 * Effective for ground OSD, groundstation HUD and Antenna tracker
 * http://www.wedontneednasa.com/2014/02/lighttelemetry-v2-en-route-to-ground-osd.html
 *
 * This implementation is for LTM v2 > 2400 baud rates
 *
 * Cleanflight implementation by Jonathan Hudson
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"

#ifdef TELEMETRY

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/pwm_rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/beeper.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

#include "telemetry/telemetry.h"
#include "telemetry/ltm.h"

#include "config/config.h"
#include "config/runtime_config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#define TELEMETRY_LTM_INITIAL_PORT_MODE MODE_TX
#define LTM_CYCLETIME   100

extern uint16_t rssi;           // FIXME dependency on mw.c
static serialPort_t *ltmPort;
static serialPortConfig_t *portConfig;
static telemetryConfig_t *telemetryConfig;
static bool ltmEnabled;
static portSharing_e ltmPortSharing;
static uint8_t ltm_crc;

static void ltm_initialise_packet(uint8_t ltm_id)
{
    ltm_crc = 0;
    serialWrite(ltmPort, '$');
    serialWrite(ltmPort, 'T');
    serialWrite(ltmPort, ltm_id);
}

static void ltm_serialise_8(uint8_t v)
{
    serialWrite(ltmPort, v);
    ltm_crc ^= v;
}

static void ltm_serialise_16(uint16_t v)
{
    ltm_serialise_8((uint8_t)v);
    ltm_serialise_8((v >> 8));
}

static void ltm_serialise_32(uint32_t v)
{
    ltm_serialise_8((uint8_t)v);
    ltm_serialise_8((v >> 8));
    ltm_serialise_8((v >> 16));
    ltm_serialise_8((v >> 24));
}

static void ltm_finalise(void)
{
    serialWrite(ltmPort, ltm_crc);
}

/*
 * GPS G-frame 5Hhz at > 2400 baud
 * LAT LON SPD ALT SAT/FIX
 */
static void ltm_gframe(void)
{
#if defined(GPS)
    uint8_t gps_fix_type = 0;
    int32_t ltm_alt;

    if (!sensors(SENSOR_GPS))
        return;

    if (!STATE(GPS_FIX))
        gps_fix_type = 1;
    else if (GPS_numSat < 5)
        gps_fix_type = 2;
    else
        gps_fix_type = 3;

    ltm_initialise_packet('G');
    ltm_serialise_32(GPS_coord[LAT]);
    ltm_serialise_32(GPS_coord[LON]);
    ltm_serialise_8((uint8_t)(GPS_speed / 100));

#if defined(BARO) || defined(SONAR)
    ltm_alt = (sensors(SENSOR_SONAR) || sensors(SENSOR_BARO)) ? altitudeHoldGetEstimatedAltitude() : GPS_altitude * 100;
#else
    ltm_alt = GPS_altitude * 100;
#endif
    ltm_serialise_32(ltm_alt);
    ltm_serialise_8((GPS_numSat << 2) | gps_fix_type);
    ltm_finalise();
#endif
}

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

static void ltm_sframe(void)
{
    uint8_t lt_flightmode;
    uint8_t lt_statemode;
    if (FLIGHT_MODE(PASSTHRU_MODE))
        lt_flightmode = 0;
    else if (FLIGHT_MODE(GPS_HOME_MODE))
        lt_flightmode = 13;
    else if (FLIGHT_MODE(GPS_HOLD_MODE))
        lt_flightmode = 9;
    else if (FLIGHT_MODE(HEADFREE_MODE))
        lt_flightmode = 4;
    else if (FLIGHT_MODE(BARO_MODE))
        lt_flightmode = 8;
    else if (FLIGHT_MODE(ANGLE_MODE))
        lt_flightmode = 2;
    else if (FLIGHT_MODE(HORIZON_MODE))
        lt_flightmode = 3;
    else
        lt_flightmode = 1;      // Rate mode

    lt_statemode = (ARMING_FLAG(ARMED)) ? 1 : 0;
    if (failsafeIsActive())
        lt_statemode |= 2;
    ltm_initialise_packet('S');
    ltm_serialise_16(vbat * 100);    //vbat converted to mv
    ltm_serialise_16(0);             //  current, not implemented
    ltm_serialise_8((uint8_t)((rssi * 254) / 1023));        // scaled RSSI (uchar)
    ltm_serialise_8(0);              // no airspeed
    ltm_serialise_8((lt_flightmode << 2) | lt_statemode);
    ltm_finalise();
}

/*
 * Attitude A-frame - 10 Hz at > 2400 baud
 *  PITCH ROLL HEADING
 */
static void ltm_aframe()
{
    ltm_initialise_packet('A');
    ltm_serialise_16(DECIDEGREES_TO_DEGREES(attitude.values.pitch));
    ltm_serialise_16(DECIDEGREES_TO_DEGREES(attitude.values.roll));
    ltm_serialise_16(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    ltm_finalise();
}

/*
 * OSD additional data frame, 1 Hz rate
 *  This frame will be ignored by Ghettostation, but processed by GhettOSD if it is used as standalone onboard OSD
 *  home pos, home alt, direction to home
 */
static void ltm_oframe()
{
    ltm_initialise_packet('O');
#if defined(GPS)
    ltm_serialise_32(GPS_home[LAT]);
    ltm_serialise_32(GPS_home[LON]);
#else
    ltm_serialise_32(0);
    ltm_serialise_32(0);
#endif
    ltm_serialise_32(0);                // Don't have GPS home altitude
    ltm_serialise_8(1);                 // OSD always ON
    ltm_serialise_8(STATE(GPS_FIX_HOME) ? 1 : 0);
    ltm_finalise();
}

static void process_ltm(void)
{
    static uint8_t ltm_scheduler;
    ltm_aframe();
    if (ltm_scheduler & 1)
        ltm_gframe();
    else
        ltm_sframe();
    if (ltm_scheduler == 0)
        ltm_oframe();
    ltm_scheduler++;
    ltm_scheduler %= 10;
}

void handleLtmTelemetry(void)
{
    static uint32_t ltm_lastCycleTime;
    uint32_t now;
    if (!ltmEnabled)
        return;
    if (!ltmPort)
        return;
    now = millis();
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

void initLtmTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
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
    ltmPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_LTM, NULL, baudRates[baudRateIndex], TELEMETRY_LTM_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);
    if (!ltmPort)
        return;
    ltmEnabled = true;
}

void checkLtmTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(ltmPortSharing);
    if (newTelemetryEnabledValue == ltmEnabled)
        return;
    if (newTelemetryEnabledValue)
        configureLtmTelemetryPort();
    else
        freeLtmTelemetryPort();
}
#endif
