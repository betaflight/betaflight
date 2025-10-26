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

/*
 * telemetry_mavlink.c
 *
 * Author: Konstantin Sharlaimov
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY_MAVLINK)

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"

#include "rx/rx.h"
#include "rx/mavlink.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/mavlink.h"

#include "build/debug.h"

// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define TELEMETRY_MAVLINK_INITIAL_PORT_MODE MODE_TX
#define TELEMETRY_MAVLINK_MAXRATE 50
#define TELEMETRY_MAVLINK_DELAY ((1000 * 1000) / TELEMETRY_MAVLINK_MAXRATE)

#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1
extern uint16_t rssi; // FIXME dependency on mw.c

static serialPort_t *mavlinkPort = NULL;
static const serialPortConfig_t *portConfig;

static bool mavlinkTelemetryEnabled =  false;
static portSharing_e mavlinkPortSharing;
static uint32_t lastMavlinkMessageTime = 0;

static mavlink_message_t mavMsg;
static uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];

static void mavlinkSerialWrite(uint8_t * buf, uint16_t length)
{
    for (int i = 0; i < length; i++)
        serialWrite(mavlinkPort, buf[i]);
}

static int16_t headingOrScaledMilliAmpereHoursDrawn(void)
{
    if (isAmperageConfigured() && telemetryConfig()->mavlink_mah_as_heading_divisor > 0) {
        // In the Connex Prosight OSD, this goes between 0 and 999, so it will need to be scaled in that range.
        return getMAhDrawn() / telemetryConfig()->mavlink_mah_as_heading_divisor;
    }
    // heading Current heading in degrees, in compass units (0..360, 0=north)
    return DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}

void freeMAVLinkTelemetryPort(void)
{
    closeSerialPort(mavlinkPort);
    mavlinkPort = NULL;
    mavlinkTelemetryEnabled = false;
}

void initMAVLinkTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    mavlinkPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MAVLINK);
}

void configureMAVLinkTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        // default rate for minimOSD
        baudRateIndex = BAUD_57600;
    }

    mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_MAVLINK_INITIAL_PORT_MODE, telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);

    if (!mavlinkPort) {
        return;
    }

    mavlinkTelemetryEnabled = true;
}

static void mavlinkSendSystemStatus(void)
{
    uint16_t msgLength;

    uint32_t onboardControlAndSensors = 35843;

    /*
    onboard_control_sensors_present Bitmask
    fedcba9876543210
    1000110000000011    For all   = 35843
    0001000000000100    With Mag  = 4100
    0010000000001000    With Baro = 8200
    0100000000100000    With GPS  = 16416
    0000001111111111
    */

    if (sensors(SENSOR_MAG))  onboardControlAndSensors |=  4100;
    if (sensors(SENSOR_BARO)) onboardControlAndSensors |=  8200;
    if (sensors(SENSOR_GPS))  onboardControlAndSensors |= 16416;

    uint16_t batteryVoltage = 0;
    int16_t batteryAmperage = -1;
    int8_t batteryRemaining = 100;

    if (getBatteryState() < BATTERY_NOT_PRESENT) {
        batteryVoltage = isBatteryVoltageConfigured() ? getBatteryVoltage() * 10 : batteryVoltage;
        batteryAmperage = isAmperageConfigured() ? getAmperage() : batteryAmperage;
        batteryRemaining = isBatteryVoltageConfigured() ? calculateBatteryPercentageRemaining() : batteryRemaining;
    }

    mavlink_msg_sys_status_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present.
        //Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure,
        // 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position,
        // 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization,
        // 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
        onboardControlAndSensors,
        // onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled
        onboardControlAndSensors,
        // onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error.
        onboardControlAndSensors & 1023,
        // load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        0,
        // voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
        batteryVoltage,
        // current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        batteryAmperage,
        // battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        batteryRemaining,
        // drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_count1 Autopilot-specific errors
        0,
        // errors_count2 Autopilot-specific errors
        0,
        // errors_count3 Autopilot-specific errors
        0,
        // errors_count4 Autopilot-specific errors
        0,
        // extended parameters, set to zero
        0,
        0,
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Packets transmit counter to debug actual data rate
    static uint32_t transmitCounter = 0;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 2, transmitCounter);
    transmitCounter = (transmitCounter + 1) % 100;
}

static void mavlinkSendRCChannelsAndRSSI(void)
{
    uint16_t msgLength;
    mavlink_msg_rc_channels_raw_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
        0,
        // chan1_raw RC channel 1 value, in microseconds
        (rxRuntimeState.channelCount >= 1) ? rcData[0] : 0,
        // chan2_raw RC channel 2 value, in microseconds
        (rxRuntimeState.channelCount >= 2) ? rcData[1] : 0,
        // chan3_raw RC channel 3 value, in microseconds
        (rxRuntimeState.channelCount >= 3) ? rcData[2] : 0,
        // chan4_raw RC channel 4 value, in microseconds
        (rxRuntimeState.channelCount >= 4) ? rcData[3] : 0,
        // chan5_raw RC channel 5 value, in microseconds
        (rxRuntimeState.channelCount >= 5) ? rcData[4] : 0,
        // chan6_raw RC channel 6 value, in microseconds
        (rxRuntimeState.channelCount >= 6) ? rcData[5] : 0,
        // chan7_raw RC channel 7 value, in microseconds
        (rxRuntimeState.channelCount >= 7) ? rcData[6] : 0,
        // chan8_raw RC channel 8 value, in microseconds
        (rxRuntimeState.channelCount >= 8) ? rcData[7] : 0,
        // rssi Receive signal strength indicator, 0: 0%, 254: 100%
        scaleRange(getRssi(), 0, RSSI_MAX_VALUE, 0, 254));
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Packets transmit counter to debug actual data rate
    static uint32_t transmitCounter = 0;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 3, transmitCounter);
    transmitCounter = (transmitCounter + 1) % 100;
}

#if defined(USE_GPS)
static void mavlinkSendPosition(void)
{
    uint16_t msgLength;
    uint8_t gpsFixType = 0;

    if (!sensors(SENSOR_GPS))
        return;

    if (!STATE(GPS_FIX)) {
        gpsFixType = 1;
    }
    else {
        if (gpsSol.numSat < GPS_MIN_SAT_COUNT) {
            gpsFixType = 2;
        }
        else {
            gpsFixType = 3;
        }
    }

    mavlink_msg_gps_raw_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        micros(),
        // fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        gpsFixType,
        // lat Latitude in 1E7 degrees
        gpsSol.llh.lat,
        // lon Longitude in 1E7 degrees
        gpsSol.llh.lon,
        // alt Altitude in 1E3 meters (millimeters) above MSL
        gpsSol.llh.altCm * 10,
        // eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        65535,
        // epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        65535,
        // vel GPS ground speed (m/s * 100). If unknown, set to: 65535
        gpsSol.groundSpeed,
        // cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
        gpsSol.groundCourse * 10,
        // satellites_visible Number of satellites visible. If unknown, set to 255
        gpsSol.numSat,
        // Extended parameters, set to zero
        0,
        0,
        0,
        0,
        0,
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Global position
    mavlink_msg_global_position_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        micros(),
        // lat Latitude in 1E7 degrees
        gpsSol.llh.lat,
        // lon Longitude in 1E7 degrees
        gpsSol.llh.lon,
        // alt Altitude in 1E3 meters (millimeters) above MSL
        gpsSol.llh.altCm * 10,
        // relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
        getEstimatedAltitudeCm() * 10,
        // Ground X Speed (Latitude), expressed as m/s * 100
        0,
        // Ground Y Speed (Longitude), expressed as m/s * 100
        0,
        // Ground Z Speed (Altitude), expressed as m/s * 100
        0,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        headingOrScaledMilliAmpereHoursDrawn()
    );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    mavlink_msg_gps_global_origin_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // Latitude (WGS84), expressed as * 1E7
        GPS_home_llh.lat,
        // Longitude (WGS84), expressed as * 1E7
        GPS_home_llh.lon,
        // Altitude(WGS84), expressed as * 1000
        0,
        // Timestamp, unused
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Packets transmit counter to debug actual data rate
    static uint32_t transmitCounter = 0;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 4, transmitCounter);
    transmitCounter = (transmitCounter + 1) % 100;
}
#endif

static void mavlinkSendAttitude(void)
{
    uint16_t msgLength;
    mavlink_msg_attitude_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // roll Roll angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.roll),
        // pitch Pitch angle (rad)
        DECIDEGREES_TO_RADIANS(-attitude.values.pitch),
        // yaw Yaw angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.yaw),
        // rollspeed Roll angular speed (rad/s)
        0,
        // pitchspeed Pitch angular speed (rad/s)
        0,
        // yawspeed Yaw angular speed (rad/s)
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Packets transmit counter to debug actual data rate
    static uint32_t transmitCounter = 0;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 5, transmitCounter);
    transmitCounter = (transmitCounter + 1) % 100;
}

static void mavlinkSendHUDAndHeartbeat(void)
{
    uint16_t msgLength;
    float mavAltitude = 0;
    float mavGroundSpeed = 0;
    float mavAirSpeed = 0;
    float mavClimbRate = 0;

#if defined(USE_GPS)
    // use ground speed if source available
    if (sensors(SENSOR_GPS)) {
        mavGroundSpeed = gpsSol.groundSpeed / 100.0f;
    }
#endif

    mavAltitude = getEstimatedAltitudeCm() / 100.0f;

    mavlink_msg_vfr_hud_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // airspeed Current airspeed in m/s
        mavAirSpeed,
        // groundspeed Current ground speed in m/s
        mavGroundSpeed,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        headingOrScaledMilliAmpereHoursDrawn(),
        // throttle Current throttle setting in integer percent, 0 to 100
        scaleRange(constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 100),
        // alt Current altitude (MSL), in meters, if we have sonar or baro use them, otherwise use GPS (less accurate)
        mavAltitude,
        // climb Current climb rate in meters/second
        mavClimbRate);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    uint8_t mavModes = MAV_MODE_MANUAL_DISARMED;
    if (ARMING_FLAG(ARMED))
        mavModes |= MAV_MODE_MANUAL_ARMED;

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode)
    {
        case MIXER_TRI:
            mavSystemType = MAV_TYPE_TRICOPTER;
            break;
        case MIXER_QUADP:
        case MIXER_QUADX:
        case MIXER_Y4:
        case MIXER_VTAIL4:
            mavSystemType = MAV_TYPE_QUADROTOR;
            break;
        case MIXER_Y6:
        case MIXER_HEX6:
        case MIXER_HEX6X:
            mavSystemType = MAV_TYPE_HEXAROTOR;
            break;
        case MIXER_OCTOX8:
        case MIXER_OCTOX8P:
        case MIXER_OCTOFLATP:
        case MIXER_OCTOFLATX:
            mavSystemType = MAV_TYPE_OCTOROTOR;
            break;
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_CUSTOM_AIRPLANE:
            mavSystemType = MAV_TYPE_FIXED_WING;
            break;
        case MIXER_HELI_120_CCPM:
        case MIXER_HELI_90_DEG:
            mavSystemType = MAV_TYPE_HELICOPTER;
            break;
        default:
            mavSystemType = MAV_TYPE_GENERIC;
            break;
    }

    // Custom mode for compatibility with APM OSDs
    uint8_t mavCustomMode = 1;  // Acro by default

    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | ALT_HOLD_MODE | POS_HOLD_MODE)) {
        mavCustomMode = 0;      //Stabilize
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    uint8_t mavSystemState = 0;
    if (ARMING_FLAG(ARMED)) {
        if (failsafeIsActive()) {
            mavSystemState = MAV_STATE_CRITICAL;
        }
        else {
            mavSystemState = MAV_STATE_ACTIVE;
        }
    }
    else {
        mavSystemState = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        mavSystemType,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        MAV_AUTOPILOT_GENERIC,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Packets transmit counter to debug actual data rate
    static uint32_t transmitCounter = 0;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 6, transmitCounter);
    transmitCounter = (transmitCounter + 1) % 100;
}

static void mavlinkSendBatteryStatus(void)
{
    uint16_t msgLength;

    uint16_t voltages[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN];
    uint16_t voltagesExt[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN];
    memset(voltages, 0xff, sizeof(voltages));
    memset(voltagesExt, 0, sizeof(voltagesExt));
    if (isBatteryVoltageConfigured()) {
        uint8_t batteryCellCount = getBatteryCellCount();
        if (batteryCellCount > 0 && telemetryConfig()->report_cell_voltage) {
            for (int cell=0; cell < batteryCellCount && cell < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN + MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN; cell++) {
                if (cell < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN) {
                    voltages[cell] = getBatteryAverageCellVoltage() * 10;
                } else {
                    voltagesExt[cell - MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN] = getBatteryAverageCellVoltage() * 10;
                }
            }
        } else {
            voltages[0] = getBatteryVoltage() * 10;
        }
    }

    // Battery amperage in centiamps (cA), -1 if not available
    int16_t batteryAmperage = -1;
    if (isAmperageConfigured()) {
        batteryAmperage = getAmperage(); // Already in cA (0.01A)
    }

    // mAh consumed, -1 if not available
    int32_t amperageConsumed = -1;
    if (isAmperageConfigured()) {
        amperageConsumed = getMAhDrawn(); // This is the key field for "Capa"
    }

    // Battery percentage remaining
    int8_t batteryRemaining = -1;
    if (isBatteryVoltageConfigured()) {
        batteryRemaining = calculateBatteryPercentageRemaining();
    }

    // Temperature: INT16_MAX if unknown
    int16_t temperature = INT16_MAX;

    mavlink_msg_battery_status_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &mavMsg,
        0,                    // id: Battery ID (0 = main battery)
        0,                    // battery_function: 0 = MAV_BATTERY_FUNCTION_UNKNOWN
        0,                    // type: 0 = MAV_BATTERY_TYPE_UNKNOWN (could use MAV_BATTERY_TYPE_LIPO = 1)
        temperature,          // temperature: INT16_MAX = unknown
        voltages,             // voltages[10]: Cell voltages in mV
        batteryAmperage,      // current_battery: Current in cA
        amperageConsumed,     // current_consumed: mAh drawn (CRITICAL for "Capa")
        -1,                   // energy_consumed: -1 = not available (could calculate from Wh if needed)
        batteryRemaining,     // battery_remaining: Percentage 0-100
        0,                    // time_remaining: 0 = not calculated
        MAV_BATTERY_CHARGE_STATE_UNDEFINED, // charge_state: 0 = MAV_BATTERY_CHARGE_STATE_UNDEFINED
        voltagesExt,          // voltages_ext[4]: Cells 11-14, not used
        0,                    // mode: 0 = normal
        0                     // fault_bitmask: 0 = no faults
    );

    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Packets transmit counter to debug actual data rate
    static uint32_t transmitCounter = 0;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 7, transmitCounter);
    transmitCounter = (transmitCounter + 1) % 100;
}

/* MAVLink telemetry data streams */
// The initialization rates value is zero, because they are installed from CLI settings
static mavlinkTelemetryStream_t mavTelemetryStreams[] = {
    [MAV_DATA_STREAM_EXTENDED_STATUS] = {
        .rate = 0,
        .updateTime = 0,
        .streamFunc = mavlinkSendSystemStatus,
    },
    [MAV_DATA_STREAM_RC_CHANNELS] = {
        .rate = 0,
        .updateTime = 0,
        .streamFunc = mavlinkSendRCChannelsAndRSSI,
    },
    [MAV_DATA_STREAM_POSITION] = {
        .rate = 0,
        .updateTime = 0,
#ifdef USE_GPS
        .streamFunc = mavlinkSendPosition,
#else
        .streamFunc = NULL,
#endif
    },
    [MAV_DATA_STREAM_EXTRA1] = {
        .rate = 0,
        .updateTime = 0,
        .streamFunc = mavlinkSendAttitude,
    },
    [MAV_DATA_STREAM_EXTRA2] = {
        .rate = 0,
        .updateTime = 0,
        .streamFunc = mavlinkSendHUDAndHeartbeat,
    },
    [MAV_DATA_STREAM_EXTRA3] = {
        .rate = 0,
        .updateTime = 0,
        .streamFunc = mavlinkSendBatteryStatus,
    }
};
#define TELEMETRIES_STREAM_COUNT ARRAYLEN(mavTelemetryStreams)

static void configureMAVLinkStreamRates(void)
{
    mavTelemetryStreams[MAV_DATA_STREAM_EXTENDED_STATUS].rate = telemetryConfig()->mavlink_extended_status_rate;
    mavTelemetryStreams[MAV_DATA_STREAM_RC_CHANNELS].rate = telemetryConfig()->mavlink_rc_channels_rate;
#ifdef USE_GPS
    mavTelemetryStreams[MAV_DATA_STREAM_POSITION].rate = telemetryConfig()->mavlink_position_rate;
#endif
    mavTelemetryStreams[MAV_DATA_STREAM_EXTRA1].rate = telemetryConfig()->mavlink_extra1_rate;
    mavTelemetryStreams[MAV_DATA_STREAM_EXTRA2].rate = telemetryConfig()->mavlink_extra2_rate;
    mavTelemetryStreams[MAV_DATA_STREAM_EXTRA3].rate = telemetryConfig()->mavlink_extra3_rate;

    // Seed timers to avoid burst on enable
    timeMs_t nowMs = millis();
    for (uint16_t i = 0; i < TELEMETRIES_STREAM_COUNT; i++) {
        const uint8_t rate = mavTelemetryStreams[i].rate;
        mavTelemetryStreams[i].updateTime = (rate > 0) ? nowMs + (timeMs_t)(1000 / rate) + 3 * i : 0;
    }
}

static void processMAVLinkTelemetry(void)
{
    timeMs_t currentTimeMs = millis();
    for (uint16_t i = 0; i < TELEMETRIES_STREAM_COUNT; i++) {
        if (mavTelemetryStreams[i].rate != 0 && mavTelemetryStreams[i].streamFunc != NULL) {
            if (currentTimeMs >= mavTelemetryStreams[i].updateTime) {
                mavTelemetryStreams[i].streamFunc();
                mavTelemetryStreams[i].updateTime = currentTimeMs + (timeMs_t)(1000 / mavTelemetryStreams[i].rate);
            }
        }
    }
}

void checkMAVLinkTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!mavlinkTelemetryEnabled && telemetrySharedPort != NULL) {
            mavlinkPort = telemetrySharedPort;
            mavlinkTelemetryEnabled = true;
            configureMAVLinkStreamRates();
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(mavlinkPortSharing);

        if (newTelemetryEnabledValue == mavlinkTelemetryEnabled) {
            return;
        }

        if (newTelemetryEnabledValue) {
            configureMAVLinkTelemetryPort();
            configureMAVLinkStreamRates();
        } else {
            freeMAVLinkTelemetryPort();
        }
    }
}

void handleMAVLinkTelemetry(void)
{
    if (!mavlinkTelemetryEnabled || !mavlinkPort) {
        return;
    }

    bool shouldSendTelemetry = false;
    uint32_t now = micros();
    if (isValidMavlinkTxBuffer()) {
        shouldSendTelemetry = shouldSendMavlinkTelemetry();
    } else if ((now - lastMavlinkMessageTime) >= TELEMETRY_MAVLINK_DELAY) {
        shouldSendTelemetry = true;
    }

    if (shouldSendTelemetry) {
        processMAVLinkTelemetry();
        lastMavlinkMessageTime = now;
    }
}

#endif
