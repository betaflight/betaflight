#pragma once
// MESSAGE GLOBAL_POSITION_SENSOR PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR 296


typedef struct __mavlink_global_position_sensor_t {
 uint64_t time_usec; /*< [us] Timestamp of message transmission (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 uint32_t processing_time; /*< [us] The time spent in processing the sensor data that is the basis for this position. The recipient can use this to improve time alignment of the data. This is the time between measurement (e.g. camera exposure time) and transmission of this message. Set to NaN if not known.*/
 int32_t lat; /*< [degE7] Latitude (WGS84)*/
 int32_t lon; /*< [degE7] Longitude (WGS84)*/
 float alt_ellipsoid; /*< [m] Altitude (WGS84 elipsoid), preferred if available*/
 float alt; /*< [m] Altitude (MSL - position-system specific value) use if no alt_ellipsoid available*/
 float eph; /*< [m] Standard deviation of horizontal position error*/
 float epv; /*< [m] Standard deviation of vertical position error*/
 uint8_t target_system; /*<  System ID (ID of target system, normally autopilot and ground station).*/
 uint8_t target_component; /*<  Component ID (normally 0 for broadcast).*/
 uint8_t id; /*<  Sensor ID*/
 uint8_t source; /*<  Source of position/estimate (such as GNSS, estimator, etc.)*/
 uint8_t flags; /*<  Status flags*/
} mavlink_global_position_sensor_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN 41
#define MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN 41
#define MAVLINK_MSG_ID_296_LEN 41
#define MAVLINK_MSG_ID_296_MIN_LEN 41

#define MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_CRC 158
#define MAVLINK_MSG_ID_296_CRC 158



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_SENSOR { \
    296, \
    "GLOBAL_POSITION_SENSOR", \
    13, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_global_position_sensor_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_global_position_sensor_t, target_component) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_global_position_sensor_t, id) }, \
         { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_global_position_sensor_t, time_usec) }, \
         { "processing_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_global_position_sensor_t, processing_time) }, \
         { "source", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_global_position_sensor_t, source) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_global_position_sensor_t, flags) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_sensor_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_position_sensor_t, lon) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_global_position_sensor_t, alt_ellipsoid) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_global_position_sensor_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_global_position_sensor_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_global_position_sensor_t, epv) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_SENSOR { \
    "GLOBAL_POSITION_SENSOR", \
    13, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_global_position_sensor_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_global_position_sensor_t, target_component) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_global_position_sensor_t, id) }, \
         { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_global_position_sensor_t, time_usec) }, \
         { "processing_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_global_position_sensor_t, processing_time) }, \
         { "source", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_global_position_sensor_t, source) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_global_position_sensor_t, flags) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_sensor_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_position_sensor_t, lon) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_global_position_sensor_t, alt_ellipsoid) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_global_position_sensor_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_global_position_sensor_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_global_position_sensor_t, epv) }, \
         } \
}
#endif

/**
 * @brief Pack a global_position_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (ID of target system, normally autopilot and ground station).
 * @param target_component  Component ID (normally 0 for broadcast).
 * @param id  Sensor ID
 * @param time_usec [us] Timestamp of message transmission (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param processing_time [us] The time spent in processing the sensor data that is the basis for this position. The recipient can use this to improve time alignment of the data. This is the time between measurement (e.g. camera exposure time) and transmission of this message. Set to NaN if not known.
 * @param source  Source of position/estimate (such as GNSS, estimator, etc.)
 * @param flags  Status flags
 * @param lat [degE7] Latitude (WGS84)
 * @param lon [degE7] Longitude (WGS84)
 * @param alt_ellipsoid [m] Altitude (WGS84 elipsoid), preferred if available
 * @param alt [m] Altitude (MSL - position-system specific value) use if no alt_ellipsoid available
 * @param eph [m] Standard deviation of horizontal position error
 * @param epv [m] Standard deviation of vertical position error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t id, uint64_t time_usec, uint32_t processing_time, uint8_t source, uint8_t flags, int32_t lat, int32_t lon, float alt_ellipsoid, float alt, float eph, float epv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, processing_time);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_ellipsoid);
    _mav_put_float(buf, 24, alt);
    _mav_put_float(buf, 28, eph);
    _mav_put_float(buf, 32, epv);
    _mav_put_uint8_t(buf, 36, target_system);
    _mav_put_uint8_t(buf, 37, target_component);
    _mav_put_uint8_t(buf, 38, id);
    _mav_put_uint8_t(buf, 39, source);
    _mav_put_uint8_t(buf, 40, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN);
#else
    mavlink_global_position_sensor_t packet;
    packet.time_usec = time_usec;
    packet.processing_time = processing_time;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.id = id;
    packet.source = source;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_CRC);
}

/**
 * @brief Pack a global_position_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (ID of target system, normally autopilot and ground station).
 * @param target_component  Component ID (normally 0 for broadcast).
 * @param id  Sensor ID
 * @param time_usec [us] Timestamp of message transmission (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param processing_time [us] The time spent in processing the sensor data that is the basis for this position. The recipient can use this to improve time alignment of the data. This is the time between measurement (e.g. camera exposure time) and transmission of this message. Set to NaN if not known.
 * @param source  Source of position/estimate (such as GNSS, estimator, etc.)
 * @param flags  Status flags
 * @param lat [degE7] Latitude (WGS84)
 * @param lon [degE7] Longitude (WGS84)
 * @param alt_ellipsoid [m] Altitude (WGS84 elipsoid), preferred if available
 * @param alt [m] Altitude (MSL - position-system specific value) use if no alt_ellipsoid available
 * @param eph [m] Standard deviation of horizontal position error
 * @param epv [m] Standard deviation of vertical position error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_sensor_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t id, uint64_t time_usec, uint32_t processing_time, uint8_t source, uint8_t flags, int32_t lat, int32_t lon, float alt_ellipsoid, float alt, float eph, float epv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, processing_time);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_ellipsoid);
    _mav_put_float(buf, 24, alt);
    _mav_put_float(buf, 28, eph);
    _mav_put_float(buf, 32, epv);
    _mav_put_uint8_t(buf, 36, target_system);
    _mav_put_uint8_t(buf, 37, target_component);
    _mav_put_uint8_t(buf, 38, id);
    _mav_put_uint8_t(buf, 39, source);
    _mav_put_uint8_t(buf, 40, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN);
#else
    mavlink_global_position_sensor_t packet;
    packet.time_usec = time_usec;
    packet.processing_time = processing_time;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.id = id;
    packet.source = source;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN);
#endif
}

/**
 * @brief Pack a global_position_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID (ID of target system, normally autopilot and ground station).
 * @param target_component  Component ID (normally 0 for broadcast).
 * @param id  Sensor ID
 * @param time_usec [us] Timestamp of message transmission (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param processing_time [us] The time spent in processing the sensor data that is the basis for this position. The recipient can use this to improve time alignment of the data. This is the time between measurement (e.g. camera exposure time) and transmission of this message. Set to NaN if not known.
 * @param source  Source of position/estimate (such as GNSS, estimator, etc.)
 * @param flags  Status flags
 * @param lat [degE7] Latitude (WGS84)
 * @param lon [degE7] Longitude (WGS84)
 * @param alt_ellipsoid [m] Altitude (WGS84 elipsoid), preferred if available
 * @param alt [m] Altitude (MSL - position-system specific value) use if no alt_ellipsoid available
 * @param eph [m] Standard deviation of horizontal position error
 * @param epv [m] Standard deviation of vertical position error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t id,uint64_t time_usec,uint32_t processing_time,uint8_t source,uint8_t flags,int32_t lat,int32_t lon,float alt_ellipsoid,float alt,float eph,float epv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, processing_time);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_ellipsoid);
    _mav_put_float(buf, 24, alt);
    _mav_put_float(buf, 28, eph);
    _mav_put_float(buf, 32, epv);
    _mav_put_uint8_t(buf, 36, target_system);
    _mav_put_uint8_t(buf, 37, target_component);
    _mav_put_uint8_t(buf, 38, id);
    _mav_put_uint8_t(buf, 39, source);
    _mav_put_uint8_t(buf, 40, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN);
#else
    mavlink_global_position_sensor_t packet;
    packet.time_usec = time_usec;
    packet.processing_time = processing_time;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.id = id;
    packet.source = source;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_CRC);
}

/**
 * @brief Encode a global_position_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_sensor_t* global_position_sensor)
{
    return mavlink_msg_global_position_sensor_pack(system_id, component_id, msg, global_position_sensor->target_system, global_position_sensor->target_component, global_position_sensor->id, global_position_sensor->time_usec, global_position_sensor->processing_time, global_position_sensor->source, global_position_sensor->flags, global_position_sensor->lat, global_position_sensor->lon, global_position_sensor->alt_ellipsoid, global_position_sensor->alt, global_position_sensor->eph, global_position_sensor->epv);
}

/**
 * @brief Encode a global_position_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_position_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_position_sensor_t* global_position_sensor)
{
    return mavlink_msg_global_position_sensor_pack_chan(system_id, component_id, chan, msg, global_position_sensor->target_system, global_position_sensor->target_component, global_position_sensor->id, global_position_sensor->time_usec, global_position_sensor->processing_time, global_position_sensor->source, global_position_sensor->flags, global_position_sensor->lat, global_position_sensor->lon, global_position_sensor->alt_ellipsoid, global_position_sensor->alt, global_position_sensor->eph, global_position_sensor->epv);
}

/**
 * @brief Encode a global_position_sensor struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param global_position_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_sensor_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_global_position_sensor_t* global_position_sensor)
{
    return mavlink_msg_global_position_sensor_pack_status(system_id, component_id, _status, msg,  global_position_sensor->target_system, global_position_sensor->target_component, global_position_sensor->id, global_position_sensor->time_usec, global_position_sensor->processing_time, global_position_sensor->source, global_position_sensor->flags, global_position_sensor->lat, global_position_sensor->lon, global_position_sensor->alt_ellipsoid, global_position_sensor->alt, global_position_sensor->eph, global_position_sensor->epv);
}

/**
 * @brief Send a global_position_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID (ID of target system, normally autopilot and ground station).
 * @param target_component  Component ID (normally 0 for broadcast).
 * @param id  Sensor ID
 * @param time_usec [us] Timestamp of message transmission (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param processing_time [us] The time spent in processing the sensor data that is the basis for this position. The recipient can use this to improve time alignment of the data. This is the time between measurement (e.g. camera exposure time) and transmission of this message. Set to NaN if not known.
 * @param source  Source of position/estimate (such as GNSS, estimator, etc.)
 * @param flags  Status flags
 * @param lat [degE7] Latitude (WGS84)
 * @param lon [degE7] Longitude (WGS84)
 * @param alt_ellipsoid [m] Altitude (WGS84 elipsoid), preferred if available
 * @param alt [m] Altitude (MSL - position-system specific value) use if no alt_ellipsoid available
 * @param eph [m] Standard deviation of horizontal position error
 * @param epv [m] Standard deviation of vertical position error
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_sensor_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t id, uint64_t time_usec, uint32_t processing_time, uint8_t source, uint8_t flags, int32_t lat, int32_t lon, float alt_ellipsoid, float alt, float eph, float epv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, processing_time);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_ellipsoid);
    _mav_put_float(buf, 24, alt);
    _mav_put_float(buf, 28, eph);
    _mav_put_float(buf, 32, epv);
    _mav_put_uint8_t(buf, 36, target_system);
    _mav_put_uint8_t(buf, 37, target_component);
    _mav_put_uint8_t(buf, 38, id);
    _mav_put_uint8_t(buf, 39, source);
    _mav_put_uint8_t(buf, 40, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_CRC);
#else
    mavlink_global_position_sensor_t packet;
    packet.time_usec = time_usec;
    packet.processing_time = processing_time;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.id = id;
    packet.source = source;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_CRC);
#endif
}

/**
 * @brief Send a global_position_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_global_position_sensor_send_struct(mavlink_channel_t chan, const mavlink_global_position_sensor_t* global_position_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_global_position_sensor_send(chan, global_position_sensor->target_system, global_position_sensor->target_component, global_position_sensor->id, global_position_sensor->time_usec, global_position_sensor->processing_time, global_position_sensor->source, global_position_sensor->flags, global_position_sensor->lat, global_position_sensor->lon, global_position_sensor->alt_ellipsoid, global_position_sensor->alt, global_position_sensor->eph, global_position_sensor->epv);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR, (const char *)global_position_sensor, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_global_position_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t id, uint64_t time_usec, uint32_t processing_time, uint8_t source, uint8_t flags, int32_t lat, int32_t lon, float alt_ellipsoid, float alt, float eph, float epv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, processing_time);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_ellipsoid);
    _mav_put_float(buf, 24, alt);
    _mav_put_float(buf, 28, eph);
    _mav_put_float(buf, 32, epv);
    _mav_put_uint8_t(buf, 36, target_system);
    _mav_put_uint8_t(buf, 37, target_component);
    _mav_put_uint8_t(buf, 38, id);
    _mav_put_uint8_t(buf, 39, source);
    _mav_put_uint8_t(buf, 40, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_CRC);
#else
    mavlink_global_position_sensor_t *packet = (mavlink_global_position_sensor_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->processing_time = processing_time;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt_ellipsoid = alt_ellipsoid;
    packet->alt = alt;
    packet->eph = eph;
    packet->epv = epv;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->id = id;
    packet->source = source;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POSITION_SENSOR UNPACKING


/**
 * @brief Get field target_system from global_position_sensor message
 *
 * @return  System ID (ID of target system, normally autopilot and ground station).
 */
static inline uint8_t mavlink_msg_global_position_sensor_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field target_component from global_position_sensor message
 *
 * @return  Component ID (normally 0 for broadcast).
 */
static inline uint8_t mavlink_msg_global_position_sensor_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field id from global_position_sensor message
 *
 * @return  Sensor ID
 */
static inline uint8_t mavlink_msg_global_position_sensor_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field time_usec from global_position_sensor message
 *
 * @return [us] Timestamp of message transmission (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_global_position_sensor_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field processing_time from global_position_sensor message
 *
 * @return [us] The time spent in processing the sensor data that is the basis for this position. The recipient can use this to improve time alignment of the data. This is the time between measurement (e.g. camera exposure time) and transmission of this message. Set to NaN if not known.
 */
static inline uint32_t mavlink_msg_global_position_sensor_get_processing_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field source from global_position_sensor message
 *
 * @return  Source of position/estimate (such as GNSS, estimator, etc.)
 */
static inline uint8_t mavlink_msg_global_position_sensor_get_source(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field flags from global_position_sensor message
 *
 * @return  Status flags
 */
static inline uint8_t mavlink_msg_global_position_sensor_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field lat from global_position_sensor message
 *
 * @return [degE7] Latitude (WGS84)
 */
static inline int32_t mavlink_msg_global_position_sensor_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field lon from global_position_sensor message
 *
 * @return [degE7] Longitude (WGS84)
 */
static inline int32_t mavlink_msg_global_position_sensor_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field alt_ellipsoid from global_position_sensor message
 *
 * @return [m] Altitude (WGS84 elipsoid), preferred if available
 */
static inline float mavlink_msg_global_position_sensor_get_alt_ellipsoid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field alt from global_position_sensor message
 *
 * @return [m] Altitude (MSL - position-system specific value) use if no alt_ellipsoid available
 */
static inline float mavlink_msg_global_position_sensor_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field eph from global_position_sensor message
 *
 * @return [m] Standard deviation of horizontal position error
 */
static inline float mavlink_msg_global_position_sensor_get_eph(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field epv from global_position_sensor message
 *
 * @return [m] Standard deviation of vertical position error
 */
static inline float mavlink_msg_global_position_sensor_get_epv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a global_position_sensor message into a struct
 *
 * @param msg The message to decode
 * @param global_position_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_sensor_decode(const mavlink_message_t* msg, mavlink_global_position_sensor_t* global_position_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    global_position_sensor->time_usec = mavlink_msg_global_position_sensor_get_time_usec(msg);
    global_position_sensor->processing_time = mavlink_msg_global_position_sensor_get_processing_time(msg);
    global_position_sensor->lat = mavlink_msg_global_position_sensor_get_lat(msg);
    global_position_sensor->lon = mavlink_msg_global_position_sensor_get_lon(msg);
    global_position_sensor->alt_ellipsoid = mavlink_msg_global_position_sensor_get_alt_ellipsoid(msg);
    global_position_sensor->alt = mavlink_msg_global_position_sensor_get_alt(msg);
    global_position_sensor->eph = mavlink_msg_global_position_sensor_get_eph(msg);
    global_position_sensor->epv = mavlink_msg_global_position_sensor_get_epv(msg);
    global_position_sensor->target_system = mavlink_msg_global_position_sensor_get_target_system(msg);
    global_position_sensor->target_component = mavlink_msg_global_position_sensor_get_target_component(msg);
    global_position_sensor->id = mavlink_msg_global_position_sensor_get_id(msg);
    global_position_sensor->source = mavlink_msg_global_position_sensor_get_source(msg);
    global_position_sensor->flags = mavlink_msg_global_position_sensor_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN;
        memset(global_position_sensor, 0, MAVLINK_MSG_ID_GLOBAL_POSITION_SENSOR_LEN);
    memcpy(global_position_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
