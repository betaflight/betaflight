#pragma once
// MESSAGE ORBIT_EXECUTION_STATUS PACKING

#define MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS 360


typedef struct __mavlink_orbit_execution_status_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float radius; /*< [m] Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.*/
 int32_t x; /*<  X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.*/
 int32_t y; /*<  Y coordinate of center point.  Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.*/
 float z; /*< [m] Altitude of center point. Coordinate system depends on frame field.*/
 uint8_t frame; /*<  The coordinate system of the fields: x, y, z.*/
} mavlink_orbit_execution_status_t;

#define MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN 25
#define MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN 25
#define MAVLINK_MSG_ID_360_LEN 25
#define MAVLINK_MSG_ID_360_MIN_LEN 25

#define MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC 11
#define MAVLINK_MSG_ID_360_CRC 11



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ORBIT_EXECUTION_STATUS { \
    360, \
    "ORBIT_EXECUTION_STATUS", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_orbit_execution_status_t, time_usec) }, \
         { "radius", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_orbit_execution_status_t, radius) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_orbit_execution_status_t, frame) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_orbit_execution_status_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_orbit_execution_status_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_orbit_execution_status_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ORBIT_EXECUTION_STATUS { \
    "ORBIT_EXECUTION_STATUS", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_orbit_execution_status_t, time_usec) }, \
         { "radius", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_orbit_execution_status_t, radius) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_orbit_execution_status_t, frame) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_orbit_execution_status_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_orbit_execution_status_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_orbit_execution_status_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a orbit_execution_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param radius [m] Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.
 * @param frame  The coordinate system of the fields: x, y, z.
 * @param x  X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 * @param y  Y coordinate of center point.  Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 * @param z [m] Altitude of center point. Coordinate system depends on frame field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_orbit_execution_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float radius, uint8_t frame, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, radius);
    _mav_put_int32_t(buf, 12, x);
    _mav_put_int32_t(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_uint8_t(buf, 24, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN);
#else
    mavlink_orbit_execution_status_t packet;
    packet.time_usec = time_usec;
    packet.radius = radius;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC);
}

/**
 * @brief Pack a orbit_execution_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param radius [m] Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.
 * @param frame  The coordinate system of the fields: x, y, z.
 * @param x  X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 * @param y  Y coordinate of center point.  Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 * @param z [m] Altitude of center point. Coordinate system depends on frame field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_orbit_execution_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, float radius, uint8_t frame, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, radius);
    _mav_put_int32_t(buf, 12, x);
    _mav_put_int32_t(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_uint8_t(buf, 24, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN);
#else
    mavlink_orbit_execution_status_t packet;
    packet.time_usec = time_usec;
    packet.radius = radius;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN);
#endif
}

/**
 * @brief Pack a orbit_execution_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param radius [m] Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.
 * @param frame  The coordinate system of the fields: x, y, z.
 * @param x  X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 * @param y  Y coordinate of center point.  Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 * @param z [m] Altitude of center point. Coordinate system depends on frame field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_orbit_execution_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float radius,uint8_t frame,int32_t x,int32_t y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, radius);
    _mav_put_int32_t(buf, 12, x);
    _mav_put_int32_t(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_uint8_t(buf, 24, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN);
#else
    mavlink_orbit_execution_status_t packet;
    packet.time_usec = time_usec;
    packet.radius = radius;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC);
}

/**
 * @brief Encode a orbit_execution_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param orbit_execution_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_orbit_execution_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_orbit_execution_status_t* orbit_execution_status)
{
    return mavlink_msg_orbit_execution_status_pack(system_id, component_id, msg, orbit_execution_status->time_usec, orbit_execution_status->radius, orbit_execution_status->frame, orbit_execution_status->x, orbit_execution_status->y, orbit_execution_status->z);
}

/**
 * @brief Encode a orbit_execution_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param orbit_execution_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_orbit_execution_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_orbit_execution_status_t* orbit_execution_status)
{
    return mavlink_msg_orbit_execution_status_pack_chan(system_id, component_id, chan, msg, orbit_execution_status->time_usec, orbit_execution_status->radius, orbit_execution_status->frame, orbit_execution_status->x, orbit_execution_status->y, orbit_execution_status->z);
}

/**
 * @brief Encode a orbit_execution_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param orbit_execution_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_orbit_execution_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_orbit_execution_status_t* orbit_execution_status)
{
    return mavlink_msg_orbit_execution_status_pack_status(system_id, component_id, _status, msg,  orbit_execution_status->time_usec, orbit_execution_status->radius, orbit_execution_status->frame, orbit_execution_status->x, orbit_execution_status->y, orbit_execution_status->z);
}

/**
 * @brief Send a orbit_execution_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param radius [m] Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.
 * @param frame  The coordinate system of the fields: x, y, z.
 * @param x  X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 * @param y  Y coordinate of center point.  Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 * @param z [m] Altitude of center point. Coordinate system depends on frame field.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_orbit_execution_status_send(mavlink_channel_t chan, uint64_t time_usec, float radius, uint8_t frame, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, radius);
    _mav_put_int32_t(buf, 12, x);
    _mav_put_int32_t(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_uint8_t(buf, 24, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS, buf, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC);
#else
    mavlink_orbit_execution_status_t packet;
    packet.time_usec = time_usec;
    packet.radius = radius;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC);
#endif
}

/**
 * @brief Send a orbit_execution_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_orbit_execution_status_send_struct(mavlink_channel_t chan, const mavlink_orbit_execution_status_t* orbit_execution_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_orbit_execution_status_send(chan, orbit_execution_status->time_usec, orbit_execution_status->radius, orbit_execution_status->frame, orbit_execution_status->x, orbit_execution_status->y, orbit_execution_status->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS, (const char *)orbit_execution_status, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_orbit_execution_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float radius, uint8_t frame, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, radius);
    _mav_put_int32_t(buf, 12, x);
    _mav_put_int32_t(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_uint8_t(buf, 24, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS, buf, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC);
#else
    mavlink_orbit_execution_status_t *packet = (mavlink_orbit_execution_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->radius = radius;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS, (const char *)packet, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ORBIT_EXECUTION_STATUS UNPACKING


/**
 * @brief Get field time_usec from orbit_execution_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_orbit_execution_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field radius from orbit_execution_status message
 *
 * @return [m] Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.
 */
static inline float mavlink_msg_orbit_execution_status_get_radius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field frame from orbit_execution_status message
 *
 * @return  The coordinate system of the fields: x, y, z.
 */
static inline uint8_t mavlink_msg_orbit_execution_status_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field x from orbit_execution_status message
 *
 * @return  X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 */
static inline int32_t mavlink_msg_orbit_execution_status_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field y from orbit_execution_status message
 *
 * @return  Y coordinate of center point.  Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
 */
static inline int32_t mavlink_msg_orbit_execution_status_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field z from orbit_execution_status message
 *
 * @return [m] Altitude of center point. Coordinate system depends on frame field.
 */
static inline float mavlink_msg_orbit_execution_status_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a orbit_execution_status message into a struct
 *
 * @param msg The message to decode
 * @param orbit_execution_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_orbit_execution_status_decode(const mavlink_message_t* msg, mavlink_orbit_execution_status_t* orbit_execution_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    orbit_execution_status->time_usec = mavlink_msg_orbit_execution_status_get_time_usec(msg);
    orbit_execution_status->radius = mavlink_msg_orbit_execution_status_get_radius(msg);
    orbit_execution_status->x = mavlink_msg_orbit_execution_status_get_x(msg);
    orbit_execution_status->y = mavlink_msg_orbit_execution_status_get_y(msg);
    orbit_execution_status->z = mavlink_msg_orbit_execution_status_get_z(msg);
    orbit_execution_status->frame = mavlink_msg_orbit_execution_status_get_frame(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN;
        memset(orbit_execution_status, 0, MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN);
    memcpy(orbit_execution_status, _MAV_PAYLOAD(msg), len);
#endif
}
