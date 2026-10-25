#pragma once
// MESSAGE FIGURE_EIGHT_EXECUTION_STATUS PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS 361


typedef struct __mavlink_figure_eight_execution_status_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float major_radius; /*< [m] Major axis radius of the figure eight. Positive: orbit the north circle clockwise. Negative: orbit the north circle counter-clockwise.*/
 float minor_radius; /*< [m] Minor axis radius of the figure eight. Defines the radius of two circles that make up the figure.*/
 float orientation; /*< [rad] Orientation of the figure eight major axis with respect to true north in [-pi,pi).*/
 int32_t x; /*<  X coordinate of center point. Coordinate system depends on frame field.*/
 int32_t y; /*<  Y coordinate of center point. Coordinate system depends on frame field.*/
 float z; /*< [m] Altitude of center point. Coordinate system depends on frame field.*/
 uint8_t frame; /*<  The coordinate system of the fields: x, y, z.*/
} mavlink_figure_eight_execution_status_t;

#define MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN 33
#define MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN 33
#define MAVLINK_MSG_ID_361_LEN 33
#define MAVLINK_MSG_ID_361_MIN_LEN 33

#define MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC 93
#define MAVLINK_MSG_ID_361_CRC 93



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FIGURE_EIGHT_EXECUTION_STATUS { \
    361, \
    "FIGURE_EIGHT_EXECUTION_STATUS", \
    8, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_figure_eight_execution_status_t, time_usec) }, \
         { "major_radius", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_figure_eight_execution_status_t, major_radius) }, \
         { "minor_radius", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_figure_eight_execution_status_t, minor_radius) }, \
         { "orientation", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_figure_eight_execution_status_t, orientation) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_figure_eight_execution_status_t, frame) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_figure_eight_execution_status_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_figure_eight_execution_status_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_figure_eight_execution_status_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FIGURE_EIGHT_EXECUTION_STATUS { \
    "FIGURE_EIGHT_EXECUTION_STATUS", \
    8, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_figure_eight_execution_status_t, time_usec) }, \
         { "major_radius", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_figure_eight_execution_status_t, major_radius) }, \
         { "minor_radius", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_figure_eight_execution_status_t, minor_radius) }, \
         { "orientation", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_figure_eight_execution_status_t, orientation) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_figure_eight_execution_status_t, frame) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_figure_eight_execution_status_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_figure_eight_execution_status_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_figure_eight_execution_status_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a figure_eight_execution_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param major_radius [m] Major axis radius of the figure eight. Positive: orbit the north circle clockwise. Negative: orbit the north circle counter-clockwise.
 * @param minor_radius [m] Minor axis radius of the figure eight. Defines the radius of two circles that make up the figure.
 * @param orientation [rad] Orientation of the figure eight major axis with respect to true north in [-pi,pi).
 * @param frame  The coordinate system of the fields: x, y, z.
 * @param x  X coordinate of center point. Coordinate system depends on frame field.
 * @param y  Y coordinate of center point. Coordinate system depends on frame field.
 * @param z [m] Altitude of center point. Coordinate system depends on frame field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_figure_eight_execution_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float major_radius, float minor_radius, float orientation, uint8_t frame, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, major_radius);
    _mav_put_float(buf, 12, minor_radius);
    _mav_put_float(buf, 16, orientation);
    _mav_put_int32_t(buf, 20, x);
    _mav_put_int32_t(buf, 24, y);
    _mav_put_float(buf, 28, z);
    _mav_put_uint8_t(buf, 32, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN);
#else
    mavlink_figure_eight_execution_status_t packet;
    packet.time_usec = time_usec;
    packet.major_radius = major_radius;
    packet.minor_radius = minor_radius;
    packet.orientation = orientation;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC);
}

/**
 * @brief Pack a figure_eight_execution_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param major_radius [m] Major axis radius of the figure eight. Positive: orbit the north circle clockwise. Negative: orbit the north circle counter-clockwise.
 * @param minor_radius [m] Minor axis radius of the figure eight. Defines the radius of two circles that make up the figure.
 * @param orientation [rad] Orientation of the figure eight major axis with respect to true north in [-pi,pi).
 * @param frame  The coordinate system of the fields: x, y, z.
 * @param x  X coordinate of center point. Coordinate system depends on frame field.
 * @param y  Y coordinate of center point. Coordinate system depends on frame field.
 * @param z [m] Altitude of center point. Coordinate system depends on frame field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_figure_eight_execution_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, float major_radius, float minor_radius, float orientation, uint8_t frame, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, major_radius);
    _mav_put_float(buf, 12, minor_radius);
    _mav_put_float(buf, 16, orientation);
    _mav_put_int32_t(buf, 20, x);
    _mav_put_int32_t(buf, 24, y);
    _mav_put_float(buf, 28, z);
    _mav_put_uint8_t(buf, 32, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN);
#else
    mavlink_figure_eight_execution_status_t packet;
    packet.time_usec = time_usec;
    packet.major_radius = major_radius;
    packet.minor_radius = minor_radius;
    packet.orientation = orientation;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN);
#endif
}

/**
 * @brief Pack a figure_eight_execution_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param major_radius [m] Major axis radius of the figure eight. Positive: orbit the north circle clockwise. Negative: orbit the north circle counter-clockwise.
 * @param minor_radius [m] Minor axis radius of the figure eight. Defines the radius of two circles that make up the figure.
 * @param orientation [rad] Orientation of the figure eight major axis with respect to true north in [-pi,pi).
 * @param frame  The coordinate system of the fields: x, y, z.
 * @param x  X coordinate of center point. Coordinate system depends on frame field.
 * @param y  Y coordinate of center point. Coordinate system depends on frame field.
 * @param z [m] Altitude of center point. Coordinate system depends on frame field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_figure_eight_execution_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float major_radius,float minor_radius,float orientation,uint8_t frame,int32_t x,int32_t y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, major_radius);
    _mav_put_float(buf, 12, minor_radius);
    _mav_put_float(buf, 16, orientation);
    _mav_put_int32_t(buf, 20, x);
    _mav_put_int32_t(buf, 24, y);
    _mav_put_float(buf, 28, z);
    _mav_put_uint8_t(buf, 32, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN);
#else
    mavlink_figure_eight_execution_status_t packet;
    packet.time_usec = time_usec;
    packet.major_radius = major_radius;
    packet.minor_radius = minor_radius;
    packet.orientation = orientation;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC);
}

/**
 * @brief Encode a figure_eight_execution_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param figure_eight_execution_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_figure_eight_execution_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_figure_eight_execution_status_t* figure_eight_execution_status)
{
    return mavlink_msg_figure_eight_execution_status_pack(system_id, component_id, msg, figure_eight_execution_status->time_usec, figure_eight_execution_status->major_radius, figure_eight_execution_status->minor_radius, figure_eight_execution_status->orientation, figure_eight_execution_status->frame, figure_eight_execution_status->x, figure_eight_execution_status->y, figure_eight_execution_status->z);
}

/**
 * @brief Encode a figure_eight_execution_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param figure_eight_execution_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_figure_eight_execution_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_figure_eight_execution_status_t* figure_eight_execution_status)
{
    return mavlink_msg_figure_eight_execution_status_pack_chan(system_id, component_id, chan, msg, figure_eight_execution_status->time_usec, figure_eight_execution_status->major_radius, figure_eight_execution_status->minor_radius, figure_eight_execution_status->orientation, figure_eight_execution_status->frame, figure_eight_execution_status->x, figure_eight_execution_status->y, figure_eight_execution_status->z);
}

/**
 * @brief Encode a figure_eight_execution_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param figure_eight_execution_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_figure_eight_execution_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_figure_eight_execution_status_t* figure_eight_execution_status)
{
    return mavlink_msg_figure_eight_execution_status_pack_status(system_id, component_id, _status, msg,  figure_eight_execution_status->time_usec, figure_eight_execution_status->major_radius, figure_eight_execution_status->minor_radius, figure_eight_execution_status->orientation, figure_eight_execution_status->frame, figure_eight_execution_status->x, figure_eight_execution_status->y, figure_eight_execution_status->z);
}

/**
 * @brief Send a figure_eight_execution_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param major_radius [m] Major axis radius of the figure eight. Positive: orbit the north circle clockwise. Negative: orbit the north circle counter-clockwise.
 * @param minor_radius [m] Minor axis radius of the figure eight. Defines the radius of two circles that make up the figure.
 * @param orientation [rad] Orientation of the figure eight major axis with respect to true north in [-pi,pi).
 * @param frame  The coordinate system of the fields: x, y, z.
 * @param x  X coordinate of center point. Coordinate system depends on frame field.
 * @param y  Y coordinate of center point. Coordinate system depends on frame field.
 * @param z [m] Altitude of center point. Coordinate system depends on frame field.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_figure_eight_execution_status_send(mavlink_channel_t chan, uint64_t time_usec, float major_radius, float minor_radius, float orientation, uint8_t frame, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, major_radius);
    _mav_put_float(buf, 12, minor_radius);
    _mav_put_float(buf, 16, orientation);
    _mav_put_int32_t(buf, 20, x);
    _mav_put_int32_t(buf, 24, y);
    _mav_put_float(buf, 28, z);
    _mav_put_uint8_t(buf, 32, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS, buf, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC);
#else
    mavlink_figure_eight_execution_status_t packet;
    packet.time_usec = time_usec;
    packet.major_radius = major_radius;
    packet.minor_radius = minor_radius;
    packet.orientation = orientation;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS, (const char *)&packet, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC);
#endif
}

/**
 * @brief Send a figure_eight_execution_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_figure_eight_execution_status_send_struct(mavlink_channel_t chan, const mavlink_figure_eight_execution_status_t* figure_eight_execution_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_figure_eight_execution_status_send(chan, figure_eight_execution_status->time_usec, figure_eight_execution_status->major_radius, figure_eight_execution_status->minor_radius, figure_eight_execution_status->orientation, figure_eight_execution_status->frame, figure_eight_execution_status->x, figure_eight_execution_status->y, figure_eight_execution_status->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS, (const char *)figure_eight_execution_status, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_figure_eight_execution_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float major_radius, float minor_radius, float orientation, uint8_t frame, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, major_radius);
    _mav_put_float(buf, 12, minor_radius);
    _mav_put_float(buf, 16, orientation);
    _mav_put_int32_t(buf, 20, x);
    _mav_put_int32_t(buf, 24, y);
    _mav_put_float(buf, 28, z);
    _mav_put_uint8_t(buf, 32, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS, buf, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC);
#else
    mavlink_figure_eight_execution_status_t *packet = (mavlink_figure_eight_execution_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->major_radius = major_radius;
    packet->minor_radius = minor_radius;
    packet->orientation = orientation;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS, (const char *)packet, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE FIGURE_EIGHT_EXECUTION_STATUS UNPACKING


/**
 * @brief Get field time_usec from figure_eight_execution_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_figure_eight_execution_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field major_radius from figure_eight_execution_status message
 *
 * @return [m] Major axis radius of the figure eight. Positive: orbit the north circle clockwise. Negative: orbit the north circle counter-clockwise.
 */
static inline float mavlink_msg_figure_eight_execution_status_get_major_radius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field minor_radius from figure_eight_execution_status message
 *
 * @return [m] Minor axis radius of the figure eight. Defines the radius of two circles that make up the figure.
 */
static inline float mavlink_msg_figure_eight_execution_status_get_minor_radius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field orientation from figure_eight_execution_status message
 *
 * @return [rad] Orientation of the figure eight major axis with respect to true north in [-pi,pi).
 */
static inline float mavlink_msg_figure_eight_execution_status_get_orientation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field frame from figure_eight_execution_status message
 *
 * @return  The coordinate system of the fields: x, y, z.
 */
static inline uint8_t mavlink_msg_figure_eight_execution_status_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field x from figure_eight_execution_status message
 *
 * @return  X coordinate of center point. Coordinate system depends on frame field.
 */
static inline int32_t mavlink_msg_figure_eight_execution_status_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field y from figure_eight_execution_status message
 *
 * @return  Y coordinate of center point. Coordinate system depends on frame field.
 */
static inline int32_t mavlink_msg_figure_eight_execution_status_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field z from figure_eight_execution_status message
 *
 * @return [m] Altitude of center point. Coordinate system depends on frame field.
 */
static inline float mavlink_msg_figure_eight_execution_status_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a figure_eight_execution_status message into a struct
 *
 * @param msg The message to decode
 * @param figure_eight_execution_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_figure_eight_execution_status_decode(const mavlink_message_t* msg, mavlink_figure_eight_execution_status_t* figure_eight_execution_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    figure_eight_execution_status->time_usec = mavlink_msg_figure_eight_execution_status_get_time_usec(msg);
    figure_eight_execution_status->major_radius = mavlink_msg_figure_eight_execution_status_get_major_radius(msg);
    figure_eight_execution_status->minor_radius = mavlink_msg_figure_eight_execution_status_get_minor_radius(msg);
    figure_eight_execution_status->orientation = mavlink_msg_figure_eight_execution_status_get_orientation(msg);
    figure_eight_execution_status->x = mavlink_msg_figure_eight_execution_status_get_x(msg);
    figure_eight_execution_status->y = mavlink_msg_figure_eight_execution_status_get_y(msg);
    figure_eight_execution_status->z = mavlink_msg_figure_eight_execution_status_get_z(msg);
    figure_eight_execution_status->frame = mavlink_msg_figure_eight_execution_status_get_frame(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN? msg->len : MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN;
        memset(figure_eight_execution_status, 0, MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN);
    memcpy(figure_eight_execution_status, _MAV_PAYLOAD(msg), len);
#endif
}
