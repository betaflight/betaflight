#pragma once
// MESSAGE SERIAL_CONTROL PACKING

#define MAVLINK_MSG_ID_SERIAL_CONTROL 126


typedef struct __mavlink_serial_control_t {
 uint32_t baudrate; /*< [bits/s] Baudrate of transfer. Zero means no change.*/
 uint16_t timeout; /*< [ms] Timeout for reply data*/
 uint8_t device; /*<  Serial control device type.*/
 uint8_t flags; /*<  Bitmap of serial control flags.*/
 uint8_t count; /*< [bytes] how many bytes in this transfer*/
 uint8_t data[70]; /*<  serial data*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
} mavlink_serial_control_t;

#define MAVLINK_MSG_ID_SERIAL_CONTROL_LEN 81
#define MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN 79
#define MAVLINK_MSG_ID_126_LEN 81
#define MAVLINK_MSG_ID_126_MIN_LEN 79

#define MAVLINK_MSG_ID_SERIAL_CONTROL_CRC 220
#define MAVLINK_MSG_ID_126_CRC 220

#define MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN 70

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_CONTROL { \
    126, \
    "SERIAL_CONTROL", \
    8, \
    {  { "device", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_serial_control_t, device) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_serial_control_t, flags) }, \
         { "timeout", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_serial_control_t, timeout) }, \
         { "baudrate", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_serial_control_t, baudrate) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_serial_control_t, count) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 70, 9, offsetof(mavlink_serial_control_t, data) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 79, offsetof(mavlink_serial_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_serial_control_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_CONTROL { \
    "SERIAL_CONTROL", \
    8, \
    {  { "device", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_serial_control_t, device) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_serial_control_t, flags) }, \
         { "timeout", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_serial_control_t, timeout) }, \
         { "baudrate", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_serial_control_t, baudrate) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_serial_control_t, count) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 70, 9, offsetof(mavlink_serial_control_t, data) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 79, offsetof(mavlink_serial_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_serial_control_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param device  Serial control device type.
 * @param flags  Bitmap of serial control flags.
 * @param timeout [ms] Timeout for reply data
 * @param baudrate [bits/s] Baudrate of transfer. Zero means no change.
 * @param count [bytes] how many bytes in this transfer
 * @param data  serial data
 * @param target_system  System ID
 * @param target_component  Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t *data, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_CONTROL_LEN];
    _mav_put_uint32_t(buf, 0, baudrate);
    _mav_put_uint16_t(buf, 4, timeout);
    _mav_put_uint8_t(buf, 6, device);
    _mav_put_uint8_t(buf, 7, flags);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_uint8_t(buf, 79, target_system);
    _mav_put_uint8_t(buf, 80, target_component);
    _mav_put_uint8_t_array(buf, 9, data, 70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN);
#else
    mavlink_serial_control_t packet;
    packet.baudrate = baudrate;
    packet.timeout = timeout;
    packet.device = device;
    packet.flags = flags;
    packet.count = count;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_assign_uint8_t(packet.data, data, 70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
}

/**
 * @brief Pack a serial_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param device  Serial control device type.
 * @param flags  Bitmap of serial control flags.
 * @param timeout [ms] Timeout for reply data
 * @param baudrate [bits/s] Baudrate of transfer. Zero means no change.
 * @param count [bytes] how many bytes in this transfer
 * @param data  serial data
 * @param target_system  System ID
 * @param target_component  Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t *data, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_CONTROL_LEN];
    _mav_put_uint32_t(buf, 0, baudrate);
    _mav_put_uint16_t(buf, 4, timeout);
    _mav_put_uint8_t(buf, 6, device);
    _mav_put_uint8_t(buf, 7, flags);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_uint8_t(buf, 79, target_system);
    _mav_put_uint8_t(buf, 80, target_component);
    _mav_put_uint8_t_array(buf, 9, data, 70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN);
#else
    mavlink_serial_control_t packet;
    packet.baudrate = baudrate;
    packet.timeout = timeout;
    packet.device = device;
    packet.flags = flags;
    packet.count = count;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a serial_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param device  Serial control device type.
 * @param flags  Bitmap of serial control flags.
 * @param timeout [ms] Timeout for reply data
 * @param baudrate [bits/s] Baudrate of transfer. Zero means no change.
 * @param count [bytes] how many bytes in this transfer
 * @param data  serial data
 * @param target_system  System ID
 * @param target_component  Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t device,uint8_t flags,uint16_t timeout,uint32_t baudrate,uint8_t count,const uint8_t *data,uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_CONTROL_LEN];
    _mav_put_uint32_t(buf, 0, baudrate);
    _mav_put_uint16_t(buf, 4, timeout);
    _mav_put_uint8_t(buf, 6, device);
    _mav_put_uint8_t(buf, 7, flags);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_uint8_t(buf, 79, target_system);
    _mav_put_uint8_t(buf, 80, target_component);
    _mav_put_uint8_t_array(buf, 9, data, 70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN);
#else
    mavlink_serial_control_t packet;
    packet.baudrate = baudrate;
    packet.timeout = timeout;
    packet.device = device;
    packet.flags = flags;
    packet.count = count;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_assign_uint8_t(packet.data, data, 70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
}

/**
 * @brief Encode a serial_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_control_t* serial_control)
{
    return mavlink_msg_serial_control_pack(system_id, component_id, msg, serial_control->device, serial_control->flags, serial_control->timeout, serial_control->baudrate, serial_control->count, serial_control->data, serial_control->target_system, serial_control->target_component);
}

/**
 * @brief Encode a serial_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_control_t* serial_control)
{
    return mavlink_msg_serial_control_pack_chan(system_id, component_id, chan, msg, serial_control->device, serial_control->flags, serial_control->timeout, serial_control->baudrate, serial_control->count, serial_control->data, serial_control->target_system, serial_control->target_component);
}

/**
 * @brief Encode a serial_control struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param serial_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_control_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_serial_control_t* serial_control)
{
    return mavlink_msg_serial_control_pack_status(system_id, component_id, _status, msg,  serial_control->device, serial_control->flags, serial_control->timeout, serial_control->baudrate, serial_control->count, serial_control->data, serial_control->target_system, serial_control->target_component);
}

/**
 * @brief Send a serial_control message
 * @param chan MAVLink channel to send the message
 *
 * @param device  Serial control device type.
 * @param flags  Bitmap of serial control flags.
 * @param timeout [ms] Timeout for reply data
 * @param baudrate [bits/s] Baudrate of transfer. Zero means no change.
 * @param count [bytes] how many bytes in this transfer
 * @param data  serial data
 * @param target_system  System ID
 * @param target_component  Component ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_control_send(mavlink_channel_t chan, uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t *data, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_CONTROL_LEN];
    _mav_put_uint32_t(buf, 0, baudrate);
    _mav_put_uint16_t(buf, 4, timeout);
    _mav_put_uint8_t(buf, 6, device);
    _mav_put_uint8_t(buf, 7, flags);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_uint8_t(buf, 79, target_system);
    _mav_put_uint8_t(buf, 80, target_component);
    _mav_put_uint8_t_array(buf, 9, data, 70);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_CONTROL, buf, MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
#else
    mavlink_serial_control_t packet;
    packet.baudrate = baudrate;
    packet.timeout = timeout;
    packet.device = device;
    packet.flags = flags;
    packet.count = count;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_assign_uint8_t(packet.data, data, 70);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
#endif
}

/**
 * @brief Send a serial_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_control_send_struct(mavlink_channel_t chan, const mavlink_serial_control_t* serial_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_control_send(chan, serial_control->device, serial_control->flags, serial_control->timeout, serial_control->baudrate, serial_control->count, serial_control->data, serial_control->target_system, serial_control->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_CONTROL, (const char *)serial_control, MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t *data, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, baudrate);
    _mav_put_uint16_t(buf, 4, timeout);
    _mav_put_uint8_t(buf, 6, device);
    _mav_put_uint8_t(buf, 7, flags);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_uint8_t(buf, 79, target_system);
    _mav_put_uint8_t(buf, 80, target_component);
    _mav_put_uint8_t_array(buf, 9, data, 70);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_CONTROL, buf, MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
#else
    mavlink_serial_control_t *packet = (mavlink_serial_control_t *)msgbuf;
    packet->baudrate = baudrate;
    packet->timeout = timeout;
    packet->device = device;
    packet->flags = flags;
    packet->count = count;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_assign_uint8_t(packet->data, data, 70);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_CONTROL, (const char *)packet, MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN, MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_CONTROL UNPACKING


/**
 * @brief Get field device from serial_control message
 *
 * @return  Serial control device type.
 */
static inline uint8_t mavlink_msg_serial_control_get_device(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field flags from serial_control message
 *
 * @return  Bitmap of serial control flags.
 */
static inline uint8_t mavlink_msg_serial_control_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field timeout from serial_control message
 *
 * @return [ms] Timeout for reply data
 */
static inline uint16_t mavlink_msg_serial_control_get_timeout(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field baudrate from serial_control message
 *
 * @return [bits/s] Baudrate of transfer. Zero means no change.
 */
static inline uint32_t mavlink_msg_serial_control_get_baudrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field count from serial_control message
 *
 * @return [bytes] how many bytes in this transfer
 */
static inline uint8_t mavlink_msg_serial_control_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field data from serial_control message
 *
 * @return  serial data
 */
static inline uint16_t mavlink_msg_serial_control_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 70,  9);
}

/**
 * @brief Get field target_system from serial_control message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_serial_control_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  79);
}

/**
 * @brief Get field target_component from serial_control message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_serial_control_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Decode a serial_control message into a struct
 *
 * @param msg The message to decode
 * @param serial_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_control_decode(const mavlink_message_t* msg, mavlink_serial_control_t* serial_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    serial_control->baudrate = mavlink_msg_serial_control_get_baudrate(msg);
    serial_control->timeout = mavlink_msg_serial_control_get_timeout(msg);
    serial_control->device = mavlink_msg_serial_control_get_device(msg);
    serial_control->flags = mavlink_msg_serial_control_get_flags(msg);
    serial_control->count = mavlink_msg_serial_control_get_count(msg);
    mavlink_msg_serial_control_get_data(msg, serial_control->data);
    serial_control->target_system = mavlink_msg_serial_control_get_target_system(msg);
    serial_control->target_component = mavlink_msg_serial_control_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_CONTROL_LEN;
        memset(serial_control, 0, MAVLINK_MSG_ID_SERIAL_CONTROL_LEN);
    memcpy(serial_control, _MAV_PAYLOAD(msg), len);
#endif
}
