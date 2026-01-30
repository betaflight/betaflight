#pragma once
// MESSAGE ESC_STATUS PACKING

#define MAVLINK_MSG_ID_ESC_STATUS 291


typedef struct __mavlink_esc_status_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
 int32_t rpm[4]; /*< [rpm] Reported motor RPM from each ESC (negative for reverse rotation).*/
 float voltage[4]; /*< [V] Voltage measured from each ESC.*/
 float current[4]; /*< [A] Current measured from each ESC.*/
 uint8_t index; /*<  Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.*/
} mavlink_esc_status_t;

#define MAVLINK_MSG_ID_ESC_STATUS_LEN 57
#define MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN 57
#define MAVLINK_MSG_ID_291_LEN 57
#define MAVLINK_MSG_ID_291_MIN_LEN 57

#define MAVLINK_MSG_ID_ESC_STATUS_CRC 10
#define MAVLINK_MSG_ID_291_CRC 10

#define MAVLINK_MSG_ESC_STATUS_FIELD_RPM_LEN 4
#define MAVLINK_MSG_ESC_STATUS_FIELD_VOLTAGE_LEN 4
#define MAVLINK_MSG_ESC_STATUS_FIELD_CURRENT_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ESC_STATUS { \
    291, \
    "ESC_STATUS", \
    5, \
    {  { "index", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_esc_status_t, index) }, \
         { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_esc_status_t, time_usec) }, \
         { "rpm", NULL, MAVLINK_TYPE_INT32_T, 4, 8, offsetof(mavlink_esc_status_t, rpm) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_esc_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 4, 40, offsetof(mavlink_esc_status_t, current) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ESC_STATUS { \
    "ESC_STATUS", \
    5, \
    {  { "index", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_esc_status_t, index) }, \
         { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_esc_status_t, time_usec) }, \
         { "rpm", NULL, MAVLINK_TYPE_INT32_T, 4, 8, offsetof(mavlink_esc_status_t, rpm) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_esc_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 4, 40, offsetof(mavlink_esc_status_t, current) }, \
         } \
}
#endif

/**
 * @brief Pack a esc_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param index  Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param rpm [rpm] Reported motor RPM from each ESC (negative for reverse rotation).
 * @param voltage [V] Voltage measured from each ESC.
 * @param current [A] Current measured from each ESC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t index, uint64_t time_usec, const int32_t *rpm, const float *voltage, const float *current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 56, index);
    _mav_put_int32_t_array(buf, 8, rpm, 4);
    _mav_put_float_array(buf, 24, voltage, 4);
    _mav_put_float_array(buf, 40, current, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#else
    mavlink_esc_status_t packet;
    packet.time_usec = time_usec;
    packet.index = index;
    mav_array_assign_int32_t(packet.rpm, rpm, 4);
    mav_array_assign_float(packet.voltage, voltage, 4);
    mav_array_assign_float(packet.current, current, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
}

/**
 * @brief Pack a esc_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param index  Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param rpm [rpm] Reported motor RPM from each ESC (negative for reverse rotation).
 * @param voltage [V] Voltage measured from each ESC.
 * @param current [A] Current measured from each ESC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t index, uint64_t time_usec, const int32_t *rpm, const float *voltage, const float *current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 56, index);
    _mav_put_int32_t_array(buf, 8, rpm, 4);
    _mav_put_float_array(buf, 24, voltage, 4);
    _mav_put_float_array(buf, 40, current, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#else
    mavlink_esc_status_t packet;
    packet.time_usec = time_usec;
    packet.index = index;
    mav_array_memcpy(packet.rpm, rpm, sizeof(int32_t)*4);
    mav_array_memcpy(packet.voltage, voltage, sizeof(float)*4);
    mav_array_memcpy(packet.current, current, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif
}

/**
 * @brief Pack a esc_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param index  Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param rpm [rpm] Reported motor RPM from each ESC (negative for reverse rotation).
 * @param voltage [V] Voltage measured from each ESC.
 * @param current [A] Current measured from each ESC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t index,uint64_t time_usec,const int32_t *rpm,const float *voltage,const float *current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 56, index);
    _mav_put_int32_t_array(buf, 8, rpm, 4);
    _mav_put_float_array(buf, 24, voltage, 4);
    _mav_put_float_array(buf, 40, current, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#else
    mavlink_esc_status_t packet;
    packet.time_usec = time_usec;
    packet.index = index;
    mav_array_assign_int32_t(packet.rpm, rpm, 4);
    mav_array_assign_float(packet.voltage, voltage, 4);
    mav_array_assign_float(packet.current, current, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
}

/**
 * @brief Encode a esc_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param esc_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_esc_status_t* esc_status)
{
    return mavlink_msg_esc_status_pack(system_id, component_id, msg, esc_status->index, esc_status->time_usec, esc_status->rpm, esc_status->voltage, esc_status->current);
}

/**
 * @brief Encode a esc_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param esc_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_esc_status_t* esc_status)
{
    return mavlink_msg_esc_status_pack_chan(system_id, component_id, chan, msg, esc_status->index, esc_status->time_usec, esc_status->rpm, esc_status->voltage, esc_status->current);
}

/**
 * @brief Encode a esc_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param esc_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_esc_status_t* esc_status)
{
    return mavlink_msg_esc_status_pack_status(system_id, component_id, _status, msg,  esc_status->index, esc_status->time_usec, esc_status->rpm, esc_status->voltage, esc_status->current);
}

/**
 * @brief Send a esc_status message
 * @param chan MAVLink channel to send the message
 *
 * @param index  Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param rpm [rpm] Reported motor RPM from each ESC (negative for reverse rotation).
 * @param voltage [V] Voltage measured from each ESC.
 * @param current [A] Current measured from each ESC.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_esc_status_send(mavlink_channel_t chan, uint8_t index, uint64_t time_usec, const int32_t *rpm, const float *voltage, const float *current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 56, index);
    _mav_put_int32_t_array(buf, 8, rpm, 4);
    _mav_put_float_array(buf, 24, voltage, 4);
    _mav_put_float_array(buf, 40, current, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_STATUS, buf, MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#else
    mavlink_esc_status_t packet;
    packet.time_usec = time_usec;
    packet.index = index;
    mav_array_assign_int32_t(packet.rpm, rpm, 4);
    mav_array_assign_float(packet.voltage, voltage, 4);
    mav_array_assign_float(packet.current, current, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#endif
}

/**
 * @brief Send a esc_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_esc_status_send_struct(mavlink_channel_t chan, const mavlink_esc_status_t* esc_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_esc_status_send(chan, esc_status->index, esc_status->time_usec, esc_status->rpm, esc_status->voltage, esc_status->current);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_STATUS, (const char *)esc_status, MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ESC_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_esc_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t index, uint64_t time_usec, const int32_t *rpm, const float *voltage, const float *current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 56, index);
    _mav_put_int32_t_array(buf, 8, rpm, 4);
    _mav_put_float_array(buf, 24, voltage, 4);
    _mav_put_float_array(buf, 40, current, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_STATUS, buf, MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#else
    mavlink_esc_status_t *packet = (mavlink_esc_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->index = index;
    mav_array_assign_int32_t(packet->rpm, rpm, 4);
    mav_array_assign_float(packet->voltage, voltage, 4);
    mav_array_assign_float(packet->current, current, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_STATUS, (const char *)packet, MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ESC_STATUS UNPACKING


/**
 * @brief Get field index from esc_status message
 *
 * @return  Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.
 */
static inline uint8_t mavlink_msg_esc_status_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Get field time_usec from esc_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 */
static inline uint64_t mavlink_msg_esc_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field rpm from esc_status message
 *
 * @return [rpm] Reported motor RPM from each ESC (negative for reverse rotation).
 */
static inline uint16_t mavlink_msg_esc_status_get_rpm(const mavlink_message_t* msg, int32_t *rpm)
{
    return _MAV_RETURN_int32_t_array(msg, rpm, 4,  8);
}

/**
 * @brief Get field voltage from esc_status message
 *
 * @return [V] Voltage measured from each ESC.
 */
static inline uint16_t mavlink_msg_esc_status_get_voltage(const mavlink_message_t* msg, float *voltage)
{
    return _MAV_RETURN_float_array(msg, voltage, 4,  24);
}

/**
 * @brief Get field current from esc_status message
 *
 * @return [A] Current measured from each ESC.
 */
static inline uint16_t mavlink_msg_esc_status_get_current(const mavlink_message_t* msg, float *current)
{
    return _MAV_RETURN_float_array(msg, current, 4,  40);
}

/**
 * @brief Decode a esc_status message into a struct
 *
 * @param msg The message to decode
 * @param esc_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_esc_status_decode(const mavlink_message_t* msg, mavlink_esc_status_t* esc_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    esc_status->time_usec = mavlink_msg_esc_status_get_time_usec(msg);
    mavlink_msg_esc_status_get_rpm(msg, esc_status->rpm);
    mavlink_msg_esc_status_get_voltage(msg, esc_status->voltage);
    mavlink_msg_esc_status_get_current(msg, esc_status->current);
    esc_status->index = mavlink_msg_esc_status_get_index(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ESC_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ESC_STATUS_LEN;
        memset(esc_status, 0, MAVLINK_MSG_ID_ESC_STATUS_LEN);
    memcpy(esc_status, _MAV_PAYLOAD(msg), len);
#endif
}
