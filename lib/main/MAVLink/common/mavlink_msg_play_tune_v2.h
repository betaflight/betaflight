#pragma once
// MESSAGE PLAY_TUNE_V2 PACKING

#define MAVLINK_MSG_ID_PLAY_TUNE_V2 400


typedef struct __mavlink_play_tune_v2_t {
 uint32_t format; /*<  Tune format*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 char tune[248]; /*<  Tune definition as a NULL-terminated string.*/
} mavlink_play_tune_v2_t;

#define MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN 254
#define MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN 254
#define MAVLINK_MSG_ID_400_LEN 254
#define MAVLINK_MSG_ID_400_MIN_LEN 254

#define MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC 110
#define MAVLINK_MSG_ID_400_CRC 110

#define MAVLINK_MSG_PLAY_TUNE_V2_FIELD_TUNE_LEN 248

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PLAY_TUNE_V2 { \
    400, \
    "PLAY_TUNE_V2", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_play_tune_v2_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_play_tune_v2_t, target_component) }, \
         { "format", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_play_tune_v2_t, format) }, \
         { "tune", NULL, MAVLINK_TYPE_CHAR, 248, 6, offsetof(mavlink_play_tune_v2_t, tune) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PLAY_TUNE_V2 { \
    "PLAY_TUNE_V2", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_play_tune_v2_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_play_tune_v2_t, target_component) }, \
         { "format", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_play_tune_v2_t, format) }, \
         { "tune", NULL, MAVLINK_TYPE_CHAR, 248, 6, offsetof(mavlink_play_tune_v2_t, tune) }, \
         } \
}
#endif

/**
 * @brief Pack a play_tune_v2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param format  Tune format
 * @param tune  Tune definition as a NULL-terminated string.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_play_tune_v2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t format, const char *tune)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN];
    _mav_put_uint32_t(buf, 0, format);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_char_array(buf, 6, tune, 248);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN);
#else
    mavlink_play_tune_v2_t packet;
    packet.format = format;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_assign_char(packet.tune, tune, 248);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLAY_TUNE_V2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC);
}

/**
 * @brief Pack a play_tune_v2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param format  Tune format
 * @param tune  Tune definition as a NULL-terminated string.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_play_tune_v2_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t format, const char *tune)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN];
    _mav_put_uint32_t(buf, 0, format);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_char_array(buf, 6, tune, 248);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN);
#else
    mavlink_play_tune_v2_t packet;
    packet.format = format;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.tune, tune, sizeof(char)*248);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLAY_TUNE_V2;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN);
#endif
}

/**
 * @brief Pack a play_tune_v2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param format  Tune format
 * @param tune  Tune definition as a NULL-terminated string.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_play_tune_v2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t format,const char *tune)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN];
    _mav_put_uint32_t(buf, 0, format);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_char_array(buf, 6, tune, 248);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN);
#else
    mavlink_play_tune_v2_t packet;
    packet.format = format;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_assign_char(packet.tune, tune, 248);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLAY_TUNE_V2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC);
}

/**
 * @brief Encode a play_tune_v2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param play_tune_v2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_play_tune_v2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_play_tune_v2_t* play_tune_v2)
{
    return mavlink_msg_play_tune_v2_pack(system_id, component_id, msg, play_tune_v2->target_system, play_tune_v2->target_component, play_tune_v2->format, play_tune_v2->tune);
}

/**
 * @brief Encode a play_tune_v2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param play_tune_v2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_play_tune_v2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_play_tune_v2_t* play_tune_v2)
{
    return mavlink_msg_play_tune_v2_pack_chan(system_id, component_id, chan, msg, play_tune_v2->target_system, play_tune_v2->target_component, play_tune_v2->format, play_tune_v2->tune);
}

/**
 * @brief Encode a play_tune_v2 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param play_tune_v2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_play_tune_v2_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_play_tune_v2_t* play_tune_v2)
{
    return mavlink_msg_play_tune_v2_pack_status(system_id, component_id, _status, msg,  play_tune_v2->target_system, play_tune_v2->target_component, play_tune_v2->format, play_tune_v2->tune);
}

/**
 * @brief Send a play_tune_v2 message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param format  Tune format
 * @param tune  Tune definition as a NULL-terminated string.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_play_tune_v2_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t format, const char *tune)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN];
    _mav_put_uint32_t(buf, 0, format);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_char_array(buf, 6, tune, 248);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE_V2, buf, MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC);
#else
    mavlink_play_tune_v2_t packet;
    packet.format = format;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_assign_char(packet.tune, tune, 248);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE_V2, (const char *)&packet, MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC);
#endif
}

/**
 * @brief Send a play_tune_v2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_play_tune_v2_send_struct(mavlink_channel_t chan, const mavlink_play_tune_v2_t* play_tune_v2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_play_tune_v2_send(chan, play_tune_v2->target_system, play_tune_v2->target_component, play_tune_v2->format, play_tune_v2->tune);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE_V2, (const char *)play_tune_v2, MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC);
#endif
}

#if MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_play_tune_v2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t format, const char *tune)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, format);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_char_array(buf, 6, tune, 248);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE_V2, buf, MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC);
#else
    mavlink_play_tune_v2_t *packet = (mavlink_play_tune_v2_t *)msgbuf;
    packet->format = format;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_assign_char(packet->tune, tune, 248);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE_V2, (const char *)packet, MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN, MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC);
#endif
}
#endif

#endif

// MESSAGE PLAY_TUNE_V2 UNPACKING


/**
 * @brief Get field target_system from play_tune_v2 message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_play_tune_v2_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from play_tune_v2 message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_play_tune_v2_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field format from play_tune_v2 message
 *
 * @return  Tune format
 */
static inline uint32_t mavlink_msg_play_tune_v2_get_format(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field tune from play_tune_v2 message
 *
 * @return  Tune definition as a NULL-terminated string.
 */
static inline uint16_t mavlink_msg_play_tune_v2_get_tune(const mavlink_message_t* msg, char *tune)
{
    return _MAV_RETURN_char_array(msg, tune, 248,  6);
}

/**
 * @brief Decode a play_tune_v2 message into a struct
 *
 * @param msg The message to decode
 * @param play_tune_v2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_play_tune_v2_decode(const mavlink_message_t* msg, mavlink_play_tune_v2_t* play_tune_v2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    play_tune_v2->format = mavlink_msg_play_tune_v2_get_format(msg);
    play_tune_v2->target_system = mavlink_msg_play_tune_v2_get_target_system(msg);
    play_tune_v2->target_component = mavlink_msg_play_tune_v2_get_target_component(msg);
    mavlink_msg_play_tune_v2_get_tune(msg, play_tune_v2->tune);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN? msg->len : MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN;
        memset(play_tune_v2, 0, MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN);
    memcpy(play_tune_v2, _MAV_PAYLOAD(msg), len);
#endif
}
