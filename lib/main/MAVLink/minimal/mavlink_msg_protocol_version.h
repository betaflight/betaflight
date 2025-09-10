#pragma once
// MESSAGE PROTOCOL_VERSION PACKING

#define MAVLINK_MSG_ID_PROTOCOL_VERSION 300


typedef struct __mavlink_protocol_version_t {
 uint16_t version; /*<  Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.*/
 uint16_t min_version; /*<  Minimum MAVLink version supported*/
 uint16_t max_version; /*<  Maximum MAVLink version supported (set to the same value as version by default)*/
 uint8_t spec_version_hash[8]; /*<  The first 8 bytes (not characters printed in hex!) of the git hash.*/
 uint8_t library_version_hash[8]; /*<  The first 8 bytes (not characters printed in hex!) of the git hash.*/
} mavlink_protocol_version_t;

#define MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN 22
#define MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN 22
#define MAVLINK_MSG_ID_300_LEN 22
#define MAVLINK_MSG_ID_300_MIN_LEN 22

#define MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC 217
#define MAVLINK_MSG_ID_300_CRC 217

#define MAVLINK_MSG_PROTOCOL_VERSION_FIELD_SPEC_VERSION_HASH_LEN 8
#define MAVLINK_MSG_PROTOCOL_VERSION_FIELD_LIBRARY_VERSION_HASH_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PROTOCOL_VERSION { \
    300, \
    "PROTOCOL_VERSION", \
    5, \
    {  { "version", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_protocol_version_t, version) }, \
         { "min_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_protocol_version_t, min_version) }, \
         { "max_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_protocol_version_t, max_version) }, \
         { "spec_version_hash", NULL, MAVLINK_TYPE_UINT8_T, 8, 6, offsetof(mavlink_protocol_version_t, spec_version_hash) }, \
         { "library_version_hash", NULL, MAVLINK_TYPE_UINT8_T, 8, 14, offsetof(mavlink_protocol_version_t, library_version_hash) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PROTOCOL_VERSION { \
    "PROTOCOL_VERSION", \
    5, \
    {  { "version", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_protocol_version_t, version) }, \
         { "min_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_protocol_version_t, min_version) }, \
         { "max_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_protocol_version_t, max_version) }, \
         { "spec_version_hash", NULL, MAVLINK_TYPE_UINT8_T, 8, 6, offsetof(mavlink_protocol_version_t, spec_version_hash) }, \
         { "library_version_hash", NULL, MAVLINK_TYPE_UINT8_T, 8, 14, offsetof(mavlink_protocol_version_t, library_version_hash) }, \
         } \
}
#endif

/**
 * @brief Pack a protocol_version message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param version  Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
 * @param min_version  Minimum MAVLink version supported
 * @param max_version  Maximum MAVLink version supported (set to the same value as version by default)
 * @param spec_version_hash  The first 8 bytes (not characters printed in hex!) of the git hash.
 * @param library_version_hash  The first 8 bytes (not characters printed in hex!) of the git hash.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_protocol_version_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t version, uint16_t min_version, uint16_t max_version, const uint8_t *spec_version_hash, const uint8_t *library_version_hash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN];
    _mav_put_uint16_t(buf, 0, version);
    _mav_put_uint16_t(buf, 2, min_version);
    _mav_put_uint16_t(buf, 4, max_version);
    _mav_put_uint8_t_array(buf, 6, spec_version_hash, 8);
    _mav_put_uint8_t_array(buf, 14, library_version_hash, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN);
#else
    mavlink_protocol_version_t packet;
    packet.version = version;
    packet.min_version = min_version;
    packet.max_version = max_version;
    mav_array_assign_uint8_t(packet.spec_version_hash, spec_version_hash, 8);
    mav_array_assign_uint8_t(packet.library_version_hash, library_version_hash, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PROTOCOL_VERSION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC);
}

/**
 * @brief Pack a protocol_version message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param version  Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
 * @param min_version  Minimum MAVLink version supported
 * @param max_version  Maximum MAVLink version supported (set to the same value as version by default)
 * @param spec_version_hash  The first 8 bytes (not characters printed in hex!) of the git hash.
 * @param library_version_hash  The first 8 bytes (not characters printed in hex!) of the git hash.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_protocol_version_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t version, uint16_t min_version, uint16_t max_version, const uint8_t *spec_version_hash, const uint8_t *library_version_hash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN];
    _mav_put_uint16_t(buf, 0, version);
    _mav_put_uint16_t(buf, 2, min_version);
    _mav_put_uint16_t(buf, 4, max_version);
    _mav_put_uint8_t_array(buf, 6, spec_version_hash, 8);
    _mav_put_uint8_t_array(buf, 14, library_version_hash, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN);
#else
    mavlink_protocol_version_t packet;
    packet.version = version;
    packet.min_version = min_version;
    packet.max_version = max_version;
    mav_array_memcpy(packet.spec_version_hash, spec_version_hash, sizeof(uint8_t)*8);
    mav_array_memcpy(packet.library_version_hash, library_version_hash, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PROTOCOL_VERSION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN);
#endif
}

/**
 * @brief Pack a protocol_version message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param version  Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
 * @param min_version  Minimum MAVLink version supported
 * @param max_version  Maximum MAVLink version supported (set to the same value as version by default)
 * @param spec_version_hash  The first 8 bytes (not characters printed in hex!) of the git hash.
 * @param library_version_hash  The first 8 bytes (not characters printed in hex!) of the git hash.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_protocol_version_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t version,uint16_t min_version,uint16_t max_version,const uint8_t *spec_version_hash,const uint8_t *library_version_hash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN];
    _mav_put_uint16_t(buf, 0, version);
    _mav_put_uint16_t(buf, 2, min_version);
    _mav_put_uint16_t(buf, 4, max_version);
    _mav_put_uint8_t_array(buf, 6, spec_version_hash, 8);
    _mav_put_uint8_t_array(buf, 14, library_version_hash, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN);
#else
    mavlink_protocol_version_t packet;
    packet.version = version;
    packet.min_version = min_version;
    packet.max_version = max_version;
    mav_array_assign_uint8_t(packet.spec_version_hash, spec_version_hash, 8);
    mav_array_assign_uint8_t(packet.library_version_hash, library_version_hash, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PROTOCOL_VERSION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC);
}

/**
 * @brief Encode a protocol_version struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param protocol_version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_protocol_version_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_protocol_version_t* protocol_version)
{
    return mavlink_msg_protocol_version_pack(system_id, component_id, msg, protocol_version->version, protocol_version->min_version, protocol_version->max_version, protocol_version->spec_version_hash, protocol_version->library_version_hash);
}

/**
 * @brief Encode a protocol_version struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param protocol_version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_protocol_version_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_protocol_version_t* protocol_version)
{
    return mavlink_msg_protocol_version_pack_chan(system_id, component_id, chan, msg, protocol_version->version, protocol_version->min_version, protocol_version->max_version, protocol_version->spec_version_hash, protocol_version->library_version_hash);
}

/**
 * @brief Encode a protocol_version struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param protocol_version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_protocol_version_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_protocol_version_t* protocol_version)
{
    return mavlink_msg_protocol_version_pack_status(system_id, component_id, _status, msg,  protocol_version->version, protocol_version->min_version, protocol_version->max_version, protocol_version->spec_version_hash, protocol_version->library_version_hash);
}

/**
 * @brief Send a protocol_version message
 * @param chan MAVLink channel to send the message
 *
 * @param version  Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
 * @param min_version  Minimum MAVLink version supported
 * @param max_version  Maximum MAVLink version supported (set to the same value as version by default)
 * @param spec_version_hash  The first 8 bytes (not characters printed in hex!) of the git hash.
 * @param library_version_hash  The first 8 bytes (not characters printed in hex!) of the git hash.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_protocol_version_send(mavlink_channel_t chan, uint16_t version, uint16_t min_version, uint16_t max_version, const uint8_t *spec_version_hash, const uint8_t *library_version_hash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN];
    _mav_put_uint16_t(buf, 0, version);
    _mav_put_uint16_t(buf, 2, min_version);
    _mav_put_uint16_t(buf, 4, max_version);
    _mav_put_uint8_t_array(buf, 6, spec_version_hash, 8);
    _mav_put_uint8_t_array(buf, 14, library_version_hash, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROTOCOL_VERSION, buf, MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC);
#else
    mavlink_protocol_version_t packet;
    packet.version = version;
    packet.min_version = min_version;
    packet.max_version = max_version;
    mav_array_assign_uint8_t(packet.spec_version_hash, spec_version_hash, 8);
    mav_array_assign_uint8_t(packet.library_version_hash, library_version_hash, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROTOCOL_VERSION, (const char *)&packet, MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC);
#endif
}

/**
 * @brief Send a protocol_version message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_protocol_version_send_struct(mavlink_channel_t chan, const mavlink_protocol_version_t* protocol_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_protocol_version_send(chan, protocol_version->version, protocol_version->min_version, protocol_version->max_version, protocol_version->spec_version_hash, protocol_version->library_version_hash);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROTOCOL_VERSION, (const char *)protocol_version, MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC);
#endif
}

#if MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_protocol_version_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t version, uint16_t min_version, uint16_t max_version, const uint8_t *spec_version_hash, const uint8_t *library_version_hash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, version);
    _mav_put_uint16_t(buf, 2, min_version);
    _mav_put_uint16_t(buf, 4, max_version);
    _mav_put_uint8_t_array(buf, 6, spec_version_hash, 8);
    _mav_put_uint8_t_array(buf, 14, library_version_hash, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROTOCOL_VERSION, buf, MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC);
#else
    mavlink_protocol_version_t *packet = (mavlink_protocol_version_t *)msgbuf;
    packet->version = version;
    packet->min_version = min_version;
    packet->max_version = max_version;
    mav_array_assign_uint8_t(packet->spec_version_hash, spec_version_hash, 8);
    mav_array_assign_uint8_t(packet->library_version_hash, library_version_hash, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROTOCOL_VERSION, (const char *)packet, MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN, MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC);
#endif
}
#endif

#endif

// MESSAGE PROTOCOL_VERSION UNPACKING


/**
 * @brief Get field version from protocol_version message
 *
 * @return  Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
 */
static inline uint16_t mavlink_msg_protocol_version_get_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field min_version from protocol_version message
 *
 * @return  Minimum MAVLink version supported
 */
static inline uint16_t mavlink_msg_protocol_version_get_min_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field max_version from protocol_version message
 *
 * @return  Maximum MAVLink version supported (set to the same value as version by default)
 */
static inline uint16_t mavlink_msg_protocol_version_get_max_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field spec_version_hash from protocol_version message
 *
 * @return  The first 8 bytes (not characters printed in hex!) of the git hash.
 */
static inline uint16_t mavlink_msg_protocol_version_get_spec_version_hash(const mavlink_message_t* msg, uint8_t *spec_version_hash)
{
    return _MAV_RETURN_uint8_t_array(msg, spec_version_hash, 8,  6);
}

/**
 * @brief Get field library_version_hash from protocol_version message
 *
 * @return  The first 8 bytes (not characters printed in hex!) of the git hash.
 */
static inline uint16_t mavlink_msg_protocol_version_get_library_version_hash(const mavlink_message_t* msg, uint8_t *library_version_hash)
{
    return _MAV_RETURN_uint8_t_array(msg, library_version_hash, 8,  14);
}

/**
 * @brief Decode a protocol_version message into a struct
 *
 * @param msg The message to decode
 * @param protocol_version C-struct to decode the message contents into
 */
static inline void mavlink_msg_protocol_version_decode(const mavlink_message_t* msg, mavlink_protocol_version_t* protocol_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    protocol_version->version = mavlink_msg_protocol_version_get_version(msg);
    protocol_version->min_version = mavlink_msg_protocol_version_get_min_version(msg);
    protocol_version->max_version = mavlink_msg_protocol_version_get_max_version(msg);
    mavlink_msg_protocol_version_get_spec_version_hash(msg, protocol_version->spec_version_hash);
    mavlink_msg_protocol_version_get_library_version_hash(msg, protocol_version->library_version_hash);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN? msg->len : MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN;
        memset(protocol_version, 0, MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN);
    memcpy(protocol_version, _MAV_PAYLOAD(msg), len);
#endif
}
