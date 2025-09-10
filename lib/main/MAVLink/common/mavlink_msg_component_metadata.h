#pragma once
// MESSAGE COMPONENT_METADATA PACKING

#define MAVLINK_MSG_ID_COMPONENT_METADATA 397


typedef struct __mavlink_component_metadata_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint32_t file_crc; /*<  CRC32 of the general metadata file.*/
 char uri[100]; /*<  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.*/
} mavlink_component_metadata_t;

#define MAVLINK_MSG_ID_COMPONENT_METADATA_LEN 108
#define MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN 108
#define MAVLINK_MSG_ID_397_LEN 108
#define MAVLINK_MSG_ID_397_MIN_LEN 108

#define MAVLINK_MSG_ID_COMPONENT_METADATA_CRC 182
#define MAVLINK_MSG_ID_397_CRC 182

#define MAVLINK_MSG_COMPONENT_METADATA_FIELD_URI_LEN 100

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMPONENT_METADATA { \
    397, \
    "COMPONENT_METADATA", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_component_metadata_t, time_boot_ms) }, \
         { "file_crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_component_metadata_t, file_crc) }, \
         { "uri", NULL, MAVLINK_TYPE_CHAR, 100, 8, offsetof(mavlink_component_metadata_t, uri) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMPONENT_METADATA { \
    "COMPONENT_METADATA", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_component_metadata_t, time_boot_ms) }, \
         { "file_crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_component_metadata_t, file_crc) }, \
         { "uri", NULL, MAVLINK_TYPE_CHAR, 100, 8, offsetof(mavlink_component_metadata_t, uri) }, \
         } \
}
#endif

/**
 * @brief Pack a component_metadata message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param file_crc  CRC32 of the general metadata file.
 * @param uri  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_metadata_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t file_crc, const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_METADATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, file_crc);
    _mav_put_char_array(buf, 8, uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN);
#else
    mavlink_component_metadata_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.file_crc = file_crc;
    mav_array_assign_char(packet.uri, uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_METADATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_CRC);
}

/**
 * @brief Pack a component_metadata message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param file_crc  CRC32 of the general metadata file.
 * @param uri  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_metadata_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t file_crc, const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_METADATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, file_crc);
    _mav_put_char_array(buf, 8, uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN);
#else
    mavlink_component_metadata_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.file_crc = file_crc;
    mav_array_memcpy(packet.uri, uri, sizeof(char)*100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_METADATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN);
#endif
}

/**
 * @brief Pack a component_metadata message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param file_crc  CRC32 of the general metadata file.
 * @param uri  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_metadata_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint32_t file_crc,const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_METADATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, file_crc);
    _mav_put_char_array(buf, 8, uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN);
#else
    mavlink_component_metadata_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.file_crc = file_crc;
    mav_array_assign_char(packet.uri, uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_METADATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_CRC);
}

/**
 * @brief Encode a component_metadata struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param component_metadata C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_metadata_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_component_metadata_t* component_metadata)
{
    return mavlink_msg_component_metadata_pack(system_id, component_id, msg, component_metadata->time_boot_ms, component_metadata->file_crc, component_metadata->uri);
}

/**
 * @brief Encode a component_metadata struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param component_metadata C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_metadata_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_component_metadata_t* component_metadata)
{
    return mavlink_msg_component_metadata_pack_chan(system_id, component_id, chan, msg, component_metadata->time_boot_ms, component_metadata->file_crc, component_metadata->uri);
}

/**
 * @brief Encode a component_metadata struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param component_metadata C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_metadata_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_component_metadata_t* component_metadata)
{
    return mavlink_msg_component_metadata_pack_status(system_id, component_id, _status, msg,  component_metadata->time_boot_ms, component_metadata->file_crc, component_metadata->uri);
}

/**
 * @brief Send a component_metadata message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param file_crc  CRC32 of the general metadata file.
 * @param uri  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_component_metadata_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint32_t file_crc, const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_METADATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, file_crc);
    _mav_put_char_array(buf, 8, uri, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_METADATA, buf, MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_CRC);
#else
    mavlink_component_metadata_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.file_crc = file_crc;
    mav_array_assign_char(packet.uri, uri, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_METADATA, (const char *)&packet, MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_CRC);
#endif
}

/**
 * @brief Send a component_metadata message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_component_metadata_send_struct(mavlink_channel_t chan, const mavlink_component_metadata_t* component_metadata)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_component_metadata_send(chan, component_metadata->time_boot_ms, component_metadata->file_crc, component_metadata->uri);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_METADATA, (const char *)component_metadata, MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMPONENT_METADATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_component_metadata_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint32_t file_crc, const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, file_crc);
    _mav_put_char_array(buf, 8, uri, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_METADATA, buf, MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_CRC);
#else
    mavlink_component_metadata_t *packet = (mavlink_component_metadata_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->file_crc = file_crc;
    mav_array_assign_char(packet->uri, uri, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_METADATA, (const char *)packet, MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN, MAVLINK_MSG_ID_COMPONENT_METADATA_CRC);
#endif
}
#endif

#endif

// MESSAGE COMPONENT_METADATA UNPACKING


/**
 * @brief Get field time_boot_ms from component_metadata message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_component_metadata_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field file_crc from component_metadata message
 *
 * @return  CRC32 of the general metadata file.
 */
static inline uint32_t mavlink_msg_component_metadata_get_file_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field uri from component_metadata message
 *
 * @return  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 */
static inline uint16_t mavlink_msg_component_metadata_get_uri(const mavlink_message_t* msg, char *uri)
{
    return _MAV_RETURN_char_array(msg, uri, 100,  8);
}

/**
 * @brief Decode a component_metadata message into a struct
 *
 * @param msg The message to decode
 * @param component_metadata C-struct to decode the message contents into
 */
static inline void mavlink_msg_component_metadata_decode(const mavlink_message_t* msg, mavlink_component_metadata_t* component_metadata)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    component_metadata->time_boot_ms = mavlink_msg_component_metadata_get_time_boot_ms(msg);
    component_metadata->file_crc = mavlink_msg_component_metadata_get_file_crc(msg);
    mavlink_msg_component_metadata_get_uri(msg, component_metadata->uri);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMPONENT_METADATA_LEN? msg->len : MAVLINK_MSG_ID_COMPONENT_METADATA_LEN;
        memset(component_metadata, 0, MAVLINK_MSG_ID_COMPONENT_METADATA_LEN);
    memcpy(component_metadata, _MAV_PAYLOAD(msg), len);
#endif
}
