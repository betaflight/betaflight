#pragma once
// MESSAGE COMPONENT_INFORMATION PACKING

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION 395


typedef struct __mavlink_component_information_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint32_t general_metadata_file_crc; /*<  CRC32 of the general metadata file (general_metadata_uri).*/
 uint32_t peripherals_metadata_file_crc; /*<  CRC32 of peripherals metadata file (peripherals_metadata_uri).*/
 char general_metadata_uri[100]; /*<  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.*/
 char peripherals_metadata_uri[100]; /*<  (Optional) MAVLink FTP URI for the peripherals metadata file (COMP_METADATA_TYPE_PERIPHERALS), which may be compressed with xz. This contains data about "attached components" such as UAVCAN nodes. The peripherals are in a separate file because the information must be generated dynamically at runtime. The string needs to be zero terminated.*/
} mavlink_component_information_t;

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN 212
#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN 212
#define MAVLINK_MSG_ID_395_LEN 212
#define MAVLINK_MSG_ID_395_MIN_LEN 212

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC 0
#define MAVLINK_MSG_ID_395_CRC 0

#define MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_GENERAL_METADATA_URI_LEN 100
#define MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_PERIPHERALS_METADATA_URI_LEN 100

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMPONENT_INFORMATION { \
    395, \
    "COMPONENT_INFORMATION", \
    5, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_component_information_t, time_boot_ms) }, \
         { "general_metadata_file_crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_component_information_t, general_metadata_file_crc) }, \
         { "general_metadata_uri", NULL, MAVLINK_TYPE_CHAR, 100, 12, offsetof(mavlink_component_information_t, general_metadata_uri) }, \
         { "peripherals_metadata_file_crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_component_information_t, peripherals_metadata_file_crc) }, \
         { "peripherals_metadata_uri", NULL, MAVLINK_TYPE_CHAR, 100, 112, offsetof(mavlink_component_information_t, peripherals_metadata_uri) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMPONENT_INFORMATION { \
    "COMPONENT_INFORMATION", \
    5, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_component_information_t, time_boot_ms) }, \
         { "general_metadata_file_crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_component_information_t, general_metadata_file_crc) }, \
         { "general_metadata_uri", NULL, MAVLINK_TYPE_CHAR, 100, 12, offsetof(mavlink_component_information_t, general_metadata_uri) }, \
         { "peripherals_metadata_file_crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_component_information_t, peripherals_metadata_file_crc) }, \
         { "peripherals_metadata_uri", NULL, MAVLINK_TYPE_CHAR, 100, 112, offsetof(mavlink_component_information_t, peripherals_metadata_uri) }, \
         } \
}
#endif

/**
 * @brief Pack a component_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param general_metadata_file_crc  CRC32 of the general metadata file (general_metadata_uri).
 * @param general_metadata_uri  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 * @param peripherals_metadata_file_crc  CRC32 of peripherals metadata file (peripherals_metadata_uri).
 * @param peripherals_metadata_uri  (Optional) MAVLink FTP URI for the peripherals metadata file (COMP_METADATA_TYPE_PERIPHERALS), which may be compressed with xz. This contains data about "attached components" such as UAVCAN nodes. The peripherals are in a separate file because the information must be generated dynamically at runtime. The string needs to be zero terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t general_metadata_file_crc, const char *general_metadata_uri, uint32_t peripherals_metadata_file_crc, const char *peripherals_metadata_uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, general_metadata_file_crc);
    _mav_put_uint32_t(buf, 8, peripherals_metadata_file_crc);
    _mav_put_char_array(buf, 12, general_metadata_uri, 100);
    _mav_put_char_array(buf, 112, peripherals_metadata_uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN);
#else
    mavlink_component_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.general_metadata_file_crc = general_metadata_file_crc;
    packet.peripherals_metadata_file_crc = peripherals_metadata_file_crc;
    mav_array_assign_char(packet.general_metadata_uri, general_metadata_uri, 100);
    mav_array_assign_char(packet.peripherals_metadata_uri, peripherals_metadata_uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC);
}

/**
 * @brief Pack a component_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param general_metadata_file_crc  CRC32 of the general metadata file (general_metadata_uri).
 * @param general_metadata_uri  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 * @param peripherals_metadata_file_crc  CRC32 of peripherals metadata file (peripherals_metadata_uri).
 * @param peripherals_metadata_uri  (Optional) MAVLink FTP URI for the peripherals metadata file (COMP_METADATA_TYPE_PERIPHERALS), which may be compressed with xz. This contains data about "attached components" such as UAVCAN nodes. The peripherals are in a separate file because the information must be generated dynamically at runtime. The string needs to be zero terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t general_metadata_file_crc, const char *general_metadata_uri, uint32_t peripherals_metadata_file_crc, const char *peripherals_metadata_uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, general_metadata_file_crc);
    _mav_put_uint32_t(buf, 8, peripherals_metadata_file_crc);
    _mav_put_char_array(buf, 12, general_metadata_uri, 100);
    _mav_put_char_array(buf, 112, peripherals_metadata_uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN);
#else
    mavlink_component_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.general_metadata_file_crc = general_metadata_file_crc;
    packet.peripherals_metadata_file_crc = peripherals_metadata_file_crc;
    mav_array_memcpy(packet.general_metadata_uri, general_metadata_uri, sizeof(char)*100);
    mav_array_memcpy(packet.peripherals_metadata_uri, peripherals_metadata_uri, sizeof(char)*100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a component_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param general_metadata_file_crc  CRC32 of the general metadata file (general_metadata_uri).
 * @param general_metadata_uri  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 * @param peripherals_metadata_file_crc  CRC32 of peripherals metadata file (peripherals_metadata_uri).
 * @param peripherals_metadata_uri  (Optional) MAVLink FTP URI for the peripherals metadata file (COMP_METADATA_TYPE_PERIPHERALS), which may be compressed with xz. This contains data about "attached components" such as UAVCAN nodes. The peripherals are in a separate file because the information must be generated dynamically at runtime. The string needs to be zero terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint32_t general_metadata_file_crc,const char *general_metadata_uri,uint32_t peripherals_metadata_file_crc,const char *peripherals_metadata_uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, general_metadata_file_crc);
    _mav_put_uint32_t(buf, 8, peripherals_metadata_file_crc);
    _mav_put_char_array(buf, 12, general_metadata_uri, 100);
    _mav_put_char_array(buf, 112, peripherals_metadata_uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN);
#else
    mavlink_component_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.general_metadata_file_crc = general_metadata_file_crc;
    packet.peripherals_metadata_file_crc = peripherals_metadata_file_crc;
    mav_array_assign_char(packet.general_metadata_uri, general_metadata_uri, 100);
    mav_array_assign_char(packet.peripherals_metadata_uri, peripherals_metadata_uri, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC);
}

/**
 * @brief Encode a component_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param component_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_component_information_t* component_information)
{
    return mavlink_msg_component_information_pack(system_id, component_id, msg, component_information->time_boot_ms, component_information->general_metadata_file_crc, component_information->general_metadata_uri, component_information->peripherals_metadata_file_crc, component_information->peripherals_metadata_uri);
}

/**
 * @brief Encode a component_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param component_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_component_information_t* component_information)
{
    return mavlink_msg_component_information_pack_chan(system_id, component_id, chan, msg, component_information->time_boot_ms, component_information->general_metadata_file_crc, component_information->general_metadata_uri, component_information->peripherals_metadata_file_crc, component_information->peripherals_metadata_uri);
}

/**
 * @brief Encode a component_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param component_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_component_information_t* component_information)
{
    return mavlink_msg_component_information_pack_status(system_id, component_id, _status, msg,  component_information->time_boot_ms, component_information->general_metadata_file_crc, component_information->general_metadata_uri, component_information->peripherals_metadata_file_crc, component_information->peripherals_metadata_uri);
}

/**
 * @brief Send a component_information message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param general_metadata_file_crc  CRC32 of the general metadata file (general_metadata_uri).
 * @param general_metadata_uri  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 * @param peripherals_metadata_file_crc  CRC32 of peripherals metadata file (peripherals_metadata_uri).
 * @param peripherals_metadata_uri  (Optional) MAVLink FTP URI for the peripherals metadata file (COMP_METADATA_TYPE_PERIPHERALS), which may be compressed with xz. This contains data about "attached components" such as UAVCAN nodes. The peripherals are in a separate file because the information must be generated dynamically at runtime. The string needs to be zero terminated.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_component_information_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint32_t general_metadata_file_crc, const char *general_metadata_uri, uint32_t peripherals_metadata_file_crc, const char *peripherals_metadata_uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, general_metadata_file_crc);
    _mav_put_uint32_t(buf, 8, peripherals_metadata_file_crc);
    _mav_put_char_array(buf, 12, general_metadata_uri, 100);
    _mav_put_char_array(buf, 112, peripherals_metadata_uri, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION, buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC);
#else
    mavlink_component_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.general_metadata_file_crc = general_metadata_file_crc;
    packet.peripherals_metadata_file_crc = peripherals_metadata_file_crc;
    mav_array_assign_char(packet.general_metadata_uri, general_metadata_uri, 100);
    mav_array_assign_char(packet.peripherals_metadata_uri, peripherals_metadata_uri, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a component_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_component_information_send_struct(mavlink_channel_t chan, const mavlink_component_information_t* component_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_component_information_send(chan, component_information->time_boot_ms, component_information->general_metadata_file_crc, component_information->general_metadata_uri, component_information->peripherals_metadata_file_crc, component_information->peripherals_metadata_uri);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION, (const char *)component_information, MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_component_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint32_t general_metadata_file_crc, const char *general_metadata_uri, uint32_t peripherals_metadata_file_crc, const char *peripherals_metadata_uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, general_metadata_file_crc);
    _mav_put_uint32_t(buf, 8, peripherals_metadata_file_crc);
    _mav_put_char_array(buf, 12, general_metadata_uri, 100);
    _mav_put_char_array(buf, 112, peripherals_metadata_uri, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION, buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC);
#else
    mavlink_component_information_t *packet = (mavlink_component_information_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->general_metadata_file_crc = general_metadata_file_crc;
    packet->peripherals_metadata_file_crc = peripherals_metadata_file_crc;
    mav_array_assign_char(packet->general_metadata_uri, general_metadata_uri, 100);
    mav_array_assign_char(packet->peripherals_metadata_uri, peripherals_metadata_uri, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE COMPONENT_INFORMATION UNPACKING


/**
 * @brief Get field time_boot_ms from component_information message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_component_information_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field general_metadata_file_crc from component_information message
 *
 * @return  CRC32 of the general metadata file (general_metadata_uri).
 */
static inline uint32_t mavlink_msg_component_information_get_general_metadata_file_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field general_metadata_uri from component_information message
 *
 * @return  MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
 */
static inline uint16_t mavlink_msg_component_information_get_general_metadata_uri(const mavlink_message_t* msg, char *general_metadata_uri)
{
    return _MAV_RETURN_char_array(msg, general_metadata_uri, 100,  12);
}

/**
 * @brief Get field peripherals_metadata_file_crc from component_information message
 *
 * @return  CRC32 of peripherals metadata file (peripherals_metadata_uri).
 */
static inline uint32_t mavlink_msg_component_information_get_peripherals_metadata_file_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field peripherals_metadata_uri from component_information message
 *
 * @return  (Optional) MAVLink FTP URI for the peripherals metadata file (COMP_METADATA_TYPE_PERIPHERALS), which may be compressed with xz. This contains data about "attached components" such as UAVCAN nodes. The peripherals are in a separate file because the information must be generated dynamically at runtime. The string needs to be zero terminated.
 */
static inline uint16_t mavlink_msg_component_information_get_peripherals_metadata_uri(const mavlink_message_t* msg, char *peripherals_metadata_uri)
{
    return _MAV_RETURN_char_array(msg, peripherals_metadata_uri, 100,  112);
}

/**
 * @brief Decode a component_information message into a struct
 *
 * @param msg The message to decode
 * @param component_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_component_information_decode(const mavlink_message_t* msg, mavlink_component_information_t* component_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    component_information->time_boot_ms = mavlink_msg_component_information_get_time_boot_ms(msg);
    component_information->general_metadata_file_crc = mavlink_msg_component_information_get_general_metadata_file_crc(msg);
    component_information->peripherals_metadata_file_crc = mavlink_msg_component_information_get_peripherals_metadata_file_crc(msg);
    mavlink_msg_component_information_get_general_metadata_uri(msg, component_information->general_metadata_uri);
    mavlink_msg_component_information_get_peripherals_metadata_uri(msg, component_information->peripherals_metadata_uri);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN;
        memset(component_information, 0, MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN);
    memcpy(component_information, _MAV_PAYLOAD(msg), len);
#endif
}
