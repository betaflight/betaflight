#pragma once
// MESSAGE COMPONENT_INFORMATION_BASIC PACKING

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC 396


typedef struct __mavlink_component_information_basic_t {
 uint64_t capabilities; /*<  Component capability flags*/
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint32_t time_manufacture_s; /*< [s] Date of manufacture as a UNIX Epoch time (since 1.1.1970) in seconds.*/
 char vendor_name[32]; /*<  Name of the component vendor. Needs to be zero terminated. The field is optional and can be empty/all zeros.*/
 char model_name[32]; /*<  Name of the component model. Needs to be zero terminated. The field is optional and can be empty/all zeros.*/
 char software_version[24]; /*<  Software version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.*/
 char hardware_version[24]; /*<  Hardware version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.*/
 char serial_number[32]; /*<  Hardware serial number. The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.*/
} mavlink_component_information_basic_t;

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN 160
#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN 160
#define MAVLINK_MSG_ID_396_LEN 160
#define MAVLINK_MSG_ID_396_MIN_LEN 160

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC 50
#define MAVLINK_MSG_ID_396_CRC 50

#define MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_VENDOR_NAME_LEN 32
#define MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_MODEL_NAME_LEN 32
#define MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_SOFTWARE_VERSION_LEN 24
#define MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_HARDWARE_VERSION_LEN 24
#define MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_SERIAL_NUMBER_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMPONENT_INFORMATION_BASIC { \
    396, \
    "COMPONENT_INFORMATION_BASIC", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_component_information_basic_t, time_boot_ms) }, \
         { "capabilities", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_component_information_basic_t, capabilities) }, \
         { "time_manufacture_s", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_component_information_basic_t, time_manufacture_s) }, \
         { "vendor_name", NULL, MAVLINK_TYPE_CHAR, 32, 16, offsetof(mavlink_component_information_basic_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_CHAR, 32, 48, offsetof(mavlink_component_information_basic_t, model_name) }, \
         { "software_version", NULL, MAVLINK_TYPE_CHAR, 24, 80, offsetof(mavlink_component_information_basic_t, software_version) }, \
         { "hardware_version", NULL, MAVLINK_TYPE_CHAR, 24, 104, offsetof(mavlink_component_information_basic_t, hardware_version) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 32, 128, offsetof(mavlink_component_information_basic_t, serial_number) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMPONENT_INFORMATION_BASIC { \
    "COMPONENT_INFORMATION_BASIC", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_component_information_basic_t, time_boot_ms) }, \
         { "capabilities", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_component_information_basic_t, capabilities) }, \
         { "time_manufacture_s", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_component_information_basic_t, time_manufacture_s) }, \
         { "vendor_name", NULL, MAVLINK_TYPE_CHAR, 32, 16, offsetof(mavlink_component_information_basic_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_CHAR, 32, 48, offsetof(mavlink_component_information_basic_t, model_name) }, \
         { "software_version", NULL, MAVLINK_TYPE_CHAR, 24, 80, offsetof(mavlink_component_information_basic_t, software_version) }, \
         { "hardware_version", NULL, MAVLINK_TYPE_CHAR, 24, 104, offsetof(mavlink_component_information_basic_t, hardware_version) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 32, 128, offsetof(mavlink_component_information_basic_t, serial_number) }, \
         } \
}
#endif

/**
 * @brief Pack a component_information_basic message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param capabilities  Component capability flags
 * @param time_manufacture_s [s] Date of manufacture as a UNIX Epoch time (since 1.1.1970) in seconds.
 * @param vendor_name  Name of the component vendor. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 * @param model_name  Name of the component model. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 * @param software_version  Software version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @param hardware_version  Hardware version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @param serial_number  Hardware serial number. The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_information_basic_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint64_t capabilities, uint32_t time_manufacture_s, const char *vendor_name, const char *model_name, const char *software_version, const char *hardware_version, const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN];
    _mav_put_uint64_t(buf, 0, capabilities);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_uint32_t(buf, 12, time_manufacture_s);
    _mav_put_char_array(buf, 16, vendor_name, 32);
    _mav_put_char_array(buf, 48, model_name, 32);
    _mav_put_char_array(buf, 80, software_version, 24);
    _mav_put_char_array(buf, 104, hardware_version, 24);
    _mav_put_char_array(buf, 128, serial_number, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN);
#else
    mavlink_component_information_basic_t packet;
    packet.capabilities = capabilities;
    packet.time_boot_ms = time_boot_ms;
    packet.time_manufacture_s = time_manufacture_s;
    mav_array_assign_char(packet.vendor_name, vendor_name, 32);
    mav_array_assign_char(packet.model_name, model_name, 32);
    mav_array_assign_char(packet.software_version, software_version, 24);
    mav_array_assign_char(packet.hardware_version, hardware_version, 24);
    mav_array_assign_char(packet.serial_number, serial_number, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC);
}

/**
 * @brief Pack a component_information_basic message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param capabilities  Component capability flags
 * @param time_manufacture_s [s] Date of manufacture as a UNIX Epoch time (since 1.1.1970) in seconds.
 * @param vendor_name  Name of the component vendor. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 * @param model_name  Name of the component model. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 * @param software_version  Software version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @param hardware_version  Hardware version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @param serial_number  Hardware serial number. The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_information_basic_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint64_t capabilities, uint32_t time_manufacture_s, const char *vendor_name, const char *model_name, const char *software_version, const char *hardware_version, const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN];
    _mav_put_uint64_t(buf, 0, capabilities);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_uint32_t(buf, 12, time_manufacture_s);
    _mav_put_char_array(buf, 16, vendor_name, 32);
    _mav_put_char_array(buf, 48, model_name, 32);
    _mav_put_char_array(buf, 80, software_version, 24);
    _mav_put_char_array(buf, 104, hardware_version, 24);
    _mav_put_char_array(buf, 128, serial_number, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN);
#else
    mavlink_component_information_basic_t packet;
    packet.capabilities = capabilities;
    packet.time_boot_ms = time_boot_ms;
    packet.time_manufacture_s = time_manufacture_s;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.software_version, software_version, sizeof(char)*24);
    mav_array_memcpy(packet.hardware_version, hardware_version, sizeof(char)*24);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN);
#endif
}

/**
 * @brief Pack a component_information_basic message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param capabilities  Component capability flags
 * @param time_manufacture_s [s] Date of manufacture as a UNIX Epoch time (since 1.1.1970) in seconds.
 * @param vendor_name  Name of the component vendor. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 * @param model_name  Name of the component model. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 * @param software_version  Software version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @param hardware_version  Hardware version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @param serial_number  Hardware serial number. The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_information_basic_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint64_t capabilities,uint32_t time_manufacture_s,const char *vendor_name,const char *model_name,const char *software_version,const char *hardware_version,const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN];
    _mav_put_uint64_t(buf, 0, capabilities);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_uint32_t(buf, 12, time_manufacture_s);
    _mav_put_char_array(buf, 16, vendor_name, 32);
    _mav_put_char_array(buf, 48, model_name, 32);
    _mav_put_char_array(buf, 80, software_version, 24);
    _mav_put_char_array(buf, 104, hardware_version, 24);
    _mav_put_char_array(buf, 128, serial_number, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN);
#else
    mavlink_component_information_basic_t packet;
    packet.capabilities = capabilities;
    packet.time_boot_ms = time_boot_ms;
    packet.time_manufacture_s = time_manufacture_s;
    mav_array_assign_char(packet.vendor_name, vendor_name, 32);
    mav_array_assign_char(packet.model_name, model_name, 32);
    mav_array_assign_char(packet.software_version, software_version, 24);
    mav_array_assign_char(packet.hardware_version, hardware_version, 24);
    mav_array_assign_char(packet.serial_number, serial_number, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC);
}

/**
 * @brief Encode a component_information_basic struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param component_information_basic C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_information_basic_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_component_information_basic_t* component_information_basic)
{
    return mavlink_msg_component_information_basic_pack(system_id, component_id, msg, component_information_basic->time_boot_ms, component_information_basic->capabilities, component_information_basic->time_manufacture_s, component_information_basic->vendor_name, component_information_basic->model_name, component_information_basic->software_version, component_information_basic->hardware_version, component_information_basic->serial_number);
}

/**
 * @brief Encode a component_information_basic struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param component_information_basic C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_information_basic_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_component_information_basic_t* component_information_basic)
{
    return mavlink_msg_component_information_basic_pack_chan(system_id, component_id, chan, msg, component_information_basic->time_boot_ms, component_information_basic->capabilities, component_information_basic->time_manufacture_s, component_information_basic->vendor_name, component_information_basic->model_name, component_information_basic->software_version, component_information_basic->hardware_version, component_information_basic->serial_number);
}

/**
 * @brief Encode a component_information_basic struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param component_information_basic C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_information_basic_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_component_information_basic_t* component_information_basic)
{
    return mavlink_msg_component_information_basic_pack_status(system_id, component_id, _status, msg,  component_information_basic->time_boot_ms, component_information_basic->capabilities, component_information_basic->time_manufacture_s, component_information_basic->vendor_name, component_information_basic->model_name, component_information_basic->software_version, component_information_basic->hardware_version, component_information_basic->serial_number);
}

/**
 * @brief Send a component_information_basic message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param capabilities  Component capability flags
 * @param time_manufacture_s [s] Date of manufacture as a UNIX Epoch time (since 1.1.1970) in seconds.
 * @param vendor_name  Name of the component vendor. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 * @param model_name  Name of the component model. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 * @param software_version  Software version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @param hardware_version  Hardware version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 * @param serial_number  Hardware serial number. The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_component_information_basic_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint64_t capabilities, uint32_t time_manufacture_s, const char *vendor_name, const char *model_name, const char *software_version, const char *hardware_version, const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN];
    _mav_put_uint64_t(buf, 0, capabilities);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_uint32_t(buf, 12, time_manufacture_s);
    _mav_put_char_array(buf, 16, vendor_name, 32);
    _mav_put_char_array(buf, 48, model_name, 32);
    _mav_put_char_array(buf, 80, software_version, 24);
    _mav_put_char_array(buf, 104, hardware_version, 24);
    _mav_put_char_array(buf, 128, serial_number, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC, buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC);
#else
    mavlink_component_information_basic_t packet;
    packet.capabilities = capabilities;
    packet.time_boot_ms = time_boot_ms;
    packet.time_manufacture_s = time_manufacture_s;
    mav_array_assign_char(packet.vendor_name, vendor_name, 32);
    mav_array_assign_char(packet.model_name, model_name, 32);
    mav_array_assign_char(packet.software_version, software_version, 24);
    mav_array_assign_char(packet.hardware_version, hardware_version, 24);
    mav_array_assign_char(packet.serial_number, serial_number, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC, (const char *)&packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC);
#endif
}

/**
 * @brief Send a component_information_basic message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_component_information_basic_send_struct(mavlink_channel_t chan, const mavlink_component_information_basic_t* component_information_basic)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_component_information_basic_send(chan, component_information_basic->time_boot_ms, component_information_basic->capabilities, component_information_basic->time_manufacture_s, component_information_basic->vendor_name, component_information_basic->model_name, component_information_basic->software_version, component_information_basic->hardware_version, component_information_basic->serial_number);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC, (const char *)component_information_basic, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_component_information_basic_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint64_t capabilities, uint32_t time_manufacture_s, const char *vendor_name, const char *model_name, const char *software_version, const char *hardware_version, const char *serial_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, capabilities);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_uint32_t(buf, 12, time_manufacture_s);
    _mav_put_char_array(buf, 16, vendor_name, 32);
    _mav_put_char_array(buf, 48, model_name, 32);
    _mav_put_char_array(buf, 80, software_version, 24);
    _mav_put_char_array(buf, 104, hardware_version, 24);
    _mav_put_char_array(buf, 128, serial_number, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC, buf, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC);
#else
    mavlink_component_information_basic_t *packet = (mavlink_component_information_basic_t *)msgbuf;
    packet->capabilities = capabilities;
    packet->time_boot_ms = time_boot_ms;
    packet->time_manufacture_s = time_manufacture_s;
    mav_array_assign_char(packet->vendor_name, vendor_name, 32);
    mav_array_assign_char(packet->model_name, model_name, 32);
    mav_array_assign_char(packet->software_version, software_version, 24);
    mav_array_assign_char(packet->hardware_version, hardware_version, 24);
    mav_array_assign_char(packet->serial_number, serial_number, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC, (const char *)packet, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC);
#endif
}
#endif

#endif

// MESSAGE COMPONENT_INFORMATION_BASIC UNPACKING


/**
 * @brief Get field time_boot_ms from component_information_basic message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_component_information_basic_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field capabilities from component_information_basic message
 *
 * @return  Component capability flags
 */
static inline uint64_t mavlink_msg_component_information_basic_get_capabilities(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_manufacture_s from component_information_basic message
 *
 * @return [s] Date of manufacture as a UNIX Epoch time (since 1.1.1970) in seconds.
 */
static inline uint32_t mavlink_msg_component_information_basic_get_time_manufacture_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field vendor_name from component_information_basic message
 *
 * @return  Name of the component vendor. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 */
static inline uint16_t mavlink_msg_component_information_basic_get_vendor_name(const mavlink_message_t* msg, char *vendor_name)
{
    return _MAV_RETURN_char_array(msg, vendor_name, 32,  16);
}

/**
 * @brief Get field model_name from component_information_basic message
 *
 * @return  Name of the component model. Needs to be zero terminated. The field is optional and can be empty/all zeros.
 */
static inline uint16_t mavlink_msg_component_information_basic_get_model_name(const mavlink_message_t* msg, char *model_name)
{
    return _MAV_RETURN_char_array(msg, model_name, 32,  48);
}

/**
 * @brief Get field software_version from component_information_basic message
 *
 * @return  Software version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 */
static inline uint16_t mavlink_msg_component_information_basic_get_software_version(const mavlink_message_t* msg, char *software_version)
{
    return _MAV_RETURN_char_array(msg, software_version, 24,  80);
}

/**
 * @brief Get field hardware_version from component_information_basic message
 *
 * @return  Hardware version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 */
static inline uint16_t mavlink_msg_component_information_basic_get_hardware_version(const mavlink_message_t* msg, char *hardware_version)
{
    return _MAV_RETURN_char_array(msg, hardware_version, 24,  104);
}

/**
 * @brief Get field serial_number from component_information_basic message
 *
 * @return  Hardware serial number. The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
 */
static inline uint16_t mavlink_msg_component_information_basic_get_serial_number(const mavlink_message_t* msg, char *serial_number)
{
    return _MAV_RETURN_char_array(msg, serial_number, 32,  128);
}

/**
 * @brief Decode a component_information_basic message into a struct
 *
 * @param msg The message to decode
 * @param component_information_basic C-struct to decode the message contents into
 */
static inline void mavlink_msg_component_information_basic_decode(const mavlink_message_t* msg, mavlink_component_information_basic_t* component_information_basic)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    component_information_basic->capabilities = mavlink_msg_component_information_basic_get_capabilities(msg);
    component_information_basic->time_boot_ms = mavlink_msg_component_information_basic_get_time_boot_ms(msg);
    component_information_basic->time_manufacture_s = mavlink_msg_component_information_basic_get_time_manufacture_s(msg);
    mavlink_msg_component_information_basic_get_vendor_name(msg, component_information_basic->vendor_name);
    mavlink_msg_component_information_basic_get_model_name(msg, component_information_basic->model_name);
    mavlink_msg_component_information_basic_get_software_version(msg, component_information_basic->software_version);
    mavlink_msg_component_information_basic_get_hardware_version(msg, component_information_basic->hardware_version);
    mavlink_msg_component_information_basic_get_serial_number(msg, component_information_basic->serial_number);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN? msg->len : MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN;
        memset(component_information_basic, 0, MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN);
    memcpy(component_information_basic, _MAV_PAYLOAD(msg), len);
#endif
}
