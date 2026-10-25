#pragma once
// MESSAGE PARAM_ERROR PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_PARAM_ERROR 345


typedef struct __mavlink_param_error_t {
 int16_t param_index; /*<  Parameter index. Will be -1 if the param ID field should be used as an identifier (else the param id will be ignored)*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 char param_id[16]; /*<  Parameter id. Terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string. Encoded as 7-bit ASCII.*/
 uint8_t error; /*<  Error being returned to client.*/
} mavlink_param_error_t;

#define MAVLINK_MSG_ID_PARAM_ERROR_LEN 21
#define MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN 21
#define MAVLINK_MSG_ID_345_LEN 21
#define MAVLINK_MSG_ID_345_MIN_LEN 21

#define MAVLINK_MSG_ID_PARAM_ERROR_CRC 209
#define MAVLINK_MSG_ID_345_CRC 209

#define MAVLINK_MSG_PARAM_ERROR_FIELD_PARAM_ID_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PARAM_ERROR { \
    345, \
    "PARAM_ERROR", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_param_error_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_param_error_t, target_component) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_param_error_t, param_id) }, \
         { "param_index", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_param_error_t, param_index) }, \
         { "error", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_param_error_t, error) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PARAM_ERROR { \
    "PARAM_ERROR", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_param_error_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_param_error_t, target_component) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_param_error_t, param_id) }, \
         { "param_index", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_param_error_t, param_index) }, \
         { "error", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_param_error_t, error) }, \
         } \
}
#endif

/**
 * @brief Pack a param_error message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param param_id  Parameter id. Terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string. Encoded as 7-bit ASCII.
 * @param param_index  Parameter index. Will be -1 if the param ID field should be used as an identifier (else the param id will be ignored)
 * @param error  Error being returned to client.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_error_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const char *param_id, int16_t param_index, uint8_t error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_ERROR_LEN];
    _mav_put_int16_t(buf, 0, param_index);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 20, error);
    _mav_put_char_array(buf, 4, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_ERROR_LEN);
#else
    mavlink_param_error_t packet;
    packet.param_index = param_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.error = error;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_ERROR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_PARAM_ERROR_LEN, MAVLINK_MSG_ID_PARAM_ERROR_CRC);
}

/**
 * @brief Pack a param_error message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param param_id  Parameter id. Terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string. Encoded as 7-bit ASCII.
 * @param param_index  Parameter index. Will be -1 if the param ID field should be used as an identifier (else the param id will be ignored)
 * @param error  Error being returned to client.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_error_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const char *param_id, int16_t param_index, uint8_t error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_ERROR_LEN];
    _mav_put_int16_t(buf, 0, param_index);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 20, error);
    _mav_put_char_array(buf, 4, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_ERROR_LEN);
#else
    mavlink_param_error_t packet;
    packet.param_index = param_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.error = error;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_ERROR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_PARAM_ERROR_LEN, MAVLINK_MSG_ID_PARAM_ERROR_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_PARAM_ERROR_LEN);
#endif
}

/**
 * @brief Pack a param_error message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param param_id  Parameter id. Terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string. Encoded as 7-bit ASCII.
 * @param param_index  Parameter index. Will be -1 if the param ID field should be used as an identifier (else the param id will be ignored)
 * @param error  Error being returned to client.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_error_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,const char *param_id,int16_t param_index,uint8_t error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_ERROR_LEN];
    _mav_put_int16_t(buf, 0, param_index);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 20, error);
    _mav_put_char_array(buf, 4, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_ERROR_LEN);
#else
    mavlink_param_error_t packet;
    packet.param_index = param_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.error = error;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_ERROR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_PARAM_ERROR_LEN, MAVLINK_MSG_ID_PARAM_ERROR_CRC);
}

/**
 * @brief Encode a param_error struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_error_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_error_t* param_error)
{
    return mavlink_msg_param_error_pack(system_id, component_id, msg, param_error->target_system, param_error->target_component, param_error->param_id, param_error->param_index, param_error->error);
}

/**
 * @brief Encode a param_error struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_error_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_param_error_t* param_error)
{
    return mavlink_msg_param_error_pack_chan(system_id, component_id, chan, msg, param_error->target_system, param_error->target_component, param_error->param_id, param_error->param_index, param_error->error);
}

/**
 * @brief Encode a param_error struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param param_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_error_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_param_error_t* param_error)
{
    return mavlink_msg_param_error_pack_status(system_id, component_id, _status, msg,  param_error->target_system, param_error->target_component, param_error->param_id, param_error->param_index, param_error->error);
}

/**
 * @brief Send a param_error message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param param_id  Parameter id. Terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string. Encoded as 7-bit ASCII.
 * @param param_index  Parameter index. Will be -1 if the param ID field should be used as an identifier (else the param id will be ignored)
 * @param error  Error being returned to client.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_error_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, const char *param_id, int16_t param_index, uint8_t error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_ERROR_LEN];
    _mav_put_int16_t(buf, 0, param_index);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 20, error);
    _mav_put_char_array(buf, 4, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_ERROR, buf, MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_PARAM_ERROR_LEN, MAVLINK_MSG_ID_PARAM_ERROR_CRC);
#else
    mavlink_param_error_t packet;
    packet.param_index = param_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.error = error;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_ERROR, (const char *)&packet, MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_PARAM_ERROR_LEN, MAVLINK_MSG_ID_PARAM_ERROR_CRC);
#endif
}

/**
 * @brief Send a param_error message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_param_error_send_struct(mavlink_channel_t chan, const mavlink_param_error_t* param_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_param_error_send(chan, param_error->target_system, param_error->target_component, param_error->param_id, param_error->param_index, param_error->error);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_ERROR, (const char *)param_error, MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_PARAM_ERROR_LEN, MAVLINK_MSG_ID_PARAM_ERROR_CRC);
#endif
}

#if MAVLINK_MSG_ID_PARAM_ERROR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_param_error_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, const char *param_id, int16_t param_index, uint8_t error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, param_index);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 20, error);
    _mav_put_char_array(buf, 4, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_ERROR, buf, MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_PARAM_ERROR_LEN, MAVLINK_MSG_ID_PARAM_ERROR_CRC);
#else
    mavlink_param_error_t *packet = (mavlink_param_error_t *)msgbuf;
    packet->param_index = param_index;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->error = error;
    mav_array_memcpy(packet->param_id, param_id, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_ERROR, (const char *)packet, MAVLINK_MSG_ID_PARAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_PARAM_ERROR_LEN, MAVLINK_MSG_ID_PARAM_ERROR_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAM_ERROR UNPACKING


/**
 * @brief Get field target_system from param_error message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_param_error_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_component from param_error message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_param_error_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field param_id from param_error message
 *
 * @return  Parameter id. Terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string. Encoded as 7-bit ASCII.
 */
static inline uint16_t mavlink_msg_param_error_get_param_id(const mavlink_message_t* msg, char *param_id)
{
    return _MAV_RETURN_char_array(msg, param_id, 16,  4);
}

/**
 * @brief Get field param_index from param_error message
 *
 * @return  Parameter index. Will be -1 if the param ID field should be used as an identifier (else the param id will be ignored)
 */
static inline int16_t mavlink_msg_param_error_get_param_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field error from param_error message
 *
 * @return  Error being returned to client.
 */
static inline uint8_t mavlink_msg_param_error_get_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a param_error message into a struct
 *
 * @param msg The message to decode
 * @param param_error C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_error_decode(const mavlink_message_t* msg, mavlink_param_error_t* param_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    param_error->param_index = mavlink_msg_param_error_get_param_index(msg);
    param_error->target_system = mavlink_msg_param_error_get_target_system(msg);
    param_error->target_component = mavlink_msg_param_error_get_target_component(msg);
    mavlink_msg_param_error_get_param_id(msg, param_error->param_id);
    param_error->error = mavlink_msg_param_error_get_error(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PARAM_ERROR_LEN? msg->len : MAVLINK_MSG_ID_PARAM_ERROR_LEN;
        memset(param_error, 0, MAVLINK_MSG_ID_PARAM_ERROR_LEN);
    memcpy(param_error, _MAV_PAYLOAD(msg), len);
#endif
}
