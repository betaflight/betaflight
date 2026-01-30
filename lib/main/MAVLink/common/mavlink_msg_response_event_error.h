#pragma once
// MESSAGE RESPONSE_EVENT_ERROR PACKING

#define MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR 413


typedef struct __mavlink_response_event_error_t {
 uint16_t sequence; /*<  Sequence number.*/
 uint16_t sequence_oldest_available; /*<  Oldest Sequence number that is still available after the sequence set in REQUEST_EVENT.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t reason; /*<  Error reason.*/
} mavlink_response_event_error_t;

#define MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN 7
#define MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN 7
#define MAVLINK_MSG_ID_413_LEN 7
#define MAVLINK_MSG_ID_413_MIN_LEN 7

#define MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC 77
#define MAVLINK_MSG_ID_413_CRC 77



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RESPONSE_EVENT_ERROR { \
    413, \
    "RESPONSE_EVENT_ERROR", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_response_event_error_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_response_event_error_t, target_component) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_response_event_error_t, sequence) }, \
         { "sequence_oldest_available", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_response_event_error_t, sequence_oldest_available) }, \
         { "reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_response_event_error_t, reason) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RESPONSE_EVENT_ERROR { \
    "RESPONSE_EVENT_ERROR", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_response_event_error_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_response_event_error_t, target_component) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_response_event_error_t, sequence) }, \
         { "sequence_oldest_available", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_response_event_error_t, sequence_oldest_available) }, \
         { "reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_response_event_error_t, reason) }, \
         } \
}
#endif

/**
 * @brief Pack a response_event_error message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param sequence  Sequence number.
 * @param sequence_oldest_available  Oldest Sequence number that is still available after the sequence set in REQUEST_EVENT.
 * @param reason  Error reason.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_response_event_error_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t sequence, uint16_t sequence_oldest_available, uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint16_t(buf, 2, sequence_oldest_available);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, reason);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN);
#else
    mavlink_response_event_error_t packet;
    packet.sequence = sequence;
    packet.sequence_oldest_available = sequence_oldest_available;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.reason = reason;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC);
}

/**
 * @brief Pack a response_event_error message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param sequence  Sequence number.
 * @param sequence_oldest_available  Oldest Sequence number that is still available after the sequence set in REQUEST_EVENT.
 * @param reason  Error reason.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_response_event_error_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t sequence, uint16_t sequence_oldest_available, uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint16_t(buf, 2, sequence_oldest_available);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, reason);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN);
#else
    mavlink_response_event_error_t packet;
    packet.sequence = sequence;
    packet.sequence_oldest_available = sequence_oldest_available;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.reason = reason;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN);
#endif
}

/**
 * @brief Pack a response_event_error message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param sequence  Sequence number.
 * @param sequence_oldest_available  Oldest Sequence number that is still available after the sequence set in REQUEST_EVENT.
 * @param reason  Error reason.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_response_event_error_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t sequence,uint16_t sequence_oldest_available,uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint16_t(buf, 2, sequence_oldest_available);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, reason);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN);
#else
    mavlink_response_event_error_t packet;
    packet.sequence = sequence;
    packet.sequence_oldest_available = sequence_oldest_available;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.reason = reason;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC);
}

/**
 * @brief Encode a response_event_error struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param response_event_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_response_event_error_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_response_event_error_t* response_event_error)
{
    return mavlink_msg_response_event_error_pack(system_id, component_id, msg, response_event_error->target_system, response_event_error->target_component, response_event_error->sequence, response_event_error->sequence_oldest_available, response_event_error->reason);
}

/**
 * @brief Encode a response_event_error struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param response_event_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_response_event_error_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_response_event_error_t* response_event_error)
{
    return mavlink_msg_response_event_error_pack_chan(system_id, component_id, chan, msg, response_event_error->target_system, response_event_error->target_component, response_event_error->sequence, response_event_error->sequence_oldest_available, response_event_error->reason);
}

/**
 * @brief Encode a response_event_error struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param response_event_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_response_event_error_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_response_event_error_t* response_event_error)
{
    return mavlink_msg_response_event_error_pack_status(system_id, component_id, _status, msg,  response_event_error->target_system, response_event_error->target_component, response_event_error->sequence, response_event_error->sequence_oldest_available, response_event_error->reason);
}

/**
 * @brief Send a response_event_error message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param sequence  Sequence number.
 * @param sequence_oldest_available  Oldest Sequence number that is still available after the sequence set in REQUEST_EVENT.
 * @param reason  Error reason.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_response_event_error_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t sequence, uint16_t sequence_oldest_available, uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint16_t(buf, 2, sequence_oldest_available);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, reason);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR, buf, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC);
#else
    mavlink_response_event_error_t packet;
    packet.sequence = sequence;
    packet.sequence_oldest_available = sequence_oldest_available;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.reason = reason;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR, (const char *)&packet, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC);
#endif
}

/**
 * @brief Send a response_event_error message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_response_event_error_send_struct(mavlink_channel_t chan, const mavlink_response_event_error_t* response_event_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_response_event_error_send(chan, response_event_error->target_system, response_event_error->target_component, response_event_error->sequence, response_event_error->sequence_oldest_available, response_event_error->reason);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR, (const char *)response_event_error, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC);
#endif
}

#if MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_response_event_error_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t sequence, uint16_t sequence_oldest_available, uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint16_t(buf, 2, sequence_oldest_available);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, reason);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR, buf, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC);
#else
    mavlink_response_event_error_t *packet = (mavlink_response_event_error_t *)msgbuf;
    packet->sequence = sequence;
    packet->sequence_oldest_available = sequence_oldest_available;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->reason = reason;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR, (const char *)packet, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC);
#endif
}
#endif

#endif

// MESSAGE RESPONSE_EVENT_ERROR UNPACKING


/**
 * @brief Get field target_system from response_event_error message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_response_event_error_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from response_event_error message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_response_event_error_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field sequence from response_event_error message
 *
 * @return  Sequence number.
 */
static inline uint16_t mavlink_msg_response_event_error_get_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field sequence_oldest_available from response_event_error message
 *
 * @return  Oldest Sequence number that is still available after the sequence set in REQUEST_EVENT.
 */
static inline uint16_t mavlink_msg_response_event_error_get_sequence_oldest_available(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field reason from response_event_error message
 *
 * @return  Error reason.
 */
static inline uint8_t mavlink_msg_response_event_error_get_reason(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a response_event_error message into a struct
 *
 * @param msg The message to decode
 * @param response_event_error C-struct to decode the message contents into
 */
static inline void mavlink_msg_response_event_error_decode(const mavlink_message_t* msg, mavlink_response_event_error_t* response_event_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    response_event_error->sequence = mavlink_msg_response_event_error_get_sequence(msg);
    response_event_error->sequence_oldest_available = mavlink_msg_response_event_error_get_sequence_oldest_available(msg);
    response_event_error->target_system = mavlink_msg_response_event_error_get_target_system(msg);
    response_event_error->target_component = mavlink_msg_response_event_error_get_target_component(msg);
    response_event_error->reason = mavlink_msg_response_event_error_get_reason(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN? msg->len : MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN;
        memset(response_event_error, 0, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN);
    memcpy(response_event_error, _MAV_PAYLOAD(msg), len);
#endif
}
