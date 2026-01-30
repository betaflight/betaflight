#pragma once
// MESSAGE REQUEST_EVENT PACKING

#define MAVLINK_MSG_ID_REQUEST_EVENT 412


typedef struct __mavlink_request_event_t {
 uint16_t first_sequence; /*<  First sequence number of the requested event.*/
 uint16_t last_sequence; /*<  Last sequence number of the requested event.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
} mavlink_request_event_t;

#define MAVLINK_MSG_ID_REQUEST_EVENT_LEN 6
#define MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN 6
#define MAVLINK_MSG_ID_412_LEN 6
#define MAVLINK_MSG_ID_412_MIN_LEN 6

#define MAVLINK_MSG_ID_REQUEST_EVENT_CRC 33
#define MAVLINK_MSG_ID_412_CRC 33



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_REQUEST_EVENT { \
    412, \
    "REQUEST_EVENT", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_request_event_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_request_event_t, target_component) }, \
         { "first_sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_request_event_t, first_sequence) }, \
         { "last_sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_request_event_t, last_sequence) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_REQUEST_EVENT { \
    "REQUEST_EVENT", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_request_event_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_request_event_t, target_component) }, \
         { "first_sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_request_event_t, first_sequence) }, \
         { "last_sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_request_event_t, last_sequence) }, \
         } \
}
#endif

/**
 * @brief Pack a request_event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param first_sequence  First sequence number of the requested event.
 * @param last_sequence  Last sequence number of the requested event.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_event_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t first_sequence, uint16_t last_sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REQUEST_EVENT_LEN];
    _mav_put_uint16_t(buf, 0, first_sequence);
    _mav_put_uint16_t(buf, 2, last_sequence);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_EVENT_LEN);
#else
    mavlink_request_event_t packet;
    packet.first_sequence = first_sequence;
    packet.last_sequence = last_sequence;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_EVENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REQUEST_EVENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_CRC);
}

/**
 * @brief Pack a request_event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param first_sequence  First sequence number of the requested event.
 * @param last_sequence  Last sequence number of the requested event.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_event_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t first_sequence, uint16_t last_sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REQUEST_EVENT_LEN];
    _mav_put_uint16_t(buf, 0, first_sequence);
    _mav_put_uint16_t(buf, 2, last_sequence);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_EVENT_LEN);
#else
    mavlink_request_event_t packet;
    packet.first_sequence = first_sequence;
    packet.last_sequence = last_sequence;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_EVENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REQUEST_EVENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_LEN);
#endif
}

/**
 * @brief Pack a request_event message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param first_sequence  First sequence number of the requested event.
 * @param last_sequence  Last sequence number of the requested event.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_event_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t first_sequence,uint16_t last_sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REQUEST_EVENT_LEN];
    _mav_put_uint16_t(buf, 0, first_sequence);
    _mav_put_uint16_t(buf, 2, last_sequence);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_EVENT_LEN);
#else
    mavlink_request_event_t packet;
    packet.first_sequence = first_sequence;
    packet.last_sequence = last_sequence;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_EVENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REQUEST_EVENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_CRC);
}

/**
 * @brief Encode a request_event struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param request_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_event_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_event_t* request_event)
{
    return mavlink_msg_request_event_pack(system_id, component_id, msg, request_event->target_system, request_event->target_component, request_event->first_sequence, request_event->last_sequence);
}

/**
 * @brief Encode a request_event struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param request_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_event_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_request_event_t* request_event)
{
    return mavlink_msg_request_event_pack_chan(system_id, component_id, chan, msg, request_event->target_system, request_event->target_component, request_event->first_sequence, request_event->last_sequence);
}

/**
 * @brief Encode a request_event struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param request_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_event_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_request_event_t* request_event)
{
    return mavlink_msg_request_event_pack_status(system_id, component_id, _status, msg,  request_event->target_system, request_event->target_component, request_event->first_sequence, request_event->last_sequence);
}

/**
 * @brief Send a request_event message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param first_sequence  First sequence number of the requested event.
 * @param last_sequence  Last sequence number of the requested event.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_event_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t first_sequence, uint16_t last_sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REQUEST_EVENT_LEN];
    _mav_put_uint16_t(buf, 0, first_sequence);
    _mav_put_uint16_t(buf, 2, last_sequence);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT, buf, MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_CRC);
#else
    mavlink_request_event_t packet;
    packet.first_sequence = first_sequence;
    packet.last_sequence = last_sequence;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT, (const char *)&packet, MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_CRC);
#endif
}

/**
 * @brief Send a request_event message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_request_event_send_struct(mavlink_channel_t chan, const mavlink_request_event_t* request_event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_request_event_send(chan, request_event->target_system, request_event->target_component, request_event->first_sequence, request_event->last_sequence);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT, (const char *)request_event, MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_REQUEST_EVENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_request_event_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t first_sequence, uint16_t last_sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, first_sequence);
    _mav_put_uint16_t(buf, 2, last_sequence);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT, buf, MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_CRC);
#else
    mavlink_request_event_t *packet = (mavlink_request_event_t *)msgbuf;
    packet->first_sequence = first_sequence;
    packet->last_sequence = last_sequence;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT, (const char *)packet, MAVLINK_MSG_ID_REQUEST_EVENT_MIN_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_CRC);
#endif
}
#endif

#endif

// MESSAGE REQUEST_EVENT UNPACKING


/**
 * @brief Get field target_system from request_event message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_request_event_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from request_event message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_request_event_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field first_sequence from request_event message
 *
 * @return  First sequence number of the requested event.
 */
static inline uint16_t mavlink_msg_request_event_get_first_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field last_sequence from request_event message
 *
 * @return  Last sequence number of the requested event.
 */
static inline uint16_t mavlink_msg_request_event_get_last_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a request_event message into a struct
 *
 * @param msg The message to decode
 * @param request_event C-struct to decode the message contents into
 */
static inline void mavlink_msg_request_event_decode(const mavlink_message_t* msg, mavlink_request_event_t* request_event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    request_event->first_sequence = mavlink_msg_request_event_get_first_sequence(msg);
    request_event->last_sequence = mavlink_msg_request_event_get_last_sequence(msg);
    request_event->target_system = mavlink_msg_request_event_get_target_system(msg);
    request_event->target_component = mavlink_msg_request_event_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_REQUEST_EVENT_LEN? msg->len : MAVLINK_MSG_ID_REQUEST_EVENT_LEN;
        memset(request_event, 0, MAVLINK_MSG_ID_REQUEST_EVENT_LEN);
    memcpy(request_event, _MAV_PAYLOAD(msg), len);
#endif
}
