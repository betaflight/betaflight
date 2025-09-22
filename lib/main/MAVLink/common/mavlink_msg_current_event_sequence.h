#pragma once
// MESSAGE CURRENT_EVENT_SEQUENCE PACKING

#define MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE 411


typedef struct __mavlink_current_event_sequence_t {
 uint16_t sequence; /*<  Sequence number.*/
 uint8_t flags; /*<  Flag bitset.*/
} mavlink_current_event_sequence_t;

#define MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN 3
#define MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN 3
#define MAVLINK_MSG_ID_411_LEN 3
#define MAVLINK_MSG_ID_411_MIN_LEN 3

#define MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_CRC 106
#define MAVLINK_MSG_ID_411_CRC 106



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CURRENT_EVENT_SEQUENCE { \
    411, \
    "CURRENT_EVENT_SEQUENCE", \
    2, \
    {  { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_current_event_sequence_t, sequence) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_current_event_sequence_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CURRENT_EVENT_SEQUENCE { \
    "CURRENT_EVENT_SEQUENCE", \
    2, \
    {  { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_current_event_sequence_t, sequence) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_current_event_sequence_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a current_event_sequence message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sequence  Sequence number.
 * @param flags  Flag bitset.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_event_sequence_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t sequence, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint8_t(buf, 2, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN);
#else
    mavlink_current_event_sequence_t packet;
    packet.sequence = sequence;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_CRC);
}

/**
 * @brief Pack a current_event_sequence message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param sequence  Sequence number.
 * @param flags  Flag bitset.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_event_sequence_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t sequence, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint8_t(buf, 2, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN);
#else
    mavlink_current_event_sequence_t packet;
    packet.sequence = sequence;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN);
#endif
}

/**
 * @brief Pack a current_event_sequence message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sequence  Sequence number.
 * @param flags  Flag bitset.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_event_sequence_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t sequence,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint8_t(buf, 2, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN);
#else
    mavlink_current_event_sequence_t packet;
    packet.sequence = sequence;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_CRC);
}

/**
 * @brief Encode a current_event_sequence struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param current_event_sequence C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_event_sequence_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_current_event_sequence_t* current_event_sequence)
{
    return mavlink_msg_current_event_sequence_pack(system_id, component_id, msg, current_event_sequence->sequence, current_event_sequence->flags);
}

/**
 * @brief Encode a current_event_sequence struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param current_event_sequence C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_event_sequence_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_current_event_sequence_t* current_event_sequence)
{
    return mavlink_msg_current_event_sequence_pack_chan(system_id, component_id, chan, msg, current_event_sequence->sequence, current_event_sequence->flags);
}

/**
 * @brief Encode a current_event_sequence struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param current_event_sequence C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_event_sequence_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_current_event_sequence_t* current_event_sequence)
{
    return mavlink_msg_current_event_sequence_pack_status(system_id, component_id, _status, msg,  current_event_sequence->sequence, current_event_sequence->flags);
}

/**
 * @brief Send a current_event_sequence message
 * @param chan MAVLink channel to send the message
 *
 * @param sequence  Sequence number.
 * @param flags  Flag bitset.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_current_event_sequence_send(mavlink_channel_t chan, uint16_t sequence, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint8_t(buf, 2, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE, buf, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_CRC);
#else
    mavlink_current_event_sequence_t packet;
    packet.sequence = sequence;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE, (const char *)&packet, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_CRC);
#endif
}

/**
 * @brief Send a current_event_sequence message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_current_event_sequence_send_struct(mavlink_channel_t chan, const mavlink_current_event_sequence_t* current_event_sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_current_event_sequence_send(chan, current_event_sequence->sequence, current_event_sequence->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE, (const char *)current_event_sequence, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_CRC);
#endif
}

#if MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_current_event_sequence_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t sequence, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint8_t(buf, 2, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE, buf, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_CRC);
#else
    mavlink_current_event_sequence_t *packet = (mavlink_current_event_sequence_t *)msgbuf;
    packet->sequence = sequence;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE, (const char *)packet, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_CRC);
#endif
}
#endif

#endif

// MESSAGE CURRENT_EVENT_SEQUENCE UNPACKING


/**
 * @brief Get field sequence from current_event_sequence message
 *
 * @return  Sequence number.
 */
static inline uint16_t mavlink_msg_current_event_sequence_get_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field flags from current_event_sequence message
 *
 * @return  Flag bitset.
 */
static inline uint8_t mavlink_msg_current_event_sequence_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a current_event_sequence message into a struct
 *
 * @param msg The message to decode
 * @param current_event_sequence C-struct to decode the message contents into
 */
static inline void mavlink_msg_current_event_sequence_decode(const mavlink_message_t* msg, mavlink_current_event_sequence_t* current_event_sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    current_event_sequence->sequence = mavlink_msg_current_event_sequence_get_sequence(msg);
    current_event_sequence->flags = mavlink_msg_current_event_sequence_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN? msg->len : MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN;
        memset(current_event_sequence, 0, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN);
    memcpy(current_event_sequence, _MAV_PAYLOAD(msg), len);
#endif
}
