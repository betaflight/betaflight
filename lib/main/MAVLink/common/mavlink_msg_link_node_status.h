#pragma once
// MESSAGE LINK_NODE_STATUS PACKING

#define MAVLINK_MSG_ID_LINK_NODE_STATUS 8


typedef struct __mavlink_link_node_status_t {
 uint64_t timestamp; /*< [ms] Timestamp (time since system boot).*/
 uint32_t tx_rate; /*< [bytes/s] Transmit rate*/
 uint32_t rx_rate; /*< [bytes/s] Receive rate*/
 uint32_t messages_sent; /*<  Messages sent*/
 uint32_t messages_received; /*<  Messages received (estimated from counting seq)*/
 uint32_t messages_lost; /*<  Messages lost (estimated from counting seq)*/
 uint16_t rx_parse_err; /*< [bytes] Number of bytes that could not be parsed correctly.*/
 uint16_t tx_overflows; /*< [bytes] Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX*/
 uint16_t rx_overflows; /*< [bytes] Receive buffer overflows. This number wraps around as it reaches UINT16_MAX*/
 uint8_t tx_buf; /*< [%] Remaining free transmit buffer space*/
 uint8_t rx_buf; /*< [%] Remaining free receive buffer space*/
} mavlink_link_node_status_t;

#define MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN 36
#define MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN 36
#define MAVLINK_MSG_ID_8_LEN 36
#define MAVLINK_MSG_ID_8_MIN_LEN 36

#define MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC 117
#define MAVLINK_MSG_ID_8_CRC 117



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LINK_NODE_STATUS { \
    8, \
    "LINK_NODE_STATUS", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_link_node_status_t, timestamp) }, \
         { "tx_buf", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_link_node_status_t, tx_buf) }, \
         { "rx_buf", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_link_node_status_t, rx_buf) }, \
         { "tx_rate", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_link_node_status_t, tx_rate) }, \
         { "rx_rate", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_link_node_status_t, rx_rate) }, \
         { "rx_parse_err", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_link_node_status_t, rx_parse_err) }, \
         { "tx_overflows", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_link_node_status_t, tx_overflows) }, \
         { "rx_overflows", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_link_node_status_t, rx_overflows) }, \
         { "messages_sent", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_link_node_status_t, messages_sent) }, \
         { "messages_received", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_link_node_status_t, messages_received) }, \
         { "messages_lost", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_link_node_status_t, messages_lost) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LINK_NODE_STATUS { \
    "LINK_NODE_STATUS", \
    11, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_link_node_status_t, timestamp) }, \
         { "tx_buf", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_link_node_status_t, tx_buf) }, \
         { "rx_buf", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_link_node_status_t, rx_buf) }, \
         { "tx_rate", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_link_node_status_t, tx_rate) }, \
         { "rx_rate", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_link_node_status_t, rx_rate) }, \
         { "rx_parse_err", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_link_node_status_t, rx_parse_err) }, \
         { "tx_overflows", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_link_node_status_t, tx_overflows) }, \
         { "rx_overflows", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_link_node_status_t, rx_overflows) }, \
         { "messages_sent", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_link_node_status_t, messages_sent) }, \
         { "messages_received", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_link_node_status_t, messages_received) }, \
         { "messages_lost", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_link_node_status_t, messages_lost) }, \
         } \
}
#endif

/**
 * @brief Pack a link_node_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp (time since system boot).
 * @param tx_buf [%] Remaining free transmit buffer space
 * @param rx_buf [%] Remaining free receive buffer space
 * @param tx_rate [bytes/s] Transmit rate
 * @param rx_rate [bytes/s] Receive rate
 * @param rx_parse_err [bytes] Number of bytes that could not be parsed correctly.
 * @param tx_overflows [bytes] Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX
 * @param rx_overflows [bytes] Receive buffer overflows. This number wraps around as it reaches UINT16_MAX
 * @param messages_sent  Messages sent
 * @param messages_received  Messages received (estimated from counting seq)
 * @param messages_lost  Messages lost (estimated from counting seq)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_link_node_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t tx_buf, uint8_t rx_buf, uint32_t tx_rate, uint32_t rx_rate, uint16_t rx_parse_err, uint16_t tx_overflows, uint16_t rx_overflows, uint32_t messages_sent, uint32_t messages_received, uint32_t messages_lost)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, tx_rate);
    _mav_put_uint32_t(buf, 12, rx_rate);
    _mav_put_uint32_t(buf, 16, messages_sent);
    _mav_put_uint32_t(buf, 20, messages_received);
    _mav_put_uint32_t(buf, 24, messages_lost);
    _mav_put_uint16_t(buf, 28, rx_parse_err);
    _mav_put_uint16_t(buf, 30, tx_overflows);
    _mav_put_uint16_t(buf, 32, rx_overflows);
    _mav_put_uint8_t(buf, 34, tx_buf);
    _mav_put_uint8_t(buf, 35, rx_buf);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN);
#else
    mavlink_link_node_status_t packet;
    packet.timestamp = timestamp;
    packet.tx_rate = tx_rate;
    packet.rx_rate = rx_rate;
    packet.messages_sent = messages_sent;
    packet.messages_received = messages_received;
    packet.messages_lost = messages_lost;
    packet.rx_parse_err = rx_parse_err;
    packet.tx_overflows = tx_overflows;
    packet.rx_overflows = rx_overflows;
    packet.tx_buf = tx_buf;
    packet.rx_buf = rx_buf;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LINK_NODE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC);
}

/**
 * @brief Pack a link_node_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp (time since system boot).
 * @param tx_buf [%] Remaining free transmit buffer space
 * @param rx_buf [%] Remaining free receive buffer space
 * @param tx_rate [bytes/s] Transmit rate
 * @param rx_rate [bytes/s] Receive rate
 * @param rx_parse_err [bytes] Number of bytes that could not be parsed correctly.
 * @param tx_overflows [bytes] Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX
 * @param rx_overflows [bytes] Receive buffer overflows. This number wraps around as it reaches UINT16_MAX
 * @param messages_sent  Messages sent
 * @param messages_received  Messages received (estimated from counting seq)
 * @param messages_lost  Messages lost (estimated from counting seq)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_link_node_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t tx_buf, uint8_t rx_buf, uint32_t tx_rate, uint32_t rx_rate, uint16_t rx_parse_err, uint16_t tx_overflows, uint16_t rx_overflows, uint32_t messages_sent, uint32_t messages_received, uint32_t messages_lost)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, tx_rate);
    _mav_put_uint32_t(buf, 12, rx_rate);
    _mav_put_uint32_t(buf, 16, messages_sent);
    _mav_put_uint32_t(buf, 20, messages_received);
    _mav_put_uint32_t(buf, 24, messages_lost);
    _mav_put_uint16_t(buf, 28, rx_parse_err);
    _mav_put_uint16_t(buf, 30, tx_overflows);
    _mav_put_uint16_t(buf, 32, rx_overflows);
    _mav_put_uint8_t(buf, 34, tx_buf);
    _mav_put_uint8_t(buf, 35, rx_buf);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN);
#else
    mavlink_link_node_status_t packet;
    packet.timestamp = timestamp;
    packet.tx_rate = tx_rate;
    packet.rx_rate = rx_rate;
    packet.messages_sent = messages_sent;
    packet.messages_received = messages_received;
    packet.messages_lost = messages_lost;
    packet.rx_parse_err = rx_parse_err;
    packet.tx_overflows = tx_overflows;
    packet.rx_overflows = rx_overflows;
    packet.tx_buf = tx_buf;
    packet.rx_buf = rx_buf;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LINK_NODE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a link_node_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] Timestamp (time since system boot).
 * @param tx_buf [%] Remaining free transmit buffer space
 * @param rx_buf [%] Remaining free receive buffer space
 * @param tx_rate [bytes/s] Transmit rate
 * @param rx_rate [bytes/s] Receive rate
 * @param rx_parse_err [bytes] Number of bytes that could not be parsed correctly.
 * @param tx_overflows [bytes] Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX
 * @param rx_overflows [bytes] Receive buffer overflows. This number wraps around as it reaches UINT16_MAX
 * @param messages_sent  Messages sent
 * @param messages_received  Messages received (estimated from counting seq)
 * @param messages_lost  Messages lost (estimated from counting seq)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_link_node_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t tx_buf,uint8_t rx_buf,uint32_t tx_rate,uint32_t rx_rate,uint16_t rx_parse_err,uint16_t tx_overflows,uint16_t rx_overflows,uint32_t messages_sent,uint32_t messages_received,uint32_t messages_lost)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, tx_rate);
    _mav_put_uint32_t(buf, 12, rx_rate);
    _mav_put_uint32_t(buf, 16, messages_sent);
    _mav_put_uint32_t(buf, 20, messages_received);
    _mav_put_uint32_t(buf, 24, messages_lost);
    _mav_put_uint16_t(buf, 28, rx_parse_err);
    _mav_put_uint16_t(buf, 30, tx_overflows);
    _mav_put_uint16_t(buf, 32, rx_overflows);
    _mav_put_uint8_t(buf, 34, tx_buf);
    _mav_put_uint8_t(buf, 35, rx_buf);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN);
#else
    mavlink_link_node_status_t packet;
    packet.timestamp = timestamp;
    packet.tx_rate = tx_rate;
    packet.rx_rate = rx_rate;
    packet.messages_sent = messages_sent;
    packet.messages_received = messages_received;
    packet.messages_lost = messages_lost;
    packet.rx_parse_err = rx_parse_err;
    packet.tx_overflows = tx_overflows;
    packet.rx_overflows = rx_overflows;
    packet.tx_buf = tx_buf;
    packet.rx_buf = rx_buf;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LINK_NODE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC);
}

/**
 * @brief Encode a link_node_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param link_node_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_link_node_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_link_node_status_t* link_node_status)
{
    return mavlink_msg_link_node_status_pack(system_id, component_id, msg, link_node_status->timestamp, link_node_status->tx_buf, link_node_status->rx_buf, link_node_status->tx_rate, link_node_status->rx_rate, link_node_status->rx_parse_err, link_node_status->tx_overflows, link_node_status->rx_overflows, link_node_status->messages_sent, link_node_status->messages_received, link_node_status->messages_lost);
}

/**
 * @brief Encode a link_node_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param link_node_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_link_node_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_link_node_status_t* link_node_status)
{
    return mavlink_msg_link_node_status_pack_chan(system_id, component_id, chan, msg, link_node_status->timestamp, link_node_status->tx_buf, link_node_status->rx_buf, link_node_status->tx_rate, link_node_status->rx_rate, link_node_status->rx_parse_err, link_node_status->tx_overflows, link_node_status->rx_overflows, link_node_status->messages_sent, link_node_status->messages_received, link_node_status->messages_lost);
}

/**
 * @brief Encode a link_node_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param link_node_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_link_node_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_link_node_status_t* link_node_status)
{
    return mavlink_msg_link_node_status_pack_status(system_id, component_id, _status, msg,  link_node_status->timestamp, link_node_status->tx_buf, link_node_status->rx_buf, link_node_status->tx_rate, link_node_status->rx_rate, link_node_status->rx_parse_err, link_node_status->tx_overflows, link_node_status->rx_overflows, link_node_status->messages_sent, link_node_status->messages_received, link_node_status->messages_lost);
}

/**
 * @brief Send a link_node_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] Timestamp (time since system boot).
 * @param tx_buf [%] Remaining free transmit buffer space
 * @param rx_buf [%] Remaining free receive buffer space
 * @param tx_rate [bytes/s] Transmit rate
 * @param rx_rate [bytes/s] Receive rate
 * @param rx_parse_err [bytes] Number of bytes that could not be parsed correctly.
 * @param tx_overflows [bytes] Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX
 * @param rx_overflows [bytes] Receive buffer overflows. This number wraps around as it reaches UINT16_MAX
 * @param messages_sent  Messages sent
 * @param messages_received  Messages received (estimated from counting seq)
 * @param messages_lost  Messages lost (estimated from counting seq)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_link_node_status_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t tx_buf, uint8_t rx_buf, uint32_t tx_rate, uint32_t rx_rate, uint16_t rx_parse_err, uint16_t tx_overflows, uint16_t rx_overflows, uint32_t messages_sent, uint32_t messages_received, uint32_t messages_lost)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, tx_rate);
    _mav_put_uint32_t(buf, 12, rx_rate);
    _mav_put_uint32_t(buf, 16, messages_sent);
    _mav_put_uint32_t(buf, 20, messages_received);
    _mav_put_uint32_t(buf, 24, messages_lost);
    _mav_put_uint16_t(buf, 28, rx_parse_err);
    _mav_put_uint16_t(buf, 30, tx_overflows);
    _mav_put_uint16_t(buf, 32, rx_overflows);
    _mav_put_uint8_t(buf, 34, tx_buf);
    _mav_put_uint8_t(buf, 35, rx_buf);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LINK_NODE_STATUS, buf, MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC);
#else
    mavlink_link_node_status_t packet;
    packet.timestamp = timestamp;
    packet.tx_rate = tx_rate;
    packet.rx_rate = rx_rate;
    packet.messages_sent = messages_sent;
    packet.messages_received = messages_received;
    packet.messages_lost = messages_lost;
    packet.rx_parse_err = rx_parse_err;
    packet.tx_overflows = tx_overflows;
    packet.rx_overflows = rx_overflows;
    packet.tx_buf = tx_buf;
    packet.rx_buf = rx_buf;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LINK_NODE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC);
#endif
}

/**
 * @brief Send a link_node_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_link_node_status_send_struct(mavlink_channel_t chan, const mavlink_link_node_status_t* link_node_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_link_node_status_send(chan, link_node_status->timestamp, link_node_status->tx_buf, link_node_status->rx_buf, link_node_status->tx_rate, link_node_status->rx_rate, link_node_status->rx_parse_err, link_node_status->tx_overflows, link_node_status->rx_overflows, link_node_status->messages_sent, link_node_status->messages_received, link_node_status->messages_lost);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LINK_NODE_STATUS, (const char *)link_node_status, MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_link_node_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t tx_buf, uint8_t rx_buf, uint32_t tx_rate, uint32_t rx_rate, uint16_t rx_parse_err, uint16_t tx_overflows, uint16_t rx_overflows, uint32_t messages_sent, uint32_t messages_received, uint32_t messages_lost)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, tx_rate);
    _mav_put_uint32_t(buf, 12, rx_rate);
    _mav_put_uint32_t(buf, 16, messages_sent);
    _mav_put_uint32_t(buf, 20, messages_received);
    _mav_put_uint32_t(buf, 24, messages_lost);
    _mav_put_uint16_t(buf, 28, rx_parse_err);
    _mav_put_uint16_t(buf, 30, tx_overflows);
    _mav_put_uint16_t(buf, 32, rx_overflows);
    _mav_put_uint8_t(buf, 34, tx_buf);
    _mav_put_uint8_t(buf, 35, rx_buf);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LINK_NODE_STATUS, buf, MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC);
#else
    mavlink_link_node_status_t *packet = (mavlink_link_node_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->tx_rate = tx_rate;
    packet->rx_rate = rx_rate;
    packet->messages_sent = messages_sent;
    packet->messages_received = messages_received;
    packet->messages_lost = messages_lost;
    packet->rx_parse_err = rx_parse_err;
    packet->tx_overflows = tx_overflows;
    packet->rx_overflows = rx_overflows;
    packet->tx_buf = tx_buf;
    packet->rx_buf = rx_buf;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LINK_NODE_STATUS, (const char *)packet, MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN, MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE LINK_NODE_STATUS UNPACKING


/**
 * @brief Get field timestamp from link_node_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint64_t mavlink_msg_link_node_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field tx_buf from link_node_status message
 *
 * @return [%] Remaining free transmit buffer space
 */
static inline uint8_t mavlink_msg_link_node_status_get_tx_buf(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field rx_buf from link_node_status message
 *
 * @return [%] Remaining free receive buffer space
 */
static inline uint8_t mavlink_msg_link_node_status_get_rx_buf(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field tx_rate from link_node_status message
 *
 * @return [bytes/s] Transmit rate
 */
static inline uint32_t mavlink_msg_link_node_status_get_tx_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field rx_rate from link_node_status message
 *
 * @return [bytes/s] Receive rate
 */
static inline uint32_t mavlink_msg_link_node_status_get_rx_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field rx_parse_err from link_node_status message
 *
 * @return [bytes] Number of bytes that could not be parsed correctly.
 */
static inline uint16_t mavlink_msg_link_node_status_get_rx_parse_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field tx_overflows from link_node_status message
 *
 * @return [bytes] Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX
 */
static inline uint16_t mavlink_msg_link_node_status_get_tx_overflows(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field rx_overflows from link_node_status message
 *
 * @return [bytes] Receive buffer overflows. This number wraps around as it reaches UINT16_MAX
 */
static inline uint16_t mavlink_msg_link_node_status_get_rx_overflows(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field messages_sent from link_node_status message
 *
 * @return  Messages sent
 */
static inline uint32_t mavlink_msg_link_node_status_get_messages_sent(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field messages_received from link_node_status message
 *
 * @return  Messages received (estimated from counting seq)
 */
static inline uint32_t mavlink_msg_link_node_status_get_messages_received(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field messages_lost from link_node_status message
 *
 * @return  Messages lost (estimated from counting seq)
 */
static inline uint32_t mavlink_msg_link_node_status_get_messages_lost(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Decode a link_node_status message into a struct
 *
 * @param msg The message to decode
 * @param link_node_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_link_node_status_decode(const mavlink_message_t* msg, mavlink_link_node_status_t* link_node_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    link_node_status->timestamp = mavlink_msg_link_node_status_get_timestamp(msg);
    link_node_status->tx_rate = mavlink_msg_link_node_status_get_tx_rate(msg);
    link_node_status->rx_rate = mavlink_msg_link_node_status_get_rx_rate(msg);
    link_node_status->messages_sent = mavlink_msg_link_node_status_get_messages_sent(msg);
    link_node_status->messages_received = mavlink_msg_link_node_status_get_messages_received(msg);
    link_node_status->messages_lost = mavlink_msg_link_node_status_get_messages_lost(msg);
    link_node_status->rx_parse_err = mavlink_msg_link_node_status_get_rx_parse_err(msg);
    link_node_status->tx_overflows = mavlink_msg_link_node_status_get_tx_overflows(msg);
    link_node_status->rx_overflows = mavlink_msg_link_node_status_get_rx_overflows(msg);
    link_node_status->tx_buf = mavlink_msg_link_node_status_get_tx_buf(msg);
    link_node_status->rx_buf = mavlink_msg_link_node_status_get_rx_buf(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN;
        memset(link_node_status, 0, MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN);
    memcpy(link_node_status, _MAV_PAYLOAD(msg), len);
#endif
}
