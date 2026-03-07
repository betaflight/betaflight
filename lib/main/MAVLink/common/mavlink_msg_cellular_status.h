#pragma once
// MESSAGE CELLULAR_STATUS PACKING

#define MAVLINK_MSG_ID_CELLULAR_STATUS 334


typedef struct __mavlink_cellular_status_t {
 uint16_t mcc; /*<  Mobile country code. If unknown, set to UINT16_MAX*/
 uint16_t mnc; /*<  Mobile network code. If unknown, set to UINT16_MAX*/
 uint16_t lac; /*<  Location area code. If unknown, set to 0*/
 uint8_t status; /*<  Cellular modem status*/
 uint8_t failure_reason; /*<  Failure reason when status in in CELLULAR_STATUS_FLAG_FAILED*/
 uint8_t type; /*<  Cellular network radio type: gsm, cdma, lte...*/
 uint8_t quality; /*<  Signal quality in percent. If unknown, set to UINT8_MAX*/
} mavlink_cellular_status_t;

#define MAVLINK_MSG_ID_CELLULAR_STATUS_LEN 10
#define MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN 10
#define MAVLINK_MSG_ID_334_LEN 10
#define MAVLINK_MSG_ID_334_MIN_LEN 10

#define MAVLINK_MSG_ID_CELLULAR_STATUS_CRC 72
#define MAVLINK_MSG_ID_334_CRC 72



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CELLULAR_STATUS { \
    334, \
    "CELLULAR_STATUS", \
    7, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_cellular_status_t, status) }, \
         { "failure_reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_cellular_status_t, failure_reason) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_cellular_status_t, type) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_cellular_status_t, quality) }, \
         { "mcc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_cellular_status_t, mcc) }, \
         { "mnc", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_cellular_status_t, mnc) }, \
         { "lac", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_cellular_status_t, lac) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CELLULAR_STATUS { \
    "CELLULAR_STATUS", \
    7, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_cellular_status_t, status) }, \
         { "failure_reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_cellular_status_t, failure_reason) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_cellular_status_t, type) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_cellular_status_t, quality) }, \
         { "mcc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_cellular_status_t, mcc) }, \
         { "mnc", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_cellular_status_t, mnc) }, \
         { "lac", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_cellular_status_t, lac) }, \
         } \
}
#endif

/**
 * @brief Pack a cellular_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Cellular modem status
 * @param failure_reason  Failure reason when status in in CELLULAR_STATUS_FLAG_FAILED
 * @param type  Cellular network radio type: gsm, cdma, lte...
 * @param quality  Signal quality in percent. If unknown, set to UINT8_MAX
 * @param mcc  Mobile country code. If unknown, set to UINT16_MAX
 * @param mnc  Mobile network code. If unknown, set to UINT16_MAX
 * @param lac  Location area code. If unknown, set to 0
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cellular_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mcc);
    _mav_put_uint16_t(buf, 2, mnc);
    _mav_put_uint16_t(buf, 4, lac);
    _mav_put_uint8_t(buf, 6, status);
    _mav_put_uint8_t(buf, 7, failure_reason);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#else
    mavlink_cellular_status_t packet;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.lac = lac;
    packet.status = status;
    packet.failure_reason = failure_reason;
    packet.type = type;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CELLULAR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
}

/**
 * @brief Pack a cellular_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Cellular modem status
 * @param failure_reason  Failure reason when status in in CELLULAR_STATUS_FLAG_FAILED
 * @param type  Cellular network radio type: gsm, cdma, lte...
 * @param quality  Signal quality in percent. If unknown, set to UINT8_MAX
 * @param mcc  Mobile country code. If unknown, set to UINT16_MAX
 * @param mnc  Mobile network code. If unknown, set to UINT16_MAX
 * @param lac  Location area code. If unknown, set to 0
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cellular_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mcc);
    _mav_put_uint16_t(buf, 2, mnc);
    _mav_put_uint16_t(buf, 4, lac);
    _mav_put_uint8_t(buf, 6, status);
    _mav_put_uint8_t(buf, 7, failure_reason);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#else
    mavlink_cellular_status_t packet;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.lac = lac;
    packet.status = status;
    packet.failure_reason = failure_reason;
    packet.type = type;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CELLULAR_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#endif
}

/**
 * @brief Pack a cellular_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  Cellular modem status
 * @param failure_reason  Failure reason when status in in CELLULAR_STATUS_FLAG_FAILED
 * @param type  Cellular network radio type: gsm, cdma, lte...
 * @param quality  Signal quality in percent. If unknown, set to UINT8_MAX
 * @param mcc  Mobile country code. If unknown, set to UINT16_MAX
 * @param mnc  Mobile network code. If unknown, set to UINT16_MAX
 * @param lac  Location area code. If unknown, set to 0
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cellular_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t status,uint8_t failure_reason,uint8_t type,uint8_t quality,uint16_t mcc,uint16_t mnc,uint16_t lac)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mcc);
    _mav_put_uint16_t(buf, 2, mnc);
    _mav_put_uint16_t(buf, 4, lac);
    _mav_put_uint8_t(buf, 6, status);
    _mav_put_uint8_t(buf, 7, failure_reason);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#else
    mavlink_cellular_status_t packet;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.lac = lac;
    packet.status = status;
    packet.failure_reason = failure_reason;
    packet.type = type;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CELLULAR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
}

/**
 * @brief Encode a cellular_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cellular_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cellular_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cellular_status_t* cellular_status)
{
    return mavlink_msg_cellular_status_pack(system_id, component_id, msg, cellular_status->status, cellular_status->failure_reason, cellular_status->type, cellular_status->quality, cellular_status->mcc, cellular_status->mnc, cellular_status->lac);
}

/**
 * @brief Encode a cellular_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cellular_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cellular_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cellular_status_t* cellular_status)
{
    return mavlink_msg_cellular_status_pack_chan(system_id, component_id, chan, msg, cellular_status->status, cellular_status->failure_reason, cellular_status->type, cellular_status->quality, cellular_status->mcc, cellular_status->mnc, cellular_status->lac);
}

/**
 * @brief Encode a cellular_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param cellular_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cellular_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_cellular_status_t* cellular_status)
{
    return mavlink_msg_cellular_status_pack_status(system_id, component_id, _status, msg,  cellular_status->status, cellular_status->failure_reason, cellular_status->type, cellular_status->quality, cellular_status->mcc, cellular_status->mnc, cellular_status->lac);
}

/**
 * @brief Send a cellular_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status  Cellular modem status
 * @param failure_reason  Failure reason when status in in CELLULAR_STATUS_FLAG_FAILED
 * @param type  Cellular network radio type: gsm, cdma, lte...
 * @param quality  Signal quality in percent. If unknown, set to UINT8_MAX
 * @param mcc  Mobile country code. If unknown, set to UINT16_MAX
 * @param mnc  Mobile network code. If unknown, set to UINT16_MAX
 * @param lac  Location area code. If unknown, set to 0
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cellular_status_send(mavlink_channel_t chan, uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mcc);
    _mav_put_uint16_t(buf, 2, mnc);
    _mav_put_uint16_t(buf, 4, lac);
    _mav_put_uint8_t(buf, 6, status);
    _mav_put_uint8_t(buf, 7, failure_reason);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, buf, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#else
    mavlink_cellular_status_t packet;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.lac = lac;
    packet.status = status;
    packet.failure_reason = failure_reason;
    packet.type = type;
    packet.quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#endif
}

/**
 * @brief Send a cellular_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cellular_status_send_struct(mavlink_channel_t chan, const mavlink_cellular_status_t* cellular_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cellular_status_send(chan, cellular_status->status, cellular_status->failure_reason, cellular_status->type, cellular_status->quality, cellular_status->mcc, cellular_status->mnc, cellular_status->lac);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, (const char *)cellular_status, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CELLULAR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cellular_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, mcc);
    _mav_put_uint16_t(buf, 2, mnc);
    _mav_put_uint16_t(buf, 4, lac);
    _mav_put_uint8_t(buf, 6, status);
    _mav_put_uint8_t(buf, 7, failure_reason);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, buf, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#else
    mavlink_cellular_status_t *packet = (mavlink_cellular_status_t *)msgbuf;
    packet->mcc = mcc;
    packet->mnc = mnc;
    packet->lac = lac;
    packet->status = status;
    packet->failure_reason = failure_reason;
    packet->type = type;
    packet->quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, (const char *)packet, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CELLULAR_STATUS UNPACKING


/**
 * @brief Get field status from cellular_status message
 *
 * @return  Cellular modem status
 */
static inline uint8_t mavlink_msg_cellular_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field failure_reason from cellular_status message
 *
 * @return  Failure reason when status in in CELLULAR_STATUS_FLAG_FAILED
 */
static inline uint8_t mavlink_msg_cellular_status_get_failure_reason(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field type from cellular_status message
 *
 * @return  Cellular network radio type: gsm, cdma, lte...
 */
static inline uint8_t mavlink_msg_cellular_status_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field quality from cellular_status message
 *
 * @return  Signal quality in percent. If unknown, set to UINT8_MAX
 */
static inline uint8_t mavlink_msg_cellular_status_get_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field mcc from cellular_status message
 *
 * @return  Mobile country code. If unknown, set to UINT16_MAX
 */
static inline uint16_t mavlink_msg_cellular_status_get_mcc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field mnc from cellular_status message
 *
 * @return  Mobile network code. If unknown, set to UINT16_MAX
 */
static inline uint16_t mavlink_msg_cellular_status_get_mnc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field lac from cellular_status message
 *
 * @return  Location area code. If unknown, set to 0
 */
static inline uint16_t mavlink_msg_cellular_status_get_lac(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a cellular_status message into a struct
 *
 * @param msg The message to decode
 * @param cellular_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_cellular_status_decode(const mavlink_message_t* msg, mavlink_cellular_status_t* cellular_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cellular_status->mcc = mavlink_msg_cellular_status_get_mcc(msg);
    cellular_status->mnc = mavlink_msg_cellular_status_get_mnc(msg);
    cellular_status->lac = mavlink_msg_cellular_status_get_lac(msg);
    cellular_status->status = mavlink_msg_cellular_status_get_status(msg);
    cellular_status->failure_reason = mavlink_msg_cellular_status_get_failure_reason(msg);
    cellular_status->type = mavlink_msg_cellular_status_get_type(msg);
    cellular_status->quality = mavlink_msg_cellular_status_get_quality(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CELLULAR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CELLULAR_STATUS_LEN;
        memset(cellular_status, 0, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
    memcpy(cellular_status, _MAV_PAYLOAD(msg), len);
#endif
}
