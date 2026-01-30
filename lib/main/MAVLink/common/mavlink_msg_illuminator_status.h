#pragma once
// MESSAGE ILLUMINATOR_STATUS PACKING

#define MAVLINK_MSG_ID_ILLUMINATOR_STATUS 440


typedef struct __mavlink_illuminator_status_t {
 uint32_t uptime_ms; /*< [ms] Time since the start-up of the illuminator in ms*/
 uint32_t error_status; /*<  Errors*/
 float brightness; /*< [%] Illuminator brightness*/
 float strobe_period; /*< [s] Illuminator strobing period in seconds*/
 float strobe_duty_cycle; /*< [%] Illuminator strobing duty cycle*/
 float temp_c; /*<  Temperature in Celsius*/
 float min_strobe_period; /*< [s] Minimum strobing period in seconds*/
 float max_strobe_period; /*< [s] Maximum strobing period in seconds*/
 uint8_t enable; /*<  0: Illuminators OFF, 1: Illuminators ON*/
 uint8_t mode_bitmask; /*<  Supported illuminator modes*/
 uint8_t mode; /*<  Illuminator mode*/
} mavlink_illuminator_status_t;

#define MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN 35
#define MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN 35
#define MAVLINK_MSG_ID_440_LEN 35
#define MAVLINK_MSG_ID_440_MIN_LEN 35

#define MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC 66
#define MAVLINK_MSG_ID_440_CRC 66



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ILLUMINATOR_STATUS { \
    440, \
    "ILLUMINATOR_STATUS", \
    11, \
    {  { "uptime_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_illuminator_status_t, uptime_ms) }, \
         { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_illuminator_status_t, enable) }, \
         { "mode_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_illuminator_status_t, mode_bitmask) }, \
         { "error_status", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_illuminator_status_t, error_status) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_illuminator_status_t, mode) }, \
         { "brightness", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_illuminator_status_t, brightness) }, \
         { "strobe_period", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_illuminator_status_t, strobe_period) }, \
         { "strobe_duty_cycle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_illuminator_status_t, strobe_duty_cycle) }, \
         { "temp_c", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_illuminator_status_t, temp_c) }, \
         { "min_strobe_period", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_illuminator_status_t, min_strobe_period) }, \
         { "max_strobe_period", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_illuminator_status_t, max_strobe_period) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ILLUMINATOR_STATUS { \
    "ILLUMINATOR_STATUS", \
    11, \
    {  { "uptime_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_illuminator_status_t, uptime_ms) }, \
         { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_illuminator_status_t, enable) }, \
         { "mode_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_illuminator_status_t, mode_bitmask) }, \
         { "error_status", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_illuminator_status_t, error_status) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_illuminator_status_t, mode) }, \
         { "brightness", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_illuminator_status_t, brightness) }, \
         { "strobe_period", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_illuminator_status_t, strobe_period) }, \
         { "strobe_duty_cycle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_illuminator_status_t, strobe_duty_cycle) }, \
         { "temp_c", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_illuminator_status_t, temp_c) }, \
         { "min_strobe_period", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_illuminator_status_t, min_strobe_period) }, \
         { "max_strobe_period", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_illuminator_status_t, max_strobe_period) }, \
         } \
}
#endif

/**
 * @brief Pack a illuminator_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param uptime_ms [ms] Time since the start-up of the illuminator in ms
 * @param enable  0: Illuminators OFF, 1: Illuminators ON
 * @param mode_bitmask  Supported illuminator modes
 * @param error_status  Errors
 * @param mode  Illuminator mode
 * @param brightness [%] Illuminator brightness
 * @param strobe_period [s] Illuminator strobing period in seconds
 * @param strobe_duty_cycle [%] Illuminator strobing duty cycle
 * @param temp_c  Temperature in Celsius
 * @param min_strobe_period [s] Minimum strobing period in seconds
 * @param max_strobe_period [s] Maximum strobing period in seconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_illuminator_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t uptime_ms, uint8_t enable, uint8_t mode_bitmask, uint32_t error_status, uint8_t mode, float brightness, float strobe_period, float strobe_duty_cycle, float temp_c, float min_strobe_period, float max_strobe_period)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, uptime_ms);
    _mav_put_uint32_t(buf, 4, error_status);
    _mav_put_float(buf, 8, brightness);
    _mav_put_float(buf, 12, strobe_period);
    _mav_put_float(buf, 16, strobe_duty_cycle);
    _mav_put_float(buf, 20, temp_c);
    _mav_put_float(buf, 24, min_strobe_period);
    _mav_put_float(buf, 28, max_strobe_period);
    _mav_put_uint8_t(buf, 32, enable);
    _mav_put_uint8_t(buf, 33, mode_bitmask);
    _mav_put_uint8_t(buf, 34, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN);
#else
    mavlink_illuminator_status_t packet;
    packet.uptime_ms = uptime_ms;
    packet.error_status = error_status;
    packet.brightness = brightness;
    packet.strobe_period = strobe_period;
    packet.strobe_duty_cycle = strobe_duty_cycle;
    packet.temp_c = temp_c;
    packet.min_strobe_period = min_strobe_period;
    packet.max_strobe_period = max_strobe_period;
    packet.enable = enable;
    packet.mode_bitmask = mode_bitmask;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ILLUMINATOR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC);
}

/**
 * @brief Pack a illuminator_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param uptime_ms [ms] Time since the start-up of the illuminator in ms
 * @param enable  0: Illuminators OFF, 1: Illuminators ON
 * @param mode_bitmask  Supported illuminator modes
 * @param error_status  Errors
 * @param mode  Illuminator mode
 * @param brightness [%] Illuminator brightness
 * @param strobe_period [s] Illuminator strobing period in seconds
 * @param strobe_duty_cycle [%] Illuminator strobing duty cycle
 * @param temp_c  Temperature in Celsius
 * @param min_strobe_period [s] Minimum strobing period in seconds
 * @param max_strobe_period [s] Maximum strobing period in seconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_illuminator_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t uptime_ms, uint8_t enable, uint8_t mode_bitmask, uint32_t error_status, uint8_t mode, float brightness, float strobe_period, float strobe_duty_cycle, float temp_c, float min_strobe_period, float max_strobe_period)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, uptime_ms);
    _mav_put_uint32_t(buf, 4, error_status);
    _mav_put_float(buf, 8, brightness);
    _mav_put_float(buf, 12, strobe_period);
    _mav_put_float(buf, 16, strobe_duty_cycle);
    _mav_put_float(buf, 20, temp_c);
    _mav_put_float(buf, 24, min_strobe_period);
    _mav_put_float(buf, 28, max_strobe_period);
    _mav_put_uint8_t(buf, 32, enable);
    _mav_put_uint8_t(buf, 33, mode_bitmask);
    _mav_put_uint8_t(buf, 34, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN);
#else
    mavlink_illuminator_status_t packet;
    packet.uptime_ms = uptime_ms;
    packet.error_status = error_status;
    packet.brightness = brightness;
    packet.strobe_period = strobe_period;
    packet.strobe_duty_cycle = strobe_duty_cycle;
    packet.temp_c = temp_c;
    packet.min_strobe_period = min_strobe_period;
    packet.max_strobe_period = max_strobe_period;
    packet.enable = enable;
    packet.mode_bitmask = mode_bitmask;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ILLUMINATOR_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN);
#endif
}

/**
 * @brief Pack a illuminator_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uptime_ms [ms] Time since the start-up of the illuminator in ms
 * @param enable  0: Illuminators OFF, 1: Illuminators ON
 * @param mode_bitmask  Supported illuminator modes
 * @param error_status  Errors
 * @param mode  Illuminator mode
 * @param brightness [%] Illuminator brightness
 * @param strobe_period [s] Illuminator strobing period in seconds
 * @param strobe_duty_cycle [%] Illuminator strobing duty cycle
 * @param temp_c  Temperature in Celsius
 * @param min_strobe_period [s] Minimum strobing period in seconds
 * @param max_strobe_period [s] Maximum strobing period in seconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_illuminator_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t uptime_ms,uint8_t enable,uint8_t mode_bitmask,uint32_t error_status,uint8_t mode,float brightness,float strobe_period,float strobe_duty_cycle,float temp_c,float min_strobe_period,float max_strobe_period)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, uptime_ms);
    _mav_put_uint32_t(buf, 4, error_status);
    _mav_put_float(buf, 8, brightness);
    _mav_put_float(buf, 12, strobe_period);
    _mav_put_float(buf, 16, strobe_duty_cycle);
    _mav_put_float(buf, 20, temp_c);
    _mav_put_float(buf, 24, min_strobe_period);
    _mav_put_float(buf, 28, max_strobe_period);
    _mav_put_uint8_t(buf, 32, enable);
    _mav_put_uint8_t(buf, 33, mode_bitmask);
    _mav_put_uint8_t(buf, 34, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN);
#else
    mavlink_illuminator_status_t packet;
    packet.uptime_ms = uptime_ms;
    packet.error_status = error_status;
    packet.brightness = brightness;
    packet.strobe_period = strobe_period;
    packet.strobe_duty_cycle = strobe_duty_cycle;
    packet.temp_c = temp_c;
    packet.min_strobe_period = min_strobe_period;
    packet.max_strobe_period = max_strobe_period;
    packet.enable = enable;
    packet.mode_bitmask = mode_bitmask;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ILLUMINATOR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC);
}

/**
 * @brief Encode a illuminator_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param illuminator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_illuminator_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_illuminator_status_t* illuminator_status)
{
    return mavlink_msg_illuminator_status_pack(system_id, component_id, msg, illuminator_status->uptime_ms, illuminator_status->enable, illuminator_status->mode_bitmask, illuminator_status->error_status, illuminator_status->mode, illuminator_status->brightness, illuminator_status->strobe_period, illuminator_status->strobe_duty_cycle, illuminator_status->temp_c, illuminator_status->min_strobe_period, illuminator_status->max_strobe_period);
}

/**
 * @brief Encode a illuminator_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param illuminator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_illuminator_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_illuminator_status_t* illuminator_status)
{
    return mavlink_msg_illuminator_status_pack_chan(system_id, component_id, chan, msg, illuminator_status->uptime_ms, illuminator_status->enable, illuminator_status->mode_bitmask, illuminator_status->error_status, illuminator_status->mode, illuminator_status->brightness, illuminator_status->strobe_period, illuminator_status->strobe_duty_cycle, illuminator_status->temp_c, illuminator_status->min_strobe_period, illuminator_status->max_strobe_period);
}

/**
 * @brief Encode a illuminator_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param illuminator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_illuminator_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_illuminator_status_t* illuminator_status)
{
    return mavlink_msg_illuminator_status_pack_status(system_id, component_id, _status, msg,  illuminator_status->uptime_ms, illuminator_status->enable, illuminator_status->mode_bitmask, illuminator_status->error_status, illuminator_status->mode, illuminator_status->brightness, illuminator_status->strobe_period, illuminator_status->strobe_duty_cycle, illuminator_status->temp_c, illuminator_status->min_strobe_period, illuminator_status->max_strobe_period);
}

/**
 * @brief Send a illuminator_status message
 * @param chan MAVLink channel to send the message
 *
 * @param uptime_ms [ms] Time since the start-up of the illuminator in ms
 * @param enable  0: Illuminators OFF, 1: Illuminators ON
 * @param mode_bitmask  Supported illuminator modes
 * @param error_status  Errors
 * @param mode  Illuminator mode
 * @param brightness [%] Illuminator brightness
 * @param strobe_period [s] Illuminator strobing period in seconds
 * @param strobe_duty_cycle [%] Illuminator strobing duty cycle
 * @param temp_c  Temperature in Celsius
 * @param min_strobe_period [s] Minimum strobing period in seconds
 * @param max_strobe_period [s] Maximum strobing period in seconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_illuminator_status_send(mavlink_channel_t chan, uint32_t uptime_ms, uint8_t enable, uint8_t mode_bitmask, uint32_t error_status, uint8_t mode, float brightness, float strobe_period, float strobe_duty_cycle, float temp_c, float min_strobe_period, float max_strobe_period)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, uptime_ms);
    _mav_put_uint32_t(buf, 4, error_status);
    _mav_put_float(buf, 8, brightness);
    _mav_put_float(buf, 12, strobe_period);
    _mav_put_float(buf, 16, strobe_duty_cycle);
    _mav_put_float(buf, 20, temp_c);
    _mav_put_float(buf, 24, min_strobe_period);
    _mav_put_float(buf, 28, max_strobe_period);
    _mav_put_uint8_t(buf, 32, enable);
    _mav_put_uint8_t(buf, 33, mode_bitmask);
    _mav_put_uint8_t(buf, 34, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ILLUMINATOR_STATUS, buf, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC);
#else
    mavlink_illuminator_status_t packet;
    packet.uptime_ms = uptime_ms;
    packet.error_status = error_status;
    packet.brightness = brightness;
    packet.strobe_period = strobe_period;
    packet.strobe_duty_cycle = strobe_duty_cycle;
    packet.temp_c = temp_c;
    packet.min_strobe_period = min_strobe_period;
    packet.max_strobe_period = max_strobe_period;
    packet.enable = enable;
    packet.mode_bitmask = mode_bitmask;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ILLUMINATOR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC);
#endif
}

/**
 * @brief Send a illuminator_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_illuminator_status_send_struct(mavlink_channel_t chan, const mavlink_illuminator_status_t* illuminator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_illuminator_status_send(chan, illuminator_status->uptime_ms, illuminator_status->enable, illuminator_status->mode_bitmask, illuminator_status->error_status, illuminator_status->mode, illuminator_status->brightness, illuminator_status->strobe_period, illuminator_status->strobe_duty_cycle, illuminator_status->temp_c, illuminator_status->min_strobe_period, illuminator_status->max_strobe_period);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ILLUMINATOR_STATUS, (const char *)illuminator_status, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_illuminator_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t uptime_ms, uint8_t enable, uint8_t mode_bitmask, uint32_t error_status, uint8_t mode, float brightness, float strobe_period, float strobe_duty_cycle, float temp_c, float min_strobe_period, float max_strobe_period)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, uptime_ms);
    _mav_put_uint32_t(buf, 4, error_status);
    _mav_put_float(buf, 8, brightness);
    _mav_put_float(buf, 12, strobe_period);
    _mav_put_float(buf, 16, strobe_duty_cycle);
    _mav_put_float(buf, 20, temp_c);
    _mav_put_float(buf, 24, min_strobe_period);
    _mav_put_float(buf, 28, max_strobe_period);
    _mav_put_uint8_t(buf, 32, enable);
    _mav_put_uint8_t(buf, 33, mode_bitmask);
    _mav_put_uint8_t(buf, 34, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ILLUMINATOR_STATUS, buf, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC);
#else
    mavlink_illuminator_status_t *packet = (mavlink_illuminator_status_t *)msgbuf;
    packet->uptime_ms = uptime_ms;
    packet->error_status = error_status;
    packet->brightness = brightness;
    packet->strobe_period = strobe_period;
    packet->strobe_duty_cycle = strobe_duty_cycle;
    packet->temp_c = temp_c;
    packet->min_strobe_period = min_strobe_period;
    packet->max_strobe_period = max_strobe_period;
    packet->enable = enable;
    packet->mode_bitmask = mode_bitmask;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ILLUMINATOR_STATUS, (const char *)packet, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ILLUMINATOR_STATUS UNPACKING


/**
 * @brief Get field uptime_ms from illuminator_status message
 *
 * @return [ms] Time since the start-up of the illuminator in ms
 */
static inline uint32_t mavlink_msg_illuminator_status_get_uptime_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field enable from illuminator_status message
 *
 * @return  0: Illuminators OFF, 1: Illuminators ON
 */
static inline uint8_t mavlink_msg_illuminator_status_get_enable(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field mode_bitmask from illuminator_status message
 *
 * @return  Supported illuminator modes
 */
static inline uint8_t mavlink_msg_illuminator_status_get_mode_bitmask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field error_status from illuminator_status message
 *
 * @return  Errors
 */
static inline uint32_t mavlink_msg_illuminator_status_get_error_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field mode from illuminator_status message
 *
 * @return  Illuminator mode
 */
static inline uint8_t mavlink_msg_illuminator_status_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field brightness from illuminator_status message
 *
 * @return [%] Illuminator brightness
 */
static inline float mavlink_msg_illuminator_status_get_brightness(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field strobe_period from illuminator_status message
 *
 * @return [s] Illuminator strobing period in seconds
 */
static inline float mavlink_msg_illuminator_status_get_strobe_period(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field strobe_duty_cycle from illuminator_status message
 *
 * @return [%] Illuminator strobing duty cycle
 */
static inline float mavlink_msg_illuminator_status_get_strobe_duty_cycle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field temp_c from illuminator_status message
 *
 * @return  Temperature in Celsius
 */
static inline float mavlink_msg_illuminator_status_get_temp_c(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field min_strobe_period from illuminator_status message
 *
 * @return [s] Minimum strobing period in seconds
 */
static inline float mavlink_msg_illuminator_status_get_min_strobe_period(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field max_strobe_period from illuminator_status message
 *
 * @return [s] Maximum strobing period in seconds
 */
static inline float mavlink_msg_illuminator_status_get_max_strobe_period(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a illuminator_status message into a struct
 *
 * @param msg The message to decode
 * @param illuminator_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_illuminator_status_decode(const mavlink_message_t* msg, mavlink_illuminator_status_t* illuminator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    illuminator_status->uptime_ms = mavlink_msg_illuminator_status_get_uptime_ms(msg);
    illuminator_status->error_status = mavlink_msg_illuminator_status_get_error_status(msg);
    illuminator_status->brightness = mavlink_msg_illuminator_status_get_brightness(msg);
    illuminator_status->strobe_period = mavlink_msg_illuminator_status_get_strobe_period(msg);
    illuminator_status->strobe_duty_cycle = mavlink_msg_illuminator_status_get_strobe_duty_cycle(msg);
    illuminator_status->temp_c = mavlink_msg_illuminator_status_get_temp_c(msg);
    illuminator_status->min_strobe_period = mavlink_msg_illuminator_status_get_min_strobe_period(msg);
    illuminator_status->max_strobe_period = mavlink_msg_illuminator_status_get_max_strobe_period(msg);
    illuminator_status->enable = mavlink_msg_illuminator_status_get_enable(msg);
    illuminator_status->mode_bitmask = mavlink_msg_illuminator_status_get_mode_bitmask(msg);
    illuminator_status->mode = mavlink_msg_illuminator_status_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN;
        memset(illuminator_status, 0, MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN);
    memcpy(illuminator_status, _MAV_PAYLOAD(msg), len);
#endif
}
