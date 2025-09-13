#pragma once
// MESSAGE CELLULAR_CONFIG PACKING

#define MAVLINK_MSG_ID_CELLULAR_CONFIG 336


typedef struct __mavlink_cellular_config_t {
 uint8_t enable_lte; /*<  Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.*/
 uint8_t enable_pin; /*<  Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.*/
 char pin[16]; /*<  PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.*/
 char new_pin[16]; /*<  New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a response.*/
 char apn[32]; /*<  Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.*/
 char puk[16]; /*<  Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message is sent back as a response.*/
 uint8_t roaming; /*<  Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.*/
 uint8_t response; /*<  Message acceptance response (sent back to GS).*/
} mavlink_cellular_config_t;

#define MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN 84
#define MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN 84
#define MAVLINK_MSG_ID_336_LEN 84
#define MAVLINK_MSG_ID_336_MIN_LEN 84

#define MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC 245
#define MAVLINK_MSG_ID_336_CRC 245

#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_LEN 16
#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_LEN 16
#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_LEN 32
#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CELLULAR_CONFIG { \
    336, \
    "CELLULAR_CONFIG", \
    8, \
    {  { "enable_lte", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_cellular_config_t, enable_lte) }, \
         { "enable_pin", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_cellular_config_t, enable_pin) }, \
         { "pin", NULL, MAVLINK_TYPE_CHAR, 16, 2, offsetof(mavlink_cellular_config_t, pin) }, \
         { "new_pin", NULL, MAVLINK_TYPE_CHAR, 16, 18, offsetof(mavlink_cellular_config_t, new_pin) }, \
         { "apn", NULL, MAVLINK_TYPE_CHAR, 32, 34, offsetof(mavlink_cellular_config_t, apn) }, \
         { "puk", NULL, MAVLINK_TYPE_CHAR, 16, 66, offsetof(mavlink_cellular_config_t, puk) }, \
         { "roaming", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_cellular_config_t, roaming) }, \
         { "response", NULL, MAVLINK_TYPE_UINT8_T, 0, 83, offsetof(mavlink_cellular_config_t, response) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CELLULAR_CONFIG { \
    "CELLULAR_CONFIG", \
    8, \
    {  { "enable_lte", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_cellular_config_t, enable_lte) }, \
         { "enable_pin", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_cellular_config_t, enable_pin) }, \
         { "pin", NULL, MAVLINK_TYPE_CHAR, 16, 2, offsetof(mavlink_cellular_config_t, pin) }, \
         { "new_pin", NULL, MAVLINK_TYPE_CHAR, 16, 18, offsetof(mavlink_cellular_config_t, new_pin) }, \
         { "apn", NULL, MAVLINK_TYPE_CHAR, 32, 34, offsetof(mavlink_cellular_config_t, apn) }, \
         { "puk", NULL, MAVLINK_TYPE_CHAR, 16, 66, offsetof(mavlink_cellular_config_t, puk) }, \
         { "roaming", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_cellular_config_t, roaming) }, \
         { "response", NULL, MAVLINK_TYPE_UINT8_T, 0, 83, offsetof(mavlink_cellular_config_t, response) }, \
         } \
}
#endif

/**
 * @brief Pack a cellular_config message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enable_lte  Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param enable_pin  Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param pin  PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.
 * @param new_pin  New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a response.
 * @param apn  Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.
 * @param puk  Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message is sent back as a response.
 * @param roaming  Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param response  Message acceptance response (sent back to GS).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cellular_config_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t enable_lte, uint8_t enable_pin, const char *pin, const char *new_pin, const char *apn, const char *puk, uint8_t roaming, uint8_t response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN];
    _mav_put_uint8_t(buf, 0, enable_lte);
    _mav_put_uint8_t(buf, 1, enable_pin);
    _mav_put_uint8_t(buf, 82, roaming);
    _mav_put_uint8_t(buf, 83, response);
    _mav_put_char_array(buf, 2, pin, 16);
    _mav_put_char_array(buf, 18, new_pin, 16);
    _mav_put_char_array(buf, 34, apn, 32);
    _mav_put_char_array(buf, 66, puk, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN);
#else
    mavlink_cellular_config_t packet;
    packet.enable_lte = enable_lte;
    packet.enable_pin = enable_pin;
    packet.roaming = roaming;
    packet.response = response;
    mav_array_assign_char(packet.pin, pin, 16);
    mav_array_assign_char(packet.new_pin, new_pin, 16);
    mav_array_assign_char(packet.apn, apn, 32);
    mav_array_assign_char(packet.puk, puk, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CELLULAR_CONFIG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC);
}

/**
 * @brief Pack a cellular_config message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param enable_lte  Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param enable_pin  Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param pin  PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.
 * @param new_pin  New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a response.
 * @param apn  Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.
 * @param puk  Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message is sent back as a response.
 * @param roaming  Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param response  Message acceptance response (sent back to GS).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cellular_config_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t enable_lte, uint8_t enable_pin, const char *pin, const char *new_pin, const char *apn, const char *puk, uint8_t roaming, uint8_t response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN];
    _mav_put_uint8_t(buf, 0, enable_lte);
    _mav_put_uint8_t(buf, 1, enable_pin);
    _mav_put_uint8_t(buf, 82, roaming);
    _mav_put_uint8_t(buf, 83, response);
    _mav_put_char_array(buf, 2, pin, 16);
    _mav_put_char_array(buf, 18, new_pin, 16);
    _mav_put_char_array(buf, 34, apn, 32);
    _mav_put_char_array(buf, 66, puk, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN);
#else
    mavlink_cellular_config_t packet;
    packet.enable_lte = enable_lte;
    packet.enable_pin = enable_pin;
    packet.roaming = roaming;
    packet.response = response;
    mav_array_memcpy(packet.pin, pin, sizeof(char)*16);
    mav_array_memcpy(packet.new_pin, new_pin, sizeof(char)*16);
    mav_array_memcpy(packet.apn, apn, sizeof(char)*32);
    mav_array_memcpy(packet.puk, puk, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CELLULAR_CONFIG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN);
#endif
}

/**
 * @brief Pack a cellular_config message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param enable_lte  Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param enable_pin  Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param pin  PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.
 * @param new_pin  New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a response.
 * @param apn  Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.
 * @param puk  Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message is sent back as a response.
 * @param roaming  Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param response  Message acceptance response (sent back to GS).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cellular_config_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t enable_lte,uint8_t enable_pin,const char *pin,const char *new_pin,const char *apn,const char *puk,uint8_t roaming,uint8_t response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN];
    _mav_put_uint8_t(buf, 0, enable_lte);
    _mav_put_uint8_t(buf, 1, enable_pin);
    _mav_put_uint8_t(buf, 82, roaming);
    _mav_put_uint8_t(buf, 83, response);
    _mav_put_char_array(buf, 2, pin, 16);
    _mav_put_char_array(buf, 18, new_pin, 16);
    _mav_put_char_array(buf, 34, apn, 32);
    _mav_put_char_array(buf, 66, puk, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN);
#else
    mavlink_cellular_config_t packet;
    packet.enable_lte = enable_lte;
    packet.enable_pin = enable_pin;
    packet.roaming = roaming;
    packet.response = response;
    mav_array_assign_char(packet.pin, pin, 16);
    mav_array_assign_char(packet.new_pin, new_pin, 16);
    mav_array_assign_char(packet.apn, apn, 32);
    mav_array_assign_char(packet.puk, puk, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CELLULAR_CONFIG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC);
}

/**
 * @brief Encode a cellular_config struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cellular_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cellular_config_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cellular_config_t* cellular_config)
{
    return mavlink_msg_cellular_config_pack(system_id, component_id, msg, cellular_config->enable_lte, cellular_config->enable_pin, cellular_config->pin, cellular_config->new_pin, cellular_config->apn, cellular_config->puk, cellular_config->roaming, cellular_config->response);
}

/**
 * @brief Encode a cellular_config struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cellular_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cellular_config_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cellular_config_t* cellular_config)
{
    return mavlink_msg_cellular_config_pack_chan(system_id, component_id, chan, msg, cellular_config->enable_lte, cellular_config->enable_pin, cellular_config->pin, cellular_config->new_pin, cellular_config->apn, cellular_config->puk, cellular_config->roaming, cellular_config->response);
}

/**
 * @brief Encode a cellular_config struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param cellular_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cellular_config_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_cellular_config_t* cellular_config)
{
    return mavlink_msg_cellular_config_pack_status(system_id, component_id, _status, msg,  cellular_config->enable_lte, cellular_config->enable_pin, cellular_config->pin, cellular_config->new_pin, cellular_config->apn, cellular_config->puk, cellular_config->roaming, cellular_config->response);
}

/**
 * @brief Send a cellular_config message
 * @param chan MAVLink channel to send the message
 *
 * @param enable_lte  Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param enable_pin  Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param pin  PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.
 * @param new_pin  New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a response.
 * @param apn  Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.
 * @param puk  Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message is sent back as a response.
 * @param roaming  Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 * @param response  Message acceptance response (sent back to GS).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cellular_config_send(mavlink_channel_t chan, uint8_t enable_lte, uint8_t enable_pin, const char *pin, const char *new_pin, const char *apn, const char *puk, uint8_t roaming, uint8_t response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN];
    _mav_put_uint8_t(buf, 0, enable_lte);
    _mav_put_uint8_t(buf, 1, enable_pin);
    _mav_put_uint8_t(buf, 82, roaming);
    _mav_put_uint8_t(buf, 83, response);
    _mav_put_char_array(buf, 2, pin, 16);
    _mav_put_char_array(buf, 18, new_pin, 16);
    _mav_put_char_array(buf, 34, apn, 32);
    _mav_put_char_array(buf, 66, puk, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_CONFIG, buf, MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC);
#else
    mavlink_cellular_config_t packet;
    packet.enable_lte = enable_lte;
    packet.enable_pin = enable_pin;
    packet.roaming = roaming;
    packet.response = response;
    mav_array_assign_char(packet.pin, pin, 16);
    mav_array_assign_char(packet.new_pin, new_pin, 16);
    mav_array_assign_char(packet.apn, apn, 32);
    mav_array_assign_char(packet.puk, puk, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_CONFIG, (const char *)&packet, MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC);
#endif
}

/**
 * @brief Send a cellular_config message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cellular_config_send_struct(mavlink_channel_t chan, const mavlink_cellular_config_t* cellular_config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cellular_config_send(chan, cellular_config->enable_lte, cellular_config->enable_pin, cellular_config->pin, cellular_config->new_pin, cellular_config->apn, cellular_config->puk, cellular_config->roaming, cellular_config->response);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_CONFIG, (const char *)cellular_config, MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC);
#endif
}

#if MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cellular_config_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t enable_lte, uint8_t enable_pin, const char *pin, const char *new_pin, const char *apn, const char *puk, uint8_t roaming, uint8_t response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, enable_lte);
    _mav_put_uint8_t(buf, 1, enable_pin);
    _mav_put_uint8_t(buf, 82, roaming);
    _mav_put_uint8_t(buf, 83, response);
    _mav_put_char_array(buf, 2, pin, 16);
    _mav_put_char_array(buf, 18, new_pin, 16);
    _mav_put_char_array(buf, 34, apn, 32);
    _mav_put_char_array(buf, 66, puk, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_CONFIG, buf, MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC);
#else
    mavlink_cellular_config_t *packet = (mavlink_cellular_config_t *)msgbuf;
    packet->enable_lte = enable_lte;
    packet->enable_pin = enable_pin;
    packet->roaming = roaming;
    packet->response = response;
    mav_array_assign_char(packet->pin, pin, 16);
    mav_array_assign_char(packet->new_pin, new_pin, 16);
    mav_array_assign_char(packet->apn, apn, 32);
    mav_array_assign_char(packet->puk, puk, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_CONFIG, (const char *)packet, MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN, MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC);
#endif
}
#endif

#endif

// MESSAGE CELLULAR_CONFIG UNPACKING


/**
 * @brief Get field enable_lte from cellular_config message
 *
 * @return  Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 */
static inline uint8_t mavlink_msg_cellular_config_get_enable_lte(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field enable_pin from cellular_config message
 *
 * @return  Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 */
static inline uint8_t mavlink_msg_cellular_config_get_enable_pin(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field pin from cellular_config message
 *
 * @return  PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.
 */
static inline uint16_t mavlink_msg_cellular_config_get_pin(const mavlink_message_t* msg, char *pin)
{
    return _MAV_RETURN_char_array(msg, pin, 16,  2);
}

/**
 * @brief Get field new_pin from cellular_config message
 *
 * @return  New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a response.
 */
static inline uint16_t mavlink_msg_cellular_config_get_new_pin(const mavlink_message_t* msg, char *new_pin)
{
    return _MAV_RETURN_char_array(msg, new_pin, 16,  18);
}

/**
 * @brief Get field apn from cellular_config message
 *
 * @return  Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.
 */
static inline uint16_t mavlink_msg_cellular_config_get_apn(const mavlink_message_t* msg, char *apn)
{
    return _MAV_RETURN_char_array(msg, apn, 32,  34);
}

/**
 * @brief Get field puk from cellular_config message
 *
 * @return  Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message is sent back as a response.
 */
static inline uint16_t mavlink_msg_cellular_config_get_puk(const mavlink_message_t* msg, char *puk)
{
    return _MAV_RETURN_char_array(msg, puk, 16,  66);
}

/**
 * @brief Get field roaming from cellular_config message
 *
 * @return  Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
 */
static inline uint8_t mavlink_msg_cellular_config_get_roaming(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  82);
}

/**
 * @brief Get field response from cellular_config message
 *
 * @return  Message acceptance response (sent back to GS).
 */
static inline uint8_t mavlink_msg_cellular_config_get_response(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  83);
}

/**
 * @brief Decode a cellular_config message into a struct
 *
 * @param msg The message to decode
 * @param cellular_config C-struct to decode the message contents into
 */
static inline void mavlink_msg_cellular_config_decode(const mavlink_message_t* msg, mavlink_cellular_config_t* cellular_config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cellular_config->enable_lte = mavlink_msg_cellular_config_get_enable_lte(msg);
    cellular_config->enable_pin = mavlink_msg_cellular_config_get_enable_pin(msg);
    mavlink_msg_cellular_config_get_pin(msg, cellular_config->pin);
    mavlink_msg_cellular_config_get_new_pin(msg, cellular_config->new_pin);
    mavlink_msg_cellular_config_get_apn(msg, cellular_config->apn);
    mavlink_msg_cellular_config_get_puk(msg, cellular_config->puk);
    cellular_config->roaming = mavlink_msg_cellular_config_get_roaming(msg);
    cellular_config->response = mavlink_msg_cellular_config_get_response(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN? msg->len : MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN;
        memset(cellular_config, 0, MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN);
    memcpy(cellular_config, _MAV_PAYLOAD(msg), len);
#endif
}
