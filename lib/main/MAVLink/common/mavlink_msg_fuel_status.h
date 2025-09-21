#pragma once
// MESSAGE FUEL_STATUS PACKING

#define MAVLINK_MSG_ID_FUEL_STATUS 371


typedef struct __mavlink_fuel_status_t {
 float maximum_fuel; /*<  Capacity when full. Must be provided.*/
 float consumed_fuel; /*<  Consumed fuel (measured). This value should not be inferred: if not measured set to NaN. NaN: field not provided.*/
 float remaining_fuel; /*<  Remaining fuel until empty (measured). The value should not be inferred: if not measured set to NaN. NaN: field not provided.*/
 float flow_rate; /*<  Positive value when emptying/using, and negative if filling/replacing. NaN: field not provided.*/
 float temperature; /*< [K] Fuel temperature. NaN: field not provided.*/
 uint32_t fuel_type; /*<  Fuel type. Defines units for fuel capacity and consumption fields above.*/
 uint8_t id; /*<  Fuel ID. Must match ID of other messages for same fuel system, such as BATTERY_STATUS_V2.*/
 uint8_t percent_remaining; /*< [%] Percentage of remaining fuel, relative to full. Values: [0-100], UINT8_MAX: field not provided.*/
} mavlink_fuel_status_t;

#define MAVLINK_MSG_ID_FUEL_STATUS_LEN 26
#define MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN 26
#define MAVLINK_MSG_ID_371_LEN 26
#define MAVLINK_MSG_ID_371_MIN_LEN 26

#define MAVLINK_MSG_ID_FUEL_STATUS_CRC 10
#define MAVLINK_MSG_ID_371_CRC 10



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FUEL_STATUS { \
    371, \
    "FUEL_STATUS", \
    8, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_fuel_status_t, id) }, \
         { "maximum_fuel", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_fuel_status_t, maximum_fuel) }, \
         { "consumed_fuel", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_fuel_status_t, consumed_fuel) }, \
         { "remaining_fuel", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_fuel_status_t, remaining_fuel) }, \
         { "percent_remaining", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_fuel_status_t, percent_remaining) }, \
         { "flow_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_fuel_status_t, flow_rate) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_fuel_status_t, temperature) }, \
         { "fuel_type", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_fuel_status_t, fuel_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FUEL_STATUS { \
    "FUEL_STATUS", \
    8, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_fuel_status_t, id) }, \
         { "maximum_fuel", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_fuel_status_t, maximum_fuel) }, \
         { "consumed_fuel", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_fuel_status_t, consumed_fuel) }, \
         { "remaining_fuel", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_fuel_status_t, remaining_fuel) }, \
         { "percent_remaining", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_fuel_status_t, percent_remaining) }, \
         { "flow_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_fuel_status_t, flow_rate) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_fuel_status_t, temperature) }, \
         { "fuel_type", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_fuel_status_t, fuel_type) }, \
         } \
}
#endif

/**
 * @brief Pack a fuel_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Fuel ID. Must match ID of other messages for same fuel system, such as BATTERY_STATUS_V2.
 * @param maximum_fuel  Capacity when full. Must be provided.
 * @param consumed_fuel  Consumed fuel (measured). This value should not be inferred: if not measured set to NaN. NaN: field not provided.
 * @param remaining_fuel  Remaining fuel until empty (measured). The value should not be inferred: if not measured set to NaN. NaN: field not provided.
 * @param percent_remaining [%] Percentage of remaining fuel, relative to full. Values: [0-100], UINT8_MAX: field not provided.
 * @param flow_rate  Positive value when emptying/using, and negative if filling/replacing. NaN: field not provided.
 * @param temperature [K] Fuel temperature. NaN: field not provided.
 * @param fuel_type  Fuel type. Defines units for fuel capacity and consumption fields above.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fuel_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, float maximum_fuel, float consumed_fuel, float remaining_fuel, uint8_t percent_remaining, float flow_rate, float temperature, uint32_t fuel_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FUEL_STATUS_LEN];
    _mav_put_float(buf, 0, maximum_fuel);
    _mav_put_float(buf, 4, consumed_fuel);
    _mav_put_float(buf, 8, remaining_fuel);
    _mav_put_float(buf, 12, flow_rate);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint32_t(buf, 20, fuel_type);
    _mav_put_uint8_t(buf, 24, id);
    _mav_put_uint8_t(buf, 25, percent_remaining);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FUEL_STATUS_LEN);
#else
    mavlink_fuel_status_t packet;
    packet.maximum_fuel = maximum_fuel;
    packet.consumed_fuel = consumed_fuel;
    packet.remaining_fuel = remaining_fuel;
    packet.flow_rate = flow_rate;
    packet.temperature = temperature;
    packet.fuel_type = fuel_type;
    packet.id = id;
    packet.percent_remaining = percent_remaining;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FUEL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FUEL_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN, MAVLINK_MSG_ID_FUEL_STATUS_LEN, MAVLINK_MSG_ID_FUEL_STATUS_CRC);
}

/**
 * @brief Pack a fuel_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Fuel ID. Must match ID of other messages for same fuel system, such as BATTERY_STATUS_V2.
 * @param maximum_fuel  Capacity when full. Must be provided.
 * @param consumed_fuel  Consumed fuel (measured). This value should not be inferred: if not measured set to NaN. NaN: field not provided.
 * @param remaining_fuel  Remaining fuel until empty (measured). The value should not be inferred: if not measured set to NaN. NaN: field not provided.
 * @param percent_remaining [%] Percentage of remaining fuel, relative to full. Values: [0-100], UINT8_MAX: field not provided.
 * @param flow_rate  Positive value when emptying/using, and negative if filling/replacing. NaN: field not provided.
 * @param temperature [K] Fuel temperature. NaN: field not provided.
 * @param fuel_type  Fuel type. Defines units for fuel capacity and consumption fields above.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fuel_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t id, float maximum_fuel, float consumed_fuel, float remaining_fuel, uint8_t percent_remaining, float flow_rate, float temperature, uint32_t fuel_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FUEL_STATUS_LEN];
    _mav_put_float(buf, 0, maximum_fuel);
    _mav_put_float(buf, 4, consumed_fuel);
    _mav_put_float(buf, 8, remaining_fuel);
    _mav_put_float(buf, 12, flow_rate);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint32_t(buf, 20, fuel_type);
    _mav_put_uint8_t(buf, 24, id);
    _mav_put_uint8_t(buf, 25, percent_remaining);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FUEL_STATUS_LEN);
#else
    mavlink_fuel_status_t packet;
    packet.maximum_fuel = maximum_fuel;
    packet.consumed_fuel = consumed_fuel;
    packet.remaining_fuel = remaining_fuel;
    packet.flow_rate = flow_rate;
    packet.temperature = temperature;
    packet.fuel_type = fuel_type;
    packet.id = id;
    packet.percent_remaining = percent_remaining;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FUEL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FUEL_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN, MAVLINK_MSG_ID_FUEL_STATUS_LEN, MAVLINK_MSG_ID_FUEL_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN, MAVLINK_MSG_ID_FUEL_STATUS_LEN);
#endif
}

/**
 * @brief Pack a fuel_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Fuel ID. Must match ID of other messages for same fuel system, such as BATTERY_STATUS_V2.
 * @param maximum_fuel  Capacity when full. Must be provided.
 * @param consumed_fuel  Consumed fuel (measured). This value should not be inferred: if not measured set to NaN. NaN: field not provided.
 * @param remaining_fuel  Remaining fuel until empty (measured). The value should not be inferred: if not measured set to NaN. NaN: field not provided.
 * @param percent_remaining [%] Percentage of remaining fuel, relative to full. Values: [0-100], UINT8_MAX: field not provided.
 * @param flow_rate  Positive value when emptying/using, and negative if filling/replacing. NaN: field not provided.
 * @param temperature [K] Fuel temperature. NaN: field not provided.
 * @param fuel_type  Fuel type. Defines units for fuel capacity and consumption fields above.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fuel_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,float maximum_fuel,float consumed_fuel,float remaining_fuel,uint8_t percent_remaining,float flow_rate,float temperature,uint32_t fuel_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FUEL_STATUS_LEN];
    _mav_put_float(buf, 0, maximum_fuel);
    _mav_put_float(buf, 4, consumed_fuel);
    _mav_put_float(buf, 8, remaining_fuel);
    _mav_put_float(buf, 12, flow_rate);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint32_t(buf, 20, fuel_type);
    _mav_put_uint8_t(buf, 24, id);
    _mav_put_uint8_t(buf, 25, percent_remaining);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FUEL_STATUS_LEN);
#else
    mavlink_fuel_status_t packet;
    packet.maximum_fuel = maximum_fuel;
    packet.consumed_fuel = consumed_fuel;
    packet.remaining_fuel = remaining_fuel;
    packet.flow_rate = flow_rate;
    packet.temperature = temperature;
    packet.fuel_type = fuel_type;
    packet.id = id;
    packet.percent_remaining = percent_remaining;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FUEL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FUEL_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN, MAVLINK_MSG_ID_FUEL_STATUS_LEN, MAVLINK_MSG_ID_FUEL_STATUS_CRC);
}

/**
 * @brief Encode a fuel_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fuel_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fuel_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fuel_status_t* fuel_status)
{
    return mavlink_msg_fuel_status_pack(system_id, component_id, msg, fuel_status->id, fuel_status->maximum_fuel, fuel_status->consumed_fuel, fuel_status->remaining_fuel, fuel_status->percent_remaining, fuel_status->flow_rate, fuel_status->temperature, fuel_status->fuel_type);
}

/**
 * @brief Encode a fuel_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fuel_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fuel_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fuel_status_t* fuel_status)
{
    return mavlink_msg_fuel_status_pack_chan(system_id, component_id, chan, msg, fuel_status->id, fuel_status->maximum_fuel, fuel_status->consumed_fuel, fuel_status->remaining_fuel, fuel_status->percent_remaining, fuel_status->flow_rate, fuel_status->temperature, fuel_status->fuel_type);
}

/**
 * @brief Encode a fuel_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param fuel_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fuel_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_fuel_status_t* fuel_status)
{
    return mavlink_msg_fuel_status_pack_status(system_id, component_id, _status, msg,  fuel_status->id, fuel_status->maximum_fuel, fuel_status->consumed_fuel, fuel_status->remaining_fuel, fuel_status->percent_remaining, fuel_status->flow_rate, fuel_status->temperature, fuel_status->fuel_type);
}

/**
 * @brief Send a fuel_status message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Fuel ID. Must match ID of other messages for same fuel system, such as BATTERY_STATUS_V2.
 * @param maximum_fuel  Capacity when full. Must be provided.
 * @param consumed_fuel  Consumed fuel (measured). This value should not be inferred: if not measured set to NaN. NaN: field not provided.
 * @param remaining_fuel  Remaining fuel until empty (measured). The value should not be inferred: if not measured set to NaN. NaN: field not provided.
 * @param percent_remaining [%] Percentage of remaining fuel, relative to full. Values: [0-100], UINT8_MAX: field not provided.
 * @param flow_rate  Positive value when emptying/using, and negative if filling/replacing. NaN: field not provided.
 * @param temperature [K] Fuel temperature. NaN: field not provided.
 * @param fuel_type  Fuel type. Defines units for fuel capacity and consumption fields above.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fuel_status_send(mavlink_channel_t chan, uint8_t id, float maximum_fuel, float consumed_fuel, float remaining_fuel, uint8_t percent_remaining, float flow_rate, float temperature, uint32_t fuel_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FUEL_STATUS_LEN];
    _mav_put_float(buf, 0, maximum_fuel);
    _mav_put_float(buf, 4, consumed_fuel);
    _mav_put_float(buf, 8, remaining_fuel);
    _mav_put_float(buf, 12, flow_rate);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint32_t(buf, 20, fuel_type);
    _mav_put_uint8_t(buf, 24, id);
    _mav_put_uint8_t(buf, 25, percent_remaining);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FUEL_STATUS, buf, MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN, MAVLINK_MSG_ID_FUEL_STATUS_LEN, MAVLINK_MSG_ID_FUEL_STATUS_CRC);
#else
    mavlink_fuel_status_t packet;
    packet.maximum_fuel = maximum_fuel;
    packet.consumed_fuel = consumed_fuel;
    packet.remaining_fuel = remaining_fuel;
    packet.flow_rate = flow_rate;
    packet.temperature = temperature;
    packet.fuel_type = fuel_type;
    packet.id = id;
    packet.percent_remaining = percent_remaining;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FUEL_STATUS, (const char *)&packet, MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN, MAVLINK_MSG_ID_FUEL_STATUS_LEN, MAVLINK_MSG_ID_FUEL_STATUS_CRC);
#endif
}

/**
 * @brief Send a fuel_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fuel_status_send_struct(mavlink_channel_t chan, const mavlink_fuel_status_t* fuel_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fuel_status_send(chan, fuel_status->id, fuel_status->maximum_fuel, fuel_status->consumed_fuel, fuel_status->remaining_fuel, fuel_status->percent_remaining, fuel_status->flow_rate, fuel_status->temperature, fuel_status->fuel_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FUEL_STATUS, (const char *)fuel_status, MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN, MAVLINK_MSG_ID_FUEL_STATUS_LEN, MAVLINK_MSG_ID_FUEL_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_FUEL_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fuel_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, float maximum_fuel, float consumed_fuel, float remaining_fuel, uint8_t percent_remaining, float flow_rate, float temperature, uint32_t fuel_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, maximum_fuel);
    _mav_put_float(buf, 4, consumed_fuel);
    _mav_put_float(buf, 8, remaining_fuel);
    _mav_put_float(buf, 12, flow_rate);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint32_t(buf, 20, fuel_type);
    _mav_put_uint8_t(buf, 24, id);
    _mav_put_uint8_t(buf, 25, percent_remaining);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FUEL_STATUS, buf, MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN, MAVLINK_MSG_ID_FUEL_STATUS_LEN, MAVLINK_MSG_ID_FUEL_STATUS_CRC);
#else
    mavlink_fuel_status_t *packet = (mavlink_fuel_status_t *)msgbuf;
    packet->maximum_fuel = maximum_fuel;
    packet->consumed_fuel = consumed_fuel;
    packet->remaining_fuel = remaining_fuel;
    packet->flow_rate = flow_rate;
    packet->temperature = temperature;
    packet->fuel_type = fuel_type;
    packet->id = id;
    packet->percent_remaining = percent_remaining;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FUEL_STATUS, (const char *)packet, MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN, MAVLINK_MSG_ID_FUEL_STATUS_LEN, MAVLINK_MSG_ID_FUEL_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE FUEL_STATUS UNPACKING


/**
 * @brief Get field id from fuel_status message
 *
 * @return  Fuel ID. Must match ID of other messages for same fuel system, such as BATTERY_STATUS_V2.
 */
static inline uint8_t mavlink_msg_fuel_status_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field maximum_fuel from fuel_status message
 *
 * @return  Capacity when full. Must be provided.
 */
static inline float mavlink_msg_fuel_status_get_maximum_fuel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field consumed_fuel from fuel_status message
 *
 * @return  Consumed fuel (measured). This value should not be inferred: if not measured set to NaN. NaN: field not provided.
 */
static inline float mavlink_msg_fuel_status_get_consumed_fuel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field remaining_fuel from fuel_status message
 *
 * @return  Remaining fuel until empty (measured). The value should not be inferred: if not measured set to NaN. NaN: field not provided.
 */
static inline float mavlink_msg_fuel_status_get_remaining_fuel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field percent_remaining from fuel_status message
 *
 * @return [%] Percentage of remaining fuel, relative to full. Values: [0-100], UINT8_MAX: field not provided.
 */
static inline uint8_t mavlink_msg_fuel_status_get_percent_remaining(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field flow_rate from fuel_status message
 *
 * @return  Positive value when emptying/using, and negative if filling/replacing. NaN: field not provided.
 */
static inline float mavlink_msg_fuel_status_get_flow_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field temperature from fuel_status message
 *
 * @return [K] Fuel temperature. NaN: field not provided.
 */
static inline float mavlink_msg_fuel_status_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field fuel_type from fuel_status message
 *
 * @return  Fuel type. Defines units for fuel capacity and consumption fields above.
 */
static inline uint32_t mavlink_msg_fuel_status_get_fuel_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Decode a fuel_status message into a struct
 *
 * @param msg The message to decode
 * @param fuel_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_fuel_status_decode(const mavlink_message_t* msg, mavlink_fuel_status_t* fuel_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fuel_status->maximum_fuel = mavlink_msg_fuel_status_get_maximum_fuel(msg);
    fuel_status->consumed_fuel = mavlink_msg_fuel_status_get_consumed_fuel(msg);
    fuel_status->remaining_fuel = mavlink_msg_fuel_status_get_remaining_fuel(msg);
    fuel_status->flow_rate = mavlink_msg_fuel_status_get_flow_rate(msg);
    fuel_status->temperature = mavlink_msg_fuel_status_get_temperature(msg);
    fuel_status->fuel_type = mavlink_msg_fuel_status_get_fuel_type(msg);
    fuel_status->id = mavlink_msg_fuel_status_get_id(msg);
    fuel_status->percent_remaining = mavlink_msg_fuel_status_get_percent_remaining(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FUEL_STATUS_LEN? msg->len : MAVLINK_MSG_ID_FUEL_STATUS_LEN;
        memset(fuel_status, 0, MAVLINK_MSG_ID_FUEL_STATUS_LEN);
    memcpy(fuel_status, _MAV_PAYLOAD(msg), len);
#endif
}
