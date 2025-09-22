#pragma once
// MESSAGE TIME_ESTIMATE_TO_TARGET PACKING

#define MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET 380


typedef struct __mavlink_time_estimate_to_target_t {
 int32_t safe_return; /*< [s] Estimated time to complete the vehicle's configured "safe return" action from its current position (e.g. RTL, Smart RTL, etc.). -1 indicates that the vehicle is landed, or that no time estimate available.*/
 int32_t land; /*< [s] Estimated time for vehicle to complete the LAND action from its current position. -1 indicates that the vehicle is landed, or that no time estimate available.*/
 int32_t mission_next_item; /*< [s] Estimated time for reaching/completing the currently active mission item. -1 means no time estimate available.*/
 int32_t mission_end; /*< [s] Estimated time for completing the current mission. -1 means no mission active and/or no estimate available.*/
 int32_t commanded_action; /*< [s] Estimated time for completing the current commanded action (i.e. Go To, Takeoff, Land, etc.). -1 means no action active and/or no estimate available.*/
} mavlink_time_estimate_to_target_t;

#define MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN 20
#define MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN 20
#define MAVLINK_MSG_ID_380_LEN 20
#define MAVLINK_MSG_ID_380_MIN_LEN 20

#define MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC 232
#define MAVLINK_MSG_ID_380_CRC 232



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TIME_ESTIMATE_TO_TARGET { \
    380, \
    "TIME_ESTIMATE_TO_TARGET", \
    5, \
    {  { "safe_return", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_time_estimate_to_target_t, safe_return) }, \
         { "land", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_time_estimate_to_target_t, land) }, \
         { "mission_next_item", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_time_estimate_to_target_t, mission_next_item) }, \
         { "mission_end", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_time_estimate_to_target_t, mission_end) }, \
         { "commanded_action", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_time_estimate_to_target_t, commanded_action) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TIME_ESTIMATE_TO_TARGET { \
    "TIME_ESTIMATE_TO_TARGET", \
    5, \
    {  { "safe_return", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_time_estimate_to_target_t, safe_return) }, \
         { "land", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_time_estimate_to_target_t, land) }, \
         { "mission_next_item", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_time_estimate_to_target_t, mission_next_item) }, \
         { "mission_end", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_time_estimate_to_target_t, mission_end) }, \
         { "commanded_action", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_time_estimate_to_target_t, commanded_action) }, \
         } \
}
#endif

/**
 * @brief Pack a time_estimate_to_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param safe_return [s] Estimated time to complete the vehicle's configured "safe return" action from its current position (e.g. RTL, Smart RTL, etc.). -1 indicates that the vehicle is landed, or that no time estimate available.
 * @param land [s] Estimated time for vehicle to complete the LAND action from its current position. -1 indicates that the vehicle is landed, or that no time estimate available.
 * @param mission_next_item [s] Estimated time for reaching/completing the currently active mission item. -1 means no time estimate available.
 * @param mission_end [s] Estimated time for completing the current mission. -1 means no mission active and/or no estimate available.
 * @param commanded_action [s] Estimated time for completing the current commanded action (i.e. Go To, Takeoff, Land, etc.). -1 means no action active and/or no estimate available.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_time_estimate_to_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t safe_return, int32_t land, int32_t mission_next_item, int32_t mission_end, int32_t commanded_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN];
    _mav_put_int32_t(buf, 0, safe_return);
    _mav_put_int32_t(buf, 4, land);
    _mav_put_int32_t(buf, 8, mission_next_item);
    _mav_put_int32_t(buf, 12, mission_end);
    _mav_put_int32_t(buf, 16, commanded_action);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN);
#else
    mavlink_time_estimate_to_target_t packet;
    packet.safe_return = safe_return;
    packet.land = land;
    packet.mission_next_item = mission_next_item;
    packet.mission_end = mission_end;
    packet.commanded_action = commanded_action;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC);
}

/**
 * @brief Pack a time_estimate_to_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param safe_return [s] Estimated time to complete the vehicle's configured "safe return" action from its current position (e.g. RTL, Smart RTL, etc.). -1 indicates that the vehicle is landed, or that no time estimate available.
 * @param land [s] Estimated time for vehicle to complete the LAND action from its current position. -1 indicates that the vehicle is landed, or that no time estimate available.
 * @param mission_next_item [s] Estimated time for reaching/completing the currently active mission item. -1 means no time estimate available.
 * @param mission_end [s] Estimated time for completing the current mission. -1 means no mission active and/or no estimate available.
 * @param commanded_action [s] Estimated time for completing the current commanded action (i.e. Go To, Takeoff, Land, etc.). -1 means no action active and/or no estimate available.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_time_estimate_to_target_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int32_t safe_return, int32_t land, int32_t mission_next_item, int32_t mission_end, int32_t commanded_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN];
    _mav_put_int32_t(buf, 0, safe_return);
    _mav_put_int32_t(buf, 4, land);
    _mav_put_int32_t(buf, 8, mission_next_item);
    _mav_put_int32_t(buf, 12, mission_end);
    _mav_put_int32_t(buf, 16, commanded_action);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN);
#else
    mavlink_time_estimate_to_target_t packet;
    packet.safe_return = safe_return;
    packet.land = land;
    packet.mission_next_item = mission_next_item;
    packet.mission_end = mission_end;
    packet.commanded_action = commanded_action;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN);
#endif
}

/**
 * @brief Pack a time_estimate_to_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param safe_return [s] Estimated time to complete the vehicle's configured "safe return" action from its current position (e.g. RTL, Smart RTL, etc.). -1 indicates that the vehicle is landed, or that no time estimate available.
 * @param land [s] Estimated time for vehicle to complete the LAND action from its current position. -1 indicates that the vehicle is landed, or that no time estimate available.
 * @param mission_next_item [s] Estimated time for reaching/completing the currently active mission item. -1 means no time estimate available.
 * @param mission_end [s] Estimated time for completing the current mission. -1 means no mission active and/or no estimate available.
 * @param commanded_action [s] Estimated time for completing the current commanded action (i.e. Go To, Takeoff, Land, etc.). -1 means no action active and/or no estimate available.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_time_estimate_to_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t safe_return,int32_t land,int32_t mission_next_item,int32_t mission_end,int32_t commanded_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN];
    _mav_put_int32_t(buf, 0, safe_return);
    _mav_put_int32_t(buf, 4, land);
    _mav_put_int32_t(buf, 8, mission_next_item);
    _mav_put_int32_t(buf, 12, mission_end);
    _mav_put_int32_t(buf, 16, commanded_action);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN);
#else
    mavlink_time_estimate_to_target_t packet;
    packet.safe_return = safe_return;
    packet.land = land;
    packet.mission_next_item = mission_next_item;
    packet.mission_end = mission_end;
    packet.commanded_action = commanded_action;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC);
}

/**
 * @brief Encode a time_estimate_to_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param time_estimate_to_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_time_estimate_to_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_time_estimate_to_target_t* time_estimate_to_target)
{
    return mavlink_msg_time_estimate_to_target_pack(system_id, component_id, msg, time_estimate_to_target->safe_return, time_estimate_to_target->land, time_estimate_to_target->mission_next_item, time_estimate_to_target->mission_end, time_estimate_to_target->commanded_action);
}

/**
 * @brief Encode a time_estimate_to_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_estimate_to_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_time_estimate_to_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_time_estimate_to_target_t* time_estimate_to_target)
{
    return mavlink_msg_time_estimate_to_target_pack_chan(system_id, component_id, chan, msg, time_estimate_to_target->safe_return, time_estimate_to_target->land, time_estimate_to_target->mission_next_item, time_estimate_to_target->mission_end, time_estimate_to_target->commanded_action);
}

/**
 * @brief Encode a time_estimate_to_target struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param time_estimate_to_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_time_estimate_to_target_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_time_estimate_to_target_t* time_estimate_to_target)
{
    return mavlink_msg_time_estimate_to_target_pack_status(system_id, component_id, _status, msg,  time_estimate_to_target->safe_return, time_estimate_to_target->land, time_estimate_to_target->mission_next_item, time_estimate_to_target->mission_end, time_estimate_to_target->commanded_action);
}

/**
 * @brief Send a time_estimate_to_target message
 * @param chan MAVLink channel to send the message
 *
 * @param safe_return [s] Estimated time to complete the vehicle's configured "safe return" action from its current position (e.g. RTL, Smart RTL, etc.). -1 indicates that the vehicle is landed, or that no time estimate available.
 * @param land [s] Estimated time for vehicle to complete the LAND action from its current position. -1 indicates that the vehicle is landed, or that no time estimate available.
 * @param mission_next_item [s] Estimated time for reaching/completing the currently active mission item. -1 means no time estimate available.
 * @param mission_end [s] Estimated time for completing the current mission. -1 means no mission active and/or no estimate available.
 * @param commanded_action [s] Estimated time for completing the current commanded action (i.e. Go To, Takeoff, Land, etc.). -1 means no action active and/or no estimate available.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_time_estimate_to_target_send(mavlink_channel_t chan, int32_t safe_return, int32_t land, int32_t mission_next_item, int32_t mission_end, int32_t commanded_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN];
    _mav_put_int32_t(buf, 0, safe_return);
    _mav_put_int32_t(buf, 4, land);
    _mav_put_int32_t(buf, 8, mission_next_item);
    _mav_put_int32_t(buf, 12, mission_end);
    _mav_put_int32_t(buf, 16, commanded_action);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET, buf, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC);
#else
    mavlink_time_estimate_to_target_t packet;
    packet.safe_return = safe_return;
    packet.land = land;
    packet.mission_next_item = mission_next_item;
    packet.mission_end = mission_end;
    packet.commanded_action = commanded_action;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET, (const char *)&packet, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC);
#endif
}

/**
 * @brief Send a time_estimate_to_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_time_estimate_to_target_send_struct(mavlink_channel_t chan, const mavlink_time_estimate_to_target_t* time_estimate_to_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_time_estimate_to_target_send(chan, time_estimate_to_target->safe_return, time_estimate_to_target->land, time_estimate_to_target->mission_next_item, time_estimate_to_target->mission_end, time_estimate_to_target->commanded_action);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET, (const char *)time_estimate_to_target, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC);
#endif
}

#if MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_time_estimate_to_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t safe_return, int32_t land, int32_t mission_next_item, int32_t mission_end, int32_t commanded_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, safe_return);
    _mav_put_int32_t(buf, 4, land);
    _mav_put_int32_t(buf, 8, mission_next_item);
    _mav_put_int32_t(buf, 12, mission_end);
    _mav_put_int32_t(buf, 16, commanded_action);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET, buf, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC);
#else
    mavlink_time_estimate_to_target_t *packet = (mavlink_time_estimate_to_target_t *)msgbuf;
    packet->safe_return = safe_return;
    packet->land = land;
    packet->mission_next_item = mission_next_item;
    packet->mission_end = mission_end;
    packet->commanded_action = commanded_action;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET, (const char *)packet, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC);
#endif
}
#endif

#endif

// MESSAGE TIME_ESTIMATE_TO_TARGET UNPACKING


/**
 * @brief Get field safe_return from time_estimate_to_target message
 *
 * @return [s] Estimated time to complete the vehicle's configured "safe return" action from its current position (e.g. RTL, Smart RTL, etc.). -1 indicates that the vehicle is landed, or that no time estimate available.
 */
static inline int32_t mavlink_msg_time_estimate_to_target_get_safe_return(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field land from time_estimate_to_target message
 *
 * @return [s] Estimated time for vehicle to complete the LAND action from its current position. -1 indicates that the vehicle is landed, or that no time estimate available.
 */
static inline int32_t mavlink_msg_time_estimate_to_target_get_land(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field mission_next_item from time_estimate_to_target message
 *
 * @return [s] Estimated time for reaching/completing the currently active mission item. -1 means no time estimate available.
 */
static inline int32_t mavlink_msg_time_estimate_to_target_get_mission_next_item(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field mission_end from time_estimate_to_target message
 *
 * @return [s] Estimated time for completing the current mission. -1 means no mission active and/or no estimate available.
 */
static inline int32_t mavlink_msg_time_estimate_to_target_get_mission_end(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field commanded_action from time_estimate_to_target message
 *
 * @return [s] Estimated time for completing the current commanded action (i.e. Go To, Takeoff, Land, etc.). -1 means no action active and/or no estimate available.
 */
static inline int32_t mavlink_msg_time_estimate_to_target_get_commanded_action(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Decode a time_estimate_to_target message into a struct
 *
 * @param msg The message to decode
 * @param time_estimate_to_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_time_estimate_to_target_decode(const mavlink_message_t* msg, mavlink_time_estimate_to_target_t* time_estimate_to_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    time_estimate_to_target->safe_return = mavlink_msg_time_estimate_to_target_get_safe_return(msg);
    time_estimate_to_target->land = mavlink_msg_time_estimate_to_target_get_land(msg);
    time_estimate_to_target->mission_next_item = mavlink_msg_time_estimate_to_target_get_mission_next_item(msg);
    time_estimate_to_target->mission_end = mavlink_msg_time_estimate_to_target_get_mission_end(msg);
    time_estimate_to_target->commanded_action = mavlink_msg_time_estimate_to_target_get_commanded_action(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN? msg->len : MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN;
        memset(time_estimate_to_target, 0, MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN);
    memcpy(time_estimate_to_target, _MAV_PAYLOAD(msg), len);
#endif
}
