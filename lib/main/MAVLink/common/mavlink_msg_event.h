#pragma once
// MESSAGE EVENT PACKING

#define MAVLINK_MSG_ID_EVENT 410


typedef struct __mavlink_event_t {
 uint32_t id; /*<  Event ID (as defined in the component metadata)*/
 uint32_t event_time_boot_ms; /*< [ms] Timestamp (time since system boot when the event happened).*/
 uint16_t sequence; /*<  Sequence number.*/
 uint8_t destination_component; /*<  Component ID*/
 uint8_t destination_system; /*<  System ID*/
 uint8_t log_levels; /*<  Log levels: 4 bits MSB: internal (for logging purposes), 4 bits LSB: external. Levels: Emergency = 0, Alert = 1, Critical = 2, Error = 3, Warning = 4, Notice = 5, Info = 6, Debug = 7, Protocol = 8, Disabled = 9*/
 uint8_t arguments[40]; /*<  Arguments (depend on event ID).*/
} mavlink_event_t;

#define MAVLINK_MSG_ID_EVENT_LEN 53
#define MAVLINK_MSG_ID_EVENT_MIN_LEN 53
#define MAVLINK_MSG_ID_410_LEN 53
#define MAVLINK_MSG_ID_410_MIN_LEN 53

#define MAVLINK_MSG_ID_EVENT_CRC 160
#define MAVLINK_MSG_ID_410_CRC 160

#define MAVLINK_MSG_EVENT_FIELD_ARGUMENTS_LEN 40

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_EVENT { \
    410, \
    "EVENT", \
    7, \
    {  { "destination_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_event_t, destination_component) }, \
         { "destination_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_event_t, destination_system) }, \
         { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_event_t, id) }, \
         { "event_time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_event_t, event_time_boot_ms) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_event_t, sequence) }, \
         { "log_levels", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_event_t, log_levels) }, \
         { "arguments", NULL, MAVLINK_TYPE_UINT8_T, 40, 13, offsetof(mavlink_event_t, arguments) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_EVENT { \
    "EVENT", \
    7, \
    {  { "destination_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_event_t, destination_component) }, \
         { "destination_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_event_t, destination_system) }, \
         { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_event_t, id) }, \
         { "event_time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_event_t, event_time_boot_ms) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_event_t, sequence) }, \
         { "log_levels", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_event_t, log_levels) }, \
         { "arguments", NULL, MAVLINK_TYPE_UINT8_T, 40, 13, offsetof(mavlink_event_t, arguments) }, \
         } \
}
#endif

/**
 * @brief Pack a event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param destination_component  Component ID
 * @param destination_system  System ID
 * @param id  Event ID (as defined in the component metadata)
 * @param event_time_boot_ms [ms] Timestamp (time since system boot when the event happened).
 * @param sequence  Sequence number.
 * @param log_levels  Log levels: 4 bits MSB: internal (for logging purposes), 4 bits LSB: external. Levels: Emergency = 0, Alert = 1, Critical = 2, Error = 3, Warning = 4, Notice = 5, Info = 6, Debug = 7, Protocol = 8, Disabled = 9
 * @param arguments  Arguments (depend on event ID).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_event_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t destination_component, uint8_t destination_system, uint32_t id, uint32_t event_time_boot_ms, uint16_t sequence, uint8_t log_levels, const uint8_t *arguments)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EVENT_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint32_t(buf, 4, event_time_boot_ms);
    _mav_put_uint16_t(buf, 8, sequence);
    _mav_put_uint8_t(buf, 10, destination_component);
    _mav_put_uint8_t(buf, 11, destination_system);
    _mav_put_uint8_t(buf, 12, log_levels);
    _mav_put_uint8_t_array(buf, 13, arguments, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EVENT_LEN);
#else
    mavlink_event_t packet;
    packet.id = id;
    packet.event_time_boot_ms = event_time_boot_ms;
    packet.sequence = sequence;
    packet.destination_component = destination_component;
    packet.destination_system = destination_system;
    packet.log_levels = log_levels;
    mav_array_assign_uint8_t(packet.arguments, arguments, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EVENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EVENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EVENT_MIN_LEN, MAVLINK_MSG_ID_EVENT_LEN, MAVLINK_MSG_ID_EVENT_CRC);
}

/**
 * @brief Pack a event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param destination_component  Component ID
 * @param destination_system  System ID
 * @param id  Event ID (as defined in the component metadata)
 * @param event_time_boot_ms [ms] Timestamp (time since system boot when the event happened).
 * @param sequence  Sequence number.
 * @param log_levels  Log levels: 4 bits MSB: internal (for logging purposes), 4 bits LSB: external. Levels: Emergency = 0, Alert = 1, Critical = 2, Error = 3, Warning = 4, Notice = 5, Info = 6, Debug = 7, Protocol = 8, Disabled = 9
 * @param arguments  Arguments (depend on event ID).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_event_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t destination_component, uint8_t destination_system, uint32_t id, uint32_t event_time_boot_ms, uint16_t sequence, uint8_t log_levels, const uint8_t *arguments)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EVENT_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint32_t(buf, 4, event_time_boot_ms);
    _mav_put_uint16_t(buf, 8, sequence);
    _mav_put_uint8_t(buf, 10, destination_component);
    _mav_put_uint8_t(buf, 11, destination_system);
    _mav_put_uint8_t(buf, 12, log_levels);
    _mav_put_uint8_t_array(buf, 13, arguments, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EVENT_LEN);
#else
    mavlink_event_t packet;
    packet.id = id;
    packet.event_time_boot_ms = event_time_boot_ms;
    packet.sequence = sequence;
    packet.destination_component = destination_component;
    packet.destination_system = destination_system;
    packet.log_levels = log_levels;
    mav_array_memcpy(packet.arguments, arguments, sizeof(uint8_t)*40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EVENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EVENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EVENT_MIN_LEN, MAVLINK_MSG_ID_EVENT_LEN, MAVLINK_MSG_ID_EVENT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EVENT_MIN_LEN, MAVLINK_MSG_ID_EVENT_LEN);
#endif
}

/**
 * @brief Pack a event message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param destination_component  Component ID
 * @param destination_system  System ID
 * @param id  Event ID (as defined in the component metadata)
 * @param event_time_boot_ms [ms] Timestamp (time since system boot when the event happened).
 * @param sequence  Sequence number.
 * @param log_levels  Log levels: 4 bits MSB: internal (for logging purposes), 4 bits LSB: external. Levels: Emergency = 0, Alert = 1, Critical = 2, Error = 3, Warning = 4, Notice = 5, Info = 6, Debug = 7, Protocol = 8, Disabled = 9
 * @param arguments  Arguments (depend on event ID).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_event_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t destination_component,uint8_t destination_system,uint32_t id,uint32_t event_time_boot_ms,uint16_t sequence,uint8_t log_levels,const uint8_t *arguments)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EVENT_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint32_t(buf, 4, event_time_boot_ms);
    _mav_put_uint16_t(buf, 8, sequence);
    _mav_put_uint8_t(buf, 10, destination_component);
    _mav_put_uint8_t(buf, 11, destination_system);
    _mav_put_uint8_t(buf, 12, log_levels);
    _mav_put_uint8_t_array(buf, 13, arguments, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EVENT_LEN);
#else
    mavlink_event_t packet;
    packet.id = id;
    packet.event_time_boot_ms = event_time_boot_ms;
    packet.sequence = sequence;
    packet.destination_component = destination_component;
    packet.destination_system = destination_system;
    packet.log_levels = log_levels;
    mav_array_assign_uint8_t(packet.arguments, arguments, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EVENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EVENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EVENT_MIN_LEN, MAVLINK_MSG_ID_EVENT_LEN, MAVLINK_MSG_ID_EVENT_CRC);
}

/**
 * @brief Encode a event struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_event_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_event_t* event)
{
    return mavlink_msg_event_pack(system_id, component_id, msg, event->destination_component, event->destination_system, event->id, event->event_time_boot_ms, event->sequence, event->log_levels, event->arguments);
}

/**
 * @brief Encode a event struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_event_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_event_t* event)
{
    return mavlink_msg_event_pack_chan(system_id, component_id, chan, msg, event->destination_component, event->destination_system, event->id, event->event_time_boot_ms, event->sequence, event->log_levels, event->arguments);
}

/**
 * @brief Encode a event struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_event_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_event_t* event)
{
    return mavlink_msg_event_pack_status(system_id, component_id, _status, msg,  event->destination_component, event->destination_system, event->id, event->event_time_boot_ms, event->sequence, event->log_levels, event->arguments);
}

/**
 * @brief Send a event message
 * @param chan MAVLink channel to send the message
 *
 * @param destination_component  Component ID
 * @param destination_system  System ID
 * @param id  Event ID (as defined in the component metadata)
 * @param event_time_boot_ms [ms] Timestamp (time since system boot when the event happened).
 * @param sequence  Sequence number.
 * @param log_levels  Log levels: 4 bits MSB: internal (for logging purposes), 4 bits LSB: external. Levels: Emergency = 0, Alert = 1, Critical = 2, Error = 3, Warning = 4, Notice = 5, Info = 6, Debug = 7, Protocol = 8, Disabled = 9
 * @param arguments  Arguments (depend on event ID).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_event_send(mavlink_channel_t chan, uint8_t destination_component, uint8_t destination_system, uint32_t id, uint32_t event_time_boot_ms, uint16_t sequence, uint8_t log_levels, const uint8_t *arguments)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EVENT_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint32_t(buf, 4, event_time_boot_ms);
    _mav_put_uint16_t(buf, 8, sequence);
    _mav_put_uint8_t(buf, 10, destination_component);
    _mav_put_uint8_t(buf, 11, destination_system);
    _mav_put_uint8_t(buf, 12, log_levels);
    _mav_put_uint8_t_array(buf, 13, arguments, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EVENT, buf, MAVLINK_MSG_ID_EVENT_MIN_LEN, MAVLINK_MSG_ID_EVENT_LEN, MAVLINK_MSG_ID_EVENT_CRC);
#else
    mavlink_event_t packet;
    packet.id = id;
    packet.event_time_boot_ms = event_time_boot_ms;
    packet.sequence = sequence;
    packet.destination_component = destination_component;
    packet.destination_system = destination_system;
    packet.log_levels = log_levels;
    mav_array_assign_uint8_t(packet.arguments, arguments, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EVENT, (const char *)&packet, MAVLINK_MSG_ID_EVENT_MIN_LEN, MAVLINK_MSG_ID_EVENT_LEN, MAVLINK_MSG_ID_EVENT_CRC);
#endif
}

/**
 * @brief Send a event message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_event_send_struct(mavlink_channel_t chan, const mavlink_event_t* event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_event_send(chan, event->destination_component, event->destination_system, event->id, event->event_time_boot_ms, event->sequence, event->log_levels, event->arguments);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EVENT, (const char *)event, MAVLINK_MSG_ID_EVENT_MIN_LEN, MAVLINK_MSG_ID_EVENT_LEN, MAVLINK_MSG_ID_EVENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_EVENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_event_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t destination_component, uint8_t destination_system, uint32_t id, uint32_t event_time_boot_ms, uint16_t sequence, uint8_t log_levels, const uint8_t *arguments)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint32_t(buf, 4, event_time_boot_ms);
    _mav_put_uint16_t(buf, 8, sequence);
    _mav_put_uint8_t(buf, 10, destination_component);
    _mav_put_uint8_t(buf, 11, destination_system);
    _mav_put_uint8_t(buf, 12, log_levels);
    _mav_put_uint8_t_array(buf, 13, arguments, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EVENT, buf, MAVLINK_MSG_ID_EVENT_MIN_LEN, MAVLINK_MSG_ID_EVENT_LEN, MAVLINK_MSG_ID_EVENT_CRC);
#else
    mavlink_event_t *packet = (mavlink_event_t *)msgbuf;
    packet->id = id;
    packet->event_time_boot_ms = event_time_boot_ms;
    packet->sequence = sequence;
    packet->destination_component = destination_component;
    packet->destination_system = destination_system;
    packet->log_levels = log_levels;
    mav_array_assign_uint8_t(packet->arguments, arguments, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EVENT, (const char *)packet, MAVLINK_MSG_ID_EVENT_MIN_LEN, MAVLINK_MSG_ID_EVENT_LEN, MAVLINK_MSG_ID_EVENT_CRC);
#endif
}
#endif

#endif

// MESSAGE EVENT UNPACKING


/**
 * @brief Get field destination_component from event message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_event_get_destination_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field destination_system from event message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_event_get_destination_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field id from event message
 *
 * @return  Event ID (as defined in the component metadata)
 */
static inline uint32_t mavlink_msg_event_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field event_time_boot_ms from event message
 *
 * @return [ms] Timestamp (time since system boot when the event happened).
 */
static inline uint32_t mavlink_msg_event_get_event_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field sequence from event message
 *
 * @return  Sequence number.
 */
static inline uint16_t mavlink_msg_event_get_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field log_levels from event message
 *
 * @return  Log levels: 4 bits MSB: internal (for logging purposes), 4 bits LSB: external. Levels: Emergency = 0, Alert = 1, Critical = 2, Error = 3, Warning = 4, Notice = 5, Info = 6, Debug = 7, Protocol = 8, Disabled = 9
 */
static inline uint8_t mavlink_msg_event_get_log_levels(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field arguments from event message
 *
 * @return  Arguments (depend on event ID).
 */
static inline uint16_t mavlink_msg_event_get_arguments(const mavlink_message_t* msg, uint8_t *arguments)
{
    return _MAV_RETURN_uint8_t_array(msg, arguments, 40,  13);
}

/**
 * @brief Decode a event message into a struct
 *
 * @param msg The message to decode
 * @param event C-struct to decode the message contents into
 */
static inline void mavlink_msg_event_decode(const mavlink_message_t* msg, mavlink_event_t* event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    event->id = mavlink_msg_event_get_id(msg);
    event->event_time_boot_ms = mavlink_msg_event_get_event_time_boot_ms(msg);
    event->sequence = mavlink_msg_event_get_sequence(msg);
    event->destination_component = mavlink_msg_event_get_destination_component(msg);
    event->destination_system = mavlink_msg_event_get_destination_system(msg);
    event->log_levels = mavlink_msg_event_get_log_levels(msg);
    mavlink_msg_event_get_arguments(msg, event->arguments);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_EVENT_LEN? msg->len : MAVLINK_MSG_ID_EVENT_LEN;
        memset(event, 0, MAVLINK_MSG_ID_EVENT_LEN);
    memcpy(event, _MAV_PAYLOAD(msg), len);
#endif
}
