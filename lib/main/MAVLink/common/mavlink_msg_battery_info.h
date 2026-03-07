#pragma once
// MESSAGE BATTERY_INFO PACKING

#define MAVLINK_MSG_ID_BATTERY_INFO 372


typedef struct __mavlink_battery_info_t {
 float discharge_minimum_voltage; /*< [V] Minimum per-cell voltage when discharging. 0: field not provided.*/
 float charging_minimum_voltage; /*< [V] Minimum per-cell voltage when charging. 0: field not provided.*/
 float resting_minimum_voltage; /*< [V] Minimum per-cell voltage when resting. 0: field not provided.*/
 float charging_maximum_voltage; /*< [V] Maximum per-cell voltage when charged. 0: field not provided.*/
 float charging_maximum_current; /*< [A] Maximum pack continuous charge current. 0: field not provided.*/
 float nominal_voltage; /*< [V] Battery nominal voltage. Used for conversion between Wh and Ah. 0: field not provided.*/
 float discharge_maximum_current; /*< [A] Maximum pack discharge current. 0: field not provided.*/
 float discharge_maximum_burst_current; /*< [A] Maximum pack discharge burst current. 0: field not provided.*/
 float design_capacity; /*< [Ah] Fully charged design capacity. 0: field not provided.*/
 float full_charge_capacity; /*< [Ah] Predicted battery capacity when fully charged (accounting for battery degradation). NAN: field not provided.*/
 uint16_t cycle_count; /*<  Lifetime count of the number of charge/discharge cycles (https://en.wikipedia.org/wiki/Charge_cycle). UINT16_MAX: field not provided.*/
 uint16_t weight; /*< [g] Battery weight. 0: field not provided.*/
 uint8_t id; /*<  Battery ID*/
 uint8_t battery_function; /*<  Function of the battery.*/
 uint8_t type; /*<  Type (chemistry) of the battery.*/
 uint8_t state_of_health; /*< [%] State of Health (SOH) estimate. Typically 100% at the time of manufacture and will decrease over time and use. -1: field not provided.*/
 uint8_t cells_in_series; /*<  Number of battery cells in series. 0: field not provided.*/
 char manufacture_date[9]; /*<  Manufacture date (DDMMYYYY) in ASCII characters, 0 terminated. All 0: field not provided.*/
 char serial_number[32]; /*<  Serial number in ASCII characters, 0 terminated. All 0: field not provided.*/
 char name[50]; /*<  Battery device name. Formatted as manufacturer name then product name, separated with an underscore (in ASCII characters), 0 terminated. All 0: field not provided.*/
} mavlink_battery_info_t;

#define MAVLINK_MSG_ID_BATTERY_INFO_LEN 140
#define MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN 140
#define MAVLINK_MSG_ID_372_LEN 140
#define MAVLINK_MSG_ID_372_MIN_LEN 140

#define MAVLINK_MSG_ID_BATTERY_INFO_CRC 26
#define MAVLINK_MSG_ID_372_CRC 26

#define MAVLINK_MSG_BATTERY_INFO_FIELD_MANUFACTURE_DATE_LEN 9
#define MAVLINK_MSG_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN 32
#define MAVLINK_MSG_BATTERY_INFO_FIELD_NAME_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BATTERY_INFO { \
    372, \
    "BATTERY_INFO", \
    20, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_battery_info_t, id) }, \
         { "battery_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_battery_info_t, battery_function) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_battery_info_t, type) }, \
         { "state_of_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_battery_info_t, state_of_health) }, \
         { "cells_in_series", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_battery_info_t, cells_in_series) }, \
         { "cycle_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_battery_info_t, cycle_count) }, \
         { "weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_battery_info_t, weight) }, \
         { "discharge_minimum_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_battery_info_t, discharge_minimum_voltage) }, \
         { "charging_minimum_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_battery_info_t, charging_minimum_voltage) }, \
         { "resting_minimum_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_battery_info_t, resting_minimum_voltage) }, \
         { "charging_maximum_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_battery_info_t, charging_maximum_voltage) }, \
         { "charging_maximum_current", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_battery_info_t, charging_maximum_current) }, \
         { "nominal_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_battery_info_t, nominal_voltage) }, \
         { "discharge_maximum_current", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_battery_info_t, discharge_maximum_current) }, \
         { "discharge_maximum_burst_current", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_battery_info_t, discharge_maximum_burst_current) }, \
         { "design_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_battery_info_t, design_capacity) }, \
         { "full_charge_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_battery_info_t, full_charge_capacity) }, \
         { "manufacture_date", NULL, MAVLINK_TYPE_CHAR, 9, 49, offsetof(mavlink_battery_info_t, manufacture_date) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 32, 58, offsetof(mavlink_battery_info_t, serial_number) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 50, 90, offsetof(mavlink_battery_info_t, name) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BATTERY_INFO { \
    "BATTERY_INFO", \
    20, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_battery_info_t, id) }, \
         { "battery_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_battery_info_t, battery_function) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_battery_info_t, type) }, \
         { "state_of_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_battery_info_t, state_of_health) }, \
         { "cells_in_series", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_battery_info_t, cells_in_series) }, \
         { "cycle_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_battery_info_t, cycle_count) }, \
         { "weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_battery_info_t, weight) }, \
         { "discharge_minimum_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_battery_info_t, discharge_minimum_voltage) }, \
         { "charging_minimum_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_battery_info_t, charging_minimum_voltage) }, \
         { "resting_minimum_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_battery_info_t, resting_minimum_voltage) }, \
         { "charging_maximum_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_battery_info_t, charging_maximum_voltage) }, \
         { "charging_maximum_current", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_battery_info_t, charging_maximum_current) }, \
         { "nominal_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_battery_info_t, nominal_voltage) }, \
         { "discharge_maximum_current", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_battery_info_t, discharge_maximum_current) }, \
         { "discharge_maximum_burst_current", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_battery_info_t, discharge_maximum_burst_current) }, \
         { "design_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_battery_info_t, design_capacity) }, \
         { "full_charge_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_battery_info_t, full_charge_capacity) }, \
         { "manufacture_date", NULL, MAVLINK_TYPE_CHAR, 9, 49, offsetof(mavlink_battery_info_t, manufacture_date) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 32, 58, offsetof(mavlink_battery_info_t, serial_number) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 50, 90, offsetof(mavlink_battery_info_t, name) }, \
         } \
}
#endif

/**
 * @brief Pack a battery_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Battery ID
 * @param battery_function  Function of the battery.
 * @param type  Type (chemistry) of the battery.
 * @param state_of_health [%] State of Health (SOH) estimate. Typically 100% at the time of manufacture and will decrease over time and use. -1: field not provided.
 * @param cells_in_series  Number of battery cells in series. 0: field not provided.
 * @param cycle_count  Lifetime count of the number of charge/discharge cycles (https://en.wikipedia.org/wiki/Charge_cycle). UINT16_MAX: field not provided.
 * @param weight [g] Battery weight. 0: field not provided.
 * @param discharge_minimum_voltage [V] Minimum per-cell voltage when discharging. 0: field not provided.
 * @param charging_minimum_voltage [V] Minimum per-cell voltage when charging. 0: field not provided.
 * @param resting_minimum_voltage [V] Minimum per-cell voltage when resting. 0: field not provided.
 * @param charging_maximum_voltage [V] Maximum per-cell voltage when charged. 0: field not provided.
 * @param charging_maximum_current [A] Maximum pack continuous charge current. 0: field not provided.
 * @param nominal_voltage [V] Battery nominal voltage. Used for conversion between Wh and Ah. 0: field not provided.
 * @param discharge_maximum_current [A] Maximum pack discharge current. 0: field not provided.
 * @param discharge_maximum_burst_current [A] Maximum pack discharge burst current. 0: field not provided.
 * @param design_capacity [Ah] Fully charged design capacity. 0: field not provided.
 * @param full_charge_capacity [Ah] Predicted battery capacity when fully charged (accounting for battery degradation). NAN: field not provided.
 * @param manufacture_date  Manufacture date (DDMMYYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 * @param serial_number  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 * @param name  Battery device name. Formatted as manufacturer name then product name, separated with an underscore (in ASCII characters), 0 terminated. All 0: field not provided.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint8_t battery_function, uint8_t type, uint8_t state_of_health, uint8_t cells_in_series, uint16_t cycle_count, uint16_t weight, float discharge_minimum_voltage, float charging_minimum_voltage, float resting_minimum_voltage, float charging_maximum_voltage, float charging_maximum_current, float nominal_voltage, float discharge_maximum_current, float discharge_maximum_burst_current, float design_capacity, float full_charge_capacity, const char *manufacture_date, const char *serial_number, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_INFO_LEN];
    _mav_put_float(buf, 0, discharge_minimum_voltage);
    _mav_put_float(buf, 4, charging_minimum_voltage);
    _mav_put_float(buf, 8, resting_minimum_voltage);
    _mav_put_float(buf, 12, charging_maximum_voltage);
    _mav_put_float(buf, 16, charging_maximum_current);
    _mav_put_float(buf, 20, nominal_voltage);
    _mav_put_float(buf, 24, discharge_maximum_current);
    _mav_put_float(buf, 28, discharge_maximum_burst_current);
    _mav_put_float(buf, 32, design_capacity);
    _mav_put_float(buf, 36, full_charge_capacity);
    _mav_put_uint16_t(buf, 40, cycle_count);
    _mav_put_uint16_t(buf, 42, weight);
    _mav_put_uint8_t(buf, 44, id);
    _mav_put_uint8_t(buf, 45, battery_function);
    _mav_put_uint8_t(buf, 46, type);
    _mav_put_uint8_t(buf, 47, state_of_health);
    _mav_put_uint8_t(buf, 48, cells_in_series);
    _mav_put_char_array(buf, 49, manufacture_date, 9);
    _mav_put_char_array(buf, 58, serial_number, 32);
    _mav_put_char_array(buf, 90, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_INFO_LEN);
#else
    mavlink_battery_info_t packet;
    packet.discharge_minimum_voltage = discharge_minimum_voltage;
    packet.charging_minimum_voltage = charging_minimum_voltage;
    packet.resting_minimum_voltage = resting_minimum_voltage;
    packet.charging_maximum_voltage = charging_maximum_voltage;
    packet.charging_maximum_current = charging_maximum_current;
    packet.nominal_voltage = nominal_voltage;
    packet.discharge_maximum_current = discharge_maximum_current;
    packet.discharge_maximum_burst_current = discharge_maximum_burst_current;
    packet.design_capacity = design_capacity;
    packet.full_charge_capacity = full_charge_capacity;
    packet.cycle_count = cycle_count;
    packet.weight = weight;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.state_of_health = state_of_health;
    packet.cells_in_series = cells_in_series;
    mav_array_assign_char(packet.manufacture_date, manufacture_date, 9);
    mav_array_assign_char(packet.serial_number, serial_number, 32);
    mav_array_assign_char(packet.name, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_BATTERY_INFO_LEN, MAVLINK_MSG_ID_BATTERY_INFO_CRC);
}

/**
 * @brief Pack a battery_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Battery ID
 * @param battery_function  Function of the battery.
 * @param type  Type (chemistry) of the battery.
 * @param state_of_health [%] State of Health (SOH) estimate. Typically 100% at the time of manufacture and will decrease over time and use. -1: field not provided.
 * @param cells_in_series  Number of battery cells in series. 0: field not provided.
 * @param cycle_count  Lifetime count of the number of charge/discharge cycles (https://en.wikipedia.org/wiki/Charge_cycle). UINT16_MAX: field not provided.
 * @param weight [g] Battery weight. 0: field not provided.
 * @param discharge_minimum_voltage [V] Minimum per-cell voltage when discharging. 0: field not provided.
 * @param charging_minimum_voltage [V] Minimum per-cell voltage when charging. 0: field not provided.
 * @param resting_minimum_voltage [V] Minimum per-cell voltage when resting. 0: field not provided.
 * @param charging_maximum_voltage [V] Maximum per-cell voltage when charged. 0: field not provided.
 * @param charging_maximum_current [A] Maximum pack continuous charge current. 0: field not provided.
 * @param nominal_voltage [V] Battery nominal voltage. Used for conversion between Wh and Ah. 0: field not provided.
 * @param discharge_maximum_current [A] Maximum pack discharge current. 0: field not provided.
 * @param discharge_maximum_burst_current [A] Maximum pack discharge burst current. 0: field not provided.
 * @param design_capacity [Ah] Fully charged design capacity. 0: field not provided.
 * @param full_charge_capacity [Ah] Predicted battery capacity when fully charged (accounting for battery degradation). NAN: field not provided.
 * @param manufacture_date  Manufacture date (DDMMYYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 * @param serial_number  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 * @param name  Battery device name. Formatted as manufacturer name then product name, separated with an underscore (in ASCII characters), 0 terminated. All 0: field not provided.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t id, uint8_t battery_function, uint8_t type, uint8_t state_of_health, uint8_t cells_in_series, uint16_t cycle_count, uint16_t weight, float discharge_minimum_voltage, float charging_minimum_voltage, float resting_minimum_voltage, float charging_maximum_voltage, float charging_maximum_current, float nominal_voltage, float discharge_maximum_current, float discharge_maximum_burst_current, float design_capacity, float full_charge_capacity, const char *manufacture_date, const char *serial_number, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_INFO_LEN];
    _mav_put_float(buf, 0, discharge_minimum_voltage);
    _mav_put_float(buf, 4, charging_minimum_voltage);
    _mav_put_float(buf, 8, resting_minimum_voltage);
    _mav_put_float(buf, 12, charging_maximum_voltage);
    _mav_put_float(buf, 16, charging_maximum_current);
    _mav_put_float(buf, 20, nominal_voltage);
    _mav_put_float(buf, 24, discharge_maximum_current);
    _mav_put_float(buf, 28, discharge_maximum_burst_current);
    _mav_put_float(buf, 32, design_capacity);
    _mav_put_float(buf, 36, full_charge_capacity);
    _mav_put_uint16_t(buf, 40, cycle_count);
    _mav_put_uint16_t(buf, 42, weight);
    _mav_put_uint8_t(buf, 44, id);
    _mav_put_uint8_t(buf, 45, battery_function);
    _mav_put_uint8_t(buf, 46, type);
    _mav_put_uint8_t(buf, 47, state_of_health);
    _mav_put_uint8_t(buf, 48, cells_in_series);
    _mav_put_char_array(buf, 49, manufacture_date, 9);
    _mav_put_char_array(buf, 58, serial_number, 32);
    _mav_put_char_array(buf, 90, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_INFO_LEN);
#else
    mavlink_battery_info_t packet;
    packet.discharge_minimum_voltage = discharge_minimum_voltage;
    packet.charging_minimum_voltage = charging_minimum_voltage;
    packet.resting_minimum_voltage = resting_minimum_voltage;
    packet.charging_maximum_voltage = charging_maximum_voltage;
    packet.charging_maximum_current = charging_maximum_current;
    packet.nominal_voltage = nominal_voltage;
    packet.discharge_maximum_current = discharge_maximum_current;
    packet.discharge_maximum_burst_current = discharge_maximum_burst_current;
    packet.design_capacity = design_capacity;
    packet.full_charge_capacity = full_charge_capacity;
    packet.cycle_count = cycle_count;
    packet.weight = weight;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.state_of_health = state_of_health;
    packet.cells_in_series = cells_in_series;
    mav_array_memcpy(packet.manufacture_date, manufacture_date, sizeof(char)*9);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet.name, name, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_BATTERY_INFO_LEN, MAVLINK_MSG_ID_BATTERY_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_BATTERY_INFO_LEN);
#endif
}

/**
 * @brief Pack a battery_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Battery ID
 * @param battery_function  Function of the battery.
 * @param type  Type (chemistry) of the battery.
 * @param state_of_health [%] State of Health (SOH) estimate. Typically 100% at the time of manufacture and will decrease over time and use. -1: field not provided.
 * @param cells_in_series  Number of battery cells in series. 0: field not provided.
 * @param cycle_count  Lifetime count of the number of charge/discharge cycles (https://en.wikipedia.org/wiki/Charge_cycle). UINT16_MAX: field not provided.
 * @param weight [g] Battery weight. 0: field not provided.
 * @param discharge_minimum_voltage [V] Minimum per-cell voltage when discharging. 0: field not provided.
 * @param charging_minimum_voltage [V] Minimum per-cell voltage when charging. 0: field not provided.
 * @param resting_minimum_voltage [V] Minimum per-cell voltage when resting. 0: field not provided.
 * @param charging_maximum_voltage [V] Maximum per-cell voltage when charged. 0: field not provided.
 * @param charging_maximum_current [A] Maximum pack continuous charge current. 0: field not provided.
 * @param nominal_voltage [V] Battery nominal voltage. Used for conversion between Wh and Ah. 0: field not provided.
 * @param discharge_maximum_current [A] Maximum pack discharge current. 0: field not provided.
 * @param discharge_maximum_burst_current [A] Maximum pack discharge burst current. 0: field not provided.
 * @param design_capacity [Ah] Fully charged design capacity. 0: field not provided.
 * @param full_charge_capacity [Ah] Predicted battery capacity when fully charged (accounting for battery degradation). NAN: field not provided.
 * @param manufacture_date  Manufacture date (DDMMYYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 * @param serial_number  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 * @param name  Battery device name. Formatted as manufacturer name then product name, separated with an underscore (in ASCII characters), 0 terminated. All 0: field not provided.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint8_t battery_function,uint8_t type,uint8_t state_of_health,uint8_t cells_in_series,uint16_t cycle_count,uint16_t weight,float discharge_minimum_voltage,float charging_minimum_voltage,float resting_minimum_voltage,float charging_maximum_voltage,float charging_maximum_current,float nominal_voltage,float discharge_maximum_current,float discharge_maximum_burst_current,float design_capacity,float full_charge_capacity,const char *manufacture_date,const char *serial_number,const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_INFO_LEN];
    _mav_put_float(buf, 0, discharge_minimum_voltage);
    _mav_put_float(buf, 4, charging_minimum_voltage);
    _mav_put_float(buf, 8, resting_minimum_voltage);
    _mav_put_float(buf, 12, charging_maximum_voltage);
    _mav_put_float(buf, 16, charging_maximum_current);
    _mav_put_float(buf, 20, nominal_voltage);
    _mav_put_float(buf, 24, discharge_maximum_current);
    _mav_put_float(buf, 28, discharge_maximum_burst_current);
    _mav_put_float(buf, 32, design_capacity);
    _mav_put_float(buf, 36, full_charge_capacity);
    _mav_put_uint16_t(buf, 40, cycle_count);
    _mav_put_uint16_t(buf, 42, weight);
    _mav_put_uint8_t(buf, 44, id);
    _mav_put_uint8_t(buf, 45, battery_function);
    _mav_put_uint8_t(buf, 46, type);
    _mav_put_uint8_t(buf, 47, state_of_health);
    _mav_put_uint8_t(buf, 48, cells_in_series);
    _mav_put_char_array(buf, 49, manufacture_date, 9);
    _mav_put_char_array(buf, 58, serial_number, 32);
    _mav_put_char_array(buf, 90, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_INFO_LEN);
#else
    mavlink_battery_info_t packet;
    packet.discharge_minimum_voltage = discharge_minimum_voltage;
    packet.charging_minimum_voltage = charging_minimum_voltage;
    packet.resting_minimum_voltage = resting_minimum_voltage;
    packet.charging_maximum_voltage = charging_maximum_voltage;
    packet.charging_maximum_current = charging_maximum_current;
    packet.nominal_voltage = nominal_voltage;
    packet.discharge_maximum_current = discharge_maximum_current;
    packet.discharge_maximum_burst_current = discharge_maximum_burst_current;
    packet.design_capacity = design_capacity;
    packet.full_charge_capacity = full_charge_capacity;
    packet.cycle_count = cycle_count;
    packet.weight = weight;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.state_of_health = state_of_health;
    packet.cells_in_series = cells_in_series;
    mav_array_assign_char(packet.manufacture_date, manufacture_date, 9);
    mav_array_assign_char(packet.serial_number, serial_number, 32);
    mav_array_assign_char(packet.name, name, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_BATTERY_INFO_LEN, MAVLINK_MSG_ID_BATTERY_INFO_CRC);
}

/**
 * @brief Encode a battery_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param battery_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_info_t* battery_info)
{
    return mavlink_msg_battery_info_pack(system_id, component_id, msg, battery_info->id, battery_info->battery_function, battery_info->type, battery_info->state_of_health, battery_info->cells_in_series, battery_info->cycle_count, battery_info->weight, battery_info->discharge_minimum_voltage, battery_info->charging_minimum_voltage, battery_info->resting_minimum_voltage, battery_info->charging_maximum_voltage, battery_info->charging_maximum_current, battery_info->nominal_voltage, battery_info->discharge_maximum_current, battery_info->discharge_maximum_burst_current, battery_info->design_capacity, battery_info->full_charge_capacity, battery_info->manufacture_date, battery_info->serial_number, battery_info->name);
}

/**
 * @brief Encode a battery_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_battery_info_t* battery_info)
{
    return mavlink_msg_battery_info_pack_chan(system_id, component_id, chan, msg, battery_info->id, battery_info->battery_function, battery_info->type, battery_info->state_of_health, battery_info->cells_in_series, battery_info->cycle_count, battery_info->weight, battery_info->discharge_minimum_voltage, battery_info->charging_minimum_voltage, battery_info->resting_minimum_voltage, battery_info->charging_maximum_voltage, battery_info->charging_maximum_current, battery_info->nominal_voltage, battery_info->discharge_maximum_current, battery_info->discharge_maximum_burst_current, battery_info->design_capacity, battery_info->full_charge_capacity, battery_info->manufacture_date, battery_info->serial_number, battery_info->name);
}

/**
 * @brief Encode a battery_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param battery_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_battery_info_t* battery_info)
{
    return mavlink_msg_battery_info_pack_status(system_id, component_id, _status, msg,  battery_info->id, battery_info->battery_function, battery_info->type, battery_info->state_of_health, battery_info->cells_in_series, battery_info->cycle_count, battery_info->weight, battery_info->discharge_minimum_voltage, battery_info->charging_minimum_voltage, battery_info->resting_minimum_voltage, battery_info->charging_maximum_voltage, battery_info->charging_maximum_current, battery_info->nominal_voltage, battery_info->discharge_maximum_current, battery_info->discharge_maximum_burst_current, battery_info->design_capacity, battery_info->full_charge_capacity, battery_info->manufacture_date, battery_info->serial_number, battery_info->name);
}

/**
 * @brief Send a battery_info message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Battery ID
 * @param battery_function  Function of the battery.
 * @param type  Type (chemistry) of the battery.
 * @param state_of_health [%] State of Health (SOH) estimate. Typically 100% at the time of manufacture and will decrease over time and use. -1: field not provided.
 * @param cells_in_series  Number of battery cells in series. 0: field not provided.
 * @param cycle_count  Lifetime count of the number of charge/discharge cycles (https://en.wikipedia.org/wiki/Charge_cycle). UINT16_MAX: field not provided.
 * @param weight [g] Battery weight. 0: field not provided.
 * @param discharge_minimum_voltage [V] Minimum per-cell voltage when discharging. 0: field not provided.
 * @param charging_minimum_voltage [V] Minimum per-cell voltage when charging. 0: field not provided.
 * @param resting_minimum_voltage [V] Minimum per-cell voltage when resting. 0: field not provided.
 * @param charging_maximum_voltage [V] Maximum per-cell voltage when charged. 0: field not provided.
 * @param charging_maximum_current [A] Maximum pack continuous charge current. 0: field not provided.
 * @param nominal_voltage [V] Battery nominal voltage. Used for conversion between Wh and Ah. 0: field not provided.
 * @param discharge_maximum_current [A] Maximum pack discharge current. 0: field not provided.
 * @param discharge_maximum_burst_current [A] Maximum pack discharge burst current. 0: field not provided.
 * @param design_capacity [Ah] Fully charged design capacity. 0: field not provided.
 * @param full_charge_capacity [Ah] Predicted battery capacity when fully charged (accounting for battery degradation). NAN: field not provided.
 * @param manufacture_date  Manufacture date (DDMMYYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 * @param serial_number  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 * @param name  Battery device name. Formatted as manufacturer name then product name, separated with an underscore (in ASCII characters), 0 terminated. All 0: field not provided.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_battery_info_send(mavlink_channel_t chan, uint8_t id, uint8_t battery_function, uint8_t type, uint8_t state_of_health, uint8_t cells_in_series, uint16_t cycle_count, uint16_t weight, float discharge_minimum_voltage, float charging_minimum_voltage, float resting_minimum_voltage, float charging_maximum_voltage, float charging_maximum_current, float nominal_voltage, float discharge_maximum_current, float discharge_maximum_burst_current, float design_capacity, float full_charge_capacity, const char *manufacture_date, const char *serial_number, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_INFO_LEN];
    _mav_put_float(buf, 0, discharge_minimum_voltage);
    _mav_put_float(buf, 4, charging_minimum_voltage);
    _mav_put_float(buf, 8, resting_minimum_voltage);
    _mav_put_float(buf, 12, charging_maximum_voltage);
    _mav_put_float(buf, 16, charging_maximum_current);
    _mav_put_float(buf, 20, nominal_voltage);
    _mav_put_float(buf, 24, discharge_maximum_current);
    _mav_put_float(buf, 28, discharge_maximum_burst_current);
    _mav_put_float(buf, 32, design_capacity);
    _mav_put_float(buf, 36, full_charge_capacity);
    _mav_put_uint16_t(buf, 40, cycle_count);
    _mav_put_uint16_t(buf, 42, weight);
    _mav_put_uint8_t(buf, 44, id);
    _mav_put_uint8_t(buf, 45, battery_function);
    _mav_put_uint8_t(buf, 46, type);
    _mav_put_uint8_t(buf, 47, state_of_health);
    _mav_put_uint8_t(buf, 48, cells_in_series);
    _mav_put_char_array(buf, 49, manufacture_date, 9);
    _mav_put_char_array(buf, 58, serial_number, 32);
    _mav_put_char_array(buf, 90, name, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_INFO, buf, MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_BATTERY_INFO_LEN, MAVLINK_MSG_ID_BATTERY_INFO_CRC);
#else
    mavlink_battery_info_t packet;
    packet.discharge_minimum_voltage = discharge_minimum_voltage;
    packet.charging_minimum_voltage = charging_minimum_voltage;
    packet.resting_minimum_voltage = resting_minimum_voltage;
    packet.charging_maximum_voltage = charging_maximum_voltage;
    packet.charging_maximum_current = charging_maximum_current;
    packet.nominal_voltage = nominal_voltage;
    packet.discharge_maximum_current = discharge_maximum_current;
    packet.discharge_maximum_burst_current = discharge_maximum_burst_current;
    packet.design_capacity = design_capacity;
    packet.full_charge_capacity = full_charge_capacity;
    packet.cycle_count = cycle_count;
    packet.weight = weight;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.state_of_health = state_of_health;
    packet.cells_in_series = cells_in_series;
    mav_array_assign_char(packet.manufacture_date, manufacture_date, 9);
    mav_array_assign_char(packet.serial_number, serial_number, 32);
    mav_array_assign_char(packet.name, name, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_INFO, (const char *)&packet, MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_BATTERY_INFO_LEN, MAVLINK_MSG_ID_BATTERY_INFO_CRC);
#endif
}

/**
 * @brief Send a battery_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_battery_info_send_struct(mavlink_channel_t chan, const mavlink_battery_info_t* battery_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_battery_info_send(chan, battery_info->id, battery_info->battery_function, battery_info->type, battery_info->state_of_health, battery_info->cells_in_series, battery_info->cycle_count, battery_info->weight, battery_info->discharge_minimum_voltage, battery_info->charging_minimum_voltage, battery_info->resting_minimum_voltage, battery_info->charging_maximum_voltage, battery_info->charging_maximum_current, battery_info->nominal_voltage, battery_info->discharge_maximum_current, battery_info->discharge_maximum_burst_current, battery_info->design_capacity, battery_info->full_charge_capacity, battery_info->manufacture_date, battery_info->serial_number, battery_info->name);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_INFO, (const char *)battery_info, MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_BATTERY_INFO_LEN, MAVLINK_MSG_ID_BATTERY_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_BATTERY_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_battery_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint8_t battery_function, uint8_t type, uint8_t state_of_health, uint8_t cells_in_series, uint16_t cycle_count, uint16_t weight, float discharge_minimum_voltage, float charging_minimum_voltage, float resting_minimum_voltage, float charging_maximum_voltage, float charging_maximum_current, float nominal_voltage, float discharge_maximum_current, float discharge_maximum_burst_current, float design_capacity, float full_charge_capacity, const char *manufacture_date, const char *serial_number, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, discharge_minimum_voltage);
    _mav_put_float(buf, 4, charging_minimum_voltage);
    _mav_put_float(buf, 8, resting_minimum_voltage);
    _mav_put_float(buf, 12, charging_maximum_voltage);
    _mav_put_float(buf, 16, charging_maximum_current);
    _mav_put_float(buf, 20, nominal_voltage);
    _mav_put_float(buf, 24, discharge_maximum_current);
    _mav_put_float(buf, 28, discharge_maximum_burst_current);
    _mav_put_float(buf, 32, design_capacity);
    _mav_put_float(buf, 36, full_charge_capacity);
    _mav_put_uint16_t(buf, 40, cycle_count);
    _mav_put_uint16_t(buf, 42, weight);
    _mav_put_uint8_t(buf, 44, id);
    _mav_put_uint8_t(buf, 45, battery_function);
    _mav_put_uint8_t(buf, 46, type);
    _mav_put_uint8_t(buf, 47, state_of_health);
    _mav_put_uint8_t(buf, 48, cells_in_series);
    _mav_put_char_array(buf, 49, manufacture_date, 9);
    _mav_put_char_array(buf, 58, serial_number, 32);
    _mav_put_char_array(buf, 90, name, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_INFO, buf, MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_BATTERY_INFO_LEN, MAVLINK_MSG_ID_BATTERY_INFO_CRC);
#else
    mavlink_battery_info_t *packet = (mavlink_battery_info_t *)msgbuf;
    packet->discharge_minimum_voltage = discharge_minimum_voltage;
    packet->charging_minimum_voltage = charging_minimum_voltage;
    packet->resting_minimum_voltage = resting_minimum_voltage;
    packet->charging_maximum_voltage = charging_maximum_voltage;
    packet->charging_maximum_current = charging_maximum_current;
    packet->nominal_voltage = nominal_voltage;
    packet->discharge_maximum_current = discharge_maximum_current;
    packet->discharge_maximum_burst_current = discharge_maximum_burst_current;
    packet->design_capacity = design_capacity;
    packet->full_charge_capacity = full_charge_capacity;
    packet->cycle_count = cycle_count;
    packet->weight = weight;
    packet->id = id;
    packet->battery_function = battery_function;
    packet->type = type;
    packet->state_of_health = state_of_health;
    packet->cells_in_series = cells_in_series;
    mav_array_assign_char(packet->manufacture_date, manufacture_date, 9);
    mav_array_assign_char(packet->serial_number, serial_number, 32);
    mav_array_assign_char(packet->name, name, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_INFO, (const char *)packet, MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_BATTERY_INFO_LEN, MAVLINK_MSG_ID_BATTERY_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE BATTERY_INFO UNPACKING


/**
 * @brief Get field id from battery_info message
 *
 * @return  Battery ID
 */
static inline uint8_t mavlink_msg_battery_info_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field battery_function from battery_info message
 *
 * @return  Function of the battery.
 */
static inline uint8_t mavlink_msg_battery_info_get_battery_function(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field type from battery_info message
 *
 * @return  Type (chemistry) of the battery.
 */
static inline uint8_t mavlink_msg_battery_info_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field state_of_health from battery_info message
 *
 * @return [%] State of Health (SOH) estimate. Typically 100% at the time of manufacture and will decrease over time and use. -1: field not provided.
 */
static inline uint8_t mavlink_msg_battery_info_get_state_of_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Get field cells_in_series from battery_info message
 *
 * @return  Number of battery cells in series. 0: field not provided.
 */
static inline uint8_t mavlink_msg_battery_info_get_cells_in_series(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field cycle_count from battery_info message
 *
 * @return  Lifetime count of the number of charge/discharge cycles (https://en.wikipedia.org/wiki/Charge_cycle). UINT16_MAX: field not provided.
 */
static inline uint16_t mavlink_msg_battery_info_get_cycle_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field weight from battery_info message
 *
 * @return [g] Battery weight. 0: field not provided.
 */
static inline uint16_t mavlink_msg_battery_info_get_weight(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  42);
}

/**
 * @brief Get field discharge_minimum_voltage from battery_info message
 *
 * @return [V] Minimum per-cell voltage when discharging. 0: field not provided.
 */
static inline float mavlink_msg_battery_info_get_discharge_minimum_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field charging_minimum_voltage from battery_info message
 *
 * @return [V] Minimum per-cell voltage when charging. 0: field not provided.
 */
static inline float mavlink_msg_battery_info_get_charging_minimum_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field resting_minimum_voltage from battery_info message
 *
 * @return [V] Minimum per-cell voltage when resting. 0: field not provided.
 */
static inline float mavlink_msg_battery_info_get_resting_minimum_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field charging_maximum_voltage from battery_info message
 *
 * @return [V] Maximum per-cell voltage when charged. 0: field not provided.
 */
static inline float mavlink_msg_battery_info_get_charging_maximum_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field charging_maximum_current from battery_info message
 *
 * @return [A] Maximum pack continuous charge current. 0: field not provided.
 */
static inline float mavlink_msg_battery_info_get_charging_maximum_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field nominal_voltage from battery_info message
 *
 * @return [V] Battery nominal voltage. Used for conversion between Wh and Ah. 0: field not provided.
 */
static inline float mavlink_msg_battery_info_get_nominal_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field discharge_maximum_current from battery_info message
 *
 * @return [A] Maximum pack discharge current. 0: field not provided.
 */
static inline float mavlink_msg_battery_info_get_discharge_maximum_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field discharge_maximum_burst_current from battery_info message
 *
 * @return [A] Maximum pack discharge burst current. 0: field not provided.
 */
static inline float mavlink_msg_battery_info_get_discharge_maximum_burst_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field design_capacity from battery_info message
 *
 * @return [Ah] Fully charged design capacity. 0: field not provided.
 */
static inline float mavlink_msg_battery_info_get_design_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field full_charge_capacity from battery_info message
 *
 * @return [Ah] Predicted battery capacity when fully charged (accounting for battery degradation). NAN: field not provided.
 */
static inline float mavlink_msg_battery_info_get_full_charge_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field manufacture_date from battery_info message
 *
 * @return  Manufacture date (DDMMYYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 */
static inline uint16_t mavlink_msg_battery_info_get_manufacture_date(const mavlink_message_t* msg, char *manufacture_date)
{
    return _MAV_RETURN_char_array(msg, manufacture_date, 9,  49);
}

/**
 * @brief Get field serial_number from battery_info message
 *
 * @return  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 */
static inline uint16_t mavlink_msg_battery_info_get_serial_number(const mavlink_message_t* msg, char *serial_number)
{
    return _MAV_RETURN_char_array(msg, serial_number, 32,  58);
}

/**
 * @brief Get field name from battery_info message
 *
 * @return  Battery device name. Formatted as manufacturer name then product name, separated with an underscore (in ASCII characters), 0 terminated. All 0: field not provided.
 */
static inline uint16_t mavlink_msg_battery_info_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 50,  90);
}

/**
 * @brief Decode a battery_info message into a struct
 *
 * @param msg The message to decode
 * @param battery_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_battery_info_decode(const mavlink_message_t* msg, mavlink_battery_info_t* battery_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    battery_info->discharge_minimum_voltage = mavlink_msg_battery_info_get_discharge_minimum_voltage(msg);
    battery_info->charging_minimum_voltage = mavlink_msg_battery_info_get_charging_minimum_voltage(msg);
    battery_info->resting_minimum_voltage = mavlink_msg_battery_info_get_resting_minimum_voltage(msg);
    battery_info->charging_maximum_voltage = mavlink_msg_battery_info_get_charging_maximum_voltage(msg);
    battery_info->charging_maximum_current = mavlink_msg_battery_info_get_charging_maximum_current(msg);
    battery_info->nominal_voltage = mavlink_msg_battery_info_get_nominal_voltage(msg);
    battery_info->discharge_maximum_current = mavlink_msg_battery_info_get_discharge_maximum_current(msg);
    battery_info->discharge_maximum_burst_current = mavlink_msg_battery_info_get_discharge_maximum_burst_current(msg);
    battery_info->design_capacity = mavlink_msg_battery_info_get_design_capacity(msg);
    battery_info->full_charge_capacity = mavlink_msg_battery_info_get_full_charge_capacity(msg);
    battery_info->cycle_count = mavlink_msg_battery_info_get_cycle_count(msg);
    battery_info->weight = mavlink_msg_battery_info_get_weight(msg);
    battery_info->id = mavlink_msg_battery_info_get_id(msg);
    battery_info->battery_function = mavlink_msg_battery_info_get_battery_function(msg);
    battery_info->type = mavlink_msg_battery_info_get_type(msg);
    battery_info->state_of_health = mavlink_msg_battery_info_get_state_of_health(msg);
    battery_info->cells_in_series = mavlink_msg_battery_info_get_cells_in_series(msg);
    mavlink_msg_battery_info_get_manufacture_date(msg, battery_info->manufacture_date);
    mavlink_msg_battery_info_get_serial_number(msg, battery_info->serial_number);
    mavlink_msg_battery_info_get_name(msg, battery_info->name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BATTERY_INFO_LEN? msg->len : MAVLINK_MSG_ID_BATTERY_INFO_LEN;
        memset(battery_info, 0, MAVLINK_MSG_ID_BATTERY_INFO_LEN);
    memcpy(battery_info, _MAV_PAYLOAD(msg), len);
#endif
}
