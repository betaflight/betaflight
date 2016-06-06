// MESSAGE SENS_BATMON PACKING

#define MAVLINK_MSG_ID_SENS_BATMON 209

typedef struct __mavlink_sens_batmon_t
{
 uint16_t temperature; ///< Battery pack temperature in [0.1 K]
 uint16_t voltage; ///< Battery pack voltage in [mV]
 uint16_t current; ///< Battery pack current in [mA]
 uint16_t batterystatus; ///< Battery monitor status report bits in Hex
 uint16_t serialnumber; ///< Battery monitor serial number in Hex
 uint16_t hostfetcontrol; ///< Battery monitor sensor host FET control in Hex
 uint16_t cellvoltage1; ///< Battery pack cell 1 voltage in [mV]
 uint16_t cellvoltage2; ///< Battery pack cell 2 voltage in [mV]
 uint16_t cellvoltage3; ///< Battery pack cell 3 voltage in [mV]
 uint16_t cellvoltage4; ///< Battery pack cell 4 voltage in [mV]
 uint16_t cellvoltage5; ///< Battery pack cell 5 voltage in [mV]
 uint16_t cellvoltage6; ///< Battery pack cell 6 voltage in [mV]
} mavlink_sens_batmon_t;

#define MAVLINK_MSG_ID_SENS_BATMON_LEN 24
#define MAVLINK_MSG_ID_209_LEN 24

#define MAVLINK_MSG_ID_SENS_BATMON_CRC 12
#define MAVLINK_MSG_ID_209_CRC 12



#define MAVLINK_MESSAGE_INFO_SENS_BATMON { \
	"SENS_BATMON", \
	12, \
	{  { "temperature", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_sens_batmon_t, temperature) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_sens_batmon_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_sens_batmon_t, current) }, \
         { "batterystatus", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_sens_batmon_t, batterystatus) }, \
         { "serialnumber", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_sens_batmon_t, serialnumber) }, \
         { "hostfetcontrol", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_sens_batmon_t, hostfetcontrol) }, \
         { "cellvoltage1", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_sens_batmon_t, cellvoltage1) }, \
         { "cellvoltage2", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_sens_batmon_t, cellvoltage2) }, \
         { "cellvoltage3", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_sens_batmon_t, cellvoltage3) }, \
         { "cellvoltage4", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_sens_batmon_t, cellvoltage4) }, \
         { "cellvoltage5", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_sens_batmon_t, cellvoltage5) }, \
         { "cellvoltage6", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_sens_batmon_t, cellvoltage6) }, \
         } \
}


/**
 * @brief Pack a sens_batmon message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature Battery pack temperature in [0.1 K]
 * @param voltage Battery pack voltage in [mV]
 * @param current Battery pack current in [mA]
 * @param batterystatus Battery monitor status report bits in Hex
 * @param serialnumber Battery monitor serial number in Hex
 * @param hostfetcontrol Battery monitor sensor host FET control in Hex
 * @param cellvoltage1 Battery pack cell 1 voltage in [mV]
 * @param cellvoltage2 Battery pack cell 2 voltage in [mV]
 * @param cellvoltage3 Battery pack cell 3 voltage in [mV]
 * @param cellvoltage4 Battery pack cell 4 voltage in [mV]
 * @param cellvoltage5 Battery pack cell 5 voltage in [mV]
 * @param cellvoltage6 Battery pack cell 6 voltage in [mV]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_batmon_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t temperature, uint16_t voltage, uint16_t current, uint16_t batterystatus, uint16_t serialnumber, uint16_t hostfetcontrol, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SENS_BATMON_LEN];
	_mav_put_uint16_t(buf, 0, temperature);
	_mav_put_uint16_t(buf, 2, voltage);
	_mav_put_uint16_t(buf, 4, current);
	_mav_put_uint16_t(buf, 6, batterystatus);
	_mav_put_uint16_t(buf, 8, serialnumber);
	_mav_put_uint16_t(buf, 10, hostfetcontrol);
	_mav_put_uint16_t(buf, 12, cellvoltage1);
	_mav_put_uint16_t(buf, 14, cellvoltage2);
	_mav_put_uint16_t(buf, 16, cellvoltage3);
	_mav_put_uint16_t(buf, 18, cellvoltage4);
	_mav_put_uint16_t(buf, 20, cellvoltage5);
	_mav_put_uint16_t(buf, 22, cellvoltage6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#else
	mavlink_sens_batmon_t packet;
	packet.temperature = temperature;
	packet.voltage = voltage;
	packet.current = current;
	packet.batterystatus = batterystatus;
	packet.serialnumber = serialnumber;
	packet.hostfetcontrol = hostfetcontrol;
	packet.cellvoltage1 = cellvoltage1;
	packet.cellvoltage2 = cellvoltage2;
	packet.cellvoltage3 = cellvoltage3;
	packet.cellvoltage4 = cellvoltage4;
	packet.cellvoltage5 = cellvoltage5;
	packet.cellvoltage6 = cellvoltage6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SENS_BATMON;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif
}

/**
 * @brief Pack a sens_batmon message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperature Battery pack temperature in [0.1 K]
 * @param voltage Battery pack voltage in [mV]
 * @param current Battery pack current in [mA]
 * @param batterystatus Battery monitor status report bits in Hex
 * @param serialnumber Battery monitor serial number in Hex
 * @param hostfetcontrol Battery monitor sensor host FET control in Hex
 * @param cellvoltage1 Battery pack cell 1 voltage in [mV]
 * @param cellvoltage2 Battery pack cell 2 voltage in [mV]
 * @param cellvoltage3 Battery pack cell 3 voltage in [mV]
 * @param cellvoltage4 Battery pack cell 4 voltage in [mV]
 * @param cellvoltage5 Battery pack cell 5 voltage in [mV]
 * @param cellvoltage6 Battery pack cell 6 voltage in [mV]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_batmon_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t temperature,uint16_t voltage,uint16_t current,uint16_t batterystatus,uint16_t serialnumber,uint16_t hostfetcontrol,uint16_t cellvoltage1,uint16_t cellvoltage2,uint16_t cellvoltage3,uint16_t cellvoltage4,uint16_t cellvoltage5,uint16_t cellvoltage6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SENS_BATMON_LEN];
	_mav_put_uint16_t(buf, 0, temperature);
	_mav_put_uint16_t(buf, 2, voltage);
	_mav_put_uint16_t(buf, 4, current);
	_mav_put_uint16_t(buf, 6, batterystatus);
	_mav_put_uint16_t(buf, 8, serialnumber);
	_mav_put_uint16_t(buf, 10, hostfetcontrol);
	_mav_put_uint16_t(buf, 12, cellvoltage1);
	_mav_put_uint16_t(buf, 14, cellvoltage2);
	_mav_put_uint16_t(buf, 16, cellvoltage3);
	_mav_put_uint16_t(buf, 18, cellvoltage4);
	_mav_put_uint16_t(buf, 20, cellvoltage5);
	_mav_put_uint16_t(buf, 22, cellvoltage6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#else
	mavlink_sens_batmon_t packet;
	packet.temperature = temperature;
	packet.voltage = voltage;
	packet.current = current;
	packet.batterystatus = batterystatus;
	packet.serialnumber = serialnumber;
	packet.hostfetcontrol = hostfetcontrol;
	packet.cellvoltage1 = cellvoltage1;
	packet.cellvoltage2 = cellvoltage2;
	packet.cellvoltage3 = cellvoltage3;
	packet.cellvoltage4 = cellvoltage4;
	packet.cellvoltage5 = cellvoltage5;
	packet.cellvoltage6 = cellvoltage6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SENS_BATMON;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif
}

/**
 * @brief Encode a sens_batmon struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sens_batmon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_batmon_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sens_batmon_t* sens_batmon)
{
	return mavlink_msg_sens_batmon_pack(system_id, component_id, msg, sens_batmon->temperature, sens_batmon->voltage, sens_batmon->current, sens_batmon->batterystatus, sens_batmon->serialnumber, sens_batmon->hostfetcontrol, sens_batmon->cellvoltage1, sens_batmon->cellvoltage2, sens_batmon->cellvoltage3, sens_batmon->cellvoltage4, sens_batmon->cellvoltage5, sens_batmon->cellvoltage6);
}

/**
 * @brief Encode a sens_batmon struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sens_batmon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_batmon_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sens_batmon_t* sens_batmon)
{
	return mavlink_msg_sens_batmon_pack_chan(system_id, component_id, chan, msg, sens_batmon->temperature, sens_batmon->voltage, sens_batmon->current, sens_batmon->batterystatus, sens_batmon->serialnumber, sens_batmon->hostfetcontrol, sens_batmon->cellvoltage1, sens_batmon->cellvoltage2, sens_batmon->cellvoltage3, sens_batmon->cellvoltage4, sens_batmon->cellvoltage5, sens_batmon->cellvoltage6);
}

/**
 * @brief Send a sens_batmon message
 * @param chan MAVLink channel to send the message
 *
 * @param temperature Battery pack temperature in [0.1 K]
 * @param voltage Battery pack voltage in [mV]
 * @param current Battery pack current in [mA]
 * @param batterystatus Battery monitor status report bits in Hex
 * @param serialnumber Battery monitor serial number in Hex
 * @param hostfetcontrol Battery monitor sensor host FET control in Hex
 * @param cellvoltage1 Battery pack cell 1 voltage in [mV]
 * @param cellvoltage2 Battery pack cell 2 voltage in [mV]
 * @param cellvoltage3 Battery pack cell 3 voltage in [mV]
 * @param cellvoltage4 Battery pack cell 4 voltage in [mV]
 * @param cellvoltage5 Battery pack cell 5 voltage in [mV]
 * @param cellvoltage6 Battery pack cell 6 voltage in [mV]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sens_batmon_send(mavlink_channel_t chan, uint16_t temperature, uint16_t voltage, uint16_t current, uint16_t batterystatus, uint16_t serialnumber, uint16_t hostfetcontrol, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SENS_BATMON_LEN];
	_mav_put_uint16_t(buf, 0, temperature);
	_mav_put_uint16_t(buf, 2, voltage);
	_mav_put_uint16_t(buf, 4, current);
	_mav_put_uint16_t(buf, 6, batterystatus);
	_mav_put_uint16_t(buf, 8, serialnumber);
	_mav_put_uint16_t(buf, 10, hostfetcontrol);
	_mav_put_uint16_t(buf, 12, cellvoltage1);
	_mav_put_uint16_t(buf, 14, cellvoltage2);
	_mav_put_uint16_t(buf, 16, cellvoltage3);
	_mav_put_uint16_t(buf, 18, cellvoltage4);
	_mav_put_uint16_t(buf, 20, cellvoltage5);
	_mav_put_uint16_t(buf, 22, cellvoltage6);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, buf, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, buf, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif
#else
	mavlink_sens_batmon_t packet;
	packet.temperature = temperature;
	packet.voltage = voltage;
	packet.current = current;
	packet.batterystatus = batterystatus;
	packet.serialnumber = serialnumber;
	packet.hostfetcontrol = hostfetcontrol;
	packet.cellvoltage1 = cellvoltage1;
	packet.cellvoltage2 = cellvoltage2;
	packet.cellvoltage3 = cellvoltage3;
	packet.cellvoltage4 = cellvoltage4;
	packet.cellvoltage5 = cellvoltage5;
	packet.cellvoltage6 = cellvoltage6;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, (const char *)&packet, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, (const char *)&packet, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SENS_BATMON_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sens_batmon_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t temperature, uint16_t voltage, uint16_t current, uint16_t batterystatus, uint16_t serialnumber, uint16_t hostfetcontrol, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, temperature);
	_mav_put_uint16_t(buf, 2, voltage);
	_mav_put_uint16_t(buf, 4, current);
	_mav_put_uint16_t(buf, 6, batterystatus);
	_mav_put_uint16_t(buf, 8, serialnumber);
	_mav_put_uint16_t(buf, 10, hostfetcontrol);
	_mav_put_uint16_t(buf, 12, cellvoltage1);
	_mav_put_uint16_t(buf, 14, cellvoltage2);
	_mav_put_uint16_t(buf, 16, cellvoltage3);
	_mav_put_uint16_t(buf, 18, cellvoltage4);
	_mav_put_uint16_t(buf, 20, cellvoltage5);
	_mav_put_uint16_t(buf, 22, cellvoltage6);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, buf, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, buf, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif
#else
	mavlink_sens_batmon_t *packet = (mavlink_sens_batmon_t *)msgbuf;
	packet->temperature = temperature;
	packet->voltage = voltage;
	packet->current = current;
	packet->batterystatus = batterystatus;
	packet->serialnumber = serialnumber;
	packet->hostfetcontrol = hostfetcontrol;
	packet->cellvoltage1 = cellvoltage1;
	packet->cellvoltage2 = cellvoltage2;
	packet->cellvoltage3 = cellvoltage3;
	packet->cellvoltage4 = cellvoltage4;
	packet->cellvoltage5 = cellvoltage5;
	packet->cellvoltage6 = cellvoltage6;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, (const char *)packet, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, (const char *)packet, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SENS_BATMON UNPACKING


/**
 * @brief Get field temperature from sens_batmon message
 *
 * @return Battery pack temperature in [0.1 K]
 */
static inline uint16_t mavlink_msg_sens_batmon_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field voltage from sens_batmon message
 *
 * @return Battery pack voltage in [mV]
 */
static inline uint16_t mavlink_msg_sens_batmon_get_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field current from sens_batmon message
 *
 * @return Battery pack current in [mA]
 */
static inline uint16_t mavlink_msg_sens_batmon_get_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field batterystatus from sens_batmon message
 *
 * @return Battery monitor status report bits in Hex
 */
static inline uint16_t mavlink_msg_sens_batmon_get_batterystatus(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field serialnumber from sens_batmon message
 *
 * @return Battery monitor serial number in Hex
 */
static inline uint16_t mavlink_msg_sens_batmon_get_serialnumber(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field hostfetcontrol from sens_batmon message
 *
 * @return Battery monitor sensor host FET control in Hex
 */
static inline uint16_t mavlink_msg_sens_batmon_get_hostfetcontrol(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field cellvoltage1 from sens_batmon message
 *
 * @return Battery pack cell 1 voltage in [mV]
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field cellvoltage2 from sens_batmon message
 *
 * @return Battery pack cell 2 voltage in [mV]
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field cellvoltage3 from sens_batmon message
 *
 * @return Battery pack cell 3 voltage in [mV]
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field cellvoltage4 from sens_batmon message
 *
 * @return Battery pack cell 4 voltage in [mV]
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field cellvoltage5 from sens_batmon message
 *
 * @return Battery pack cell 5 voltage in [mV]
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field cellvoltage6 from sens_batmon message
 *
 * @return Battery pack cell 6 voltage in [mV]
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Decode a sens_batmon message into a struct
 *
 * @param msg The message to decode
 * @param sens_batmon C-struct to decode the message contents into
 */
static inline void mavlink_msg_sens_batmon_decode(const mavlink_message_t* msg, mavlink_sens_batmon_t* sens_batmon)
{
#if MAVLINK_NEED_BYTE_SWAP
	sens_batmon->temperature = mavlink_msg_sens_batmon_get_temperature(msg);
	sens_batmon->voltage = mavlink_msg_sens_batmon_get_voltage(msg);
	sens_batmon->current = mavlink_msg_sens_batmon_get_current(msg);
	sens_batmon->batterystatus = mavlink_msg_sens_batmon_get_batterystatus(msg);
	sens_batmon->serialnumber = mavlink_msg_sens_batmon_get_serialnumber(msg);
	sens_batmon->hostfetcontrol = mavlink_msg_sens_batmon_get_hostfetcontrol(msg);
	sens_batmon->cellvoltage1 = mavlink_msg_sens_batmon_get_cellvoltage1(msg);
	sens_batmon->cellvoltage2 = mavlink_msg_sens_batmon_get_cellvoltage2(msg);
	sens_batmon->cellvoltage3 = mavlink_msg_sens_batmon_get_cellvoltage3(msg);
	sens_batmon->cellvoltage4 = mavlink_msg_sens_batmon_get_cellvoltage4(msg);
	sens_batmon->cellvoltage5 = mavlink_msg_sens_batmon_get_cellvoltage5(msg);
	sens_batmon->cellvoltage6 = mavlink_msg_sens_batmon_get_cellvoltage6(msg);
#else
	memcpy(sens_batmon, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif
}
