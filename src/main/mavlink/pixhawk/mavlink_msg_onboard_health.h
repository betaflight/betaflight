// MESSAGE ONBOARD_HEALTH PACKING

#define MAVLINK_MSG_ID_ONBOARD_HEALTH 206

typedef struct __mavlink_onboard_health_t
{
 uint32_t uptime; ///< Uptime of system
 float ram_total; ///< RAM size in GiB
 float swap_total; ///< Swap size in GiB
 float disk_total; ///< Disk total in GiB
 float temp; ///< Temperature
 float voltage; ///< Supply voltage V
 float network_load_in; ///< Network load inbound KiB/s
 float network_load_out; ///< Network load outbound in KiB/s 
 uint16_t cpu_freq; ///< CPU frequency
 uint8_t cpu_load; ///< CPU load in percent
 uint8_t ram_usage; ///< RAM usage in percent
 uint8_t swap_usage; ///< Swap usage in percent
 int8_t disk_health; ///< Disk health (-1: N/A, 0: ERR, 1: RO, 2: RW)
 uint8_t disk_usage; ///< Disk usage in percent
} mavlink_onboard_health_t;

#define MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN 39
#define MAVLINK_MSG_ID_206_LEN 39

#define MAVLINK_MSG_ID_ONBOARD_HEALTH_CRC 19
#define MAVLINK_MSG_ID_206_CRC 19



#define MAVLINK_MESSAGE_INFO_ONBOARD_HEALTH { \
	"ONBOARD_HEALTH", \
	14, \
	{  { "uptime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_onboard_health_t, uptime) }, \
         { "ram_total", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_onboard_health_t, ram_total) }, \
         { "swap_total", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_onboard_health_t, swap_total) }, \
         { "disk_total", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_onboard_health_t, disk_total) }, \
         { "temp", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_onboard_health_t, temp) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_onboard_health_t, voltage) }, \
         { "network_load_in", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_onboard_health_t, network_load_in) }, \
         { "network_load_out", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_onboard_health_t, network_load_out) }, \
         { "cpu_freq", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_onboard_health_t, cpu_freq) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_onboard_health_t, cpu_load) }, \
         { "ram_usage", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_onboard_health_t, ram_usage) }, \
         { "swap_usage", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_onboard_health_t, swap_usage) }, \
         { "disk_health", NULL, MAVLINK_TYPE_INT8_T, 0, 37, offsetof(mavlink_onboard_health_t, disk_health) }, \
         { "disk_usage", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_onboard_health_t, disk_usage) }, \
         } \
}


/**
 * @brief Pack a onboard_health message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param uptime Uptime of system
 * @param cpu_freq CPU frequency
 * @param cpu_load CPU load in percent
 * @param ram_usage RAM usage in percent
 * @param ram_total RAM size in GiB
 * @param swap_usage Swap usage in percent
 * @param swap_total Swap size in GiB
 * @param disk_health Disk health (-1: N/A, 0: ERR, 1: RO, 2: RW)
 * @param disk_usage Disk usage in percent
 * @param disk_total Disk total in GiB
 * @param temp Temperature
 * @param voltage Supply voltage V
 * @param network_load_in Network load inbound KiB/s
 * @param network_load_out Network load outbound in KiB/s 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_health_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t uptime, uint16_t cpu_freq, uint8_t cpu_load, uint8_t ram_usage, float ram_total, uint8_t swap_usage, float swap_total, int8_t disk_health, uint8_t disk_usage, float disk_total, float temp, float voltage, float network_load_in, float network_load_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN];
	_mav_put_uint32_t(buf, 0, uptime);
	_mav_put_float(buf, 4, ram_total);
	_mav_put_float(buf, 8, swap_total);
	_mav_put_float(buf, 12, disk_total);
	_mav_put_float(buf, 16, temp);
	_mav_put_float(buf, 20, voltage);
	_mav_put_float(buf, 24, network_load_in);
	_mav_put_float(buf, 28, network_load_out);
	_mav_put_uint16_t(buf, 32, cpu_freq);
	_mav_put_uint8_t(buf, 34, cpu_load);
	_mav_put_uint8_t(buf, 35, ram_usage);
	_mav_put_uint8_t(buf, 36, swap_usage);
	_mav_put_int8_t(buf, 37, disk_health);
	_mav_put_uint8_t(buf, 38, disk_usage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#else
	mavlink_onboard_health_t packet;
	packet.uptime = uptime;
	packet.ram_total = ram_total;
	packet.swap_total = swap_total;
	packet.disk_total = disk_total;
	packet.temp = temp;
	packet.voltage = voltage;
	packet.network_load_in = network_load_in;
	packet.network_load_out = network_load_out;
	packet.cpu_freq = cpu_freq;
	packet.cpu_load = cpu_load;
	packet.ram_usage = ram_usage;
	packet.swap_usage = swap_usage;
	packet.disk_health = disk_health;
	packet.disk_usage = disk_usage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ONBOARD_HEALTH;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN, MAVLINK_MSG_ID_ONBOARD_HEALTH_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#endif
}

/**
 * @brief Pack a onboard_health message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uptime Uptime of system
 * @param cpu_freq CPU frequency
 * @param cpu_load CPU load in percent
 * @param ram_usage RAM usage in percent
 * @param ram_total RAM size in GiB
 * @param swap_usage Swap usage in percent
 * @param swap_total Swap size in GiB
 * @param disk_health Disk health (-1: N/A, 0: ERR, 1: RO, 2: RW)
 * @param disk_usage Disk usage in percent
 * @param disk_total Disk total in GiB
 * @param temp Temperature
 * @param voltage Supply voltage V
 * @param network_load_in Network load inbound KiB/s
 * @param network_load_out Network load outbound in KiB/s 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_health_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t uptime,uint16_t cpu_freq,uint8_t cpu_load,uint8_t ram_usage,float ram_total,uint8_t swap_usage,float swap_total,int8_t disk_health,uint8_t disk_usage,float disk_total,float temp,float voltage,float network_load_in,float network_load_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN];
	_mav_put_uint32_t(buf, 0, uptime);
	_mav_put_float(buf, 4, ram_total);
	_mav_put_float(buf, 8, swap_total);
	_mav_put_float(buf, 12, disk_total);
	_mav_put_float(buf, 16, temp);
	_mav_put_float(buf, 20, voltage);
	_mav_put_float(buf, 24, network_load_in);
	_mav_put_float(buf, 28, network_load_out);
	_mav_put_uint16_t(buf, 32, cpu_freq);
	_mav_put_uint8_t(buf, 34, cpu_load);
	_mav_put_uint8_t(buf, 35, ram_usage);
	_mav_put_uint8_t(buf, 36, swap_usage);
	_mav_put_int8_t(buf, 37, disk_health);
	_mav_put_uint8_t(buf, 38, disk_usage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#else
	mavlink_onboard_health_t packet;
	packet.uptime = uptime;
	packet.ram_total = ram_total;
	packet.swap_total = swap_total;
	packet.disk_total = disk_total;
	packet.temp = temp;
	packet.voltage = voltage;
	packet.network_load_in = network_load_in;
	packet.network_load_out = network_load_out;
	packet.cpu_freq = cpu_freq;
	packet.cpu_load = cpu_load;
	packet.ram_usage = ram_usage;
	packet.swap_usage = swap_usage;
	packet.disk_health = disk_health;
	packet.disk_usage = disk_usage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ONBOARD_HEALTH;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN, MAVLINK_MSG_ID_ONBOARD_HEALTH_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#endif
}

/**
 * @brief Encode a onboard_health struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param onboard_health C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_health_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_onboard_health_t* onboard_health)
{
	return mavlink_msg_onboard_health_pack(system_id, component_id, msg, onboard_health->uptime, onboard_health->cpu_freq, onboard_health->cpu_load, onboard_health->ram_usage, onboard_health->ram_total, onboard_health->swap_usage, onboard_health->swap_total, onboard_health->disk_health, onboard_health->disk_usage, onboard_health->disk_total, onboard_health->temp, onboard_health->voltage, onboard_health->network_load_in, onboard_health->network_load_out);
}

/**
 * @brief Encode a onboard_health struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param onboard_health C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_health_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_onboard_health_t* onboard_health)
{
	return mavlink_msg_onboard_health_pack_chan(system_id, component_id, chan, msg, onboard_health->uptime, onboard_health->cpu_freq, onboard_health->cpu_load, onboard_health->ram_usage, onboard_health->ram_total, onboard_health->swap_usage, onboard_health->swap_total, onboard_health->disk_health, onboard_health->disk_usage, onboard_health->disk_total, onboard_health->temp, onboard_health->voltage, onboard_health->network_load_in, onboard_health->network_load_out);
}

/**
 * @brief Send a onboard_health message
 * @param chan MAVLink channel to send the message
 *
 * @param uptime Uptime of system
 * @param cpu_freq CPU frequency
 * @param cpu_load CPU load in percent
 * @param ram_usage RAM usage in percent
 * @param ram_total RAM size in GiB
 * @param swap_usage Swap usage in percent
 * @param swap_total Swap size in GiB
 * @param disk_health Disk health (-1: N/A, 0: ERR, 1: RO, 2: RW)
 * @param disk_usage Disk usage in percent
 * @param disk_total Disk total in GiB
 * @param temp Temperature
 * @param voltage Supply voltage V
 * @param network_load_in Network load inbound KiB/s
 * @param network_load_out Network load outbound in KiB/s 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_onboard_health_send(mavlink_channel_t chan, uint32_t uptime, uint16_t cpu_freq, uint8_t cpu_load, uint8_t ram_usage, float ram_total, uint8_t swap_usage, float swap_total, int8_t disk_health, uint8_t disk_usage, float disk_total, float temp, float voltage, float network_load_in, float network_load_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN];
	_mav_put_uint32_t(buf, 0, uptime);
	_mav_put_float(buf, 4, ram_total);
	_mav_put_float(buf, 8, swap_total);
	_mav_put_float(buf, 12, disk_total);
	_mav_put_float(buf, 16, temp);
	_mav_put_float(buf, 20, voltage);
	_mav_put_float(buf, 24, network_load_in);
	_mav_put_float(buf, 28, network_load_out);
	_mav_put_uint16_t(buf, 32, cpu_freq);
	_mav_put_uint8_t(buf, 34, cpu_load);
	_mav_put_uint8_t(buf, 35, ram_usage);
	_mav_put_uint8_t(buf, 36, swap_usage);
	_mav_put_int8_t(buf, 37, disk_health);
	_mav_put_uint8_t(buf, 38, disk_usage);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_HEALTH, buf, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN, MAVLINK_MSG_ID_ONBOARD_HEALTH_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_HEALTH, buf, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#endif
#else
	mavlink_onboard_health_t packet;
	packet.uptime = uptime;
	packet.ram_total = ram_total;
	packet.swap_total = swap_total;
	packet.disk_total = disk_total;
	packet.temp = temp;
	packet.voltage = voltage;
	packet.network_load_in = network_load_in;
	packet.network_load_out = network_load_out;
	packet.cpu_freq = cpu_freq;
	packet.cpu_load = cpu_load;
	packet.ram_usage = ram_usage;
	packet.swap_usage = swap_usage;
	packet.disk_health = disk_health;
	packet.disk_usage = disk_usage;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_HEALTH, (const char *)&packet, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN, MAVLINK_MSG_ID_ONBOARD_HEALTH_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_HEALTH, (const char *)&packet, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_onboard_health_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t uptime, uint16_t cpu_freq, uint8_t cpu_load, uint8_t ram_usage, float ram_total, uint8_t swap_usage, float swap_total, int8_t disk_health, uint8_t disk_usage, float disk_total, float temp, float voltage, float network_load_in, float network_load_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, uptime);
	_mav_put_float(buf, 4, ram_total);
	_mav_put_float(buf, 8, swap_total);
	_mav_put_float(buf, 12, disk_total);
	_mav_put_float(buf, 16, temp);
	_mav_put_float(buf, 20, voltage);
	_mav_put_float(buf, 24, network_load_in);
	_mav_put_float(buf, 28, network_load_out);
	_mav_put_uint16_t(buf, 32, cpu_freq);
	_mav_put_uint8_t(buf, 34, cpu_load);
	_mav_put_uint8_t(buf, 35, ram_usage);
	_mav_put_uint8_t(buf, 36, swap_usage);
	_mav_put_int8_t(buf, 37, disk_health);
	_mav_put_uint8_t(buf, 38, disk_usage);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_HEALTH, buf, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN, MAVLINK_MSG_ID_ONBOARD_HEALTH_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_HEALTH, buf, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#endif
#else
	mavlink_onboard_health_t *packet = (mavlink_onboard_health_t *)msgbuf;
	packet->uptime = uptime;
	packet->ram_total = ram_total;
	packet->swap_total = swap_total;
	packet->disk_total = disk_total;
	packet->temp = temp;
	packet->voltage = voltage;
	packet->network_load_in = network_load_in;
	packet->network_load_out = network_load_out;
	packet->cpu_freq = cpu_freq;
	packet->cpu_load = cpu_load;
	packet->ram_usage = ram_usage;
	packet->swap_usage = swap_usage;
	packet->disk_health = disk_health;
	packet->disk_usage = disk_usage;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_HEALTH, (const char *)packet, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN, MAVLINK_MSG_ID_ONBOARD_HEALTH_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_HEALTH, (const char *)packet, MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ONBOARD_HEALTH UNPACKING


/**
 * @brief Get field uptime from onboard_health message
 *
 * @return Uptime of system
 */
static inline uint32_t mavlink_msg_onboard_health_get_uptime(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cpu_freq from onboard_health message
 *
 * @return CPU frequency
 */
static inline uint16_t mavlink_msg_onboard_health_get_cpu_freq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field cpu_load from onboard_health message
 *
 * @return CPU load in percent
 */
static inline uint8_t mavlink_msg_onboard_health_get_cpu_load(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field ram_usage from onboard_health message
 *
 * @return RAM usage in percent
 */
static inline uint8_t mavlink_msg_onboard_health_get_ram_usage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field ram_total from onboard_health message
 *
 * @return RAM size in GiB
 */
static inline float mavlink_msg_onboard_health_get_ram_total(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field swap_usage from onboard_health message
 *
 * @return Swap usage in percent
 */
static inline uint8_t mavlink_msg_onboard_health_get_swap_usage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field swap_total from onboard_health message
 *
 * @return Swap size in GiB
 */
static inline float mavlink_msg_onboard_health_get_swap_total(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field disk_health from onboard_health message
 *
 * @return Disk health (-1: N/A, 0: ERR, 1: RO, 2: RW)
 */
static inline int8_t mavlink_msg_onboard_health_get_disk_health(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  37);
}

/**
 * @brief Get field disk_usage from onboard_health message
 *
 * @return Disk usage in percent
 */
static inline uint8_t mavlink_msg_onboard_health_get_disk_usage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field disk_total from onboard_health message
 *
 * @return Disk total in GiB
 */
static inline float mavlink_msg_onboard_health_get_disk_total(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field temp from onboard_health message
 *
 * @return Temperature
 */
static inline float mavlink_msg_onboard_health_get_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field voltage from onboard_health message
 *
 * @return Supply voltage V
 */
static inline float mavlink_msg_onboard_health_get_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field network_load_in from onboard_health message
 *
 * @return Network load inbound KiB/s
 */
static inline float mavlink_msg_onboard_health_get_network_load_in(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field network_load_out from onboard_health message
 *
 * @return Network load outbound in KiB/s 
 */
static inline float mavlink_msg_onboard_health_get_network_load_out(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a onboard_health message into a struct
 *
 * @param msg The message to decode
 * @param onboard_health C-struct to decode the message contents into
 */
static inline void mavlink_msg_onboard_health_decode(const mavlink_message_t* msg, mavlink_onboard_health_t* onboard_health)
{
#if MAVLINK_NEED_BYTE_SWAP
	onboard_health->uptime = mavlink_msg_onboard_health_get_uptime(msg);
	onboard_health->ram_total = mavlink_msg_onboard_health_get_ram_total(msg);
	onboard_health->swap_total = mavlink_msg_onboard_health_get_swap_total(msg);
	onboard_health->disk_total = mavlink_msg_onboard_health_get_disk_total(msg);
	onboard_health->temp = mavlink_msg_onboard_health_get_temp(msg);
	onboard_health->voltage = mavlink_msg_onboard_health_get_voltage(msg);
	onboard_health->network_load_in = mavlink_msg_onboard_health_get_network_load_in(msg);
	onboard_health->network_load_out = mavlink_msg_onboard_health_get_network_load_out(msg);
	onboard_health->cpu_freq = mavlink_msg_onboard_health_get_cpu_freq(msg);
	onboard_health->cpu_load = mavlink_msg_onboard_health_get_cpu_load(msg);
	onboard_health->ram_usage = mavlink_msg_onboard_health_get_ram_usage(msg);
	onboard_health->swap_usage = mavlink_msg_onboard_health_get_swap_usage(msg);
	onboard_health->disk_health = mavlink_msg_onboard_health_get_disk_health(msg);
	onboard_health->disk_usage = mavlink_msg_onboard_health_get_disk_usage(msg);
#else
	memcpy(onboard_health, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ONBOARD_HEALTH_LEN);
#endif
}
