// MESSAGE DETECTION_STATS PACKING

#define MAVLINK_MSG_ID_DETECTION_STATS 205

typedef struct __mavlink_detection_stats_t
{
 uint32_t detections; ///< Number of detections
 uint32_t cluster_iters; ///< Number of cluster iterations
 float best_score; ///< Best score
 int32_t best_lat; ///< Latitude of the best detection * 1E7
 int32_t best_lon; ///< Longitude of the best detection * 1E7
 int32_t best_alt; ///< Altitude of the best detection * 1E3
 uint32_t best_detection_id; ///< Best detection ID
 uint32_t best_cluster_id; ///< Best cluster ID
 uint32_t best_cluster_iter_id; ///< Best cluster ID
 uint32_t images_done; ///< Number of images already processed
 uint32_t images_todo; ///< Number of images still to process
 float fps; ///< Average images per seconds processed
} mavlink_detection_stats_t;

#define MAVLINK_MSG_ID_DETECTION_STATS_LEN 48
#define MAVLINK_MSG_ID_205_LEN 48

#define MAVLINK_MSG_ID_DETECTION_STATS_CRC 87
#define MAVLINK_MSG_ID_205_CRC 87



#define MAVLINK_MESSAGE_INFO_DETECTION_STATS { \
	"DETECTION_STATS", \
	12, \
	{  { "detections", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_detection_stats_t, detections) }, \
         { "cluster_iters", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_detection_stats_t, cluster_iters) }, \
         { "best_score", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_detection_stats_t, best_score) }, \
         { "best_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_detection_stats_t, best_lat) }, \
         { "best_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_detection_stats_t, best_lon) }, \
         { "best_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_detection_stats_t, best_alt) }, \
         { "best_detection_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_detection_stats_t, best_detection_id) }, \
         { "best_cluster_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_detection_stats_t, best_cluster_id) }, \
         { "best_cluster_iter_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_detection_stats_t, best_cluster_iter_id) }, \
         { "images_done", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_detection_stats_t, images_done) }, \
         { "images_todo", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_detection_stats_t, images_todo) }, \
         { "fps", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_detection_stats_t, fps) }, \
         } \
}


/**
 * @brief Pack a detection_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param detections Number of detections
 * @param cluster_iters Number of cluster iterations
 * @param best_score Best score
 * @param best_lat Latitude of the best detection * 1E7
 * @param best_lon Longitude of the best detection * 1E7
 * @param best_alt Altitude of the best detection * 1E3
 * @param best_detection_id Best detection ID
 * @param best_cluster_id Best cluster ID
 * @param best_cluster_iter_id Best cluster ID
 * @param images_done Number of images already processed
 * @param images_todo Number of images still to process
 * @param fps Average images per seconds processed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_detection_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t detections, uint32_t cluster_iters, float best_score, int32_t best_lat, int32_t best_lon, int32_t best_alt, uint32_t best_detection_id, uint32_t best_cluster_id, uint32_t best_cluster_iter_id, uint32_t images_done, uint32_t images_todo, float fps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DETECTION_STATS_LEN];
	_mav_put_uint32_t(buf, 0, detections);
	_mav_put_uint32_t(buf, 4, cluster_iters);
	_mav_put_float(buf, 8, best_score);
	_mav_put_int32_t(buf, 12, best_lat);
	_mav_put_int32_t(buf, 16, best_lon);
	_mav_put_int32_t(buf, 20, best_alt);
	_mav_put_uint32_t(buf, 24, best_detection_id);
	_mav_put_uint32_t(buf, 28, best_cluster_id);
	_mav_put_uint32_t(buf, 32, best_cluster_iter_id);
	_mav_put_uint32_t(buf, 36, images_done);
	_mav_put_uint32_t(buf, 40, images_todo);
	_mav_put_float(buf, 44, fps);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#else
	mavlink_detection_stats_t packet;
	packet.detections = detections;
	packet.cluster_iters = cluster_iters;
	packet.best_score = best_score;
	packet.best_lat = best_lat;
	packet.best_lon = best_lon;
	packet.best_alt = best_alt;
	packet.best_detection_id = best_detection_id;
	packet.best_cluster_id = best_cluster_id;
	packet.best_cluster_iter_id = best_cluster_iter_id;
	packet.images_done = images_done;
	packet.images_todo = images_todo;
	packet.fps = fps;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DETECTION_STATS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DETECTION_STATS_LEN, MAVLINK_MSG_ID_DETECTION_STATS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#endif
}

/**
 * @brief Pack a detection_stats message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param detections Number of detections
 * @param cluster_iters Number of cluster iterations
 * @param best_score Best score
 * @param best_lat Latitude of the best detection * 1E7
 * @param best_lon Longitude of the best detection * 1E7
 * @param best_alt Altitude of the best detection * 1E3
 * @param best_detection_id Best detection ID
 * @param best_cluster_id Best cluster ID
 * @param best_cluster_iter_id Best cluster ID
 * @param images_done Number of images already processed
 * @param images_todo Number of images still to process
 * @param fps Average images per seconds processed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_detection_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t detections,uint32_t cluster_iters,float best_score,int32_t best_lat,int32_t best_lon,int32_t best_alt,uint32_t best_detection_id,uint32_t best_cluster_id,uint32_t best_cluster_iter_id,uint32_t images_done,uint32_t images_todo,float fps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DETECTION_STATS_LEN];
	_mav_put_uint32_t(buf, 0, detections);
	_mav_put_uint32_t(buf, 4, cluster_iters);
	_mav_put_float(buf, 8, best_score);
	_mav_put_int32_t(buf, 12, best_lat);
	_mav_put_int32_t(buf, 16, best_lon);
	_mav_put_int32_t(buf, 20, best_alt);
	_mav_put_uint32_t(buf, 24, best_detection_id);
	_mav_put_uint32_t(buf, 28, best_cluster_id);
	_mav_put_uint32_t(buf, 32, best_cluster_iter_id);
	_mav_put_uint32_t(buf, 36, images_done);
	_mav_put_uint32_t(buf, 40, images_todo);
	_mav_put_float(buf, 44, fps);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#else
	mavlink_detection_stats_t packet;
	packet.detections = detections;
	packet.cluster_iters = cluster_iters;
	packet.best_score = best_score;
	packet.best_lat = best_lat;
	packet.best_lon = best_lon;
	packet.best_alt = best_alt;
	packet.best_detection_id = best_detection_id;
	packet.best_cluster_id = best_cluster_id;
	packet.best_cluster_iter_id = best_cluster_iter_id;
	packet.images_done = images_done;
	packet.images_todo = images_todo;
	packet.fps = fps;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DETECTION_STATS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DETECTION_STATS_LEN, MAVLINK_MSG_ID_DETECTION_STATS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#endif
}

/**
 * @brief Encode a detection_stats struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param detection_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_detection_stats_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_detection_stats_t* detection_stats)
{
	return mavlink_msg_detection_stats_pack(system_id, component_id, msg, detection_stats->detections, detection_stats->cluster_iters, detection_stats->best_score, detection_stats->best_lat, detection_stats->best_lon, detection_stats->best_alt, detection_stats->best_detection_id, detection_stats->best_cluster_id, detection_stats->best_cluster_iter_id, detection_stats->images_done, detection_stats->images_todo, detection_stats->fps);
}

/**
 * @brief Encode a detection_stats struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param detection_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_detection_stats_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_detection_stats_t* detection_stats)
{
	return mavlink_msg_detection_stats_pack_chan(system_id, component_id, chan, msg, detection_stats->detections, detection_stats->cluster_iters, detection_stats->best_score, detection_stats->best_lat, detection_stats->best_lon, detection_stats->best_alt, detection_stats->best_detection_id, detection_stats->best_cluster_id, detection_stats->best_cluster_iter_id, detection_stats->images_done, detection_stats->images_todo, detection_stats->fps);
}

/**
 * @brief Send a detection_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param detections Number of detections
 * @param cluster_iters Number of cluster iterations
 * @param best_score Best score
 * @param best_lat Latitude of the best detection * 1E7
 * @param best_lon Longitude of the best detection * 1E7
 * @param best_alt Altitude of the best detection * 1E3
 * @param best_detection_id Best detection ID
 * @param best_cluster_id Best cluster ID
 * @param best_cluster_iter_id Best cluster ID
 * @param images_done Number of images already processed
 * @param images_todo Number of images still to process
 * @param fps Average images per seconds processed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_detection_stats_send(mavlink_channel_t chan, uint32_t detections, uint32_t cluster_iters, float best_score, int32_t best_lat, int32_t best_lon, int32_t best_alt, uint32_t best_detection_id, uint32_t best_cluster_id, uint32_t best_cluster_iter_id, uint32_t images_done, uint32_t images_todo, float fps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DETECTION_STATS_LEN];
	_mav_put_uint32_t(buf, 0, detections);
	_mav_put_uint32_t(buf, 4, cluster_iters);
	_mav_put_float(buf, 8, best_score);
	_mav_put_int32_t(buf, 12, best_lat);
	_mav_put_int32_t(buf, 16, best_lon);
	_mav_put_int32_t(buf, 20, best_alt);
	_mav_put_uint32_t(buf, 24, best_detection_id);
	_mav_put_uint32_t(buf, 28, best_cluster_id);
	_mav_put_uint32_t(buf, 32, best_cluster_iter_id);
	_mav_put_uint32_t(buf, 36, images_done);
	_mav_put_uint32_t(buf, 40, images_todo);
	_mav_put_float(buf, 44, fps);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DETECTION_STATS, buf, MAVLINK_MSG_ID_DETECTION_STATS_LEN, MAVLINK_MSG_ID_DETECTION_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DETECTION_STATS, buf, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#endif
#else
	mavlink_detection_stats_t packet;
	packet.detections = detections;
	packet.cluster_iters = cluster_iters;
	packet.best_score = best_score;
	packet.best_lat = best_lat;
	packet.best_lon = best_lon;
	packet.best_alt = best_alt;
	packet.best_detection_id = best_detection_id;
	packet.best_cluster_id = best_cluster_id;
	packet.best_cluster_iter_id = best_cluster_iter_id;
	packet.images_done = images_done;
	packet.images_todo = images_todo;
	packet.fps = fps;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DETECTION_STATS, (const char *)&packet, MAVLINK_MSG_ID_DETECTION_STATS_LEN, MAVLINK_MSG_ID_DETECTION_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DETECTION_STATS, (const char *)&packet, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_DETECTION_STATS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_detection_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t detections, uint32_t cluster_iters, float best_score, int32_t best_lat, int32_t best_lon, int32_t best_alt, uint32_t best_detection_id, uint32_t best_cluster_id, uint32_t best_cluster_iter_id, uint32_t images_done, uint32_t images_todo, float fps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, detections);
	_mav_put_uint32_t(buf, 4, cluster_iters);
	_mav_put_float(buf, 8, best_score);
	_mav_put_int32_t(buf, 12, best_lat);
	_mav_put_int32_t(buf, 16, best_lon);
	_mav_put_int32_t(buf, 20, best_alt);
	_mav_put_uint32_t(buf, 24, best_detection_id);
	_mav_put_uint32_t(buf, 28, best_cluster_id);
	_mav_put_uint32_t(buf, 32, best_cluster_iter_id);
	_mav_put_uint32_t(buf, 36, images_done);
	_mav_put_uint32_t(buf, 40, images_todo);
	_mav_put_float(buf, 44, fps);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DETECTION_STATS, buf, MAVLINK_MSG_ID_DETECTION_STATS_LEN, MAVLINK_MSG_ID_DETECTION_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DETECTION_STATS, buf, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#endif
#else
	mavlink_detection_stats_t *packet = (mavlink_detection_stats_t *)msgbuf;
	packet->detections = detections;
	packet->cluster_iters = cluster_iters;
	packet->best_score = best_score;
	packet->best_lat = best_lat;
	packet->best_lon = best_lon;
	packet->best_alt = best_alt;
	packet->best_detection_id = best_detection_id;
	packet->best_cluster_id = best_cluster_id;
	packet->best_cluster_iter_id = best_cluster_iter_id;
	packet->images_done = images_done;
	packet->images_todo = images_todo;
	packet->fps = fps;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DETECTION_STATS, (const char *)packet, MAVLINK_MSG_ID_DETECTION_STATS_LEN, MAVLINK_MSG_ID_DETECTION_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DETECTION_STATS, (const char *)packet, MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE DETECTION_STATS UNPACKING


/**
 * @brief Get field detections from detection_stats message
 *
 * @return Number of detections
 */
static inline uint32_t mavlink_msg_detection_stats_get_detections(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cluster_iters from detection_stats message
 *
 * @return Number of cluster iterations
 */
static inline uint32_t mavlink_msg_detection_stats_get_cluster_iters(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field best_score from detection_stats message
 *
 * @return Best score
 */
static inline float mavlink_msg_detection_stats_get_best_score(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field best_lat from detection_stats message
 *
 * @return Latitude of the best detection * 1E7
 */
static inline int32_t mavlink_msg_detection_stats_get_best_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field best_lon from detection_stats message
 *
 * @return Longitude of the best detection * 1E7
 */
static inline int32_t mavlink_msg_detection_stats_get_best_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field best_alt from detection_stats message
 *
 * @return Altitude of the best detection * 1E3
 */
static inline int32_t mavlink_msg_detection_stats_get_best_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field best_detection_id from detection_stats message
 *
 * @return Best detection ID
 */
static inline uint32_t mavlink_msg_detection_stats_get_best_detection_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field best_cluster_id from detection_stats message
 *
 * @return Best cluster ID
 */
static inline uint32_t mavlink_msg_detection_stats_get_best_cluster_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field best_cluster_iter_id from detection_stats message
 *
 * @return Best cluster ID
 */
static inline uint32_t mavlink_msg_detection_stats_get_best_cluster_iter_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Get field images_done from detection_stats message
 *
 * @return Number of images already processed
 */
static inline uint32_t mavlink_msg_detection_stats_get_images_done(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  36);
}

/**
 * @brief Get field images_todo from detection_stats message
 *
 * @return Number of images still to process
 */
static inline uint32_t mavlink_msg_detection_stats_get_images_todo(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  40);
}

/**
 * @brief Get field fps from detection_stats message
 *
 * @return Average images per seconds processed
 */
static inline float mavlink_msg_detection_stats_get_fps(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a detection_stats message into a struct
 *
 * @param msg The message to decode
 * @param detection_stats C-struct to decode the message contents into
 */
static inline void mavlink_msg_detection_stats_decode(const mavlink_message_t* msg, mavlink_detection_stats_t* detection_stats)
{
#if MAVLINK_NEED_BYTE_SWAP
	detection_stats->detections = mavlink_msg_detection_stats_get_detections(msg);
	detection_stats->cluster_iters = mavlink_msg_detection_stats_get_cluster_iters(msg);
	detection_stats->best_score = mavlink_msg_detection_stats_get_best_score(msg);
	detection_stats->best_lat = mavlink_msg_detection_stats_get_best_lat(msg);
	detection_stats->best_lon = mavlink_msg_detection_stats_get_best_lon(msg);
	detection_stats->best_alt = mavlink_msg_detection_stats_get_best_alt(msg);
	detection_stats->best_detection_id = mavlink_msg_detection_stats_get_best_detection_id(msg);
	detection_stats->best_cluster_id = mavlink_msg_detection_stats_get_best_cluster_id(msg);
	detection_stats->best_cluster_iter_id = mavlink_msg_detection_stats_get_best_cluster_iter_id(msg);
	detection_stats->images_done = mavlink_msg_detection_stats_get_images_done(msg);
	detection_stats->images_todo = mavlink_msg_detection_stats_get_images_todo(msg);
	detection_stats->fps = mavlink_msg_detection_stats_get_fps(msg);
#else
	memcpy(detection_stats, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DETECTION_STATS_LEN);
#endif
}
