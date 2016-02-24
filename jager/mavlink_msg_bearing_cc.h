// MESSAGE BEARING_CC PACKING

#define MAVLINK_MSG_ID_BEARING_CC 216

typedef struct __mavlink_bearing_cc_t
{
 uint64_t timestamp_usec; ///< The timestamp of the measurement (from the pixhawk)
 double bearing; ///< The calculated bearing
 int32_t lat; ///< Latitude where the measurement was taken
 int32_t lon; ///< Longitutde where the measurement was taken
 float alt; ///< Altitude of the measurement
} mavlink_bearing_cc_t;

#define MAVLINK_MSG_ID_BEARING_CC_LEN 28
#define MAVLINK_MSG_ID_216_LEN 28

#define MAVLINK_MSG_ID_BEARING_CC_CRC 52
#define MAVLINK_MSG_ID_216_CRC 52



#define MAVLINK_MESSAGE_INFO_BEARING_CC { \
	"BEARING_CC", \
	5, \
	{  { "timestamp_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_bearing_cc_t, timestamp_usec) }, \
         { "bearing", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_bearing_cc_t, bearing) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_bearing_cc_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_bearing_cc_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_bearing_cc_t, alt) }, \
         } \
}


/**
 * @brief Pack a bearing_cc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_usec The timestamp of the measurement (from the pixhawk)
 * @param bearing The calculated bearing
 * @param lat Latitude where the measurement was taken
 * @param lon Longitutde where the measurement was taken
 * @param alt Altitude of the measurement
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bearing_cc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp_usec, double bearing, int32_t lat, int32_t lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BEARING_CC_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_double(buf, 8, bearing);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_float(buf, 24, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BEARING_CC_LEN);
#else
	mavlink_bearing_cc_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.bearing = bearing;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BEARING_CC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BEARING_CC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BEARING_CC_LEN, MAVLINK_MSG_ID_BEARING_CC_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BEARING_CC_LEN);
#endif
}

/**
 * @brief Pack a bearing_cc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_usec The timestamp of the measurement (from the pixhawk)
 * @param bearing The calculated bearing
 * @param lat Latitude where the measurement was taken
 * @param lon Longitutde where the measurement was taken
 * @param alt Altitude of the measurement
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bearing_cc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp_usec,double bearing,int32_t lat,int32_t lon,float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BEARING_CC_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_double(buf, 8, bearing);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_float(buf, 24, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BEARING_CC_LEN);
#else
	mavlink_bearing_cc_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.bearing = bearing;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BEARING_CC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BEARING_CC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BEARING_CC_LEN, MAVLINK_MSG_ID_BEARING_CC_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BEARING_CC_LEN);
#endif
}

/**
 * @brief Encode a bearing_cc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bearing_cc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bearing_cc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bearing_cc_t* bearing_cc)
{
	return mavlink_msg_bearing_cc_pack(system_id, component_id, msg, bearing_cc->timestamp_usec, bearing_cc->bearing, bearing_cc->lat, bearing_cc->lon, bearing_cc->alt);
}

/**
 * @brief Encode a bearing_cc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bearing_cc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bearing_cc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bearing_cc_t* bearing_cc)
{
	return mavlink_msg_bearing_cc_pack_chan(system_id, component_id, chan, msg, bearing_cc->timestamp_usec, bearing_cc->bearing, bearing_cc->lat, bearing_cc->lon, bearing_cc->alt);
}

/**
 * @brief Send a bearing_cc message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_usec The timestamp of the measurement (from the pixhawk)
 * @param bearing The calculated bearing
 * @param lat Latitude where the measurement was taken
 * @param lon Longitutde where the measurement was taken
 * @param alt Altitude of the measurement
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bearing_cc_send(mavlink_channel_t chan, uint64_t timestamp_usec, double bearing, int32_t lat, int32_t lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BEARING_CC_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_double(buf, 8, bearing);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_float(buf, 24, alt);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING_CC, buf, MAVLINK_MSG_ID_BEARING_CC_LEN, MAVLINK_MSG_ID_BEARING_CC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING_CC, buf, MAVLINK_MSG_ID_BEARING_CC_LEN);
#endif
#else
	mavlink_bearing_cc_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.bearing = bearing;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING_CC, (const char *)&packet, MAVLINK_MSG_ID_BEARING_CC_LEN, MAVLINK_MSG_ID_BEARING_CC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING_CC, (const char *)&packet, MAVLINK_MSG_ID_BEARING_CC_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_BEARING_CC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bearing_cc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_usec, double bearing, int32_t lat, int32_t lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_double(buf, 8, bearing);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_float(buf, 24, alt);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING_CC, buf, MAVLINK_MSG_ID_BEARING_CC_LEN, MAVLINK_MSG_ID_BEARING_CC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING_CC, buf, MAVLINK_MSG_ID_BEARING_CC_LEN);
#endif
#else
	mavlink_bearing_cc_t *packet = (mavlink_bearing_cc_t *)msgbuf;
	packet->timestamp_usec = timestamp_usec;
	packet->bearing = bearing;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING_CC, (const char *)packet, MAVLINK_MSG_ID_BEARING_CC_LEN, MAVLINK_MSG_ID_BEARING_CC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING_CC, (const char *)packet, MAVLINK_MSG_ID_BEARING_CC_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE BEARING_CC UNPACKING


/**
 * @brief Get field timestamp_usec from bearing_cc message
 *
 * @return The timestamp of the measurement (from the pixhawk)
 */
static inline uint64_t mavlink_msg_bearing_cc_get_timestamp_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field bearing from bearing_cc message
 *
 * @return The calculated bearing
 */
static inline double mavlink_msg_bearing_cc_get_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field lat from bearing_cc message
 *
 * @return Latitude where the measurement was taken
 */
static inline int32_t mavlink_msg_bearing_cc_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field lon from bearing_cc message
 *
 * @return Longitutde where the measurement was taken
 */
static inline int32_t mavlink_msg_bearing_cc_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field alt from bearing_cc message
 *
 * @return Altitude of the measurement
 */
static inline float mavlink_msg_bearing_cc_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a bearing_cc message into a struct
 *
 * @param msg The message to decode
 * @param bearing_cc C-struct to decode the message contents into
 */
static inline void mavlink_msg_bearing_cc_decode(const mavlink_message_t* msg, mavlink_bearing_cc_t* bearing_cc)
{
#if MAVLINK_NEED_BYTE_SWAP
	bearing_cc->timestamp_usec = mavlink_msg_bearing_cc_get_timestamp_usec(msg);
	bearing_cc->bearing = mavlink_msg_bearing_cc_get_bearing(msg);
	bearing_cc->lat = mavlink_msg_bearing_cc_get_lat(msg);
	bearing_cc->lon = mavlink_msg_bearing_cc_get_lon(msg);
	bearing_cc->alt = mavlink_msg_bearing_cc_get_alt(msg);
#else
	memcpy(bearing_cc, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_BEARING_CC_LEN);
#endif
}
