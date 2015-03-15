// MESSAGE BEARING PACKING

#define MAVLINK_MSG_ID_BEARING 216

typedef struct __mavlink_bearing_t
{
 double bearing; ///< The calculated bearing
 int32_t lat; ///< Latitude where the measurement was taken
 int32_t lon; ///< Longitutde where the measurement was taken
 float alt; ///< Altitude of the measurement
} mavlink_bearing_t;

#define MAVLINK_MSG_ID_BEARING_LEN 20
#define MAVLINK_MSG_ID_216_LEN 20

#define MAVLINK_MSG_ID_BEARING_CRC 16
#define MAVLINK_MSG_ID_216_CRC 16



#define MAVLINK_MESSAGE_INFO_BEARING { \
	"BEARING", \
	4, \
	{  { "bearing", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_bearing_t, bearing) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_bearing_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_bearing_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_bearing_t, alt) }, \
         } \
}


/**
 * @brief Pack a bearing message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param bearing The calculated bearing
 * @param lat Latitude where the measurement was taken
 * @param lon Longitutde where the measurement was taken
 * @param alt Altitude of the measurement
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bearing_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       double bearing, int32_t lat, int32_t lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BEARING_LEN];
	_mav_put_double(buf, 0, bearing);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BEARING_LEN);
#else
	mavlink_bearing_t packet;
	packet.bearing = bearing;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BEARING_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BEARING;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BEARING_LEN, MAVLINK_MSG_ID_BEARING_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BEARING_LEN);
#endif
}

/**
 * @brief Pack a bearing message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bearing The calculated bearing
 * @param lat Latitude where the measurement was taken
 * @param lon Longitutde where the measurement was taken
 * @param alt Altitude of the measurement
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bearing_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           double bearing,int32_t lat,int32_t lon,float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BEARING_LEN];
	_mav_put_double(buf, 0, bearing);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BEARING_LEN);
#else
	mavlink_bearing_t packet;
	packet.bearing = bearing;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BEARING_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BEARING;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BEARING_LEN, MAVLINK_MSG_ID_BEARING_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BEARING_LEN);
#endif
}

/**
 * @brief Encode a bearing struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bearing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bearing_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bearing_t* bearing)
{
	return mavlink_msg_bearing_pack(system_id, component_id, msg, bearing->bearing, bearing->lat, bearing->lon, bearing->alt);
}

/**
 * @brief Encode a bearing struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bearing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bearing_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bearing_t* bearing)
{
	return mavlink_msg_bearing_pack_chan(system_id, component_id, chan, msg, bearing->bearing, bearing->lat, bearing->lon, bearing->alt);
}

/**
 * @brief Send a bearing message
 * @param chan MAVLink channel to send the message
 *
 * @param bearing The calculated bearing
 * @param lat Latitude where the measurement was taken
 * @param lon Longitutde where the measurement was taken
 * @param alt Altitude of the measurement
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bearing_send(mavlink_channel_t chan, double bearing, int32_t lat, int32_t lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BEARING_LEN];
	_mav_put_double(buf, 0, bearing);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING, buf, MAVLINK_MSG_ID_BEARING_LEN, MAVLINK_MSG_ID_BEARING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING, buf, MAVLINK_MSG_ID_BEARING_LEN);
#endif
#else
	mavlink_bearing_t packet;
	packet.bearing = bearing;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING, (const char *)&packet, MAVLINK_MSG_ID_BEARING_LEN, MAVLINK_MSG_ID_BEARING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING, (const char *)&packet, MAVLINK_MSG_ID_BEARING_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_BEARING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bearing_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double bearing, int32_t lat, int32_t lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_double(buf, 0, bearing);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING, buf, MAVLINK_MSG_ID_BEARING_LEN, MAVLINK_MSG_ID_BEARING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING, buf, MAVLINK_MSG_ID_BEARING_LEN);
#endif
#else
	mavlink_bearing_t *packet = (mavlink_bearing_t *)msgbuf;
	packet->bearing = bearing;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING, (const char *)packet, MAVLINK_MSG_ID_BEARING_LEN, MAVLINK_MSG_ID_BEARING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BEARING, (const char *)packet, MAVLINK_MSG_ID_BEARING_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE BEARING UNPACKING


/**
 * @brief Get field bearing from bearing message
 *
 * @return The calculated bearing
 */
static inline double mavlink_msg_bearing_get_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field lat from bearing message
 *
 * @return Latitude where the measurement was taken
 */
static inline int32_t mavlink_msg_bearing_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from bearing message
 *
 * @return Longitutde where the measurement was taken
 */
static inline int32_t mavlink_msg_bearing_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from bearing message
 *
 * @return Altitude of the measurement
 */
static inline float mavlink_msg_bearing_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a bearing message into a struct
 *
 * @param msg The message to decode
 * @param bearing C-struct to decode the message contents into
 */
static inline void mavlink_msg_bearing_decode(const mavlink_message_t* msg, mavlink_bearing_t* bearing)
{
#if MAVLINK_NEED_BYTE_SWAP
	bearing->bearing = mavlink_msg_bearing_get_bearing(msg);
	bearing->lat = mavlink_msg_bearing_get_lat(msg);
	bearing->lon = mavlink_msg_bearing_get_lon(msg);
	bearing->alt = mavlink_msg_bearing_get_alt(msg);
#else
	memcpy(bearing, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_BEARING_LEN);
#endif
}
