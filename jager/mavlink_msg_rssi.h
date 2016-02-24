// MESSAGE RSSI PACKING

#define MAVLINK_MSG_ID_RSSI 215

typedef struct __mavlink_rssi_t
{
 uint64_t timestamp_usec; /*< The timestamp of the measurement (from the pixhawk)*/
 int32_t rssi_value; /*< The rssi value measured*/
 int32_t rssi_value2; /*< The rssi value measured (for the second antenna)*/
 int32_t lat; /*< Latitude where the measurement was taken*/
 int32_t lon; /*< Longitutde where the measurement was taken*/
 float alt; /*< Altitude of the measurement*/
 int16_t heading; /*< The heading for the current measurement*/
} mavlink_rssi_t;

#define MAVLINK_MSG_ID_RSSI_LEN 30
#define MAVLINK_MSG_ID_215_LEN 30

#define MAVLINK_MSG_ID_RSSI_CRC 227
#define MAVLINK_MSG_ID_215_CRC 227



#define MAVLINK_MESSAGE_INFO_RSSI { \
	"RSSI", \
	7, \
	{  { "timestamp_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rssi_t, timestamp_usec) }, \
         { "rssi_value", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_rssi_t, rssi_value) }, \
         { "rssi_value2", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_rssi_t, rssi_value2) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_rssi_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_rssi_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rssi_t, alt) }, \
         { "heading", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_rssi_t, heading) }, \
         } \
}


/**
 * @brief Pack a rssi message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_usec The timestamp of the measurement (from the pixhawk)
 * @param rssi_value The rssi value measured
 * @param rssi_value2 The rssi value measured (for the second antenna)
 * @param heading The heading for the current measurement
 * @param lat Latitude where the measurement was taken
 * @param lon Longitutde where the measurement was taken
 * @param alt Altitude of the measurement
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rssi_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp_usec, int32_t rssi_value, int32_t rssi_value2, int16_t heading, int32_t lat, int32_t lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RSSI_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_int32_t(buf, 8, rssi_value);
	_mav_put_int32_t(buf, 12, rssi_value2);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_float(buf, 24, alt);
	_mav_put_int16_t(buf, 28, heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RSSI_LEN);
#else
	mavlink_rssi_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.rssi_value = rssi_value;
	packet.rssi_value2 = rssi_value2;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.heading = heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RSSI_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RSSI;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RSSI_LEN, MAVLINK_MSG_ID_RSSI_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RSSI_LEN);
#endif
}

/**
 * @brief Pack a rssi message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_usec The timestamp of the measurement (from the pixhawk)
 * @param rssi_value The rssi value measured
 * @param rssi_value2 The rssi value measured (for the second antenna)
 * @param heading The heading for the current measurement
 * @param lat Latitude where the measurement was taken
 * @param lon Longitutde where the measurement was taken
 * @param alt Altitude of the measurement
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rssi_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp_usec,int32_t rssi_value,int32_t rssi_value2,int16_t heading,int32_t lat,int32_t lon,float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RSSI_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_int32_t(buf, 8, rssi_value);
	_mav_put_int32_t(buf, 12, rssi_value2);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_float(buf, 24, alt);
	_mav_put_int16_t(buf, 28, heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RSSI_LEN);
#else
	mavlink_rssi_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.rssi_value = rssi_value;
	packet.rssi_value2 = rssi_value2;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.heading = heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RSSI_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RSSI;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RSSI_LEN, MAVLINK_MSG_ID_RSSI_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RSSI_LEN);
#endif
}

/**
 * @brief Encode a rssi struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rssi C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rssi_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rssi_t* rssi)
{
	return mavlink_msg_rssi_pack(system_id, component_id, msg, rssi->timestamp_usec, rssi->rssi_value, rssi->rssi_value2, rssi->heading, rssi->lat, rssi->lon, rssi->alt);
}

/**
 * @brief Encode a rssi struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rssi C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rssi_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rssi_t* rssi)
{
	return mavlink_msg_rssi_pack_chan(system_id, component_id, chan, msg, rssi->timestamp_usec, rssi->rssi_value, rssi->rssi_value2, rssi->heading, rssi->lat, rssi->lon, rssi->alt);
}

/**
 * @brief Send a rssi message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_usec The timestamp of the measurement (from the pixhawk)
 * @param rssi_value The rssi value measured
 * @param rssi_value2 The rssi value measured (for the second antenna)
 * @param heading The heading for the current measurement
 * @param lat Latitude where the measurement was taken
 * @param lon Longitutde where the measurement was taken
 * @param alt Altitude of the measurement
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rssi_send(mavlink_channel_t chan, uint64_t timestamp_usec, int32_t rssi_value, int32_t rssi_value2, int16_t heading, int32_t lat, int32_t lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RSSI_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_int32_t(buf, 8, rssi_value);
	_mav_put_int32_t(buf, 12, rssi_value2);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_float(buf, 24, alt);
	_mav_put_int16_t(buf, 28, heading);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RSSI, buf, MAVLINK_MSG_ID_RSSI_LEN, MAVLINK_MSG_ID_RSSI_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RSSI, buf, MAVLINK_MSG_ID_RSSI_LEN);
#endif
#else
	mavlink_rssi_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.rssi_value = rssi_value;
	packet.rssi_value2 = rssi_value2;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.heading = heading;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RSSI, (const char *)&packet, MAVLINK_MSG_ID_RSSI_LEN, MAVLINK_MSG_ID_RSSI_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RSSI, (const char *)&packet, MAVLINK_MSG_ID_RSSI_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RSSI_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rssi_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_usec, int32_t rssi_value, int32_t rssi_value2, int16_t heading, int32_t lat, int32_t lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_int32_t(buf, 8, rssi_value);
	_mav_put_int32_t(buf, 12, rssi_value2);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_float(buf, 24, alt);
	_mav_put_int16_t(buf, 28, heading);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RSSI, buf, MAVLINK_MSG_ID_RSSI_LEN, MAVLINK_MSG_ID_RSSI_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RSSI, buf, MAVLINK_MSG_ID_RSSI_LEN);
#endif
#else
	mavlink_rssi_t *packet = (mavlink_rssi_t *)msgbuf;
	packet->timestamp_usec = timestamp_usec;
	packet->rssi_value = rssi_value;
	packet->rssi_value2 = rssi_value2;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->heading = heading;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RSSI, (const char *)packet, MAVLINK_MSG_ID_RSSI_LEN, MAVLINK_MSG_ID_RSSI_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RSSI, (const char *)packet, MAVLINK_MSG_ID_RSSI_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RSSI UNPACKING


/**
 * @brief Get field timestamp_usec from rssi message
 *
 * @return The timestamp of the measurement (from the pixhawk)
 */
static inline uint64_t mavlink_msg_rssi_get_timestamp_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field rssi_value from rssi message
 *
 * @return The rssi value measured
 */
static inline int32_t mavlink_msg_rssi_get_rssi_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field rssi_value2 from rssi message
 *
 * @return The rssi value measured (for the second antenna)
 */
static inline int32_t mavlink_msg_rssi_get_rssi_value2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field heading from rssi message
 *
 * @return The heading for the current measurement
 */
static inline int16_t mavlink_msg_rssi_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field lat from rssi message
 *
 * @return Latitude where the measurement was taken
 */
static inline int32_t mavlink_msg_rssi_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field lon from rssi message
 *
 * @return Longitutde where the measurement was taken
 */
static inline int32_t mavlink_msg_rssi_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field alt from rssi message
 *
 * @return Altitude of the measurement
 */
static inline float mavlink_msg_rssi_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a rssi message into a struct
 *
 * @param msg The message to decode
 * @param rssi C-struct to decode the message contents into
 */
static inline void mavlink_msg_rssi_decode(const mavlink_message_t* msg, mavlink_rssi_t* rssi)
{
#if MAVLINK_NEED_BYTE_SWAP
	rssi->timestamp_usec = mavlink_msg_rssi_get_timestamp_usec(msg);
	rssi->rssi_value = mavlink_msg_rssi_get_rssi_value(msg);
	rssi->rssi_value2 = mavlink_msg_rssi_get_rssi_value2(msg);
	rssi->lat = mavlink_msg_rssi_get_lat(msg);
	rssi->lon = mavlink_msg_rssi_get_lon(msg);
	rssi->alt = mavlink_msg_rssi_get_alt(msg);
	rssi->heading = mavlink_msg_rssi_get_heading(msg);
#else
	memcpy(rssi, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RSSI_LEN);
#endif
}
