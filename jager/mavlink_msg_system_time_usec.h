// MESSAGE SYSTEM_TIME_USEC PACKING

#define MAVLINK_MSG_ID_SYSTEM_TIME_USEC 214

typedef struct __mavlink_system_time_usec_t
{
 uint64_t time_unix_usec; /*< Timestamp of the master clock in microseconds since UNIX epoch.*/
 uint64_t time_boot_usec; /*< Timestamp of the component clock since boot time in microseconds.*/
} mavlink_system_time_usec_t;

#define MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN 16
#define MAVLINK_MSG_ID_214_LEN 16

#define MAVLINK_MSG_ID_SYSTEM_TIME_USEC_CRC 143
#define MAVLINK_MSG_ID_214_CRC 143



#define MAVLINK_MESSAGE_INFO_SYSTEM_TIME_USEC { \
	"SYSTEM_TIME_USEC", \
	2, \
	{  { "time_unix_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_system_time_usec_t, time_unix_usec) }, \
         { "time_boot_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_system_time_usec_t, time_boot_usec) }, \
         } \
}


/**
 * @brief Pack a system_time_usec message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_usec Timestamp of the component clock since boot time in microseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_time_usec_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_unix_usec, uint64_t time_boot_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN];
	_mav_put_uint64_t(buf, 0, time_unix_usec);
	_mav_put_uint64_t(buf, 8, time_boot_usec);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#else
	mavlink_system_time_usec_t packet;
	packet.time_unix_usec = time_unix_usec;
	packet.time_boot_usec = time_boot_usec;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME_USEC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#endif
}

/**
 * @brief Pack a system_time_usec message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_usec Timestamp of the component clock since boot time in microseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_time_usec_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_unix_usec,uint64_t time_boot_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN];
	_mav_put_uint64_t(buf, 0, time_unix_usec);
	_mav_put_uint64_t(buf, 8, time_boot_usec);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#else
	mavlink_system_time_usec_t packet;
	packet.time_unix_usec = time_unix_usec;
	packet.time_boot_usec = time_boot_usec;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME_USEC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#endif
}

/**
 * @brief Encode a system_time_usec struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_time_usec C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_time_usec_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_system_time_usec_t* system_time_usec)
{
	return mavlink_msg_system_time_usec_pack(system_id, component_id, msg, system_time_usec->time_unix_usec, system_time_usec->time_boot_usec);
}

/**
 * @brief Encode a system_time_usec struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param system_time_usec C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_time_usec_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_system_time_usec_t* system_time_usec)
{
	return mavlink_msg_system_time_usec_pack_chan(system_id, component_id, chan, msg, system_time_usec->time_unix_usec, system_time_usec->time_boot_usec);
}

/**
 * @brief Send a system_time_usec message
 * @param chan MAVLink channel to send the message
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_usec Timestamp of the component clock since boot time in microseconds.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_system_time_usec_send(mavlink_channel_t chan, uint64_t time_unix_usec, uint64_t time_boot_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN];
	_mav_put_uint64_t(buf, 0, time_unix_usec);
	_mav_put_uint64_t(buf, 8, time_boot_usec);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC, buf, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC, buf, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#endif
#else
	mavlink_system_time_usec_t packet;
	packet.time_unix_usec = time_unix_usec;
	packet.time_boot_usec = time_boot_usec;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC, (const char *)&packet, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC, (const char *)&packet, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_system_time_usec_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_unix_usec, uint64_t time_boot_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_unix_usec);
	_mav_put_uint64_t(buf, 8, time_boot_usec);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC, buf, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC, buf, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#endif
#else
	mavlink_system_time_usec_t *packet = (mavlink_system_time_usec_t *)msgbuf;
	packet->time_unix_usec = time_unix_usec;
	packet->time_boot_usec = time_boot_usec;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC, (const char *)packet, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_USEC, (const char *)packet, MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SYSTEM_TIME_USEC UNPACKING


/**
 * @brief Get field time_unix_usec from system_time_usec message
 *
 * @return Timestamp of the master clock in microseconds since UNIX epoch.
 */
static inline uint64_t mavlink_msg_system_time_usec_get_time_unix_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_boot_usec from system_time_usec message
 *
 * @return Timestamp of the component clock since boot time in microseconds.
 */
static inline uint64_t mavlink_msg_system_time_usec_get_time_boot_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Decode a system_time_usec message into a struct
 *
 * @param msg The message to decode
 * @param system_time_usec C-struct to decode the message contents into
 */
static inline void mavlink_msg_system_time_usec_decode(const mavlink_message_t* msg, mavlink_system_time_usec_t* system_time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP
	system_time_usec->time_unix_usec = mavlink_msg_system_time_usec_get_time_unix_usec(msg);
	system_time_usec->time_boot_usec = mavlink_msg_system_time_usec_get_time_boot_usec(msg);
#else
	memcpy(system_time_usec, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SYSTEM_TIME_USEC_LEN);
#endif
}
