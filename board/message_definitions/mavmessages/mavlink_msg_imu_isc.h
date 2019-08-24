#pragma once
// MESSAGE IMU_isc PACKING

#define MAVLINK_MSG_ID_IMU_isc 152

MAVPACKED(
typedef struct __mavlink_imu_isc_t {
 float accel[3]; /*<  accelerations*/
 float magn[3]; /*<  compass (magnetometer) vector*/
 float quaternion[4]; /*<  quaternion for translating RSC to ISC*/
 float time; /*<  current time*/
}) mavlink_imu_isc_t;

#define MAVLINK_MSG_ID_IMU_isc_LEN 44
#define MAVLINK_MSG_ID_IMU_isc_MIN_LEN 44
#define MAVLINK_MSG_ID_152_LEN 44
#define MAVLINK_MSG_ID_152_MIN_LEN 44

#define MAVLINK_MSG_ID_IMU_isc_CRC 23
#define MAVLINK_MSG_ID_152_CRC 23

#define MAVLINK_MSG_IMU_isc_FIELD_ACCEL_LEN 3
#define MAVLINK_MSG_IMU_isc_FIELD_MAGN_LEN 3
#define MAVLINK_MSG_IMU_isc_FIELD_QUATERNION_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMU_isc { \
    152, \
    "IMU_isc", \
    4, \
    {  { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_imu_isc_t, accel) }, \
         { "magn", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_imu_isc_t, magn) }, \
         { "quaternion", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_imu_isc_t, quaternion) }, \
         { "time", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_imu_isc_t, time) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMU_isc { \
    "IMU_isc", \
    4, \
    {  { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_imu_isc_t, accel) }, \
         { "magn", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_imu_isc_t, magn) }, \
         { "quaternion", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_imu_isc_t, quaternion) }, \
         { "time", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_imu_isc_t, time) }, \
         } \
}
#endif

/**
 * @brief Pack a imu_isc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param accel  accelerations
 * @param magn  compass (magnetometer) vector
 * @param quaternion  quaternion for translating RSC to ISC
 * @param time  current time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_isc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *accel, const float *magn, const float *quaternion, float time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_isc_LEN];
    _mav_put_float(buf, 40, time);
    _mav_put_float_array(buf, 0, accel, 3);
    _mav_put_float_array(buf, 12, magn, 3);
    _mav_put_float_array(buf, 24, quaternion, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_isc_LEN);
#else
    mavlink_imu_isc_t packet;
    packet.time = time;
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet.magn, magn, sizeof(float)*3);
    mav_array_memcpy(packet.quaternion, quaternion, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_isc_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_isc;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_isc_MIN_LEN, MAVLINK_MSG_ID_IMU_isc_LEN, MAVLINK_MSG_ID_IMU_isc_CRC);
}

/**
 * @brief Pack a imu_isc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param accel  accelerations
 * @param magn  compass (magnetometer) vector
 * @param quaternion  quaternion for translating RSC to ISC
 * @param time  current time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_isc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *accel,const float *magn,const float *quaternion,float time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_isc_LEN];
    _mav_put_float(buf, 40, time);
    _mav_put_float_array(buf, 0, accel, 3);
    _mav_put_float_array(buf, 12, magn, 3);
    _mav_put_float_array(buf, 24, quaternion, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_isc_LEN);
#else
    mavlink_imu_isc_t packet;
    packet.time = time;
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet.magn, magn, sizeof(float)*3);
    mav_array_memcpy(packet.quaternion, quaternion, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_isc_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_isc;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_isc_MIN_LEN, MAVLINK_MSG_ID_IMU_isc_LEN, MAVLINK_MSG_ID_IMU_isc_CRC);
}

/**
 * @brief Encode a imu_isc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu_isc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_isc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_isc_t* imu_isc)
{
    return mavlink_msg_imu_isc_pack(system_id, component_id, msg, imu_isc->accel, imu_isc->magn, imu_isc->quaternion, imu_isc->time);
}

/**
 * @brief Encode a imu_isc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu_isc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_isc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_isc_t* imu_isc)
{
    return mavlink_msg_imu_isc_pack_chan(system_id, component_id, chan, msg, imu_isc->accel, imu_isc->magn, imu_isc->quaternion, imu_isc->time);
}

/**
 * @brief Send a imu_isc message
 * @param chan MAVLink channel to send the message
 *
 * @param accel  accelerations
 * @param magn  compass (magnetometer) vector
 * @param quaternion  quaternion for translating RSC to ISC
 * @param time  current time
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_isc_send(mavlink_channel_t chan, const float *accel, const float *magn, const float *quaternion, float time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_isc_LEN];
    _mav_put_float(buf, 40, time);
    _mav_put_float_array(buf, 0, accel, 3);
    _mav_put_float_array(buf, 12, magn, 3);
    _mav_put_float_array(buf, 24, quaternion, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_isc, buf, MAVLINK_MSG_ID_IMU_isc_MIN_LEN, MAVLINK_MSG_ID_IMU_isc_LEN, MAVLINK_MSG_ID_IMU_isc_CRC);
#else
    mavlink_imu_isc_t packet;
    packet.time = time;
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet.magn, magn, sizeof(float)*3);
    mav_array_memcpy(packet.quaternion, quaternion, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_isc, (const char *)&packet, MAVLINK_MSG_ID_IMU_isc_MIN_LEN, MAVLINK_MSG_ID_IMU_isc_LEN, MAVLINK_MSG_ID_IMU_isc_CRC);
#endif
}

/**
 * @brief Send a imu_isc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_imu_isc_send_struct(mavlink_channel_t chan, const mavlink_imu_isc_t* imu_isc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_imu_isc_send(chan, imu_isc->accel, imu_isc->magn, imu_isc->quaternion, imu_isc->time);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_isc, (const char *)imu_isc, MAVLINK_MSG_ID_IMU_isc_MIN_LEN, MAVLINK_MSG_ID_IMU_isc_LEN, MAVLINK_MSG_ID_IMU_isc_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMU_isc_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_isc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *accel, const float *magn, const float *quaternion, float time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 40, time);
    _mav_put_float_array(buf, 0, accel, 3);
    _mav_put_float_array(buf, 12, magn, 3);
    _mav_put_float_array(buf, 24, quaternion, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_isc, buf, MAVLINK_MSG_ID_IMU_isc_MIN_LEN, MAVLINK_MSG_ID_IMU_isc_LEN, MAVLINK_MSG_ID_IMU_isc_CRC);
#else
    mavlink_imu_isc_t *packet = (mavlink_imu_isc_t *)msgbuf;
    packet->time = time;
    mav_array_memcpy(packet->accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet->magn, magn, sizeof(float)*3);
    mav_array_memcpy(packet->quaternion, quaternion, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_isc, (const char *)packet, MAVLINK_MSG_ID_IMU_isc_MIN_LEN, MAVLINK_MSG_ID_IMU_isc_LEN, MAVLINK_MSG_ID_IMU_isc_CRC);
#endif
}
#endif

#endif

// MESSAGE IMU_isc UNPACKING


/**
 * @brief Get field accel from imu_isc message
 *
 * @return  accelerations
 */
static inline uint16_t mavlink_msg_imu_isc_get_accel(const mavlink_message_t* msg, float *accel)
{
    return _MAV_RETURN_float_array(msg, accel, 3,  0);
}

/**
 * @brief Get field magn from imu_isc message
 *
 * @return  compass (magnetometer) vector
 */
static inline uint16_t mavlink_msg_imu_isc_get_magn(const mavlink_message_t* msg, float *magn)
{
    return _MAV_RETURN_float_array(msg, magn, 3,  12);
}

/**
 * @brief Get field quaternion from imu_isc message
 *
 * @return  quaternion for translating RSC to ISC
 */
static inline uint16_t mavlink_msg_imu_isc_get_quaternion(const mavlink_message_t* msg, float *quaternion)
{
    return _MAV_RETURN_float_array(msg, quaternion, 4,  24);
}

/**
 * @brief Get field time from imu_isc message
 *
 * @return  current time
 */
static inline float mavlink_msg_imu_isc_get_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a imu_isc message into a struct
 *
 * @param msg The message to decode
 * @param imu_isc C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_isc_decode(const mavlink_message_t* msg, mavlink_imu_isc_t* imu_isc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_imu_isc_get_accel(msg, imu_isc->accel);
    mavlink_msg_imu_isc_get_magn(msg, imu_isc->magn);
    mavlink_msg_imu_isc_get_quaternion(msg, imu_isc->quaternion);
    imu_isc->time = mavlink_msg_imu_isc_get_time(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMU_isc_LEN? msg->len : MAVLINK_MSG_ID_IMU_isc_LEN;
        memset(imu_isc, 0, MAVLINK_MSG_ID_IMU_isc_LEN);
    memcpy(imu_isc, _MAV_PAYLOAD(msg), len);
#endif
}
