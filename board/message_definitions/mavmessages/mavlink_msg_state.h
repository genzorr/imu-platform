#pragma once
// MESSAGE STATE PACKING

#define MAVLINK_MSG_ID_STATE 150

MAVPACKED(
typedef struct __mavlink_state_t {
 float time; /*<  current time*/
 uint8_t MPU_state; /*<  state of MPU9255*/
 uint8_t NRF_state; /*<  state of nRF24L01*/
}) mavlink_state_t;

#define MAVLINK_MSG_ID_STATE_LEN 6
#define MAVLINK_MSG_ID_STATE_MIN_LEN 6
#define MAVLINK_MSG_ID_150_LEN 6
#define MAVLINK_MSG_ID_150_MIN_LEN 6

#define MAVLINK_MSG_ID_STATE_CRC 18
#define MAVLINK_MSG_ID_150_CRC 18



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATE { \
    150, \
    "STATE", \
    3, \
    {  { "time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_state_t, time) }, \
         { "MPU_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_state_t, MPU_state) }, \
         { "NRF_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_state_t, NRF_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATE { \
    "STATE", \
    3, \
    {  { "time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_state_t, time) }, \
         { "MPU_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_state_t, MPU_state) }, \
         { "NRF_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_state_t, NRF_state) }, \
         } \
}
#endif

/**
 * @brief Pack a state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time  current time
 * @param MPU_state  state of MPU9255
 * @param NRF_state  state of nRF24L01
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float time, uint8_t MPU_state, uint8_t NRF_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_LEN];
    _mav_put_float(buf, 0, time);
    _mav_put_uint8_t(buf, 4, MPU_state);
    _mav_put_uint8_t(buf, 5, NRF_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_LEN);
#else
    mavlink_state_t packet;
    packet.time = time;
    packet.MPU_state = MPU_state;
    packet.NRF_state = NRF_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATE_MIN_LEN, MAVLINK_MSG_ID_STATE_LEN, MAVLINK_MSG_ID_STATE_CRC);
}

/**
 * @brief Pack a state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time  current time
 * @param MPU_state  state of MPU9255
 * @param NRF_state  state of nRF24L01
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float time,uint8_t MPU_state,uint8_t NRF_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_LEN];
    _mav_put_float(buf, 0, time);
    _mav_put_uint8_t(buf, 4, MPU_state);
    _mav_put_uint8_t(buf, 5, NRF_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_LEN);
#else
    mavlink_state_t packet;
    packet.time = time;
    packet.MPU_state = MPU_state;
    packet.NRF_state = NRF_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATE_MIN_LEN, MAVLINK_MSG_ID_STATE_LEN, MAVLINK_MSG_ID_STATE_CRC);
}

/**
 * @brief Encode a state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_state_t* state)
{
    return mavlink_msg_state_pack(system_id, component_id, msg, state->time, state->MPU_state, state->NRF_state);
}

/**
 * @brief Encode a state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_state_t* state)
{
    return mavlink_msg_state_pack_chan(system_id, component_id, chan, msg, state->time, state->MPU_state, state->NRF_state);
}

/**
 * @brief Send a state message
 * @param chan MAVLink channel to send the message
 *
 * @param time  current time
 * @param MPU_state  state of MPU9255
 * @param NRF_state  state of nRF24L01
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_send(mavlink_channel_t chan, float time, uint8_t MPU_state, uint8_t NRF_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_LEN];
    _mav_put_float(buf, 0, time);
    _mav_put_uint8_t(buf, 4, MPU_state);
    _mav_put_uint8_t(buf, 5, NRF_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE, buf, MAVLINK_MSG_ID_STATE_MIN_LEN, MAVLINK_MSG_ID_STATE_LEN, MAVLINK_MSG_ID_STATE_CRC);
#else
    mavlink_state_t packet;
    packet.time = time;
    packet.MPU_state = MPU_state;
    packet.NRF_state = NRF_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE, (const char *)&packet, MAVLINK_MSG_ID_STATE_MIN_LEN, MAVLINK_MSG_ID_STATE_LEN, MAVLINK_MSG_ID_STATE_CRC);
#endif
}

/**
 * @brief Send a state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_state_send_struct(mavlink_channel_t chan, const mavlink_state_t* state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_state_send(chan, state->time, state->MPU_state, state->NRF_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE, (const char *)state, MAVLINK_MSG_ID_STATE_MIN_LEN, MAVLINK_MSG_ID_STATE_LEN, MAVLINK_MSG_ID_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float time, uint8_t MPU_state, uint8_t NRF_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, time);
    _mav_put_uint8_t(buf, 4, MPU_state);
    _mav_put_uint8_t(buf, 5, NRF_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE, buf, MAVLINK_MSG_ID_STATE_MIN_LEN, MAVLINK_MSG_ID_STATE_LEN, MAVLINK_MSG_ID_STATE_CRC);
#else
    mavlink_state_t *packet = (mavlink_state_t *)msgbuf;
    packet->time = time;
    packet->MPU_state = MPU_state;
    packet->NRF_state = NRF_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE, (const char *)packet, MAVLINK_MSG_ID_STATE_MIN_LEN, MAVLINK_MSG_ID_STATE_LEN, MAVLINK_MSG_ID_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE STATE UNPACKING


/**
 * @brief Get field time from state message
 *
 * @return  current time
 */
static inline float mavlink_msg_state_get_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field MPU_state from state message
 *
 * @return  state of MPU9255
 */
static inline uint8_t mavlink_msg_state_get_MPU_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field NRF_state from state message
 *
 * @return  state of nRF24L01
 */
static inline uint8_t mavlink_msg_state_get_NRF_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a state message into a struct
 *
 * @param msg The message to decode
 * @param state C-struct to decode the message contents into
 */
static inline void mavlink_msg_state_decode(const mavlink_message_t* msg, mavlink_state_t* state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    state->time = mavlink_msg_state_get_time(msg);
    state->MPU_state = mavlink_msg_state_get_MPU_state(msg);
    state->NRF_state = mavlink_msg_state_get_NRF_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATE_LEN? msg->len : MAVLINK_MSG_ID_STATE_LEN;
        memset(state, 0, MAVLINK_MSG_ID_STATE_LEN);
    memcpy(state, _MAV_PAYLOAD(msg), len);
#endif
}
