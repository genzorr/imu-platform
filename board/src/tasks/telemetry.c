/*
 * telemetry.c
 *
 *  Created on: Jul 19, 2019
 *      Author: michael
 */
#include <stdint.h>
#include "diag/Trace.h"

#include "state.h"
#include "nRF24L01.h"
#include "mavmessages/mavlink.h"


uint8_t mavlink_msg_state_send()
{
	mavlink_state_t msg_state;
	msg_state.time = (float)HAL_GetTick() / 1000;
//taskENTER_CRITICAL();
	msg_state.MPU_state = state_system.MPU_state;
	msg_state.NRF_state = state_system.NRF_state;
//taskEXIT_CRITICAL();

	mavlink_message_t msg;
	uint16_t len = mavlink_msg_state_encode(0x00, 0x00, &msg, &msg_state);
	uint8_t buffer[50];
	len = mavlink_msg_to_send_buffer(buffer, &msg);
	uint8_t error = nRF24L01_send(&spi_nRF24L01, buffer, len, 1);

	return error;
}


uint8_t mavlink_msg_imu_rsc_send()
{
	mavlink_imu_rsc_t msg_imu_rsc;
	msg_imu_rsc.time = (float)HAL_GetTick() / 1000;
//taskENTER_CRITICAL();
	for (int i = 0; i < 3; i++)
	{
		msg_imu_rsc.accel[i] = stateIMU_rsc.accel[i];
		msg_imu_rsc.gyro[i] = stateIMU_rsc.gyro[i];
		msg_imu_rsc.magn[i] = stateIMU_rsc.magn[i];
	}
//taskEXIT_CRITICAL();

	mavlink_message_t msg;
	uint16_t len = mavlink_msg_imu_rsc_encode(0x00, 0x00, &msg, &msg_imu_rsc);
	uint8_t buffer[100];
	len = mavlink_msg_to_send_buffer(buffer, &msg);
	uint8_t error = nRF24L01_send(&spi_nRF24L01, buffer, len, 1);

	return error;
}

uint8_t mavlink_msg_imu_isc_send()
{
	mavlink_imu_isc_t msg_imu_isc;
	msg_imu_isc.time = (float)HAL_GetTick() / 1000;
//taskENTER_CRITICAL();
	for (int i = 0; i < 3; i++)
	{
		msg_imu_isc.accel[i] = stateIMU_isc.accel[i];
		msg_imu_isc.magn[i] = stateIMU_isc.magn[i];
	}
	for (int j = 0; j < 4; j++) {
		msg_imu_isc.quaternion[j] = stateIMU_isc.quaternion[j];
	}
//taskEXIT_CRITICAL();

	mavlink_message_t msg;
	uint16_t len = mavlink_msg_imu_isc_encode(0x00, 0x00, &msg, &msg_imu_isc);
	uint8_t buffer[100];
	len = mavlink_msg_to_send_buffer(buffer, &msg);
	uint8_t error = nRF24L01_send(&spi_nRF24L01, buffer, len, 1);

	return error;
}


void TM_Init()
{
	uint8_t error = nRF24L01_init(&spi_nRF24L01);
	state_system.NRF_state = error;
	trace_printf("nRF: %d\n", error);
}

