#include <stdint.h>
#include "diag/Trace.h"

#include "state.h"
#include "nRF24L01.h"
#include "mavmessages/mavlink.h"

uint8_t mavlink_msg_state_send(void);
uint8_t mavlink_msg_imu_rsc_send(void);
uint8_t mavlink_msg_imu_isc_send(void);

void TM_Init(void);


USART_HandleTypeDef	usart_data;


uint8_t mavlink_msg_state_send(void)
{
	uint8_t error = 0;

	mavlink_state_t msg_state;
	msg_state.time = (float)HAL_GetTick() / 1000;
//taskENTER_CRITICAL();
	msg_state.MPU_state = state_system.MPU_state;
	msg_state.NRF_state = state_system.NRF_state;
//taskEXIT_CRITICAL();

	mavlink_message_t msg;
	uint16_t len = mavlink_msg_state_encode(1, 1, &msg, &msg_state);
	uint8_t buffer[50] = {};
	len = mavlink_msg_to_send_buffer(buffer, &msg);

	if (RF)
		error = nRF24L01_send(&spi_nRF24L01, buffer, len, 1);
	if (UDATA)
		error |= HAL_USART_Transmit(&usart_data, buffer, len, 10);

	return error;
}


uint8_t mavlink_msg_imu_rsc_send(void)
{
	uint8_t error = 0;

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
	uint16_t len = mavlink_msg_imu_rsc_encode(1, 1, &msg, &msg_imu_rsc);
	uint8_t buffer[100];
	len = mavlink_msg_to_send_buffer(buffer, &msg);

	if (RF)
		error = nRF24L01_send(&spi_nRF24L01, buffer, len, 1);
	if (UDATA)
		error |= HAL_USART_Transmit(&usart_data, buffer, len, 10);

	return error;
}

uint8_t mavlink_msg_imu_isc_send(void)
{
	uint8_t error = 0;

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
	uint16_t len = mavlink_msg_imu_isc_encode(1, 1, &msg, &msg_imu_isc);
	uint8_t buffer[100];
	len = mavlink_msg_to_send_buffer(buffer, &msg);

	if (RF)
		error = nRF24L01_send(&spi_nRF24L01, buffer, len, 1);
	if (UDATA)
		error |= HAL_USART_Transmit(&usart_data, buffer, len, 10);

	return error;
}


void TM_Init(void)
{
	uint8_t error = 0;

	if (RF)
	{
		error = nRF24L01_init(&spi_nRF24L01);
		state_system.NRF_state = error;
		trace_printf("nRF: %d\n", error);
	}
	if (UDATA)
	{
		usart_data.Instance = USART1;
		usart_data.Init.BaudRate = 115200;
		usart_data.Init.WordLength = UART_WORDLENGTH_8B;
		usart_data.Init.StopBits = UART_STOPBITS_1;
		usart_data.Init.Parity = UART_PARITY_NONE;
		usart_data.Init.Mode = UART_MODE_TX_RX;

		error = HAL_USART_Init(&usart_data);
		trace_printf("uDATA: %d\n", error);
	}
}

