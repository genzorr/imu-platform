#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_cortex.h>
#include <stm32f4xx_hal_i2c.h>
#include <stm32f4xx_hal_usart.h>
#include <stm32f4xx_hal_dma.h>
#include <stm32f4xx_hal_gpio.h>


#include "diag/Trace.h"
#include "state.h"
#include "sensors.h"
#include "telemetry.h"
#include "MPU9255.h"
#include "lsm6ds3.h"
#include "nRF24L01.h"
#include "xprintf.h"
#include "dbgu.h"

//#include "mavmessages/mavlink.h"


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


// глобальные структуры
USART_HandleTypeDef usart_dbg;

//stateGPS_t 			stateGPS;
stateIMU_rsc_t 		stateIMU_rsc;
stateIMU_isc_t 		stateIMU_isc;
state_system_t 		state_system;
state_zero_t		state_zero;

stateIMU_isc_t		stateIMU_isc_prev;
state_system_t		state_system_prev;


void _init_leds();



int main(int argc, char* argv[])
{
	//	Global structures init
	memset(&stateIMU_rsc, 			0x00, sizeof(stateIMU_rsc));
	memset(&stateIMU_isc, 			0x00, sizeof(stateIMU_isc));
	memset(&state_system, 			0x00, sizeof(state_system));

	memset(&stateIMU_isc_prev, 		0x00, sizeof(stateIMU_isc_prev));
	memset(&state_system_prev, 		0x00, sizeof(state_system_prev));

	state_system.MPU_state = 111;
	state_system.NRF_state = 111;


	_init_leds();

	if (DBGU)
		_init_usart_dbg();

	//	Peripheral initialization
	if (IMU)
	{
		IMU_Init();
		get_staticShifts();
	}

	if (RF)
		TM_Init();


	for (; ; )
	{
		#if (IMU)
//			IMU_updateDataAll();
//			_IMUtask_updateData();
			struct lsm6ds3_raw_data_s rd = {{0,0,0},{0,0,0}};
			float ddx[3] = {}, ddg[3] = {};
			lsm6ds3_gxl_pull(&hlsm6ds3, &rd);
			lsm6ds3_scale_xl(&hlsm6ds3.conf.xl, rd.xl, ddx, 3);
			lsm6ds3_scale_g(&hlsm6ds3.conf.g, rd.g, ddg, 3);

			stateIMU_rsc.accel[0] = ddx[0];
			stateIMU_rsc.accel[1] = ddx[1];
			stateIMU_rsc.accel[2] = ddx[2];

			stateIMU_rsc.gyro[0] = ddg[0];
			stateIMU_rsc.gyro[1] = ddg[1];
			stateIMU_rsc.gyro[2] = ddg[2];

		#endif

		#if (RF)
			mavlink_msg_state_send();
			mavlink_msg_imu_rsc_send();
			mavlink_msg_imu_isc_send();
		#endif

		#if (DBGU)
//			mavlink_message_t msg;
//			//	uint16_t len = mavlink_msg_imu_isc_encode(0x00, 0x00, &msg, &msg_imu_isc);
//			uint16_t len = mavlink_msg_imu_isc_pack(1, 1, &msg, stateIMU_isc.accel, stateIMU_isc.magn, stateIMU_isc.quaternion, (float)HAL_GetTick() / 1000);
//			uint8_t buffer[100];
//			len = mavlink_msg_to_send_buffer(buffer, &msg);
//			HAL_USART_Transmit(&usart_dbg, buffer, len, 1);
		#endif

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);

		HAL_Delay(50);
	}

	return 0;
}


void _init_leds()
{
	__GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &gpio);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
