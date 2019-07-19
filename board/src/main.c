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
#include "sensors.h"
#include "telemetry.h"
//#include <FreeRTOS.h>
//#include <FreeRTOSConfig.h>
//#include <tasks/control_task.h>
//#include <tasks/sensors_task.h>
//#include <tasks/telemetry.h>
//#include "task.h"
//#include "mavlink/UNISAT/mavlink.h"


#include "state.h"
#include "MPU9255.h"
#include "nRF24L01.h"

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

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


void Init_led(){
	GPIO_InitTypeDef gpioc;
	gpioc.Mode = GPIO_MODE_OUTPUT_PP;
	gpioc.Pin = GPIO_PIN_12;
	gpioc.Pull = GPIO_NOPULL;
	gpioc.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &gpioc);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
}

void led(){
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12))
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
}


//void LED_task(){
//	for(;;){
//		taskENTER_CRITICAL();
//		if (/*(state_system.BMP_state == 0) & */(state_system.IMU_BMP_state == 0) & (state_system.MPU_state == 0)
//				& /*(state_system.GPS_state == 0) & (state_system.NRF_state == 0) & */ (state_system.SD_state == 0))
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
//		taskEXIT_CRITICAL();
//		vTaskDelay(30);
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
//		vTaskDelay(250);
//	}
//}


int main(int argc, char* argv[])
{
//	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	// Инициализация структур глобального состояния (в нашем случае просто заполняем их нулями)
//	memset(&stateGPS, 				0x00, sizeof(stateGPS));
	memset(&stateIMU_rsc, 			0x00, sizeof(stateIMU_rsc));
	memset(&stateIMU_isc, 			0x00, sizeof(stateIMU_isc));
	memset(&state_system, 			0x00, sizeof(state_system));

	memset(&stateIMU_isc_prev, 		0x00, sizeof(stateIMU_isc_prev));
	memset(&state_system_prev, 		0x00, sizeof(state_system_prev));

//	__enable_irq();
//
//	HAL_InitTick(15);
//
//	vTaskStartScheduler();

	//	usart_dbg init
	usart_dbg.Instance = USART1;
	usart_dbg.Init.BaudRate = 115200;
	usart_dbg.Init.WordLength = UART_WORDLENGTH_8B;
	usart_dbg.Init.StopBits = UART_STOPBITS_1;
	usart_dbg.Init.Parity = UART_PARITY_NONE;
	usart_dbg.Init.Mode = UART_MODE_TX_RX;
	HAL_USART_Init(&usart_dbg);

	state_msg_t msg;

	IMU_Init();
	get_staticShifts();

	__enable_irq();

	for (; ; )
	{
		IMU_updateDataAll();
		_IMUtask_updateData();

//		stateMsg_fill(&msg);
//		HAL_USART_Transmit(&usart_dbg, (uint8_t*)&msg, sizeof(msg), 10);

		mavlink_msg_imu_rsc_send();
		mavlink_msg_imu_isc_send();

//		HAL_Delay(93);
	}

	return 0;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
