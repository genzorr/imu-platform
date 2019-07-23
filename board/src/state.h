/*
 * state.h
 *
 *  Created on: 11 нояб. 2017 г.
 *      Author: Korr237i
 *
 *  This header included main structure to share data from module to module
 */

#ifndef STATE_H_
#define STATE_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dma.h"

#include "quaternion.h"

#define DBGU			1
//#define SD 			0
#define RF				1
#define IMU				0
#define IMU_CALIBRATION	1
//#define GPS			0


// if error set value and go to end
#define PROCESS_ERROR(x) if (0 != (error = (x))) { goto end; }


/*#################################################*/
/*################## СТРУКТУРЫ ###################*/
/*#################################################*/

/*
 * 	Структуры состояний аппарата
 */
typedef struct {
	//GPS data
	float coordinates[2];
	float speed;
	float course;
	float time;
} stateGPS_t;


//	IMU data in RSC (related system of coordinates)
typedef struct {
	float accel[3];
	float gyro[3];
	float magn[3];
} stateIMU_rsc_t;


//	orientation and position of device in ISC (inertial system of coordinates)
typedef struct {
	//	IMU data
	float accel[3];
	float magn[3];

	//	position
	float velocities[3];
	float coordinates[3];

	//	orientation
	float quaternion[4];
} stateIMU_isc_t;

//	system parameters
typedef struct {
	uint8_t MPU_state;		//	state of MPU9255
	uint8_t SD_state;		//	state of SD
	uint8_t NRF_state;		//	state of NRF24L01
	uint8_t GPS_state;		//	state of GPS-module

	float time;				//	current time
	float magnASA[3];
} state_system_t;


typedef struct {
	//	zero params; this fields should be filled when device started it`s work
	double zero_pressure;
	float zero_quaternion[4];
	float zero_GPS[2];
	float gyro_staticShift[3];
	float accel_staticShift[3];
} state_zero_t;


/*##################################################*/
/*################### ПЕРЕМЕННЫЕ ###################*/
/*##################################################*/

extern USART_HandleTypeDef	usart_dbg;
extern SPI_HandleTypeDef	spi_nRF24L01;
extern I2C_HandleTypeDef 	i2c_mpu9255;

// глобальные структуры
extern stateGPS_t 			stateGPS;
extern stateIMU_rsc_t 		stateIMU_rsc;
extern stateIMU_isc_t 		stateIMU_isc;
extern state_system_t 		state_system;
extern state_zero_t			state_zero;

extern stateIMU_isc_t		stateIMU_isc_prev;
extern state_system_t		state_system_prev;


#endif /* STATE_H_ */
