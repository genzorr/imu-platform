/*
 * sensors.c
 *
 *  Created on: Jul 6, 2019
 *      Author: michael
 */
#include <math.h>
#include <string.h>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_i2c.h>

#include "diag/Trace.h"
#include "MadgwickAHRS.h"
#include "state.h"
#include "sensors.h"
#include "MPU9255.h"


I2C_HandleTypeDef 	i2c_mpu9255;


static uint8_t	get_gyro_staticShift(float* gyro_staticShift);
static uint8_t	get_accel_staticShift(float* gyro_staticShift, float* accel_staticShift);
void 			get_staticShifts();

void 			IMU_Init();
int 			IMU_updateDataAll();
void 			_IMUtask_updateData();


/**
  * @brief	Static function used to get gyro static shift
  * @param	gyro_staticShift	Array used to store that shift
  * @retval	Device's wire error
  */
static uint8_t get_gyro_staticShift(float* gyro_staticShift)
{
	uint8_t error = 0;
	uint16_t zero_orientCnt = 3000;

	int16_t accelData[3] = {0, 0, 0};
	int16_t gyroData[3] = {0, 0, 0};
	float gyro[3] = {0, 0, 0};

	//	Get static gyro shift
	for (int i = 0; i < zero_orientCnt; i++)
	{
		for (int k = 0; k < 3; k++)
		{
			accelData[k] = 0;
			gyroData[k] = 0;
			gyro[k] = 0;
		}

		//	Collect data
		PROCESS_ERROR(mpu9255_readIMU(accelData, gyroData));
		mpu9255_recalcGyro(gyroData, gyro);

		for (int m = 0; m < 3; m++)
			gyro_staticShift[m] += gyro[m];
	}
	for (int m = 0; m < 3; m++) {
		gyro_staticShift[m] /= zero_orientCnt;
	}
end:
	return error;
}


/**
  * @brief	Static function used to get accel static shift
  * @param	gyro_staticShift	Array used to get gyro shift from
  * @param	accel_staticShift	Array used to store accel shift
  * @retval	Device's wire error
  */
static uint8_t get_accel_staticShift(float* gyro_staticShift, float* accel_staticShift)
{
	uint8_t error = 0;
	uint16_t zero_orientCnt = 300;

	for (int i = 0; i < zero_orientCnt; i++)
	{
		int16_t accelData[3] = {0, 0, 0};
		int16_t gyroData[3] = {0, 0, 0};
		float accel[3] = {0, 0, 0};
		float gyro[3] = {0, 0, 0};

		//	Collect data
		PROCESS_ERROR(mpu9255_readIMU(accelData, gyroData));
		mpu9255_recalcGyro(gyroData, gyro);
		mpu9255_recalcAccel(accelData, accel);

		for (int k = 0; k < 3; k++) {
			gyro[k] -= gyro_staticShift[k];
		}

		//	Set accel static shift vector as (0,0,g)
		accel_staticShift[0] = 0;
		accel_staticShift[1] = 0;
		accel_staticShift[2] += sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
	}

	//	Divide shift by counter
	for (int m = 0; m < 3; m++)
		accel_staticShift[m] /= zero_orientCnt;

end:
	return error;
}


/**
  * @brief	Used to combine getting IMU static shifts
  */
void get_staticShifts()
{
	float gyro_staticShift[3] = {0, 0, 0};
	float accel_staticShift[3] = {0, 0, 0};

	if(IMU)
	{
		get_gyro_staticShift(gyro_staticShift);
		get_accel_staticShift(gyro_staticShift, accel_staticShift);
//	taskENTER_CRITICAL();
		for (int i = 0; i < 3; i++)
		{
			state_zero.gyro_staticShift[i] = gyro_staticShift[i];
			state_zero.accel_staticShift[i] = accel_staticShift[i];
		}
//	taskEXIT_CRITICAL();
	}
}


/**
  * @brief	Initializes I2C for IMU and IMU too
  */
void IMU_Init()
{
	if(IMU)
	{
		//	I2C init
		i2c_mpu9255.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		i2c_mpu9255.Init.ClockSpeed = 400000;
		i2c_mpu9255.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		i2c_mpu9255.Init.DutyCycle = I2C_DUTYCYCLE_2;
		i2c_mpu9255.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		i2c_mpu9255.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

		i2c_mpu9255.Init.OwnAddress1 = 0x00;

		i2c_mpu9255.Instance = I2C1;
		i2c_mpu9255.Mode = HAL_I2C_MODE_MASTER;

		int i2c_initError = HAL_I2C_Init(&i2c_mpu9255);
		trace_printf("i2c: %d\n", i2c_initError);

		if(i2c_initError != HAL_OK)
			return;


		//	MPU9255 init
		uint8_t mpu9255_initError = 0;
		mpu9255_initError = mpu9255_init(&i2c_mpu9255);
		if (mpu9255_initError != HAL_OK)
		{
			HAL_Delay(100);
			mpu9255_initError = mpu9255_init(&i2c_mpu9255);
		}

		trace_printf("mpu: %d\n", mpu9255_initError);
		state_system.MPU_state = mpu9255_initError;
	}
}


/**
  * @brief	Collects data from IMU, stores it and makes quat using S.Madgwick's algo
  * @retval	R/w IMU error
  */
int IMU_updateDataAll()
{
	int error = 0;

	//	Arrays
	int16_t accelData[3] = {0, 0, 0};
	int16_t gyroData[3] = {0, 0, 0};
	int16_t magnData[3] = {0, 0, 0};
	float accel[3] = {0, 0, 0};
	float gyro[3] = {0, 0, 0};
	float magn[3] = {0, 0, 0};

	if (IMU)
	{

	////////////////////////////////////////////////////
	/////////////////	GET IMU DATA  //////////////////
	////////////////////////////////////////////////////

		PROCESS_ERROR(mpu9255_readIMU(accelData, gyroData));
		PROCESS_ERROR(mpu9255_readCompass(magnData));

		//	Recalc data to floats
		mpu9255_recalcAccel(accelData, accel);
		mpu9255_recalcGyro(gyroData, gyro);
		mpu9255_recalcMagn(magnData, magn);

//	taskENTER_CRITICAL();
		float _time = (float)HAL_GetTick() / 1000;
		state_system.time = _time;
		//	пересчитываем их и записываем в структуры
		for (int k = 0; k < 3; k++) {
			stateIMU_rsc.accel[k] = accel[k];
			gyro[k] -= state_zero.gyro_staticShift[k];
			stateIMU_rsc.gyro[k] = gyro[k];
			stateIMU_rsc.magn[k] = magn[k];
		}
//	taskEXIT_CRITICAL();


	/////////////////////////////////////////////////////
	/////////////	UPDATE QUATERNION  //////////////////
	/////////////////////////////////////////////////////
		float quaternion[4] = {0, 0, 0, 0};

//	taskENTER_CRITICAL();
		float dt = _time - state_system_prev.time;
//	taskEXIT_CRITICAL();

		float beta = 0.041;
		MadgwickAHRSupdate(quaternion, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], magn[0], magn[1], magn[2], dt, beta);

		//	копируем кватернион в глобальную структуру
//	taskENTER_CRITICAL();
		stateIMU_isc.quaternion[0] = quaternion[0];
		stateIMU_isc.quaternion[1] = quaternion[1];
		stateIMU_isc.quaternion[2] = quaternion[2];
		stateIMU_isc.quaternion[3] = quaternion[3];
//	taskEXIT_CRITICAL();


	/////////////////////////////////////////////////////
	///////////  ROTATE VECTORS TO ISC  /////////////////
	/////////////////////////////////////////////////////
		float accel_ISC[3] = {0, 0, 0};
		float magn_ISC[3] = {0, 0, 0};

		//	Rotate accel and magn vectors to ISC
		vect_rotate(accel, quaternion, accel_ISC);
		vect_rotate(magn, quaternion, magn_ISC);

		//	Copy vectors to global structure
//	taskENTER_CRITICAL();
		for (int i = 0; i < 3; i++)
			accel_ISC[i] -= state_zero.accel_staticShift[i];

		stateIMU_isc.accel[0] = accel_ISC[0];
		stateIMU_isc.accel[1] = accel_ISC[1];
		stateIMU_isc.accel[2] = accel_ISC[2];
		stateIMU_isc.magn[0] = magn_ISC[0];
		stateIMU_isc.magn[1] = magn_ISC[1];
		stateIMU_isc.magn[2] = magn_ISC[2];
//	taskEXIT_CRITICAL();
	}

end:
	state_system.MPU_state = error;
	if (error)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

	return error;
}


/**
  * @brief	Special function for updating previous values structures by current values
  */
void _IMUtask_updateData()
{
//taskENTER_CRITICAL();
	memcpy(&stateIMU_isc_prev, 			&stateIMU_isc,			sizeof(stateIMU_isc));
	memcpy(&state_system_prev, 			&state_system,		 	sizeof(state_system));
//taskEXIT_CRITICAL();
}

