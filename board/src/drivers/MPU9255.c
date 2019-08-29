/*
 * MPU9255.c
 *
 *  Created on: Jan, 01 2017
 *      Author: korr237i
 */

#include <stdio.h>
#include <math.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_i2c.h>
#include "diag/Trace.h"

//#include "FreeRTOS.h"
//#include "task.h"

#include "MadgwickAHRS.h"
#include "MPU9255.h"
#include "state.h"
#include "vector.h"

//	Accelerometer bias & transform matrix
#define X_ACCEL_OFFSET		-0.200394	// -0.20
#define Y_ACCEL_OFFSET		-0.130163
#define Z_ACCEL_OFFSET		-0.408357
#define XX_ACCEL_TRANSFORM_MATIX	 1.010241
#define YY_ACCEL_TRANSFORM_MATIX	 1.022768
#define ZZ_ACCEL_TRANSFORM_MATIX	 1.001105
#define XY_ACCEL_TRANSFORM_MATIX	 0.014717
#define XZ_ACCEL_TRANSFORM_MATIX	-0.003185
#define YZ_ACCEL_TRANSFORM_MATIX	-0.002456

//	Magnetometer bias & transform matrix
#define X_COMPAS_OFFSET		 160.0755586
#define Y_COMPAS_OFFSET		-82.119244
#define Z_COMPAS_OFFSET		-169.018839
#define XX_COMPAS_TRANSFORM_MATIX	 0.003574
#define YY_COMPAS_TRANSFORM_MATIX	 0.003495
#define ZZ_COMPAS_TRANSFORM_MATIX	 0.003714
#define XY_COMPAS_TRANSFORM_MATIX	 0.000012
#define XZ_COMPAS_TRANSFORM_MATIX	-0.000150
#define YZ_COMPAS_TRANSFORM_MATIX	-0.000197


int mpu9255_readRegister(mpu9255_address_t address, uint8_t reg_address, uint8_t* dataRead, uint8_t count);
int mpu9255_writeRegister(mpu9255_address_t address, uint8_t reg_address, uint8_t dataWrite);
int mpu9255_rewriteRegister(mpu9255_address_t address, uint8_t reg_address, uint8_t dataWrite);
int mpu9255_init(I2C_HandleTypeDef* hi2c);

static int16_t _swapBytesI16(int16_t value);

int mpu9255_readIMU(int16_t * raw_accelData, int16_t * raw_gyroData);
int mpu9255_readCompass(int16_t * raw_compassData);
void mpu9255_recalcAccel(const int16_t * raw_accelData, float * accelData);
void mpu9255_recalcGyro(const int16_t * raw_gyroData, float * gyroData);
void mpu9255_recalcMagn(const int16_t * raw_magnData, float * magnData);


/**
  * @brief	Reads device's register
  * @param	address		Device's address
  * @param	reg_address	Reading register address
  * @param	dataRead	Pointer to array to store data
  * @param	count		Number of reading bytes
  * @retval	Device's wire error
  */
int mpu9255_readRegister(mpu9255_address_t address, uint8_t reg_address, uint8_t* dataRead, uint8_t count)
{
	return HAL_I2C_Mem_Read(&i2c_mpu9255, address, reg_address, I2C_MEMADD_SIZE_8BIT, dataRead, count, 0xFF);
}


/**
  * @brief	Writes device's register
  * @param	address		Device's address
  * @param	reg_address	Writing register address
  * @param	dataWrite	Writing data
  * @retval	Device's wire error
  */
int mpu9255_writeRegister(mpu9255_address_t address, uint8_t reg_address, uint8_t dataWrite)
{
	return HAL_I2C_Mem_Write(&i2c_mpu9255, address, reg_address, I2C_MEMADD_SIZE_8BIT, &dataWrite, 1, 0xFF);
}


/**
  * @brief	Rewrites device's register (adds byte to byte with | )
  * @param	address		Device's address
  * @param	reg_address	Writing register address
  * @param	dataWrite	Writing data
  * @retval	Device's wire error
  */
int mpu9255_rewriteRegister(mpu9255_address_t address, uint8_t reg_address, uint8_t dataWrite)
{
	int error = 0;
	uint8_t regData = 0x00;
	PROCESS_ERROR(mpu9255_readRegister(address, reg_address, &regData, 1));
	uint8_t regData_new = (regData | dataWrite);
	return HAL_I2C_Mem_Write(&i2c_mpu9255, address, reg_address, I2C_MEMADD_SIZE_8BIT, &regData_new, 1, 0xFF);

end:
	return error;
}


/**
  * @brief	Initialises IMU device
  * @param	hi2c	Pointer to a I2C_HandleTypeDef structure
  * @retval	Device's wire error
  */
int mpu9255_init(I2C_HandleTypeDef* hi2c)
{
	int error = 0;

	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	107,	0b10000000));	//RESET
	HAL_Delay(200);

	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	25,		0b00000001));	//Sample Rate Divider
	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	26,		0b00000101));	//config (DLPF = 101)
	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	28,		(0b00000000 | (ACCEL_RANGE << 3)))); 	//accel config (rate 4g = 01)
	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	29,		0b00000000));	//accel config 2 (Fch_b = 0, DLPF = 100)
	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	35,		0b00000000));	//FIFO enable (not enabled)
	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	56,		0b00000000));	//interrupt enable (int disable = 0)
	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	106,	0b00000000));	//user control
	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	107,	0b00000001));	//power managment 1
	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	108,	0b00000000));	//power managment 2
//	PROCESS_ERROR(mpu9255_writeRegister(GYRO_AND_ACCEL,	27,		(0b00000000 | (GYRO_RANGE << 4)) ));	//gyro config (rate 500dps = 01, Fch_b = 00)
	PROCESS_ERROR(mpu9255_rewriteRegister(GYRO_AND_ACCEL,	27,		0b000000000 | (1 << 4)));

	//  Magnetometer init
	PROCESS_ERROR(mpu9255_writeRegister(GYRO_AND_ACCEL,	55,		0b00000010));	//режим bypass on

	PROCESS_ERROR(mpu9255_writeRegister(COMPASS,	0x0A,   AK8963_MODE_POWER_DOWN));	// power down before entering fuse mode
	HAL_Delay(20);

	PROCESS_ERROR(mpu9255_writeRegister(COMPASS,	0x0A,   AK8963_MODE_FUSE_ROM));		// Enter Fuse ROM access mode
	HAL_Delay(10);

	static int8_t ASA[3] = {};
	PROCESS_ERROR(mpu9255_readRegister(COMPASS,		0x10,   (uint8_t*)ASA, 3));   //ASAX
	//  Recalc magnetometer sensitivity adjustment values to floats to store them
	state_system.magnASA[0] = (float)(ASA[0] + 128) / (256);
	state_system.magnASA[1] = (float)(ASA[1] + 128) / (256);
	state_system.magnASA[2] = (float)(ASA[2] + 128) / (256);

	PROCESS_ERROR(mpu9255_writeRegister(COMPASS,        0x0A,   AK8963_MODE_POWER_DOWN));	// power down after reading
	HAL_Delay(20);

	uint8_t state = 0;
	//	Clear status registers
	PROCESS_ERROR(mpu9255_readRegister(COMPASS, 	0x02, &state, 1));
	PROCESS_ERROR(mpu9255_readRegister(COMPASS, 	0x09, &state, 1));

	PROCESS_ERROR(mpu9255_writeRegister(COMPASS,	0x0A,   AK8963_MODE_100HZ | AK8963_BIT_16_BIT));

end:
	mpu9255_writeRegister(GYRO_AND_ACCEL, 55,     0b00000000);   		//режим bypass off
	return error;
}


/**
  * @brief	Helpful function to swap bytes in int16_t value
  * @param	value	Variable to swap bytes in
  * @retval	New value
  */
static int16_t _swapBytesI16(int16_t value)
{
	uint8_t * value_ptr = (uint8_t*)&value;
	uint8_t tmp = value_ptr[0];
	value_ptr[0] = value_ptr[1];
	value_ptr[1] = tmp;

	return value;
}


/**
  * @brief	Reads IMU data (accel, gyro)
  * @param	raw_accelData	Read accel data
  * @param	raw_gyroData	Read gyro data
  * @retval	Device's wire error
  */
int mpu9255_readIMU(int16_t * raw_accelData, int16_t * raw_gyroData)
{
	int error = 0;

	PROCESS_ERROR(mpu9255_readRegister(GYRO_AND_ACCEL, 59, (uint8_t*)raw_accelData, 6));
	PROCESS_ERROR(mpu9255_readRegister(GYRO_AND_ACCEL, 67, (uint8_t*)raw_gyroData, 6));

	for (int i = 0; i < 3; i++)
		raw_accelData[i] = _swapBytesI16(raw_accelData[i]);

	for (int i = 0; i < 3; i++)
		raw_gyroData[i] = _swapBytesI16(raw_gyroData[i]);

end:
	return error;
}


/**
  * @brief	Reads magnetometer data
  * @param	raw_compassData	Read magn data
  * @retval	Device's wire error
  */
int mpu9255_readCompass(int16_t * raw_compassData)
{
	int error = 0;

	//	state of magn (ready to give data or not)
	uint8_t magn_state = 0;

	/*
	 * Bypass mode on, get ST1 register value
	 */
	PROCESS_ERROR(mpu9255_writeRegister(GYRO_AND_ACCEL, 55, 0b00000010));	//	bypass on
	PROCESS_ERROR(mpu9255_readRegister(COMPASS, 0x02, &magn_state, 1));

	/*
	 * Check if ST1 value bit ready is set
	 */
	if (magn_state & AK8963_DATA_READY)
	{
		magn_state = 0;

		/*
		 * Read data values and read ST2 register to prove that data values is read
		 * I found that I should read 7 bytes together
		 */
		uint8_t bytes[7] = {};
		PROCESS_ERROR(mpu9255_readRegister(COMPASS, 0x03, bytes, 7));

		for (int i = 0; i < 3; i++)
			raw_compassData[i] = (int16_t)((bytes[2*i+1] << 8) | bytes[2*i]);

		magn_state = bytes[6];

		/*
		 * Check HOFL bit for overflow
		 */
		if (magn_state & AK8963_DATA_OVERFLOW)
		{
			for (uint8_t i = 0; i < 3; i++)
				raw_compassData[i] = 0;
		}
	}

end:
	mpu9255_writeRegister(GYRO_AND_ACCEL, 55, 0b00000000);	//	bypass off
	return error;
}


/**
  * @brief	Recalculates accel data
  * @param	raw_accelData	Read accel data
  * @param	accelData		Recalculated accel data
  */
void mpu9255_recalcAccel(const int16_t * raw_accelData, float * accelData)
{
	float _accelData[3] = {0, 0, 0};

	//	Consider accel range and device's orientation
	int accel_range = 1;
	for (int i = 0; i < ACCEL_RANGE; i++)
		accel_range *= 2;
	float factor = accel_range * MPU9255_ACCEL_SCALE_FACTOR;

	_accelData[0] =   (float)(raw_accelData[2]) * factor;
	_accelData[1] =   (float)(raw_accelData[1]) * factor;
	_accelData[2] = - (float)(raw_accelData[0]) * factor;

	if (!IMU_CALIBRATION)
	{
		//	Accelerometer bias and transform matrix (to provide real values)
		float offset_vector[3] = {X_ACCEL_OFFSET, Y_ACCEL_OFFSET, Z_ACCEL_OFFSET};
		float transform_matrix[3][3] =	{{XX_ACCEL_TRANSFORM_MATIX, XY_ACCEL_TRANSFORM_MATIX, XZ_ACCEL_TRANSFORM_MATIX},
										 {XY_ACCEL_TRANSFORM_MATIX, YY_ACCEL_TRANSFORM_MATIX, YZ_ACCEL_TRANSFORM_MATIX},
										 {XZ_ACCEL_TRANSFORM_MATIX, YZ_ACCEL_TRANSFORM_MATIX, ZZ_ACCEL_TRANSFORM_MATIX}};

		vmv(_accelData, offset_vector, accelData);
		mxv(transform_matrix, accelData, accelData);
	}

	for (int i = 0; i < 3; i++) {
		accelData[i] = _accelData[i];
	}
}


/**
  * @brief	Recalculates gyro data
  * @param	raw_gyroData	Read gyro data
  * @param	accelData		Recalculated gyro data
  */
void mpu9255_recalcGyro(const int16_t * raw_gyroData, float * gyroData)
{
	float _gyroData[3] = {0, 0, 0};

	//	Consider gyro range and device's orientation
	int gyro_range = 1;
	for (int i = 0; i < GYRO_RANGE; i++)
		gyro_range *= 2;
	float factor = gyro_range * MPU9255_GYRO_SCALE_FACTOR;

	_gyroData[0] =   (float)(raw_gyroData[2]) * factor;
	_gyroData[1] =   (float)(raw_gyroData[1]) * factor;
	_gyroData[2] = - (float)(raw_gyroData[0]) * factor;

	for (int i = 0; i < 3; i++) {
		gyroData[i] = _gyroData[i];
	}

}


/**
  * @brief	Recalculates magn data
  * @param	raw_accelData	Read magn data
  * @param	accelData		Recalculated magn data
  */
void mpu9255_recalcMagn(const int16_t * raw_magnData, float * magnData)
{
	//переводим систему координат магнетометра в систему координат MPU
	float raw_data[3] = {	  (float)raw_magnData[0],
							  (float)raw_magnData[2],
							- (float)raw_magnData[1]};

	//	Adjustment
//	taskENTER_CRITICAL();
	float magnASA[3] = {state_system.magnASA[0], state_system.magnASA[2], - state_system.magnASA[1]};
//	taskEXIT_CRITICAL();

	for (uint8_t i = 0; i < 3; i++)
		raw_data[i] *= magnASA[i];

	if (!IMU_CALIBRATION)
	{
		//	Magnetometer bias and transform matrix (to provide real values)
		float offset_vector[3] = {X_COMPAS_OFFSET, Y_COMPAS_OFFSET, Z_COMPAS_OFFSET};
		float transform_matrix[3][3] =	{	{XX_COMPAS_TRANSFORM_MATIX, XY_COMPAS_TRANSFORM_MATIX, XZ_COMPAS_TRANSFORM_MATIX},
											{XY_COMPAS_TRANSFORM_MATIX, YY_COMPAS_TRANSFORM_MATIX, YZ_COMPAS_TRANSFORM_MATIX},
											{XZ_COMPAS_TRANSFORM_MATIX, YZ_COMPAS_TRANSFORM_MATIX, ZZ_COMPAS_TRANSFORM_MATIX}};

		vmv(raw_data, offset_vector, magnData);
		mxv(transform_matrix, magnData, magnData);
		return;
	}

	float offset_vector[3] = {0, 0, 0};
	vmv(raw_data, offset_vector, magnData);
}
