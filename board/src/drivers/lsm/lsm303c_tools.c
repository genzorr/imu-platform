/*
 * lsm303c_tools.c
 *
 *  Created on: Aug 27, 2019
 *      Author: michael
 *  Contains platform-based functions for ST driver
 */

#include <stdint.h>
#include <stm32f4xx.h>
#include <diag/Trace.h>

#include "lis3mdl_reg.h"
#include "state.h"
#include "vector.h"


#define LSM_TIMEOUT	1000
#define MDPS_TO_RAD	M_PI / 180 / 1000

//	Magnetometer bias & transform matrix
#define X_MAGN_OFFSET		-292.920973
#define Y_MAGN_OFFSET		 224.742275
#define Z_MAGN_OFFSET		-130.841676
#define XX_MAGN_TRANSFORM_MATIX	 0.002098
#define YY_MAGN_TRANSFORM_MATIX	 0.002186
#define ZZ_MAGN_TRANSFORM_MATIX	 0.002267
#define XY_MAGN_TRANSFORM_MATIX	 0.000118
#define XZ_MAGN_TRANSFORM_MATIX	-0.000354
#define YZ_MAGN_TRANSFORM_MATIX	 0.000003


static uint8_t whoamI, rst;

I2C_HandleTypeDef	i2c_lsm303c;
lis3mdl_ctx_t 		lsm303c_dev_ctx;


static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == &i2c_lsm303c)
	{
		/* Write multiple command */
		reg |= 0x80;
		error = HAL_I2C_Mem_Write(handle, LIS3MDL_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT);
	}
	else
	{
		trace_printf("lsm303c invalid handle\n");
		error = -19;
	}

	return error;
}


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == &i2c_lsm303c)
	{
		/* Write multiple command */
		reg |= 0x80;
		error = HAL_I2C_Mem_Read(handle, LIS3MDL_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT);
	}
	else
	{
		trace_printf("lsm303c invalid handle\n");
		error = -19;
	}

	return error;
}


int32_t lsm303c_bus_init(void* handle)
{
	int error = 0;
	if (handle == &i2c_lsm303c)
	{
		//	I2C init
		i2c_lsm303c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		i2c_lsm303c.Init.ClockSpeed = 400000;
		i2c_lsm303c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		i2c_lsm303c.Init.DutyCycle = I2C_DUTYCYCLE_2;
		i2c_lsm303c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		i2c_lsm303c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		i2c_lsm303c.Init.OwnAddress1 = 0x00;
		i2c_lsm303c.Instance = I2C3;
		i2c_lsm303c.Mode = HAL_I2C_MODE_MASTER;

		error |= HAL_I2C_Init(&i2c_lsm303c);
		HAL_Delay(200);
		trace_printf("i2c_lsm303c: %d\n", error);
	}
	else
	{
		trace_printf("invalid lsm303c handle\n");
		error = -19;
	}

	return error;
}


int32_t lsm303c_platform_init()
{
	int error = 0;

	lsm303c_dev_ctx.write_reg = platform_write;
	lsm303c_dev_ctx.read_reg = platform_read;
	lsm303c_dev_ctx.handle = &i2c_lsm303c;

	//	Set needed bus parameters
	error |= lsm303c_bus_init(lsm303c_dev_ctx.handle);

	// Reset to defaults
	error |= lis3mdl_reset_set(&lsm303c_dev_ctx, PROPERTY_ENABLE);
	do {
		error = lis3mdl_reset_get(&lsm303c_dev_ctx, &rst);
	} while (rst);

	// Check who_am_i
	error |= lis3mdl_device_id_get(&lsm303c_dev_ctx, &whoamI);
	if (whoamI != LIS3MDL_ID)
	{
		trace_printf("lsm303c not found, %d\terror: %d\n", whoamI, error);
		return -19;
	}
	else
		trace_printf("lsm303c OK\n");

	error |= lis3mdl_block_data_update_set(&lsm303c_dev_ctx, PROPERTY_DISABLE);

	error |= lis3mdl_data_rate_set(&lsm303c_dev_ctx, LIS3MDL_HP_40Hz);

	error |= lis3mdl_full_scale_set(&lsm303c_dev_ctx, LIS3MDL_16_GAUSS);

	error |= lis3mdl_operating_mode_set(&lsm303c_dev_ctx, LIS3MDL_CONTINUOUS_MODE);

	return error;
}


uint32_t lsm303c_get_m_data_mG(float* magn)
{
	axis3bit16_t data_raw_magnetic;
	uint8_t error;
	//	Read data
	PROCESS_ERROR(lis3mdl_magnetic_raw_get(&lsm303c_dev_ctx, data_raw_magnetic.u8bit));
	magn[0] = 1000 * LIS3MDL_FROM_FS_16G_TO_G(data_raw_magnetic.i16bit[0]);
	magn[1] = 1000 * LIS3MDL_FROM_FS_16G_TO_G(data_raw_magnetic.i16bit[1]);
	magn[2] = 1000 * LIS3MDL_FROM_FS_16G_TO_G(data_raw_magnetic.i16bit[2]);

	if (!IMU_CALIBRATION)
	{
		//	Magnetometer bias and transform matrix (to provide real values)
		float offset_vector[3] = {X_MAGN_OFFSET, Y_MAGN_OFFSET, Z_MAGN_OFFSET};
		float transform_matrix[3][3] =	{	{XX_MAGN_TRANSFORM_MATIX, XY_MAGN_TRANSFORM_MATIX, XZ_MAGN_TRANSFORM_MATIX},
											{XY_MAGN_TRANSFORM_MATIX, YY_MAGN_TRANSFORM_MATIX, YZ_MAGN_TRANSFORM_MATIX},
											{XZ_MAGN_TRANSFORM_MATIX, YZ_MAGN_TRANSFORM_MATIX, ZZ_MAGN_TRANSFORM_MATIX}};

		vmv(magn, offset_vector, magn);
		mxv(transform_matrix, magn, magn);
	}

end:
	return error;
}
