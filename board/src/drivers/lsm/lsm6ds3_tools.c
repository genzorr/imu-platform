/*
 * lsm6ds3_tools.c
 *
 *  Created on: Aug 27, 2019
 *      Author: michael
 *  Contains platform-based functions for ST driver
 */

#include <stdint.h>
#include <math.h>
#include <stm32f4xx.h>
#include <diag/Trace.h>

#include "lsm6ds3_reg.h"

#include "state.h"

#define LSM_TIMEOUT	1000
#define MDPS_TO_RAD	M_PI / 180 / 1000

static uint8_t whoamI, rst;

SPI_HandleTypeDef	spi_lsm6ds3;
I2C_HandleTypeDef	i2c_lsm6ds3;
lsm6ds3_ctx_t lsm6ds3_dev_ctx;

#define LSM6DS3_I2C_ADD	0b11010111


static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == &i2c_lsm6ds3)
	{
		error = HAL_I2C_Mem_Write(handle, LSM6DS3_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT);
	}
	else if (handle == &spi_lsm6ds3)
	{
		reg |= 0x80;
		HAL_GPIO_WritePin(LSM6DS3_PORT, LSM6DS3_CS_PIN, GPIO_PIN_RESET);
		error |= HAL_SPI_Transmit(handle, &reg, 1, LSM_TIMEOUT);
		error |= HAL_SPI_Transmit(handle, bufp, len, LSM_TIMEOUT);
		HAL_GPIO_WritePin(LSM6DS3_PORT, LSM6DS3_CS_PIN, GPIO_PIN_SET);
	}

	return error;
}


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int error = 0;

	if (handle == &i2c_lsm6ds3)
	{
		error = HAL_I2C_Mem_Read(handle, LSM6DS3_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, LSM_TIMEOUT);
	}
	else if (handle == &spi_lsm6ds3)
	{
		reg |= 0x80;
		HAL_GPIO_WritePin(LSM6DS3_PORT, LSM6DS3_CS_PIN, GPIO_PIN_RESET);
		error |= HAL_SPI_Transmit(handle, &reg, 1, LSM_TIMEOUT);
		error |= HAL_SPI_Receive(handle, bufp, len, LSM_TIMEOUT);
		HAL_GPIO_WritePin(LSM6DS3_PORT, LSM6DS3_CS_PIN, GPIO_PIN_SET);
	}

	return error;
}


int32_t lsm6ds3_bus_init(void* handle)
{
	int error = 0;
	if (handle == &i2c_lsm6ds3)
	{
		//	I2C init
		i2c_lsm6ds3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		i2c_lsm6ds3.Init.ClockSpeed = 400000;
		i2c_lsm6ds3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		i2c_lsm6ds3.Init.DutyCycle = I2C_DUTYCYCLE_2;
		i2c_lsm6ds3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		i2c_lsm6ds3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		i2c_lsm6ds3.Init.OwnAddress1 = 0x00;
		i2c_lsm6ds3.Instance = I2C2;
		i2c_lsm6ds3.Mode = HAL_I2C_MODE_MASTER;

		error |= HAL_I2C_Init(&i2c_lsm6ds3);
		HAL_Delay(200);
		trace_printf("i2c_lsm6ds: %d\n", error);
	}
	else if (handle == &spi_lsm6ds3)
	{
		//	SPI init
		spi_lsm6ds3.Instance = SPI2;
		spi_lsm6ds3.Init.Mode = SPI_MODE_MASTER;
		spi_lsm6ds3.Init.Direction = SPI_DIRECTION_2LINES;
		spi_lsm6ds3.Init.DataSize = SPI_DATASIZE_8BIT;
		spi_lsm6ds3.Init.CLKPolarity = SPI_POLARITY_LOW;
		spi_lsm6ds3.Init.CLKPhase = SPI_PHASE_1EDGE;
		spi_lsm6ds3.Init.NSS = SPI_NSS_SOFT;
		spi_lsm6ds3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
		spi_lsm6ds3.Init.FirstBit = SPI_FIRSTBIT_MSB;
		spi_lsm6ds3.Init.TIMode = SPI_TIMODE_DISABLE;
		spi_lsm6ds3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

		error |= HAL_SPI_Init(&spi_lsm6ds3);
		HAL_Delay(200);
		trace_printf("spi_lsm6ds3: %d\n", error);
	}
	else
		trace_printf("invalid lsm6ds3 handle\n");

	return error;
}


int32_t lsm6ds3_platform_init()
{
	int error = 0;

	lsm6ds3_dev_ctx.write_reg = platform_write;
	lsm6ds3_dev_ctx.read_reg = platform_read;
	lsm6ds3_dev_ctx.handle = &i2c_lsm6ds3;

	//	Set needed bus parameters
	error |= lsm6ds3_bus_init(lsm6ds3_dev_ctx.handle);

	// Reset to defaults
	error |= lsm6ds3_reset_set(&lsm6ds3_dev_ctx, PROPERTY_ENABLE);
	do {
		error = lsm6ds3_reset_get(&lsm6ds3_dev_ctx, &rst);
	} while (rst);

	// Check who_am_i
	error |= lsm6ds3_device_id_get(&lsm6ds3_dev_ctx, &whoamI);
	if (whoamI != LSM6DS3_ID)
	{
		error |= lsm6ds3_device_id_get(&lsm6ds3_dev_ctx, &whoamI);
		if (whoamI != LSM6DS3_ID)
		{
			trace_printf("lsm6ds3 not found, %d\terror: %d\n", whoamI, error);
			return -19;
		}
		else
			trace_printf("lsm6ds3 OK\n");
	}
	else
		trace_printf("lsm6ds3 OK\n");

	error |= lsm6ds3_fifo_mode_set(&lsm6ds3_dev_ctx, PROPERTY_DISABLE);

	error |= lsm6ds3_block_data_update_set(&lsm6ds3_dev_ctx, PROPERTY_DISABLE);

	error |= lsm6ds3_xl_full_scale_set(&lsm6ds3_dev_ctx, LSM6DS3_4g);
	error |= lsm6ds3_gy_full_scale_set(&lsm6ds3_dev_ctx, LSM6DS3_1000dps);

	error |= lsm6ds3_xl_data_rate_set(&lsm6ds3_dev_ctx, LSM6DS3_XL_ODR_104Hz);
	error |= lsm6ds3_gy_data_rate_set(&lsm6ds3_dev_ctx, LSM6DS3_GY_ODR_104Hz);

	return error;
}


uint32_t lsm6ds3_get_xl_data_g(float* accel)
{
	axis3bit16_t data_raw_acceleration;
	uint8_t error;
	//	Read acceleration field data
	error = lsm6ds3_acceleration_raw_get(&lsm6ds3_dev_ctx, data_raw_acceleration.u8bit);
	accel[0] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]) / 1000;
	accel[1] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]) / 1000;
	accel[2] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]) / 1000;
	return error;
}


uint32_t lsm6ds3_get_g_data_rps(float* gyro)
{
	axis3bit16_t data_raw_angular_rate;
	uint8_t error;
	//	Read acceleration field data
	error = lsm6ds3_angular_rate_raw_get(&lsm6ds3_dev_ctx, data_raw_angular_rate.u8bit);
	gyro[0] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[0]) * MDPS_TO_RAD;
	gyro[1] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[1]) * MDPS_TO_RAD;
	gyro[2] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[2]) * MDPS_TO_RAD;
	return error;
}
