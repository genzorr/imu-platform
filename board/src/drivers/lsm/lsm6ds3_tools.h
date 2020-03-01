/*
 * lsm6ds3_tools.h
 *
 *  Created on: Aug 27, 2019
 *      Author: michael
 */

#ifndef DRIVERS_LSM_LSM6DS3_TOOLS_H_
#define DRIVERS_LSM_LSM6DS3_TOOLS_H_

#include <stdint.h>

int32_t lsm6ds3_platform_init(void);

uint32_t lsm6ds3_get_xl_data_g(float* accel);
uint32_t lsm6ds3_get_g_data_rps(float* gyro);

#endif /* DRIVERS_LSM_LSM6DS3_TOOLS_H_ */
