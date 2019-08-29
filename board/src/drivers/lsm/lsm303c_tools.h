/*
 * lsm303c_tools.h
 *
 *  Created on: Aug 29, 2019
 *      Author: michael
 */

#ifndef DRIVERS_LSM_LSM303C_TOOLS_H_
#define DRIVERS_LSM_LSM303C_TOOLS_H_

#include <stdint.h>

int32_t lsm303c_platform_init();

uint32_t lsm303c_get_m_data_mG(float* magn);


#endif /* DRIVERS_LSM_LSM303C_TOOLS_H_ */
