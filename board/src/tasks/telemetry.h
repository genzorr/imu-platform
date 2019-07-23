/*
 * telemetry.h
 *
 *  Created on: Jul 19, 2019
 *      Author: michael
 */

#ifndef SRC_TASKS_TELEMETRY_H_
#define SRC_TASKS_TELEMETRY_H_

#include <stdint.h>

uint8_t mavlink_msg_state_send();

uint8_t mavlink_msg_imu_rsc_send();

uint8_t mavlink_msg_imu_isc_send();

void TM_Init();


#endif /* SRC_TASKS_TELEMETRY_H_ */
