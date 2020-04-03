/*
 * telemetry.h
 *
 *  Created on: Jul 19, 2019
 *      Author: michael
 */

#ifndef SRC_TASKS_TELEMETRY_H_
#define SRC_TASKS_TELEMETRY_H_

#include <stdint.h>

uint8_t mavlink_msg_state_send(void);

uint8_t mavlink_msg_imu_rsc_send(void);

uint8_t mavlink_msg_imu_isc_send(void);

void TM_Init(void);


#endif /* SRC_TASKS_TELEMETRY_H_ */
