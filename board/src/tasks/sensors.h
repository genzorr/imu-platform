/*
 * sensors.h
 *
 *  Created on: Jul 6, 2019
 *      Author: michael
 */

#ifndef TASKS_SENSORS_H_
#define TASKS_SENSORS_H_


void get_staticShifts(void);
void IMU_Init(void);
int  IMU_updateDataAll(void);
void _IMUtask_updateData(void);


#endif /* TASKS_SENSORS_H_ */
