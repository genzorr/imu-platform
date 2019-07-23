/*
 * sensors.h
 *
 *  Created on: Jul 6, 2019
 *      Author: michael
 */

#ifndef TASKS_SENSORS_H_
#define TASKS_SENSORS_H_


void get_staticShifts();
void IMU_Init();
int  IMU_updateDataAll();
void _IMUtask_updateData();


#endif /* TASKS_SENSORS_H_ */
