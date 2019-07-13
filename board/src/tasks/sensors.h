/*
 * sensors.h
 *
 *  Created on: Jul 6, 2019
 *      Author: michael
 */

#ifndef TASKS_SENSORS_H_
#define TASKS_SENSORS_H_


typedef struct
{
    uint8_t descr_val1;     //	0xFF
    uint8_t descr_val2;		//	0xFE

    uint32_t number;

    float time;

    float accel[3];
    float gyro[3];
    float magn[3];

    float quaternion[4];

    uint32_t crc;

}__attribute__((packed, aligned(1))) state_msg_t;


void get_staticShifts();
void IMU_Init();
int  IMU_updateDataAll();
void _IMUtask_updateData();
void stateMsg_fill(state_msg_t* msg);


#endif /* TASKS_SENSORS_H_ */
