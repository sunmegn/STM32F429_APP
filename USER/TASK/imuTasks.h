#ifndef __IMUTASKS_H
#define __IMUTASKS_H

#include "includes.h"

void                 imuTask_Function(void const *argument);
extern QueueHandle_t IMU_Message_Queue;

typedef struct change_imu_yaw_val
{
	float imu_yaw_last;
	float imu_yaw_now;
	float imu_yaw_totalval;
}change_imu_yaw_val_typedef;

#endif
