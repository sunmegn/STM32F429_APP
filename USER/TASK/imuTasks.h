#ifndef __IMUTASKS_H
#define __IMUTASKS_H

#include "includes.h"

void                 imuTask_Function(void const *argument);
extern QueueHandle_t IMU_Message_Queue;

#endif
