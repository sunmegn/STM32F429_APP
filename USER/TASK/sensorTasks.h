#ifndef __SENSORTASKS_H
#define __SENSORTASKS_H

#include "includes.h"
extern QueueHandle_t TempHum_Message_Queue; 
extern QueueHandle_t Battery_Message_Queue;
void SensorTask_Function(void const * argument);

#endif
