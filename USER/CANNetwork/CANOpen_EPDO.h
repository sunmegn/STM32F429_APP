#ifndef __CANNOPENEPDO_H
#define __CANNOPENEPDO_H 
#include "main.h"
#include "MyCAN.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" 
#include "queue.h"
#include "Object.h"
#include "CanNetWork.h"
void SDO_EPDO_Process(int Master_ID,uint8_t* buf,int len);
bool CAN_EPDO_Send(int Master_ID,uint8_t Sub_ID,uint16_t obj);
void CANEPDOThreadCreate(int num);

#endif


