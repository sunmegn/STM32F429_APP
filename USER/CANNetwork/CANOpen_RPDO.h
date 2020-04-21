#ifndef __CANNOPENRPDO_H
#define __CANNOPENRPDO_H 
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

void CANRPDOThreadCreate(int num);
void CanRPDOThread(void const * argument);
bool CAN_RPDO_Send(int Master_ID,unsigned char Sub_ID,unsigned char* buf,unsigned char len);

void SDO_RPDO_Process(int Master_ID,uint8_t* buf,int len);

#endif
