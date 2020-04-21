#ifndef __CANNOPENTPDO_H
#define __CANNOPENTPDO_H 
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

void CANTPDOThreadCreate(int num);
void CanTPDOThread(void const * argument);
void SDO_TPDO_Process(int Master_ID,uint8_t* buf,int len);

bool CAN_TPDO_Send(int Master_ID,unsigned char Sub_ID,unsigned char* buf,unsigned char len);

#endif
