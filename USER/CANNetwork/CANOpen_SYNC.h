#ifndef __CANNOPENSYNC_H
#define __CANNOPENSYNC_H 
#include "main.h"
#include "can.h"
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
void CanSYNCThread(void const * argument);
void CANSYNCThreadCreate(int num);

void SDO_SYNC_Process(int Master_ID,uint8_t* buf,int len);
bool CAN_SYNC_Send(void);

#endif



