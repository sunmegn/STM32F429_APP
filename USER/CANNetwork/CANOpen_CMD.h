#ifndef __CANNOPENCMD_H
#define __CANNOPENCMD_H 
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

void CanCMDThread(void const * argument);
void CANCMDThreadCreate(int num);
bool CAN_CMD_Send(int Master_ID,unsigned short int object,unsigned char Sub_ID,unsigned int timeout_ms);

void SDO_TCMD_Process(int Master_ID,uint8_t* buf,int len);
void SDO_RCMD_Process(int Master_ID,uint8_t* buf,int len);
typedef struct
{
	uint8_t Master_ID;
	uint8_t buf[3];
}CAN_CMDtypeDef;
#endif

