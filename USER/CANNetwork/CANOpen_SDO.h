#ifndef __CANNOPENSDO_H
#define __CANNOPENSDO_H 
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
void CANSDOThreadCreate(int num);
void CanSDOThread(void const * argument);
void SDO_ASK_Process(int Master_ID,uint8_t* buf,int len);
void SDO_Response_Process(int Master_ID,uint8_t* buf,int len);
bool CAN_SDO_Send(int Master_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int len,unsigned int timeout_ms);
bool CAN_SDO_Read(int Master_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int* len,unsigned int timeout_ms);
#endif

