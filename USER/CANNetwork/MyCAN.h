#ifndef __CANOPEN_H
#define __CANOPEN_H 
#include "main.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" 
#include "queue.h"
#include "can.h"
typedef struct
{
	uint32_t COB_ID;
	uint32_t len;
	uint8_t buf[8];
}CAN_TxRxtypeDef;


#define Net1_ID	HOST_ID
#define Net2_ID	0
#define Net3_ID	0
#define Net4_ID	0
#define Net5_ID	0
#define Net6_ID	0
#define Net7_ID	0
#define Net8_ID	0
#define USE_SALVE_RPDO


void CANopen_Init(void);
void timerForCan(void);
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *buf,int len,uint32_t Timeout);
HAL_StatusTypeDef CAN1_TransmitForFreeRTOS(uint32_t ID,uint8_t* buf,int len,uint32_t Timeout);
void CanTransmitThread(void const * argument);
HAL_StatusTypeDef CAN1_Send(CAN_HandleTypeDef *hcan,CAN_TxRxtypeDef *CanTransmit);
void CANTransmitThreadCreate(int num);

void CanRxThread(void const * argument);
void CANRxThreadCreate(int num);
#endif
