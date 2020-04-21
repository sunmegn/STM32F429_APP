#ifndef __CANNETWORK_H
#define __CANNETWORK_H 
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
#include "CANOpen_EPDO.h"
#include "CANOpen_SDO.h"
#include "CANOpen_TPDO.h"
#include "CANOpen_SYNC.h"
#include "CANOpen_RPDO.h"
#include "CANOpen_CMD.h"
#include "MTLink.h"
#include "Object.h"

//#define CANMasterNet
#define CANSalveNet
/**********COB_ID:Íø¶Î**************/
#define	COBID_SYNC		0X080
#define	COBID_EPDO		0X200
#define	COBID_TPDO		0X280
#define	COBID_RPDO		0X300
#define	COBID_TCMD		0X380
#define	COBID_RCMD		0X400
#define	COBID_SDO_ASK	0X580
#define	COBID_SDO_Res	0X600
/**********Object**************/
void CANOPEN_Rx_Process(int COB_ID,uint8_t* buf,int len);
void User_CAN_Transmit(unsigned int COB_ID,unsigned char* buf,int len);
bool SampleSendBufToCANBus(int Master_ID,uint16_t object,uint8_t* buf,uint32_t len,uint32_t timeout_ms);
#endif


