#include "CANOpen_RPDO.h"

unsigned char RPDOBuf[127][8]={0};

QueueHandle_t CANRPDORespandsQueue = NULL;


void SDO_RPDO_Process(int Master_ID,uint8_t* buf,int len)
{
//	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	uint32_t RxLen=len-1;
	CANObject_Dispose(Master_ID,0,buf[0],'P',&buf[1],&(RxLen));
//	memcpy(RPDOBuf[Master_ID],buf,len);
//	xQueueOverwriteFromISR(CANRPDORespandsQueue,&Master_ID, &xHigherPriorityTaskWoken);
}
/*******RPDO********************************************************/
//osThreadId CanRPDOHandle;
void CANRPDOThreadCreate(int num)
{         
//	osThreadDef(CanRPDO, CanRPDOThread, num,0, 256);
//	CanRPDOHandle = osThreadCreate(osThread(CanRPDO), NULL);
	while(CANRPDORespandsQueue == NULL){
		CANRPDORespandsQueue=xQueueCreate(1, sizeof(int));
	}
}

void CanRPDOThread(void const * argument)
{
	int Master_ID=0;
	unsigned int len=7;
	osDelay(10);
	while(1){
		if(xQueueReceive(CANRPDORespandsQueue, &Master_ID,2000)==pdTRUE){//portMAX_DELAY
			  CANObject_Dispose(Master_ID,0,RPDOBuf[Master_ID][0],'P',&RPDOBuf[Master_ID][1],&len);
		}
	}
}
/*******RPDO********************************************************/


/**
  * @brief  CAN_RPDO发送函数 
  * @param	Master_ID:设备自身主ID 
  * @param	Sub_ID:设备自身子ID 
  * @param  *buf:发送数组 
  * @param  len:数组长度 
  * @retval HAL true
  */
bool CAN_RPDO_Send(int Master_ID,unsigned char Sub_ID,unsigned char* buf,unsigned char len)
{
	unsigned int Function_ID=COBID_RPDO;
	unsigned int COB_ID=Function_ID+Master_ID;
	uint8_t sendbuf[8];
	sendbuf[0]=Sub_ID;
	memcpy(sendbuf+1,buf,len);
		User_CAN_Transmit(COB_ID,sendbuf,len+1);
	return true;
}

