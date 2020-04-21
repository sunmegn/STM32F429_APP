#include "CANOpen_TPDO.h"
unsigned char TPDOBuf[127][8]={0};
QueueHandle_t CANTPDORespandsQueue = NULL;

void SDO_TPDO_Process(int Master_ID,uint8_t* buf,int len)
{
//	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if(Master_ID != MY_ID)return;
	uint32_t RxLen=len-1;
	CANObject_Dispose(Master_ID,0,buf[0],'P',&buf[1],&RxLen);
//	memcpy(TPDOBuf[Master_ID],buf,len);
//	xQueueOverwriteFromISR(CANTPDORespandsQueue,&Master_ID, &xHigherPriorityTaskWoken);
}


/**
  * @brief  CAN_TPDO发送函数 
  * @param	Master_ID:目标主ID 
  * @param	Sub_ID:目标子ID
  * @param  *buf:发送数组 
  * @param  len:数组长度 
  * @retval HAL true
  */
bool CAN_TPDO_Send(int Master_ID,unsigned char Sub_ID,unsigned char* buf,unsigned char len)
{
	unsigned int Function_ID=COBID_TPDO;
	unsigned int COB_ID=Function_ID+Master_ID;
	uint8_t sendbuf[8];
	sendbuf[0]=Sub_ID;
	memcpy(sendbuf+1,buf,len);
		User_CAN_Transmit(COB_ID,sendbuf,len+1);
	return true;
}


/*******TPDO********************************************************/
//osThreadId CanTPDOHandle;
void CANTPDOThreadCreate(int num)
{         
//	osThreadDef(CanTPDO, CanTPDOThread, num,0, 256);
//	CanTPDOHandle = osThreadCreate(osThread(CanTPDO), NULL);
	while(CANTPDORespandsQueue == NULL){
		CANTPDORespandsQueue=xQueueCreate(1, sizeof(int));
	}
}

void CanTPDOThread(void const * argument)
{
	int Master_ID=0;
	unsigned int len=7;
	while(1){                                  
		if(xQueueReceive(CANTPDORespandsQueue, &Master_ID,5000)==pdTRUE){//portMAX_DELAY
			 CANObject_Dispose(Master_ID,0,TPDOBuf[Master_ID][0],'P',&TPDOBuf[Master_ID][1],&len);
		}
		else{
//			CAN_EPDO_Send(MY_ID,0,0x1010);//tpdo error
		}
	}
}
/*******TPDO********************************************************/
