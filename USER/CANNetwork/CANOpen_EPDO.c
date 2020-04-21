#include "CANOpen_EPDO.h"
QueueHandle_t CANEPDORespandsQueue = NULL;
void CanEPDOThread(void const * argument);
uint8_t EPDObuf[8]={0};


void SDO_EPDO_Process(int Master_ID,uint8_t* buf,int len)
{
	uint16_t Eobject;
	uint8_t rbuf[1]={0};
	uint32_t RxLen=0;
//	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	memcpy(EPDObuf,buf,len);
	memcpy(&Eobject,&EPDObuf[1],2);
	CANObject_Dispose(Master_ID,Eobject,EPDObuf[0],'E',rbuf,&RxLen);
//	xQueueOverwriteFromISR(CANEPDORespandsQueue,&Master_ID, &xHigherPriorityTaskWoken);
}


/*******EPDO********************************************************/
//osThreadId CanEPDOHandle;
void CANEPDOThreadCreate(int num)
{         
//	osThreadDef(CanEPDO, CanEPDOThread, num,0, 128);
//	while(CanEPDOHandle == NULL)
//	CanEPDOHandle = osThreadCreate(osThread(CanEPDO), NULL);
	while(CANEPDORespandsQueue == NULL){
		CANEPDORespandsQueue=xQueueCreate(1, sizeof(int));
	}
}

void CanEPDOThread(void const * argument)
{
	int Master_ID=0;
	unsigned int len=0;
	uint16_t Eobject;
	uint8_t rbuf[1]={0};

	while(1){
		if(xQueueReceive(CANEPDORespandsQueue, &Master_ID,1000)==pdTRUE){//portMAX_DELAY
				memcpy(&Eobject,&EPDObuf[1],2);
			  CANObject_Dispose(Master_ID,Eobject,EPDObuf[0],'E',rbuf,&len);
		}
		osDelay(10);
	}
}
/*******EPDO********************************************************/

/**
  * @brief  CAN_EPDO发送函数 
  * @param	Master_ID:设备自身主ID 
  * @param	Sub_ID:设备自身子ID 
  * @param  obj:对象字典
  * @retval HAL true
  */
bool CAN_EPDO_Send(int Master_ID,uint8_t Sub_ID,uint16_t obj)
{
	unsigned int Function_ID=COBID_EPDO;
	unsigned int COB_ID=Function_ID+Master_ID;
	uint8_t sendbuf[3];
	sendbuf[0]=Sub_ID;
	memcpy(sendbuf+1,&obj,2);
		User_CAN_Transmit(COB_ID,sendbuf,3);
	return true;
}
