#include "CANOpen_SYNC.h"
QueueHandle_t CANSYNCRespandsQueue = NULL;

void SDO_SYNC_Process(int Master_ID,uint8_t* buf,int len)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xQueueOverwriteFromISR(CANSYNCRespandsQueue,&Master_ID, &xHigherPriorityTaskWoken);
}


/*******SYNC********************************************************/
osThreadId CanSYNCHandle;
void CANSYNCThreadCreate(int num)
{
	osThreadDef(CanSYNC, CanSYNCThread, num,0, 256);
	CanSYNCHandle = osThreadCreate(osThread(CanSYNC), NULL);
	while(CANSYNCRespandsQueue == NULL){
		CANSYNCRespandsQueue=xQueueCreate(1, sizeof(int));
	}
}

void CanSYNCThread(void const * argument)
{
//	int Master_ID=0;
	osDelay(1000);
	while(1){//For Master
//		if(MyBox.Linkstate == true){
//			CAN_SYNC_Send();
//		}
		osDelay(500);
	}
}
/*******SYNC********************************************************/

bool CAN_SYNC_Send(void)
{
	unsigned int COB_ID=COBID_SYNC;
	uint8_t sendbuf[1];
		User_CAN_Transmit(COB_ID,sendbuf,0);
	return true;
}

