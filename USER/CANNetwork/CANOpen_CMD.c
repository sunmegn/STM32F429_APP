#include "CANOpen_CMD.h"
QueueHandle_t CANCMDAskQueue = NULL;
QueueHandle_t CANCMDRespandsQueue = NULL;
CAN_CMDtypeDef ResCMDBuf;
CAN_CMDtypeDef AskCMDBuf;
bool CAN_CMDAsk(int Master_ID,uint8_t* buf,int len);

void SDO_TCMD_Process(int Master_ID,uint8_t* buf,int len)
{
//	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if(Master_ID != MY_ID)return;
	CAN_CMDAsk(Master_ID,buf,len);
//	AskCMDBuf.Master_ID=Master_ID;
//	memcpy(AskCMDBuf.buf,buf,len);
//	xQueueOverwriteFromISR(CANCMDAskQueue,&len, &xHigherPriorityTaskWoken);
}


void SDO_RCMD_Process(int Master_ID,uint8_t* buf,int len)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	int data= Master_ID*1000 + buf[0];
	xQueueOverwriteFromISR(CANCMDRespandsQueue,&data, &xHigherPriorityTaskWoken);
}


/*******CMD********************************************************/

bool CAN_CMDAsk(int Master_ID,uint8_t* buf,int len)
{
	int sub_id=buf[0];
	uint32_t RxLen=0;
	unsigned short obj=(buf[2]<<8)|buf[1];
	if(len != 3){return false;}
	User_CAN_Transmit(COBID_RCMD+Master_ID,buf,1);
	if(CANObject_Dispose(Master_ID,obj,sub_id,'C',buf,&RxLen) == true){//½öÖ´ÐÐ
		
	}
	return true;
}




bool CAN_CMD_Send(int Master_ID,unsigned short int object,unsigned char Sub_ID,unsigned int timeout_ms)
{
	if(CANCMDRespandsQueue == NULL){
		return false;
	}
	unsigned int Function_ID=COBID_TCMD;
	unsigned int COB_ID=Function_ID+Master_ID;
	bool state=false;
	int Rdata=0;
	uint8_t buf[3];
	buf[0]=Sub_ID;
	memcpy(buf+1,&object,2);
	int i=3;
	unsigned int retry_count=0;
	do{
		User_CAN_Transmit(COB_ID,buf,3);
		for(i=0;i<3;i++){
			if(xQueueReceive(CANCMDRespandsQueue, &Rdata,timeout_ms/3)==pdTRUE){
			   if(Rdata == (Master_ID*1000+Sub_ID)){
					return true;
			   }
			}
			else{
				state=false;
			}
		}
		retry_count++;
	}while(state == false && retry_count < 3 );
	return state;
}

//osThreadId CanCMDHandle;
void CANCMDThreadCreate(int num)
{
//	osThreadDef(CanCMD, CanCMDThread, num,0, 256);
//	CanCMDHandle = osThreadCreate(osThread(CanCMD), NULL);
	while(CANCMDAskQueue == NULL){
		CANCMDAskQueue=xQueueCreate(1, sizeof(int));
	}
	while(CANCMDRespandsQueue == NULL){
		CANCMDRespandsQueue=xQueueCreate(1, sizeof(int));
	}
}



void CanCMDThread(void const * argument)
{
	int len=0;
	while(1){
		if(xQueueReceive(CANCMDAskQueue, &len,portMAX_DELAY)==pdTRUE){//portMAX_DELAY
			CAN_CMDAsk(AskCMDBuf.Master_ID,AskCMDBuf.buf,len);
		}
	}
}
/*******CMD********************************************************/


