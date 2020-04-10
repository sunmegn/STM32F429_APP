#include "MTLink.h"



/************需要修改********MTLink_UDP****************************************************/
//根据目标ID修改发送函数
bool MTLink_SendBuffer(uint8_t DID,uint8_t *buf,int len)
{
	switch(DID)
	{
		case FREE_ID:
			break;
		case HOST_ID:
			udp_send(buf,len);
			break;
		default:
			break;
	}
	return true;
}

void SendToRespandsBuf(MTLink_typedef *m,uint8_t* buf,int len)
{
	memcpy(m->AskFIFO,buf,len);
	int pkglen=len;
	if(m == &MTLink_UDP){//跑UDP的MTLink
		xQueueSendToBack(MTLinkUDPAskQueue,&pkglen,10);//发送消息 
	}
}


bool MTLink_WaitRespands(MTLink_typedef *m,uint32_t waittime_ms)
{
	uint32_t Basetime=HAL_GetTick();
	uint32_t nowtime=Basetime;
	bool state=false;
	int len;
	if(m == &MTLink_UDP){//跑UDP的MTLink
		do{
			if(xQueueReceive(MTLinkUDPAskQueue, &len,waittime_ms/5)==pdTRUE){
				if(MTLink_RespandsDecode(m)){
					return true;
				}
				else state = false;
			}
			nowtime=HAL_GetTick();
		}while(state == false && (nowtime-Basetime) <= waittime_ms);
	}
	return state;
}
/********************************************************************************/

void MTLinkPrint(MTLink_typedef *m,uint8_t SID,uint8_t DID,char* str,int len,uint32_t timeout)
{
	MTLink_Encode(&MTLink_UDP,SID,DID,0,0x0000,(uint8_t*)str,len,timeout);
}

