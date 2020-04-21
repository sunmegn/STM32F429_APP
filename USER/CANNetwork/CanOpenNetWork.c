#include "CanNetWork.h"


/**
  * @brief  CAN���ͺ��� 
  * @param	COB_ID:CAN����ID 
  * @param  *buf:�������� 
  * @param  len:���鳤�� 
  * @retval HAL void
  */
void User_CAN_Transmit(unsigned int COB_ID,unsigned char* buf,int len)
{
//	CAN_Transmit(&hcan1,COB_ID,buf,len,200);
	CAN1_TransmitForFreeRTOS(COB_ID,buf,len,200);
}

/**
  * @brief  CAN_�����жϴ�����
  * @param	COB_ID:CAN ID
  * @param	*buf:����
  * @param  len:����
  * @retval HAL void
  */
void CANOPEN_Rx_Process(int COB_ID,uint8_t* buf,int len)
{
  int Master_ID=COB_ID&0x7F;
	unsigned Function_ID=COB_ID&0xF80;
	switch(Function_ID)
	{
		case COBID_SYNC: //SYNC   //
			SDO_SYNC_Process(Master_ID,buf,len);
			break;
		case COBID_EPDO: //ERROR   //
			SDO_EPDO_Process(Master_ID,buf,len);
			break;
		case COBID_TPDO: //��������
			SDO_TPDO_Process(Master_ID,buf,len);
			break;
		case COBID_RPDO: //��ִ����   //
			SDO_RPDO_Process(Master_ID,buf,len);
			break;
		case COBID_TCMD:
			SDO_TCMD_Process(Master_ID,buf,len);
			break;
		case COBID_RCMD://�����ִ
			SDO_RCMD_Process(Master_ID,buf,len);
			break;
		case COBID_SDO_ASK:  //SDOAsk
			SDO_ASK_Process(Master_ID,buf,len);
			break;
		case COBID_SDO_Res:  //SDOReceive
			SDO_Response_Process(Master_ID,buf,len);
		default:
		break;
	}
}

bool SampleSendBufToCANBus(int Master_ID,uint16_t object,uint8_t* buf,uint32_t len,uint32_t timeout_ms)
{
    if(object > 0x2000 && object < 0x2fff){//cmd
        return CAN_CMD_Send(Master_ID,object,0,timeout_ms);
    }
    if(object > 0x1000 && object < 0x1fff ){//EPDO
        return CAN_EPDO_Send(Master_ID,buf[0],object);
    }
    if(object > 0x3000 && object < 0x3fff  && len < 8){//TPDO
        return  CAN_TPDO_Send(Master_ID,0,buf,len);//buf[0]??SubID
    }
    else{
        return CAN_SDO_Send(Master_ID,object,0,buf,len,timeout_ms);
    }
    return false;
}

