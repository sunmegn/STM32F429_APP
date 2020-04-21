#include "CANOpen_SDO.h"
static unsigned char SDORevForSendBuffer[65535] __attribute__((at(CCMDATARAM_BASE)));
//static unsigned char SDORevForSendBuffer[5000]={0};
#define Get_NowTimeMs()		HAL_GetTick()
unsigned char SDORevForReadBuffer[200]={0};
QueueHandle_t CANSDOAskQueue = NULL;
QueueHandle_t CANSDORespondsQueue = NULL;
unsigned char RespondsBuf[127][8]={0};
unsigned char ReceiveBuf[127][8]={0};


/**
  * @brief  规定时间内等待对应ID的数据包回复 
  * @param	timeout 超时时间 
  * @retval bool 等待成功: return true 否等待超时return false 
  */

bool CAN_WaitSDORespands(int COB_ID,unsigned int timeout)
{
	uint32_t Basetime=HAL_GetTick();
	uint32_t nowtime=Basetime;
	int Master_ID,NowResID=0;
	NowResID = COB_ID - COBID_SDO_ASK;
	do{
		if(xQueueReceive(CANSDORespondsQueue, &Master_ID,timeout/5)==pdTRUE){
			if(Master_ID == NowResID){
				return true;
			}
		}
		nowtime=HAL_GetTick();
	}while((nowtime-Basetime) <= timeout);
	return false;
}

/**
  * @brief  CAN SDO_短数据包发送 
  * @param	COB_ID:CAN发送ID 
  * @param	len:发送长度 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *data:发送数组  
  * @retval void
  */
void CAN_SDO_ExpeditedSend(unsigned int COB_ID,unsigned char len,unsigned short int object,unsigned char Sub_ID,unsigned char* data)
{
	unsigned char bytes[8]={0};
	unsigned char byte0=0x00,e=1,s=1,n=4-len;
	byte0|=(1<<5);
	byte0|=((n&0x03)<<2);
	byte0|=((e&0x01)<<1);
	byte0|=(s&0x01);
	bytes[0]=byte0;
	/**********/
	bytes[1]=object&0xff;
	bytes[2]=(object>>8)&0xff;
	bytes[3]=Sub_ID;
	/*************/
	memcpy(bytes+4,data,len);
	User_CAN_Transmit(COB_ID,bytes,8);
}


/**
  * @brief  CAN SDO_短数据包发送回复check
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *bytes:接收数组beyes[8] 
  * @retval bool 解析成功: return true 否则return false 
  */
bool CAN_SDO_ExpeditedSendResponds(unsigned short int object,unsigned char Sub_ID,unsigned char *bytes)
{
	unsigned short int obj=0,sub_id=0;
	if((bytes[0]>>5) == 3){
		sub_id=(bytes[3]);
		obj|=(bytes[2]<<8);
		obj|=(bytes[1]);
		if(obj == object && Sub_ID == sub_id){
			return true;
		}
	}
	return false;
}


/**
  * @brief  CAN SDO_长数据包首次发送，第一包发送数据包长 
  * @param	COB_ID:CAN发送ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  pkglen:数据包长度 
  * @retval void
  */
void CAN_SDO_NormalFirstSend(unsigned int COB_ID,unsigned char t,unsigned short int object,unsigned char Sub_ID,unsigned int pkglen)
{
	unsigned char bytes[8]={0};
	unsigned char byte0=0x00,s=0,n=0;
	byte0|=(0<<5);
	byte0|=((t&0x01)<<4);
	byte0|=((n&0x07)<<1);
	byte0|=(s&0x01);
	bytes[0]=byte0;
	/**********/
	bytes[1]=object&0xff;
	bytes[2]=(object>>8)&0xff;
	bytes[3]=Sub_ID;
	memcpy(bytes+4,&pkglen,4);
	User_CAN_Transmit(COB_ID,bytes,8);
}

/**
  * @brief  CAN SDO_长数据包首次发送回馈check
  * @param	t:toggle应回复的反转电平 0 - 1 - 0
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *bytes:CAN收到的数据帧 
  * @retval 回复成功 return true  失败 return false 
  */
bool CAN_SDO_NormalFirstSendResponds(unsigned char t,unsigned short int object,unsigned char Sub_ID,unsigned char *bytes)
{
	unsigned char toggle=0;
	unsigned short int obj=0,sub_id=0;
	toggle=(bytes[0]>>4)&0x01;
	if((bytes[0]>>5) == 0 && toggle == t){
		sub_id=(bytes[3]);
		obj|=(bytes[2]<<8);
		obj|=(bytes[1]);
		if(obj == object && Sub_ID == sub_id){
			return true;
		}
	}
	return false;
}

/**
  * @brief  CAN SDO_长数据包数据发送 
  * @param	COB_ID:CAN发送ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *buf:数据包
  * @param  len:包长=7 
  * @retval void
  */

void CAN_SDO_NormalNextSend(unsigned int COB_ID,unsigned char t,unsigned char* buf,unsigned int len)
{
	unsigned char bytes[8]={0};
	unsigned char byte0=0x00,s=0,n=0;
	byte0|=(0<<5);
	byte0|=((t&0x01)<<4);
	byte0|=((n&0x07)<<1);
	byte0|=(s&0x01);
	bytes[0]=byte0;
	/**********/
	memcpy(bytes+1,buf,7);
	User_CAN_Transmit(COB_ID,bytes,8);
}

/**
  * @brief  CAN SDO_长数据包第二次数据发送回馈check
  * @param	t:toggle应回复的反转电平 0 - 1 - 0
  * @param  *bytes:CAN收到的数据帧 
  * @retval 回复成功 return true  失败 return false 
  */
bool CAN_SDO_NormalNextSendResponds(unsigned char t,unsigned char *bytes)
{
	unsigned char toggle=0;
	toggle=(bytes[0]>>4)&0x01;
	if((bytes[0]>>5) == 0 && toggle == t){
			return true;
	}
	return false;
}



/**
  * @brief  CAN SDO_长数据包最后一次数据发送 
  * @param	COB_ID:CAN发送ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param  *buf:数据包
  * @param  len:包长 <=7 
  * @retval void
  */
void CAN_SDO_NormalEndSend(unsigned int COB_ID,unsigned char t,unsigned char* buf,unsigned int len)
{
	unsigned char bytes[8]={0};
	unsigned char byte0=0x00,s=1,n=7-len;
	byte0|=(0<<5);
	byte0|=((t&0x01)<<4);
	byte0|=((n&0x07)<<1);
	byte0|=(s&0x01);
	bytes[0]=byte0;
	/**********/
	memcpy(bytes+1,buf,len);
	User_CAN_Transmit(COB_ID,bytes,8);
}

/**
  * @brief  CAN SDO_短数据包读取 
  * @param	COB_ID:CAN发送ID 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @retval void
  */
void CAN_SDO_ExpeditedRead(unsigned int COB_ID,unsigned short int object,unsigned char Sub_ID)
{
	unsigned char bytes[8]={0};
	unsigned char byte0=0x00,e=0,s=0,n=0;
	byte0|=(2<<5);
	byte0|=((n&0x03)<<2);
	byte0|=((e&0x01)<<1);
	byte0|=(s&0x01);
	bytes[0]=byte0;
	/**********/
	bytes[1]=object&0xff;
	bytes[2]=(object>>8)&0xff;
	bytes[3]=Sub_ID;
	User_CAN_Transmit(COB_ID,bytes,8);
}


/**
  * @brief  CAN SDO_短数据包接收回复check
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *bytes:接收数组beyes[8] 
  * @param  *buf:收到的数据 
  * @retval bool 解析成功: return true 否则return false 
  */
bool CAN_SDO_ExpeditedReadResponds(unsigned short int object,unsigned char Sub_ID,unsigned char *bytes,unsigned char* buf,unsigned int* len)
{
	unsigned short int obj=0,sub_id=0,n=((bytes[0]>>2)&0x03);
	if((bytes[0]>>5) == 2 && ((bytes[0])&0x01) == 1){
		sub_id=(bytes[3]);
		obj|=(bytes[2]<<8);
		obj|=(bytes[1]);
		if(obj == object && Sub_ID == sub_id){
			memcpy(buf,bytes+4,4-n);
			len[0]=4-n;
			return true;
		}
	}
	return false;
}

/**
  * @brief  CAN SDO_长数据包读取 
  * @param	COB_ID:CAN发送ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @retval void
  */
void CAN_SDO_NormalRead(unsigned int COB_ID,unsigned char t,unsigned short int object,unsigned char Sub_ID)
{
	unsigned char bytes[8]={0};
	unsigned char byte0=0x00,e=0,s=0,n=0;
	byte0|=(3<<5);
	byte0|=(t<<4);
	bytes[0]=byte0;
	/**********/
	bytes[1]=object&0xff;
	bytes[2]=(object>>8)&0xff;
	bytes[3]=Sub_ID;
	User_CAN_Transmit(COB_ID,bytes,8);
}


/**
  * @brief  CAN SDO_长数据包接收首次回复check
  * @param	object:对象字典
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0;
  * @param	Sub_ID:子ID 
  * @param  *bytes:接收数组beyes[8] 
  * @param  *len:收到的数据包长 
  * @retval bool 解析成功: return true 否则return false 
  */
bool CAN_SDO_NormalFirstReadResponds(unsigned short int object,unsigned char t,unsigned char Sub_ID,unsigned char *bytes,unsigned int* len)
{
	unsigned short int obj=0,sub_id=0,n=((bytes[0]>>1)&0x07);
	unsigned char toggle=(bytes[0]>>4)&0x01;
	int s=bytes[0]&0x01;
	if((bytes[0]>>5) == 0 && s == 0 && toggle == t){
		sub_id=(bytes[3]);
		obj|=(bytes[2]<<8);
		obj|=(bytes[1]);
		if(obj == object && Sub_ID == sub_id){
			memcpy(len,bytes+4,4);
			return true;
		}
	}
	return false;
}

/**
  * @brief  CAN SDO_长数据包接收数据回复check
  * @param	object:对象字典
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0;
  * @param	Sub_ID:子ID 
  * @param  *bytes:接收数组beyes[8] 
  * @param  *buf:接收的数据包 
  * @param  *len:收到的数据包长 
  * @retval bool 解析成功: return true 否则return false 
  */
bool CAN_SDO_NormalNextReadResponds(unsigned short int object,unsigned char t,unsigned char Sub_ID,unsigned char *bytes,unsigned char* buf,unsigned int* len)
{
	unsigned char toggle=(bytes[0]>>4)&0x01;
	if((bytes[0]>>5) == 0 && (bytes[0]&0x01) == 0 && toggle == t){
		memcpy(buf,bytes+1,7);
		len[0]=7;
		return true;
	}
	return false;
}


/**
  * @brief  CAN SDO_长数据包接收最后一包数据回复check
  * @param	object:对象字典
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0;
  * @param	Sub_ID:子ID 
  * @param  *bytes:接收数组beyes[8] 
  * @param  *buf:接收的数据包 
  * @param  *len:收到的数据包长 
  * @retval bool 解析成功: return true 否则return false 
  */
bool CAN_SDO_NormalEndReadResponds(unsigned short int object,unsigned char t,unsigned char Sub_ID,unsigned char *bytes,unsigned char* buf,unsigned int* len)
{
	unsigned char toggle=(bytes[0]>>4)&0x01;
	int n=(bytes[0]>>1)&0x07;
	int s=bytes[0]&0x01;
	if((bytes[0]>>5) == 0 && s == 1 && toggle == t){
		memcpy(buf,bytes+1,7);
		len[0]=7-n;
		return true;
	}
	return false;
}



/**
  * @brief  CAN SDO_发送短数据包+重传机制 
  * @param	COB_ID:CAN发送ID 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *buf:发送数组 
  * @param	len:发送长度 
  * @param	timeout_ms 超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_ESend(unsigned int COB_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int len,unsigned int timeout_ms)
{
	bool state=false;
	unsigned int retry_count=0;
	do{
		CAN_SDO_ExpeditedSend(COB_ID,len,object,Sub_ID,buf);//send buf
		if(CAN_WaitSDORespands(COB_ID,timeout_ms) == true){//wait AsK timeout_ms
			state=CAN_SDO_ExpeditedSendResponds(object,Sub_ID,&RespondsBuf[COB_ID-0x580][0]);
			if(state == true){
				return true;
			}
		}
		else{
			
			state=false;
		}
		retry_count++;
	}while(state == false && retry_count < 3 );
	return false;
}


/**
  * @brief  CAN SDO_发送长数据包包长+重传机制 
  * @param	COB_ID:CAN发送ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param	pkglen:发送的包长 
  * @param	timeout_ms 超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_FirstNSend(unsigned int COB_ID,unsigned char t,unsigned short int object,unsigned char Sub_ID,unsigned int pkglen,unsigned int timeout_ms)
{
	bool state=false;
	unsigned int retry_count=0;
	do{
		CAN_SDO_NormalFirstSend(COB_ID,t,object,Sub_ID,pkglen);
		if(CAN_WaitSDORespands(COB_ID,timeout_ms) == true){//wait AsK timeout_ms
			state=CAN_SDO_NormalFirstSendResponds(t,object,Sub_ID,&RespondsBuf[COB_ID-0x580][0]);
			if(state == true){
				return true;
			}
		}
		else{
			state=false;
		}
		retry_count++;
	}while(state == false && retry_count < 3 );
	return false;
}

/**
  * @brief  CAN SDO_发送长数据包数据+重传机制 
  * @param	COB_ID:CAN发送ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param	*buf:发送数据 buf[7] 
  * @param	pkglen:发送的包长 =7 
  * @param	timeout_ms 超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_NextNSend(unsigned int COB_ID,unsigned char t,unsigned short int object,unsigned char Sub_ID,unsigned char* buf,unsigned int pkglen,unsigned int timeout_ms)
{
	bool state=false;
	unsigned int retry_count=0;
	do{
		CAN_SDO_NormalNextSend(COB_ID,t,buf,pkglen);
		if(CAN_WaitSDORespands(COB_ID,timeout_ms) == true){//wait AsK timeout_ms
		
			state=CAN_SDO_NormalNextSendResponds(t,&RespondsBuf[COB_ID-0x580][0]);
			if(state == true){
				return true;
			}
		}
		else{
			state=false;
		}
		retry_count++;
	}while(state == false && retry_count < 3 );
	return false;
}


/**
  * @brief  CAN SDO_发送长数据包最后一包+重传机制 
  * @param	COB_ID:CAN发送ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param	*buf:发送数据 buf[pkglen] 
  * @param	pkglen:发送的包长 <=7 
  * @param	timeout_ms 超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_EndNSend(unsigned int COB_ID,unsigned char t,unsigned short int object,unsigned char Sub_ID,unsigned char* buf,unsigned int pkglen,unsigned int timeout_ms)
{
	bool state=false;
	unsigned int retry_count=0;
	do{
		CAN_SDO_NormalEndSend(COB_ID,t,buf,pkglen);
		if(CAN_WaitSDORespands(COB_ID,timeout_ms) == true){//wait AsK timeout_ms
			state=CAN_SDO_NormalNextSendResponds(t,&RespondsBuf[COB_ID-0x580][0]);
			if(state == true){
				return true;
			}
		}
		else{
			state=false;
		}
		retry_count++;
	}while(state == false && retry_count < 3 );
	return false;
}


/**
  * @brief  CAN SDO_读短数据包+重传机制 
  * @param	COB_ID:CAN发送ID 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *buf:接收数组 
  * @param	len:接收长度 
  * @param	timeout_ms 超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_ERead(unsigned int COB_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int* len,unsigned int timeout_ms)
{
	
	bool state=false;
	unsigned int retry_count=0;
	do{
		CAN_SDO_ExpeditedRead(COB_ID,object,Sub_ID);//read buf
		if(CAN_WaitSDORespands(COB_ID,timeout_ms) == true){//wait AsK timeout_ms
			state=CAN_SDO_ExpeditedReadResponds(object,Sub_ID,&RespondsBuf[COB_ID-0x580][0],buf,len);
			if(state == true){
				return true;
			}
		}
		else{
			state=false;
		}
		retry_count++;
	}while(state == false && retry_count < 3 );
	return false;
}



/**
  * @brief  CAN SDO_读长数据包包长+重传机制 
  * @param	COB_ID:CAN读取ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param	pkglen:接收的总包长
  * @param	timeout_ms 超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_FirstNRead(unsigned int COB_ID,unsigned char t,unsigned short int object,unsigned char Sub_ID,unsigned int* pkglen,unsigned int timeout_ms)
{
	bool state=false;
	unsigned int retry_count=0;
	do{
		CAN_SDO_NormalRead(COB_ID,t,object,Sub_ID);
		if(CAN_WaitSDORespands(COB_ID,timeout_ms) == true){//wait AsK timeout_ms
			state=CAN_SDO_NormalFirstReadResponds(object,t,Sub_ID,&RespondsBuf[COB_ID-0x580][0],pkglen);
			if(state == true){
				return true;
			}
		}
		else{
			state=false;
		}
		retry_count++;
	}while(state == false && retry_count < 3 );
	return false;
}


/**
  * @brief  CAN SDO_读长数据包数据+重传机制 
  * @param	COB_ID:CAN读取ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param	*buf:读取的数据包 buf[7] 
  * @param	*len:接收的包长 =7 
  * @param	timeout_ms 超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_NextNRead(unsigned int COB_ID,unsigned char t,unsigned short int object,unsigned char Sub_ID,unsigned char* buf,unsigned int* len,unsigned int timeout_ms)
{
	bool state=false;
	unsigned int retry_count=0;
	do{
		CAN_SDO_NormalRead(COB_ID,t,object,Sub_ID);
		if(CAN_WaitSDORespands(COB_ID,timeout_ms) == true){//wait AsK timeout_ms
			state=CAN_SDO_NormalNextReadResponds(object,t,Sub_ID,&RespondsBuf[COB_ID-0x580][0],buf,len);
			if(state == true){
				return true;
			}
		}
		else{
			state=false;
		}
		retry_count++;
	}while(state == false && retry_count < 3 );
	return false;
}


/**
  * @brief  CAN SDO_读长数据包最后一包数据+重传机制 
  * @param	COB_ID:CAN读取ID 
  * @param	t:toggle反转电平 0 - 1 - 0 首次t=0; 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param	*buf:读取的数据包
  * @param	*len:接收的包长 <=7
  * @param	timeout_ms 超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_EndNRead(unsigned int COB_ID,unsigned char t,unsigned short int object,unsigned char Sub_ID,unsigned char* buf,unsigned int* len,unsigned int timeout_ms)
{
	bool state=false;
	unsigned int retry_count=0;
	do{
		CAN_SDO_NormalRead(COB_ID,t,object,Sub_ID);
		if(CAN_WaitSDORespands(COB_ID,timeout_ms) == true){//wait AsK timeout_ms
			state=CAN_SDO_NormalEndReadResponds(object,t,Sub_ID,&RespondsBuf[COB_ID-0x580][0],buf,len);
			if(state == true){
				return true;
			}
		}
		else{
			state=false;
		}
		retry_count++;
	}while(state == false && retry_count < 3 );
	return false;
}




/**
  * @brief  CAN SDO_发送短数据包
  * @param	COB_ID:CAN发送ID 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *buf:发送数组 
  * @param	len:发送长度 
  * @param	timeout_ms 每包超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_SendExpeditedMode(unsigned int COB_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int len,unsigned int timeout_ms)
{
	return CAN_SDO_ESend(COB_ID,object,Sub_ID,buf,len,timeout_ms);
}



/**
  * @brief  CAN SDO_读短数据包
  * @param	COB_ID:CAN读取ID 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *buf:读取数组 
  * @param	len:读取长度 
  * @param	timeout_ms 每包超时时间 
  * @retval bool 读取成功: return true 否则return false 
  */
bool CAN_SDO_ReadExpeditedMode(unsigned int COB_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int* len,unsigned int timeout_ms)
{
	return CAN_SDO_ERead(COB_ID,object,Sub_ID,buf,len,timeout_ms);
}

/**
  * @brief  CAN SDO_发送长数据包
  * @param	COB_ID:CAN发送ID 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *buf:发送数组 
  * @param	len:发送长度 
  * @param	timeout_ms 每包超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_SendNormalMode(unsigned int COB_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int len,unsigned int timeout_ms)
{
	bool state=false;
	unsigned int toggle=0,Nowlen=len,Send_Progress=0;
	/*send pkglength*******************************/
	if(CAN_SDO_FirstNSend(COB_ID,toggle,object,Sub_ID,len,timeout_ms) == false){
		return false;
	}
	/*send buffer**********************************/
	while( Nowlen > 7){
		toggle=(toggle+1)&0x01;
		if(CAN_SDO_NextNSend(COB_ID,toggle,object,Sub_ID,&buf[len-Nowlen],7,timeout_ms) == false){
			return false;
		}
		Send_Progress=(len-Nowlen)*100/len;//Downloading
		Nowlen-=7;
//		printf("Sending%d\r\n",Send_Progress);
	};
	/*send end buffer*****************************/
	toggle=(toggle+1)&0x01;
	if(CAN_SDO_EndNSend(COB_ID,toggle,object,Sub_ID,&buf[len-Nowlen],Nowlen,timeout_ms) == false){
			return false;
	}
	Send_Progress=100;
//	printf("Sending%d\r\n",Send_Progress);
	return true;
}

/**
  * @brief  CAN SDO_读长数据包
  * @param	COB_ID:CAN读取ID 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *buf:读取数组 
  * @param	len:读取长度 
  * @param	timeout_ms 每包超时时间 
  * @retval bool 读取成功: return true 否则return false 
  */
bool CAN_SDO_ReadNormalMode(unsigned int COB_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int* len,unsigned int timeout_ms)
{
	bool state=false;
	unsigned int toggle=0,Nowlen,Read_Progress=0;
	unsigned int nextRlen=7;
	/*Read pkglength*******************************/
	
	if(CAN_SDO_FirstNRead(COB_ID,toggle,object,Sub_ID,len,timeout_ms) == false){
		return false;
	}
	Nowlen=len[0];
	/*read buffer**********************************/
	while( Nowlen > 7){
		toggle=(toggle+1)&0x01;
		
		if(CAN_SDO_NextNRead(COB_ID,toggle,object,Sub_ID,&buf[(int)(len[0]-Nowlen)],&nextRlen,timeout_ms) == false){
			return false;
		}
		Read_Progress=(len[0]-Nowlen)*100/len[0];//Uploading
		Nowlen-=7;
//		printf("Reading%d\r\n",Read_Progress);
	}
	/*send end buffer*****************************/
	toggle=(toggle+1)&0x01;
	if(CAN_SDO_EndNRead(COB_ID,toggle,object,Sub_ID,&buf[(unsigned int)(len[0]-Nowlen)],&nextRlen,timeout_ms) == false){
			return false;
	}
	Read_Progress=100;
//	printf("Reading%d\r\n",Read_Progress);
	return true;
}


/**
  * @brief  CAN SDO_发送数据包
  * @param	COB_ID:CAN发送ID 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *buf:发送数组 
  * @param	len:发送长度 
  * @param	timeout_ms 每包超时时间 
  * @retval bool 发送成功: return true 否则return false 
  */
bool CAN_SDO_Send(int Master_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int len,unsigned int timeout_ms)
{
	unsigned int Function_ID=0x580;
	unsigned int COB_ID=Function_ID+Master_ID;
	bool state = false;
	if(len > 4){
		state=CAN_SDO_SendNormalMode(COB_ID,object,Sub_ID,buf,len,timeout_ms);
	}
	else{
		state=CAN_SDO_SendExpeditedMode(COB_ID,object,Sub_ID,buf,len,timeout_ms);
	}
	return state;
}

/**
  * @brief  CAN SDO_读数据包
  * @param	COB_ID:CAN读取ID 
  * @param	object:对象字典
  * @param	Sub_ID:子ID 
  * @param  *buf:读取数组 
  * @param	len:读取长度 
  * @param	timeout_ms 每包超时时间 
  * @retval bool 读取成功: return true 否则return false 
  */
bool CAN_SDO_Read(int Master_ID,unsigned short int object,unsigned char Sub_ID,unsigned char *buf,unsigned int* len,unsigned int timeout_ms)
{
	unsigned int Function_ID=0x580;
	unsigned int COB_ID=Function_ID+Master_ID;
	bool state = false;
	if(len[0] > 4){
		
		state=CAN_SDO_ReadNormalMode(COB_ID,object,Sub_ID,buf,len,timeout_ms);
	}
	else{
		state=CAN_SDO_ReadExpeditedMode(COB_ID,object,Sub_ID,buf,len,timeout_ms);
	}
	return state;
}



/************************CANOPEN_SDO_ASK**********************************************************/
/**
  * @brief  CAN SDO_发送短数据包的响应 
  * @param	Function_ID:CAN_SDO功能ID 
  * @param	Master_ID:数据主ID 
  * @param	*buf:接收的数据包 buf[8]  
  * @retval bool 回复成功: return true 否则return false 
  */
bool CAN_SDO_SendExpeditedASK(unsigned int Function_ID,int Master_ID,unsigned char *buf)
{
	unsigned int COB_ID=Function_ID+Master_ID;
	unsigned char byte0=0x00,bytes[8]={0};
	unsigned short int obj=0,sub_id=0;
	unsigned int len=4-((buf[0]>>2)&0x03);
	sub_id=(buf[3]);
	obj|=(buf[2]<<8);
	obj|=(buf[1]);
	
	/**********/	
	byte0|=(3<<5);
	bytes[0]=byte0;
	memcpy(bytes+1,buf+1,3);
	User_CAN_Transmit(COB_ID,bytes,8);
	CANObject_Dispose(Master_ID,obj,sub_id,'W',buf+4,&len);
	/*************/
	return true;
}

/**
  * @brief  CAN SDO_发送长数据包的响应 
  * @param	Function_ID:CAN_SDO功能ID 
  * @param	Master_ID:数据主ID 
  * @param	*buf:接收的数据包 buf[8] 
  * @param	timeout_ms:整包超时时间 
  * @retval bool 回复成功: return true 否则return false 
  */
bool CAN_SDO_SendNormalASK(unsigned int Function_ID,int Master_ID,unsigned char *buf,unsigned int timeout_ms)
{
	static unsigned int SendCnt=0,packageLen=0,ReceivedLen=0,NowSub_id,Nowtoggle=0;
	static unsigned short int NowObj;
	unsigned int COB_ID=Function_ID+Master_ID;
	unsigned char byte0=0x00,bytes[8]={0};
	int retoggle=(buf[0]>>4)&0x01;
	int len=7-((buf[0]>>1)&0x07);
	int s=buf[0]&0x01; 
	if(timeout_ms == 1){//包接收超时 段包处理 
		SendCnt=0;
		Nowtoggle=0;
		NowObj=0;
		NowSub_id=0;
		ReceivedLen=0; 
		packageLen=0;
	}
	if(Nowtoggle != retoggle && SendCnt > 0){//重传回复处理 
		SendCnt--;
	}
	if(SendCnt == 0){//第一次 
		NowSub_id=(buf[3]);
		NowObj|=(buf[2]<<8);
		NowObj|=(buf[1]);
		byte0|=(0<<5);
		byte0|=(retoggle<<4);
		memcpy(&packageLen,buf+4,4);//get packagelength
		memcpy(bytes+1,buf+1,3);
		User_CAN_Transmit(COB_ID,bytes,8);
		SendCnt = 1;
		Nowtoggle=(Nowtoggle+1)&0x01;
	}
	else{	
		if(s == 0){
			ReceivedLen=(SendCnt-1)*7;
			SendCnt++;
			Nowtoggle=(Nowtoggle+1)&0x01;
			memcpy(SDORevForSendBuffer+ReceivedLen,buf+1,7);
			byte0|=(0<<5);
			byte0|=(retoggle<<4);
			bytes[0]=byte0;
			User_CAN_Transmit(COB_ID,bytes,8);
//			memcpy(lowbuf,buf+1,7);
		}
		else if(s == 1){
			ReceivedLen=(SendCnt-1)*7;
			memcpy(SDORevForSendBuffer+ReceivedLen,buf+1,len);
			ReceivedLen+=len;
			byte0|=(0<<5);
			byte0|=(retoggle<<4);
			bytes[0]=byte0;
			User_CAN_Transmit(COB_ID,bytes,8);
//			memcpy(nowbuf,buf+1,len);
			if(ReceivedLen == packageLen){//接收成功 
				CANObject_Dispose(Master_ID,NowObj,NowSub_id,'W',SDORevForSendBuffer,&packageLen);
			}
//			memset(SDORevForSendBuffer,0,packageLen);
			SendCnt=0;
			Nowtoggle=0;
			NowObj=0;
			NowSub_id=0;
		}
		
	}
	/*************/
	return true;
}


/**
  * @brief  CAN SDO_读取短数据包的响应 
  * @param	Function_ID:CAN_SDO功能ID 
  * @param	Master_ID:数据主ID 
  * @param	*buf:接收的数据包 buf[8] 
  * @retval bool 回复成功: return true 否则return false 
  */
bool CAN_SDO_ReadExpeditedASK(unsigned int Function_ID,int Master_ID,unsigned char *buf)
{
	unsigned int COB_ID=Function_ID+Master_ID;
	unsigned char byte0=0x00,bytes[8]={0};
	unsigned short int obj=0,sub_id=0,n;
	unsigned int len=4;
	sub_id=(buf[3]);
	obj|=(buf[2]<<8);
	obj|=(buf[1]);
	if(CANObject_Dispose(Master_ID,obj,sub_id,'R',bytes+4,&len) == true){//从对象字典读出数据 
		n=4-len;
		byte0=0;
		byte0|=(2<<5);
		byte0|=(n<<2);
		byte0|=0x03;
	}
	else{
		memset(bytes+4,0xff,4);//error 码 
		byte0|=(0<<5);
	}
	bytes[0]=byte0&0x7f;
	memcpy(bytes+1,buf+1,3);//copy object 
	User_CAN_Transmit(COB_ID,bytes,8);
	return true;
}


/**
  * @brief  CAN SDO_读取长数据包的响应 
  * @param	Function_ID:CAN_SDO功能ID 
  * @param	Master_ID:数据主ID 
  * @param	*buf:接收的数据包 buf[8] 
  * @param	timeout_ms:整包超时时间 
  * @retval bool 回复成功: return true 否则return false 
  */
bool CAN_SDO_ReadNormalASK(unsigned int Function_ID,int Master_ID,unsigned char *buf,unsigned int timeout_ms)
{
	static unsigned int SendCnt=0,packageLen=0,ReceivedLen=0,NowSub_id,Nowtoggle=0;
	static unsigned short int NowObj;
	unsigned int COB_ID=Function_ID+Master_ID;
	unsigned char byte0=0x00,bytes[8]={0};
	int retoggle=(buf[0]>>4)&0x01;
	int len;
	if(timeout_ms == 1){//包接收超时 段包处理 
		SendCnt=0;
		Nowtoggle=0;
		NowObj=0;
		NowSub_id=0;
		ReceivedLen=0; 
	}
	if(Nowtoggle != retoggle && SendCnt > 0){//重传回复处理 
		SendCnt--;
	}
	if(SendCnt == 0){//第一次 
		NowSub_id=(buf[3]);
		NowObj|=(buf[2]<<8);
		NowObj|=(buf[1]);
		CANObject_Dispose(Master_ID,NowObj,NowSub_id,'R',SDORevForReadBuffer,&packageLen);//读取包 
		byte0|=(0<<5);
		byte0|=(retoggle<<4);
		memcpy(bytes+4,&packageLen,4);//装载包长 
		memcpy(bytes+1,buf+1,3);//装载字典 
		User_CAN_Transmit(COB_ID,bytes,8);
		SendCnt = 1;
		Nowtoggle=(Nowtoggle+1)&0x01;
	}
	else{
		ReceivedLen=(SendCnt-1)*7;
		if((packageLen -ReceivedLen) > 7){
			byte0|=(0<<5);
			byte0|=(retoggle<<4);
			SendCnt++;
			Nowtoggle=(Nowtoggle+1)&0x01;
			memcpy(bytes+1,SDORevForReadBuffer+ReceivedLen,7);//装载数据 
		}
		else{//最后一包 
			byte0|=1;//s = 1
			memcpy(bytes+1,SDORevForReadBuffer+ReceivedLen,packageLen -ReceivedLen);
			ReceivedLen+=len;
			if(ReceivedLen == packageLen){//发送成功				
			}
			SendCnt=0;
			Nowtoggle=0;
			NowObj=0;
			NowSub_id=0;
		}
		byte0|=(0<<5);
		byte0|=(retoggle<<4);
		bytes[0]=byte0;
		User_CAN_Transmit(COB_ID,bytes,8);
		
	}
	/*************/
	return true;
}
/**
  * @brief  CAN SDO_响应 
  * @param	Master_ID:数据主ID 
  * @param	*buf:接收的数据包 buf[8] 
  * @param	timeout_ms:整包超时时间 
  * @retval bool 回复成功: return true 否则return false 
  */
bool CAN_SDO_ASK(int Master_ID,unsigned char *buf,unsigned int timeout_ms)//0x600
{
	bool state = false;
	unsigned int Function_ID=0x600;
	if((buf[0]>>5) == 1){//Expedited Mode: Send Ask
		state=CAN_SDO_SendExpeditedASK(Function_ID,Master_ID,buf);
	}
	else if((buf[0]>>5) == 3){//Normal Read Ask
		state=CAN_SDO_ReadNormalASK(Function_ID,Master_ID,buf,timeout_ms);
	}
	else if((buf[0]>>5) == 2){//Expedited Mode: Read Ask 
		state=CAN_SDO_ReadExpeditedASK(Function_ID,Master_ID,buf);
	}
	else if((buf[0]>>5) == 0){//Normal Send Ask
		state=CAN_SDO_SendNormalASK(Function_ID,Master_ID,buf,timeout_ms);
	}
	return state;
}
/*******SDO********************************************************/

//osThreadId CanSDOHandle;
void CANSDOThreadCreate(int num)
{
//	osThreadDef(CanSDO, CanSDOThread, num,0, 256);
//	CanSDOHandle = osThreadCreate(osThread(CanSDO), NULL);
	while(CANSDOAskQueue == NULL){
		CANSDOAskQueue=xQueueCreate(1, sizeof(int));
	}
	while(CANSDORespondsQueue == NULL){
		CANSDORespondsQueue=xQueueCreate(1, sizeof(int));
	}
	memset(SDORevForSendBuffer,0xff,sizeof(SDORevForSendBuffer));
}

void SDO_ASK_Process(int Master_ID,uint8_t* buf,int len)
{
	static uint32_t Nowtime=0,Lasttime=0;
	int timeoutstate=0;
//	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if(Master_ID != MY_ID)return;
	Nowtime = Get_NowTimeMs();
	if((Nowtime - Lasttime) > 300){//超时 断包处理
		timeoutstate = 1;
	}
	else{
		timeoutstate = 0;
	}
	CAN_SDO_ASK(Master_ID,buf,timeoutstate);
	Lasttime = Nowtime;
//			memcpy(ReceiveBuf[Master_ID],buf,len);
//			xQueueOverwriteFromISR(CANSDOAskQueue,&Master_ID, &xHigherPriorityTaskWoken);
}


void SDO_Response_Process(int Master_ID,uint8_t* buf,int len)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	memcpy(RespondsBuf[Master_ID],buf,len);
	xQueueOverwriteFromISR(CANSDORespondsQueue,&Master_ID, &xHigherPriorityTaskWoken);
}


void CanSDOThread(void const * argument)
{
	int Master_ID,timeoutstate=0;
	osDelay(10);
	while(1){
		if(xQueueReceive(CANSDOAskQueue, &Master_ID,300)==pdTRUE){ //300ms超时
			CAN_SDO_ASK(Master_ID,ReceiveBuf[Master_ID],timeoutstate);
			timeoutstate=0;
		}
		else{//SDO Send Or Read
			timeoutstate=1;
		}	
	}
}

