#include "modbus.h"

Ammonia_UsartRec_t ammonia_rec;
AmmoniaMsg_t ammonia_msg;

u8 VALAMMONIA[6] = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01};
u8 VALTEMP_1[6] = {0x01, 0x03, 0x00, 0x14, 0x00, 0x01};

u8 buffff[] = {0x01, 0x03, 0x02, 0x02, 0x3B, 0x35, 0xCB};

/**
  * @funNm   Pressure_Init
  * @brief   初始压力检测 串口DMA接受 使能串口中断
  * @param	 无 ammonia
  * @retval  无
  */
void ammonia_Init(void)
{
	RE(0);
	HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, sizeof(ammonia_rec.RX_pData));
	__HAL_UART_CLEAR_IDLEFLAG(&AMMONIA_USART);
	__HAL_UART_ENABLE_IT(&AMMONIA_USART, UART_IT_IDLE);
}

extern QueueHandle_t ModbusRxQueue;
BaseType_t xHigherPriorityTaskWoken = NULL;

/**
  * @funNm   crc16bitbybit
  * @brief   CRC16校验计算
  * @param	 *ptr  校验的数据
  * @param	 len   校验数据的长度
  * @retval  crc   校验计算结果
  */
const uint16_t polynom = 0xA001;
uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len)
{
	uint8_t i;
	uint16_t crc = 0xffff;

	if (len == 0)
	{
		len = 1;
	}
	while (len--)
	{
		crc ^= *ptr;
		for (i = 0; i < 8; i++)
		{
			if (crc & 1)
			{
				crc >>= 1;
				crc ^= polynom;
			}
			else
			{
				crc >>= 1;
			}
		}
		ptr++;
	}
	return (crc);
}

/**
  * @funNm   HAL_USART3_Receive_IDLE
  * @brief   USART3空闲中断接受回调函数 接受数据
  * @param	 无
  * @retval  无
  */
extern DMA_HandleTypeDef hdma_usart3_rx;
void HAL_USART3_Receive_IDLE(void)
{
	if ((__HAL_UART_GET_FLAG(&AMMONIA_USART, UART_FLAG_IDLE) != RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&AMMONIA_USART);
		HAL_UART_DMAStop(&AMMONIA_USART); //DMAmuxChannel

		ammonia_rec.RX_Size = AMMONIA_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

		if (ModbusRxQueue)
		{
			xQueueSendFromISR(ModbusRxQueue, &ammonia_rec.RX_Size, &xHigherPriorityTaskWoken); //1长度队列 作用等同二值信号量
		}
	}
}

/**
  * @funNm   Analy_Date
  * @brief   解析数据，取出四字节数据位，反转并转化为浮点型数据
  * @param	 *buff  解析的数据
  * @param	 addr   数据的起始地址
  * @retval  num    解析后的浮点型数据
  */
float Analy_Date(u8 *buff, u8 addr)
{
	float num;
	uint8_t rbuff[2];
	unsigned int temp = 0;

	rbuff[1] = buff[addr];
	rbuff[0] = buff[addr + 1];

	memcpy(&temp, rbuff, 2);

	num = temp / 100.f;

	return num;
}

/**
  * @funNm   uart_send
  * @brief   串口发送不定长度数据(485)
  * @param	 *buf    发送的数据
  * @param	 len     数据长度
  * @retval  无
  */
static void uart_send(u8 *buf, u16 len)
{
	RE(1);
	HAL_UART_Transmit(&AMMONIA_USART, buf, len, 0x100);
	RE(0);
}

/**
  * @funNm   Modbus485_Send
  * @brief   Modbus协议发送函数
  * @param	 *combuf     指令缓存
  * @retval  无
  */

void Modbus485_Send(u8 *combuf)
{
	u8 sendbuff[8];
	/*添加指令校验位 并发送一次请求*/
	for (int i = 0; i < 6; i++)
	{
		sendbuff[i] = combuf[i];
	}
	//添加指令CRC16校验位
	sendbuff[6] = crc16bitbybit(combuf, 6) | 0xff00;
	sendbuff[7] = (crc16bitbybit(combuf, 6) | 0x00ff) >> 8;

	//发送指令
	uart_send(sendbuff, 8);
}

/**
  * @funNm   Modbus485_Recive
  * @brief   Modbus协议接受处理函数
  * @param	 *combuf     指令缓存
  * @param	 time        阻塞时间
  * @retval  1 获取数据   0未获取数据
  */
uint8_t Modbus485_Recive(u8 *combuf, u16 time)
{
	u8 size;

	/*等待响应 接收并处理*/
	if (xQueueReceive(ModbusRxQueue, &size, time) == pdPASS)
	{
		for (int i = 0; i < ammonia_rec.RX_Size; i++)
		{
			//检查设备地址
			if ((ammonia_rec.RX_pData[i] == combuf[0]) && (ammonia_rec.RX_pData[i + 1] == combuf[1]))
			{
				//检测长度
				if (ammonia_rec.RX_Size - i >= ammonia_rec.RX_pData[i + 2] + 5)
				{
					//CRC16校验
					if ((crc16bitbybit(ammonia_rec.RX_pData + i, ammonia_rec.RX_pData[i + 2] + 5 - 2)) ==
						((((uint16_t)(ammonia_rec.RX_pData[i + ammonia_rec.RX_pData[i + 2] + 5 - 1])) << 8) | (ammonia_rec.RX_pData[i + ammonia_rec.RX_pData[i + 2] + 5 - 2])))
					{
						memcpy(ammonia_rec.RealBuff, ammonia_rec.RX_pData + i, ammonia_rec.RX_pData[i + 2] + 5); //获取正确数据

						//继续接收并返回
						memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
						HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

						return 1;
					}
					else
					{
						//重新接受并返回
						memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
						HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

						return 0;
					}
				}
				else //接收到的字节长度不够，重新接受
				{
					memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
					HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

					return 0;
				}
			}
		}
		//未接受到正确数据，重新接收
		memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
		HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

		return 0;
	}
	else //阻塞时间超时 标定无效数据  重新接受
	{
		memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
		HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

		return 0;
	}
}

/**
  * @funNm   Pressure_GetData
  * @brief   获取压传数据
  * @param	 pressure    保存压传的结构体数据
  * @retval  无
  */
void Pressure_GetData(AmmoniaMsg_t *ammonia)
{
	static uint8_t Count[4] = {0};
	static int a = 0;

	//设备1（地址0x01）无法读取多个（四个以上）寄存器数据  且数据地址不连续 分两次获取压力值和温度值
	Modbus485_Send(VALAMMONIA);
	//    a = Modbus485_Recive(VALAMMONIA, 100);
	//	if(Modbus485_Recive(VALAMMONIA, 100) == 1)
	//	{
	ammonia->valid_1 = 1;
	//		ammonia->ammonia_val   = Analy_Date(ammonia_rec.RealBuff,3);      // 压力值 bar
	//
	//		Count[1]=0;
	//	}
	//	else
	//	{
	//		ammonia->valid_1 = 0;
	//		Count[1] ++;
	//		if(Count[1] >= 3)  //三次连续未获取到数据 将数值清零
	//		{
	//			ammonia->ammonia_val = 0;
	//			Count[1] = 0;     //清零计数标志位 重新计数
	//		}
	//	}
	//	Modbus485_Send(VALTEMP_1);
	//	if(Modbus485_Recive(VALTEMP_1, 100) == 1)
	//	{
	//		ammonia->valid_1 = 1;
	//		ammonia->ammonia_t  = Analy_Date(ammonia_rec.RealBuff,3);    //温度值 ℃
	//
	//		Count[2] = 0;
	//	}
	//	else
	//	{
	//		ammonia->valid_1 = 0;
	//		Count[2] ++;
	//		if(Count[2] >= 3)
	//		{
	//			ammonia->ammonia_t = 0.0f;
	//			Count[2] = 0;
	//		}
	//	}
}

u8 ChangeBaud115200[] = {0x01, 0x65, 0x00, 0xAA, 0x01, 0x35, 0x00, 0x00, 0x2B, 0xBD};
u8 ChangeBaud9600[] = {0x01, 0x65, 0X00, 0xAA, 0x00, 0x35, 0x00, 0x00, 0xD7, 0xBC};
u8 GetBaud[] = {0x01, 0x64, 0x00, 0xC0, 0x0A};
u8 Init[] = {0x01, 0x30, 0x34, 0x00};
//u8 lim = 0;
//void  changebaud(void)
//{
//	int size;
//	if(lim == 0)
//	{
//		huart3.Init.BaudRate = 9600;
//		uart_send(ChangeBaud,9);
//		if(xQueueReceive(ModbusRxQueue,&size,100) == pdPASS)
//		{
//			for(int i=0;i<pressure_rec.RX_Size;i++)
//			{
//				if((pressure_rec.RX_pData[i] == 0x01) && (pressure_rec.RX_pData[i+1] == 0x65) && (pressure_rec.RX_pData[i+2] == 0x00) && (pressure_rec.RX_pData[i+2] == 0x50) && (pressure_rec.RX_pData[i+2] == 0x0B))
//				{
//					huart3.Init.BaudRate = 115200;
//					lim=1;
//					memset(pressure_rec.RX_pData,'\0',PRESSURE_RX_LEN);
//					HAL_UART_Receive_DMA(&PRESSURE_USART,pressure_rec.RX_pData,PRESSURE_RX_LEN);
//				}
//			}
//		}
//	}
//}
/**
  * @funNm   Pressure_Test_Demo
  * @brief   
  * @param	 无
  * @retval  无
  */

void Pressure_Test_Demo(void)
{
}
