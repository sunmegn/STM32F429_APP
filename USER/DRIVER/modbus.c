#include "modbus.h"

Ammonia_UsartRec_t ammonia_rec;
AmmoniaMsg_t ammonia_msg;

u8 VALAMMONIA[6] = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01};
u8 VALTEMP_1[6] = {0x01, 0x03, 0x00, 0x14, 0x00, 0x01};

u8 buffff[] = {0x01, 0x03, 0x02, 0x02, 0x3B, 0x35, 0xCB};

/**
  * @funNm   Pressure_Init
  * @brief   ��ʼѹ����� ����DMA���� ʹ�ܴ����ж�
  * @param	 �� ammonia
  * @retval  ��
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
  * @brief   CRC16У�����
  * @param	 *ptr  У�������
  * @param	 len   У�����ݵĳ���
  * @retval  crc   У�������
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
  * @brief   USART3�����жϽ��ܻص����� ��������
  * @param	 ��
  * @retval  ��
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
			xQueueSendFromISR(ModbusRxQueue, &ammonia_rec.RX_Size, &xHigherPriorityTaskWoken); //1���ȶ��� ���õ�ͬ��ֵ�ź���
		}
	}
}

/**
  * @funNm   Analy_Date
  * @brief   �������ݣ�ȡ�����ֽ�����λ����ת��ת��Ϊ����������
  * @param	 *buff  ����������
  * @param	 addr   ���ݵ���ʼ��ַ
  * @retval  num    ������ĸ���������
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
  * @brief   ���ڷ��Ͳ�����������(485)
  * @param	 *buf    ���͵�����
  * @param	 len     ���ݳ���
  * @retval  ��
  */
static void uart_send(u8 *buf, u16 len)
{
	RE(1);
	HAL_UART_Transmit(&AMMONIA_USART, buf, len, 0x100);
	RE(0);
}

/**
  * @funNm   Modbus485_Send
  * @brief   ModbusЭ�鷢�ͺ���
  * @param	 *combuf     ָ���
  * @retval  ��
  */

void Modbus485_Send(u8 *combuf)
{
	u8 sendbuff[8];
	/*���ָ��У��λ ������һ������*/
	for (int i = 0; i < 6; i++)
	{
		sendbuff[i] = combuf[i];
	}
	//���ָ��CRC16У��λ
	sendbuff[6] = crc16bitbybit(combuf, 6) | 0xff00;
	sendbuff[7] = (crc16bitbybit(combuf, 6) | 0x00ff) >> 8;

	//����ָ��
	uart_send(sendbuff, 8);
}

/**
  * @funNm   Modbus485_Recive
  * @brief   ModbusЭ����ܴ�����
  * @param	 *combuf     ָ���
  * @param	 time        ����ʱ��
  * @retval  1 ��ȡ����   0δ��ȡ����
  */
uint8_t Modbus485_Recive(u8 *combuf, u16 time)
{
	u8 size;

	/*�ȴ���Ӧ ���ղ�����*/
	if (xQueueReceive(ModbusRxQueue, &size, time) == pdPASS)
	{
		for (int i = 0; i < ammonia_rec.RX_Size; i++)
		{
			//����豸��ַ
			if ((ammonia_rec.RX_pData[i] == combuf[0]) && (ammonia_rec.RX_pData[i + 1] == combuf[1]))
			{
				//��ⳤ��
				if (ammonia_rec.RX_Size - i >= ammonia_rec.RX_pData[i + 2] + 5)
				{
					//CRC16У��
					if ((crc16bitbybit(ammonia_rec.RX_pData + i, ammonia_rec.RX_pData[i + 2] + 5 - 2)) ==
						((((uint16_t)(ammonia_rec.RX_pData[i + ammonia_rec.RX_pData[i + 2] + 5 - 1])) << 8) | (ammonia_rec.RX_pData[i + ammonia_rec.RX_pData[i + 2] + 5 - 2])))
					{
						memcpy(ammonia_rec.RealBuff, ammonia_rec.RX_pData + i, ammonia_rec.RX_pData[i + 2] + 5); //��ȡ��ȷ����

						//�������ղ�����
						memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
						HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

						return 1;
					}
					else
					{
						//���½��ܲ�����
						memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
						HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

						return 0;
					}
				}
				else //���յ����ֽڳ��Ȳ��������½���
				{
					memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
					HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

					return 0;
				}
			}
		}
		//δ���ܵ���ȷ���ݣ����½���
		memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
		HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

		return 0;
	}
	else //����ʱ�䳬ʱ �궨��Ч����  ���½���
	{
		memset(ammonia_rec.RX_pData, '\0', AMMONIA_RX_LEN);
		HAL_UART_Receive_DMA(&AMMONIA_USART, ammonia_rec.RX_pData, AMMONIA_RX_LEN);

		return 0;
	}
}

/**
  * @funNm   Pressure_GetData
  * @brief   ��ȡѹ������
  * @param	 pressure    ����ѹ���Ľṹ������
  * @retval  ��
  */
void Pressure_GetData(AmmoniaMsg_t *ammonia)
{
	static uint8_t Count[4] = {0};
	static int a = 0;

	//�豸1����ַ0x01���޷���ȡ������ĸ����ϣ��Ĵ�������  �����ݵ�ַ������ �����λ�ȡѹ��ֵ���¶�ֵ
	Modbus485_Send(VALAMMONIA);
	//    a = Modbus485_Recive(VALAMMONIA, 100);
	//	if(Modbus485_Recive(VALAMMONIA, 100) == 1)
	//	{
	ammonia->valid_1 = 1;
	//		ammonia->ammonia_val   = Analy_Date(ammonia_rec.RealBuff,3);      // ѹ��ֵ bar
	//
	//		Count[1]=0;
	//	}
	//	else
	//	{
	//		ammonia->valid_1 = 0;
	//		Count[1] ++;
	//		if(Count[1] >= 3)  //��������δ��ȡ������ ����ֵ����
	//		{
	//			ammonia->ammonia_val = 0;
	//			Count[1] = 0;     //���������־λ ���¼���
	//		}
	//	}
	//	Modbus485_Send(VALTEMP_1);
	//	if(Modbus485_Recive(VALTEMP_1, 100) == 1)
	//	{
	//		ammonia->valid_1 = 1;
	//		ammonia->ammonia_t  = Analy_Date(ammonia_rec.RealBuff,3);    //�¶�ֵ ��
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
  * @param	 ��
  * @retval  ��
  */

void Pressure_Test_Demo(void)
{
}
