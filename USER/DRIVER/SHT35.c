#include "SHT35.h"
/**
  ******************************************************************************
  * @file    SHT35.c
  * @author  LiuYang
  * @brief   SHT35 IIC ��ʪ�ȶ�ȡ����
  *
  *
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */
uint8_t SHT3X_Modecommand_Buffer[2]={0x22,0x36};  //periodic mode commands 
uint8_t SHT3X_Fetchcommand_Bbuffer[2]={0xE0,0x00};//��ȡ�������

uint8_t SHT3X_Data_Buffer[6];                     //byte0,1Ϊ�¶� byte4,5Ϊʪ��
static float Humidity;                            //ʪ�� ����
static float Temperature;                         //�¶� ����Ϊ����

TempHumiMsg_t temphum;

void SHT35_Init(void)
{
	HAL_I2C_Master_Transmit(&SHT35_IIC,0x44<<1,SHT3X_Modecommand_Buffer,2,0x100); //��һ��������periodic mode commands�������������ԵĽ�����ʪ��ת��
}
void SHT35_ReadData(TempHumiMsg_t *tempTempHum)
{
	  HAL_I2C_Master_Transmit(&SHT35_IIC,0x44<<1,SHT3X_Fetchcommand_Bbuffer,2,0x100); //�ڶ�������ʱ��ȡ������������ 
    HAL_I2C_Master_Receive(&SHT35_IIC,(0x44<<1)+1,SHT3X_Data_Buffer,6,0x100); 
    Temperature =(float)((((SHT3X_Data_Buffer[0]<<8)+SHT3X_Data_Buffer[1])*175)/65535.0f)-45; //�õ����϶��¶� 
    Humidity =(((SHT3X_Data_Buffer[3]<<8)+SHT3X_Data_Buffer[4])*100)/65535.0f; //���Եõ����ʪ��
	  tempTempHum->temp = Temperature; 
	  tempTempHum->humi = Humidity;
}
	
void SHT35_Test_Demo(void)
{
//  SCB_CleanInvalidateDCache();
	SHT35_Init();

	while(1)
	{

		SHT35_ReadData(&temphum);
		delay_ms(500);
	}
	
}
