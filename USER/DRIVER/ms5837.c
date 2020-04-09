/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:42
 * @LastEditors:Robosea
 * @LastEditTime:2020-02-22 22:35:41
 * @FilePath      :\ROV_F429_APP-10-10\USER\DRIVER\ms5837.c
 * @brief         :���ѹ��,ˮ��ֵ��������ʼ��IIcЭ�飬MS5837ֵת��ΪPa
 */
#include "ms5837.h"

//��ѹ��״̬��
#define SCTemperature   0x01 //��ʼ�¶�ת��
#define CTemperatureing 0x02 //����ת���¶�
#define SCPressure      0x03 //��ʼ��ѹת��
#define SCPressureing   0x04 //����ת����ѹ

/*
C1  ѹ�������� SENS|T1
C2  ѹ������  OFF|T1
C3	�¶�ѹ��������ϵ�� TCS
C4	�¶�ϵ����ѹ������ TCO
C5	�ο��¶� T|REF
C6 	�¶�ϵ�����¶� TEMPSENS
*/
static uint16_t Cal_C[7]; //���ڴ��PROM�е�6������1-6
/*
dT ʵ�ʺͲο��¶�֮��Ĳ���
TEMP ʵ���¶�	
*/
int32_t dT, TEMP;
/*
OFF ʵ���¶Ȳ���
SENS ʵ���¶�������
*/

uint32_t D1_Pres, D2_Temp = 20; // ����ѹ��ֵ,�����¶�ֵ

int64_t OFF_, SENS, P, T2, OFF2, SENS2;

float depth;
float P_base = 0.0, Pressure = 0.0, Temperature = 0.0; //����ѹ
/*******************************************************************************
  * @��������	MS583730BA_RESET
  * @����˵��   ��λMS5611
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
void MS583703BA_RESET(void)
{
    IIC_Start();
    IIC_Send_Byte(0xEC); //CSB�ӵأ�������ַ��0XEE������ 0X77
    IIC_Wait_Ack();
    IIC_Send_Byte(0x1E); //���͸�λ����
    IIC_Wait_Ack();
    IIC_Stop();
}

/**
 * @function_name:Change_YC
 * @brief:ѹ��ԭʼֵת����ʵ��ѹ��ֵ
 * @param {type} 
 * @return:None
 */
void Change_YC(void)
{
    dT   = (int32_t)D2_Temp - ((uint32_t)Cal_C[5] << 8);
    TEMP = 2000 + ((int64_t)dT * ((int64_t)Cal_C[6]) >> 23);
    if (TEMP < 2000) // low temp
    {
        T2    = (3 * ((int64_t)dT * (int64_t)dT)) >> 33;
        OFF2  = 3 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 2;
        SENS2 = 5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 8;

        if (TEMP < -1500)
        {
            OFF2 += 7 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
            SENS2 += 4 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
        }
    }
    else
    {
        T2    = 2 * ((int64_t)dT * (int64_t)dT) >> 37;
        OFF2  = ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) >> 4;
        SENS2 = 0;
    }
    OFF_ = ((int64_t)(Cal_C[2]) << 16) + (((int64_t)(Cal_C[4]) * dT) >> 7);
    OFF_ -= OFF2;

    SENS = ((int64_t)Cal_C[1] << 15) + (((int64_t)Cal_C[3] * dT) >> 8);
    SENS -= SENS2;

    Pressure    = ((((D1_Pres * SENS) >> 21) - OFF_) >> 13) / 10;
    Temperature = (float)TEMP - T2;
    if (P_base == 0)
    {
        P_base = Pressure;
    }
    P = Pressure - P_base;
    if (P < 0)
    {
        P = 0;
    }
    depth = P * 10 / 9.8f; //
    if (depth < 0.0f)
    {
        depth = 0.0f;
    }
}

uint32_t D1_Base, D2_Base;
int32_t  D1_err, D2_err;
/**
 * @function_name:MS5837_init
 * @brief:��ʼ��MS5873ѹ������
 * @param {type} 
 * @return:None
 */
u8 MS5837_init(void)
{
    u8  inth, intl;
    int i;
    for (i = 1; i <= 6; i++)
    {
        IIC_Start();
        IIC_Send_Byte(0xEC);
        IIC_Wait_Ack();
        IIC_Send_Byte(0xA0 + (i * 2));
        IIC_Wait_Ack();
        IIC_Stop();
        delay_sys_us(5);
        IIC_Start();
        IIC_Send_Byte(0xEC + 0x01); //�������ģʽ
        delay_sys_us(1);
        IIC_Wait_Ack();
        inth = IIC_Read_Byte(1); //��ACK�Ķ�����
        delay_sys_us(1);
        intl = IIC_Read_Byte(0); //���һ���ֽ�NACK
        IIC_Stop();
        Cal_C[i] = (((uint16_t)inth << 8) | intl);
    }
    D1_Pres = MS583703BA_getConversion(0x48); //��ȡ����ѹ
    D2_Temp = MS583703BA_getConversion(0x58); //��ȡ�¶�
    D1_Base = D1_Pres;
    D2_Base = D2_Temp;
    Change_YC();
    return Cal_C[0];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:unsigned long MS561101BA_getConversion(void)
*��������:    ��ȡ MS5837 ��ת����� 
*******************************************************************************/
unsigned long MS583703BA_getConversion(uint8_t command)
{

    unsigned long conversion = 0;
    u8            temp[3];

    IIC_Start();
    IIC_Send_Byte(0xEC); //д��ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(command); //дת������
    IIC_Wait_Ack();
    IIC_Stop();

    delay_ms(9);
    IIC_Start();
    IIC_Send_Byte(0xEC); //д��ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(0); // start read sequence
    IIC_Wait_Ack();
    IIC_Stop();

    IIC_Start();
    IIC_Send_Byte(0xEC + 0x01); //�������ģʽ
    IIC_Wait_Ack();
    temp[0] = IIC_Read_Byte(1); //��ACK�Ķ�����  bit 23-16
    temp[1] = IIC_Read_Byte(1); //��ACK�Ķ�����  bit 8-15
    temp[2] = IIC_Read_Byte(0); //��NACK�Ķ����� bit 0-7
    IIC_Stop();

    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;
}

void MS5837_Init(void)
{
    //IIC_Init();	                //��ʼ��IIC
    HAL_Delay(100);
    MS583703BA_RESET(); // Reset Device  ��λMS5837
    HAL_Delay(100);     //��λ����ʱ��ע�������ʱ��һ����Ҫ�ģ��������̵��ƺ���������20ms��
    MS5837_init();      //��ʼ��MS5837
}
u8   flag_mode = 0;
void Ms5837_DataFilter(uint32_t *D1, uint32_t *D2)
{
    static uint32_t last_D1 = 0;
    static uint32_t last_D2 = 0;
    if ((*D1 == 0xFFFFFF) || (*D2 == 0xFFFFFF))
    {
        *D1 = last_D1;
        *D2 = last_D2;
    }
    else if ((*D1 == 0) || (*D2 == 0))
    {
        *D1 = last_D1;
        *D2 = last_D2;
    }
    last_D1 = *D1;
    last_D2 = *D2;
}
/***
 * @function_name: MS5837_GetData
 * @brief:ͨ��IICЭ�飬��ѹ�������ѹ��ֵ
 * @param PressureMsg_t*pressure ѹ����Ϣ�ṹ��
 * @return:None
 */
void MS5837_GetData(PressureMsg_t *pressure)//���ѹ��ֵ
{
    static uint16_t TempGetCount = 0;
    if ((TempGetCount++) % 50 == 0)
    {
        D2_Temp = MS583703BA_getConversion(0x58);
    }

    D1_Pres = MS583703BA_getConversion(0x48);
    Change_YC();
    pressure->Temperature = TEMP / 100.f;
    pressure->Pressure    = Pressure;
    pressure->init_value  = P_base;
    pressure->depth       = depth;
}

PressureMsg_t test_pressure;
u32           startTime = 0;
u32           endTime   = 0;
u32           useTime   = 0;
void          MS8537_Test_Demo(void)
{
    MS5837_Init();

    while (1)
    {
        MS5837_GetData(&test_pressure);
        delay_ms(50);
    }
}
