/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:42
 * @LastEditors:Robosea
 * @LastEditTime:2020-02-22 22:35:41
 * @FilePath      :\ROV_F429_APP-10-10\USER\DRIVER\ms5837.c
 * @brief         :获得压传,水温值，包括初始化IIc协议，MS5837值转换为Pa
 */
#include "ms5837.h"

//气压计状态机
#define SCTemperature   0x01 //开始温度转换
#define CTemperatureing 0x02 //正在转换温度
#define SCPressure      0x03 //开始气压转换
#define SCPressureing   0x04 //正在转换气压

/*
C1  压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3	温度压力灵敏度系数 TCS
C4	温度系数的压力补偿 TCO
C5	参考温度 T|REF
C6 	温度系数的温度 TEMPSENS
*/
static uint16_t Cal_C[7]; //用于存放PROM中的6组数据1-6
/*
dT 实际和参考温度之间的差异
TEMP 实际温度	
*/
int32_t dT, TEMP;
/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/

uint32_t D1_Pres, D2_Temp = 20; // 数字压力值,数字温度值

int64_t OFF_, SENS, P, T2, OFF2, SENS2;

float depth;
float P_base = 0.0, Pressure = 0.0, Temperature = 0.0; //大气压
/*******************************************************************************
  * @函数名称	MS583730BA_RESET
  * @函数说明   复位MS5611
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void MS583703BA_RESET(void)
{
    IIC_Start();
    IIC_Send_Byte(0xEC); //CSB接地，主机地址：0XEE，否则 0X77
    IIC_Wait_Ack();
    IIC_Send_Byte(0x1E); //发送复位命令
    IIC_Wait_Ack();
    IIC_Stop();
}

/**
 * @function_name:Change_YC
 * @brief:压传原始值转化成实际压力值
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
 * @brief:初始化MS5873压传驱动
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
        IIC_Send_Byte(0xEC + 0x01); //进入接收模式
        delay_sys_us(1);
        IIC_Wait_Ack();
        inth = IIC_Read_Byte(1); //带ACK的读数据
        delay_sys_us(1);
        intl = IIC_Read_Byte(0); //最后一个字节NACK
        IIC_Stop();
        Cal_C[i] = (((uint16_t)inth << 8) | intl);
    }
    D1_Pres = MS583703BA_getConversion(0x48); //获取大气压
    D2_Temp = MS583703BA_getConversion(0x58); //获取温度
    D1_Base = D1_Pres;
    D2_Base = D2_Temp;
    Change_YC();
    return Cal_C[0];
}

/**************************实现函数********************************************
*函数原型:unsigned long MS561101BA_getConversion(void)
*功　　能:    读取 MS5837 的转换结果 
*******************************************************************************/
unsigned long MS583703BA_getConversion(uint8_t command)
{

    unsigned long conversion = 0;
    u8            temp[3];

    IIC_Start();
    IIC_Send_Byte(0xEC); //写地址
    IIC_Wait_Ack();
    IIC_Send_Byte(command); //写转换命令
    IIC_Wait_Ack();
    IIC_Stop();

    delay_ms(9);
    IIC_Start();
    IIC_Send_Byte(0xEC); //写地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0); // start read sequence
    IIC_Wait_Ack();
    IIC_Stop();

    IIC_Start();
    IIC_Send_Byte(0xEC + 0x01); //进入接收模式
    IIC_Wait_Ack();
    temp[0] = IIC_Read_Byte(1); //带ACK的读数据  bit 23-16
    temp[1] = IIC_Read_Byte(1); //带ACK的读数据  bit 8-15
    temp[2] = IIC_Read_Byte(0); //带NACK的读数据 bit 0-7
    IIC_Stop();

    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;
}

void MS5837_Init(void)
{
    //IIC_Init();	                //初始化IIC
    HAL_Delay(100);
    MS583703BA_RESET(); // Reset Device  复位MS5837
    HAL_Delay(100);     //复位后延时（注意这个延时是一定必要的，可以缩短但似乎不能少于20ms）
    MS5837_init();      //初始化MS5837
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
 * @brief:通过IIC协议，从压传处获得压传值
 * @param PressureMsg_t*pressure 压传信息结构体
 * @return:None
 */
void MS5837_GetData(PressureMsg_t *pressure)//获得压传值
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
