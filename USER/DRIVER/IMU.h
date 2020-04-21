#ifndef __IMU_H
#define __IMU_H
#include "includes.h"

#define IMU_RX_LEN 256
typedef struct
{
    uint8_t  RX_flag : 1;          //IDLE receive flag
    uint16_t RX_Size;              //receive length
    uint8_t  RX_pData[IMU_RX_LEN]; //DMA receive buffer
} IMU_UsartRec_t;
#define IMU_USART huart8

#define AngToRad(x) x *PI / 180
#define RadToAng(x) x * 180 / PI
void ImuTask(void *param);
void Imu_Init(void);
int  ImuReceivePkgFromISR(u8 data);
void Imu_SerialSend(u8 *code, int length);
void Imu_SerialInit(u32 baude);
int  CompareCode(u8 *code1, u8 *code2, int length1, int length2);
void IMU_DMAReStart(void);
typedef struct IMUEuler_TypeDef
{
    float roll;
    float pitch;
    float yaw;
    bool  valid;
} IMUEulerPkg;
typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float gyrox;
    float gyroy;
    float gyroz;
    float accx;
    float accy;
    float accz;
    float Speed;
    bool  valid;
} IMUMsg_t;
typedef struct
{
    float gyroX;
    float gyroY;
    float gyroZ;
} Data_GYRO_HandleType;
typedef struct
{
    float OffSetYaw;
} CheckYawAngle_TypeDef;
typedef struct IMUAcc_TypeDef
{
    float Accx;
    float Accy;
    float Accz;
} IMUAccPkg;

extern IMUAccPkg accRawData;

typedef struct IMUGyro_TypeDef
{
    float gyroX;
    float gyroY;
    float gyroZ;
} IMUGyroPkg;
void IMU_Init(void);                                    //imu dma配置 空闲中断打开
void IMU_GetData(IMUMsg_t *imu);                        //imu 数据读取函数
void HAL_UART8_Receive_IDLE(UART_HandleTypeDef *huart); //空闲接收中断
void IMU_Test_Demo(void);                               //imu 测试demo
void IMU_Getrpy(Data_GYRO_HandleType *imurpy);
#endif
