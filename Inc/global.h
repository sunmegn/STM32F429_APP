/***
 * @author          : robosea
 * @version         : v1.0.0
 * @Date            : 2019-12-16 11:15:41
 * @LastEditors: Robosea
 * @LastEditTime: 2020-02-22 22:06:27
 * @FilePath        : \ROV_F429_APP-10-10\Inc\global.h
 * @brief           :
 */

#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#define M_PI 3.1415926

#define MY_ID      2
#define CAN_IMU_ID 3
#define HOST_ID    1
#define FREE_ID    255
#define SENSERNUM  10
#define TASKNUM    17
// #define DEVHEARTBEAT_ID 0xF001
#define SOFTVERSION_ID  0x2003
#define DEBUGMODE       0x01 //调试模式
#define TESTMODE        0x02 //推进器中值标定模式
#define NOMALMODE       0x03 //正常模式
#define MANIPULATOR_MID 1550 //机械臂中值

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;

extern float g_ptz_pos; //云台俯仰角度
extern u8    g_SW;
extern u8    g_CW;

extern u8 gc_initacc_BUTTON; //设备initacc校准
extern u8 gc_compass_BUTTON; //罗盘校准
extern u8 gc_initdep_BUTTON;
extern u8 gc_initzero_BUTTON;

extern uint8_t       g_TimeTestF;


enum
{
    Master = 0,
    NAVI   = 1,

    HUMITURE_ID = 0,
    GLASS_ID    = 1,
    VOLTAGE_ID  = 2,
    PRESSURE_ID = 3,
    Gyro_ID     = 4,
    COMPASS_ID  = 5,
    MPU6050_ID  = 6,
    HOLDER_ID   = 7,
    ARM_ID      = 8,
    NAVI_ID     = 9,

    PowerManageTask   = 0,
    HeatBeatTask      = 1,
    DataManageTask    = 2,
    MessageTask       = 3,
    SensorTask        = 4,
    NAVITask          = 5,
    ControlTask       = 6,
    MotorTask         = 7,
    HolderTask        = 8,
    ArmTask           = 9,
    NAVI_HeatBeatTask = 10,
    NAVI_PressureTask = 11,
    NAVI_GyroTask     = 12,
    NAVI_CompassTask  = 13,
    NAVI_MPU6050Task  = 14,
    NAVI_NAVITask     = 15,
    NAVI_MessageTask  = 16
};

#pragma pack(1)
typedef struct
{
    struct
    {
        uint8_t  state[2];
        uint16_t errcode[2];
    } Cabin;

    struct
    {
        uint8_t  state[SENSERNUM];
        uint16_t errcode[SENSERNUM];
        float    datafreq[SENSERNUM];
        float    packloss[SENSERNUM];
    } SenserAndPeripheral;

    struct
    {
        uint16_t taskruntime[TASKNUM];
    } TaskRunTime;
} SystemState_t;
#pragma pack()
#pragma pack(1)
typedef struct
{
    float yk;
    float yk_1;
    float yk_2;
    float uk;
    float uk_1;
    float uk_2;
    float num[3];
    float den[3];
} SOTF_typeDef;

typedef struct
{
    float amax; //AccMax(v/s)
    float ts;   //时间间隔 (s)
    float fa;   //速度v(m/s)
    float a;    //加速度(v/s)
    float lastvel;
    float nowv;
} AccLimit_t;

typedef struct
{
    int year;
    int month;
    int data;
    int hour;
    int minute;
    int second;
    int ms;
} DateTime_t;
#pragma pack()

typedef union {
    u8    ch[4];
    float val;
} floatandu8;

typedef union {
    u8  ch[2];
    u16 val;
} u16andu8;

//小端在前
typedef union {
    u32 num;        //  当前位置限制为0-524288   根据实际需求改变数据类型
    u8  numbuff[4]; //
} u32andu8;
#define CONSTRAIN(x, max, min) ((x) > max ? max : ((x) < min ? min : (x)))

/*
*系统参数
*/
#pragma pack(1)
typedef struct
{

    struct
    {
        uint8_t updataflag;
        uint8_t flagreserve[9];
    } updataflag;

    struct
    {
        uint8_t version[10];
    } versionstr;

    struct
    {
        uint8_t noNULLflag;
        uint8_t flagreserve[15];
    } EEPROMflag;

    struct
    {
        uint16_t motormidpwm1;
        uint16_t motormidpwm2;
        uint16_t motormidpwm3;
        uint16_t motormidpwm4;
        uint16_t motormidpwm5;
        uint16_t motormidpwm6;
        uint16_t motormidpwm7;
        uint16_t motormidpwm8;
    } motormidpwm;

    struct
    {
        float yawOutP;
        float yawOutI;
        float yawOutD;
        float yawInP;
        float yawInI;
        float yawInD;
    } yawPID;

    struct
    {
        float depOutP;
        float depOutI;
        float depOutD;
        float depInP;
        float depInI;
        float depInD;
    } depPID;

    struct
    {
        float rollOutP;
        float rollOutI;
        float rollOutD;
        float rollInP;
        float rollInI;
        float rollInD;
    } rollPID;

    struct
    {
        float X_YAWcompensation; //补偿
        float X_ROLLcompensation;
        float X_Zcompensation;
    } compensation;

    struct
    {
        float acc_init_x;
        float acc_init_y;
        float acc_init_z;
    } acc_init;

    struct
    {
        float initdepth;
    } depth_init;

    struct
    {
        uint32_t midholder;
        int16_t  maxang;
        int16_t  minang;
    } holder_mid;
} SystemParam_t;

extern u8    g_nowWokeMode;     //当前工作模式  默认正常工作模式
extern u8    g_HeartBeatCnt;    //心跳计数
extern u8    g_runMode;         //控制模式，00 手动模式  01 定航模式  02 定深模式  03 定深定航模式
extern u16   MotorPWMMidVal[6]; //6个推进器中值
extern u8    g_LostPC;
extern u32   g_UpdateFlag;
extern u8    g_LEDWorkMode;
extern float g_depthspeed;

float SOTFOutput(SOTF_typeDef *gs, float data);
void  Rocker_SOTF_Clear(SOTF_typeDef *gs);
void  IIR_2OrderLpf_Init(SOTF_typeDef *n, float fs, float fc);
void  AccLimit_Init(AccLimit_t *lim, float AccMax, float ts);
float AccLim(AccLimit_t *lim, float Val);

void MTLinkMsgPrintf(uint8_t SID, uint8_t DID, const char *format, int len);
#endif
