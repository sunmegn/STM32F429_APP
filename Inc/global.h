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
#define DEBUGMODE       0x01 //����ģʽ
#define TESTMODE        0x02 //�ƽ�����ֵ�궨ģʽ
#define NOMALMODE       0x03 //����ģʽ
#define MANIPULATOR_MID 1550 //��е����ֵ

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;

extern float g_ptz_pos; //��̨�����Ƕ�
extern u8    g_SW;
extern u8    g_CW;

extern u8 gc_initacc_BUTTON; //�豸initaccУ׼
extern u8 gc_compass_BUTTON; //����У׼
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
    float ts;   //ʱ���� (s)
    float fa;   //�ٶ�v(m/s)
    float a;    //���ٶ�(v/s)
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

//С����ǰ
typedef union {
    u32 num;        //  ��ǰλ������Ϊ0-524288   ����ʵ������ı���������
    u8  numbuff[4]; //
} u32andu8;
#define CONSTRAIN(x, max, min) ((x) > max ? max : ((x) < min ? min : (x)))

/*
*ϵͳ����
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
        float X_YAWcompensation; //����
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

extern u8    g_nowWokeMode;     //��ǰ����ģʽ  Ĭ����������ģʽ
extern u8    g_HeartBeatCnt;    //��������
extern u8    g_runMode;         //����ģʽ��00 �ֶ�ģʽ  01 ����ģʽ  02 ����ģʽ  03 �����ģʽ
extern u16   MotorPWMMidVal[6]; //6���ƽ�����ֵ
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
