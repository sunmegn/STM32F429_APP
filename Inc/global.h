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

#define MY_ID           2
#define CAN_IMU_ID 		3
#define HOST_ID         1
#define FREE_ID         255
#define DEVHEARTBEAT_ID 0xF001
#define SOFTVERSION_ID  0xF00F
#define DEBUGMODE       0x01 //调试模式
#define TESTMODE        0x02 //推进器中值标定模式
#define NOMALMODE       0x03 //正常模式
#define MANIPULATOR_MID 1550 //机械臂中值

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;
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
#define CONSTRAIN(x, max, min) ((x) > max ? max : ((x) < min ? min : (x)))

extern u8    g_nowWokeMode;  //当前工作模式  默认正常工作模式
extern u8    g_HeartBeatCnt; //心跳计数
extern u8    g_runMode;      //控制模式，00 手动模式  01 定航模式  02 定深模式  03 定深定航模式
extern u16   Mid_pwm[6];     //6个推进器中值
extern u8    g_LostPC;
extern u32   g_UpdateFlag;
extern u8    g_LEDWorkMode;
extern float g_depthspeed;

float SOTFOutput(SOTF_typeDef *gs, float data);
void  Rocker_SOTF_Clear(SOTF_typeDef *gs);

void IIR_2OrderLpf_Init(SOTF_typeDef *n, float fs, float fc);

void AccLimit_Init(AccLimit_t *lim, float AccMax, float ts);

float AccLim(AccLimit_t *lim, float Val);
#endif
