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
#define DEBUGMODE       0x01 //����ģʽ
#define TESTMODE        0x02 //�ƽ�����ֵ�궨ģʽ
#define NOMALMODE       0x03 //����ģʽ
#define MANIPULATOR_MID 1550 //��е����ֵ

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
    float ts;   //ʱ���� (s)
    float fa;   //�ٶ�v(m/s)
    float a;    //���ٶ�(v/s)
    float lastvel;
    float nowv;
} AccLimit_t;
#define CONSTRAIN(x, max, min) ((x) > max ? max : ((x) < min ? min : (x)))

extern u8    g_nowWokeMode;  //��ǰ����ģʽ  Ĭ����������ģʽ
extern u8    g_HeartBeatCnt; //��������
extern u8    g_runMode;      //����ģʽ��00 �ֶ�ģʽ  01 ����ģʽ  02 ����ģʽ  03 �����ģʽ
extern u16   Mid_pwm[6];     //6���ƽ�����ֵ
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
