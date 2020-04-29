/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2020-02-18 14:18:09
 * @LastEditors:smake
 * @LastEditTime:2020-04-24 11:19:47
 * @FilePath      :\ROV_F429_APP-10-10\USER\CONTROL\control.h
 * @brief         :
 */
#ifndef __CONTROL_H
#define __CONTROL_H

#include "includes.h"
#define pi 3.1415926
typedef unsigned short int uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned char      uint8_t;
typedef enum
{
    MANUAL = 0,
    AUTO
} ControlState_t;

#pragma pack(1)
typedef struct
{
    short          channel[4];
    float          depth;
    float          heading;  //����
    float          pitching; //ǰ��ƫ��
    float          rolling;  //����ƫ��
    ControlState_t depth_state;
    ControlState_t heading_state;
    //	float voltage;
} ControlMsg_t;

typedef struct
{
    u8    depth_state;   //����״̬
    u8    heading_state; //����״̬
    float gyroX;         //���ٶ�
    float gyroY;
    float gyroZ;
    float pid_head;   //����
    float pid_updown; //����
    float headP;      //����PIDϵ��
    float headI;
    float headD;
    float depP; //����PIDϵ��
    float depI;
    float depD;
    short fb_rocker;
    short lr_rocker;
    short ud_rocker;
    short turn_rocker;
    u16   FR_val; //��ǰpwm
    u16   FL_val;
    u16   BR_val;
    u16   BL_val;
    u16   UR_val;
    u16   UL_val;
} CtrlPara_t;
#pragma pack()
/**
 * @brief	�ƽ������������С�궨��
 */
#define PROPFMax 62.07
#define PROPFMin -40.6

/**
  * @funNm	ROV_FValuetypedef
  * @brief  ��ʼ��ʹ�ã������������Ƶ���ֵ
  */
typedef struct
{
    float Fxmax;     //x�����ֵ
    float Fxmin;     //x����Сֵ
    float Fymax;     //y�����ֵ
    float Fymin;     //y����Сֵ
    float Fzmax;     //z�����ֵ
    float Fzmin;     //z����Сֵ
    float Fyawmax;   //yaw�����ֵ
    float Fyawmin;   //yaw����Сֵ
    float Fpitchmax; //pitch�����ֵ
    float Fpitchmin; //pitch����Сֵ
    float Frollmax;  //roll�����ֵ
    float Frollmin;  //roll����Сֵ
} ROV_FValuetypedef;
#define MAX_SPEED 500 //490
//#define MotorPWMMidVal       1490  //������ 6��
//#define DELTA_PWM     10
#define MAX_PWM     (MotorPWMMidVal + MAX_SPEED)
#define MIN_PWM     (MotorPWMMidVal - MAX_SPEED)
#define SPEED_COEFF 4.0 //4.9
//#define CONSTRAIN(x,max,min) (x>max?max:(x<min?min:x))
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)

extern int  DogBeatCount;
extern u8   DogHeardBeat(void);
extern void FeedDogHeardBeat(void);
void        CtrlMidPwmInit(void);
void        Cal6FreedomForceValue(float Tang, float Tmin, float Tmax);
void        ThrottleToForceControl(bool Closestate, float Tx, float Ty, float Tz, float Tyaw, float Tpitch, float Troll, float yaw, float pitch, float roll);
float       SetValf32(float x, float Lval, float Hval);
float       sign(float x);
#endif
