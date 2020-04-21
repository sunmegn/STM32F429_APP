/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:47
 * @LastEditors:smake
 * @LastEditTime:2020-04-19 00:28:56
 * @brief         :
 */
#ifndef __CONTROLTASKS_H
#define __CONTROLTASKS_H
#include "includes.h"
#include "global.h"
#define ROCKERZero          20
#define MAX_YAWSPEED        30 //°/s
#define MAX_DEEPSPEED       20 //cm/s
#define YAWCODE2ANG(val)    val *MAX_YAWSPEED / 100.0
#define DEEPCODE2SPEED(val) val *MAX_DEEPSPEED / 100.0
#define SetZeroVal(x, val)  (x > val) ? (x) : ((x < -val) ? (x) : (0))
typedef struct
{
    float Tx;
    float Ty;
    float Tz;
    float Tyaw;
    float Tpith;
    float Troll;
} Tottle_Crotypedef;
typedef struct
{
    bool         OuterLoopEN; //true:使用外环
    SOTF_typeDef OIntf;       //外环输入
    SOTF_typeDef Ofbtf;       //外环反馈
    float        NVal;
    float        NKx;
    float        Oe; //外环偏差
    float        Oe1;
    float        OeI;
    float        OeImax;
    float        OP;
    float        OI;
    float        OD;
    float        Ooutput;
    float        OoutMax;
    float        OoutMin;
    SOTF_typeDef Oouttf; //外环输出
    SOTF_typeDef Ifbtf;  //内环反馈
    float        INVal;  //线性区间
    float        falnum; //0.5-1之间
    float        Ie;     //内环偏差
    float        Ie1;
    float        IeI;
    float        IeImax;
    float        IP;
    float        II;
    float        ID;
    float        ZVal;    //死区限定值，死区为【0，ZVal】
    float        Ioutput; //内环输出
    float        T;
} NLPID_Typedef;
typedef struct
{
    float amax;      //摇杆拨动加速度值
    float ts;        //时间间隔 (s)
    float fa;        //摇杆拨动速度v
    float a;         //加速度
    float lastvel;   //上次摇杆值
    float nowv;      //现在值
    float Rokerval;  //摇杆值（-100->0<-100）
    float ZVal;      //摇杆死区，[0,ZVal]为死区，对摇杆值不做响应
    float Rockermin; //摇杆最小值
    float Rockermax; //摇杆最大值
} RockerLimit_t;
extern QueueHandle_t Control_Message_Queue;
void                 NLPID_Init(void);
void                 ControlTask_Function(void const *argument);
float                YawErrorDispose(float now, float obj);
float                NLPID_Control(NLPID_Typedef *nl, float Onow, float Inow, float obj);
float                YAWPID_Control(NLPID_Typedef *nl, float Onow, float Inow, float obj);
void                 Rocker_SOTF_Init(SOTF_typeDef *gs);
float                NLfal(float x, float a, float val);
float                ProNLPID_Control(NLPID_Typedef *nl, float Onow, float Inow, float obj);
void                 NLPID_Clear(NLPID_Typedef *nl);
/**************************************************************/
void  RockerLimit_Init(RockerLimit_t *lim, float AccMax, float ts, float ZeroVal, float Rockermin, float Rockermax);
float RockerValDispose(RockerLimit_t *lim, float RockerVal, float RZeroAccKx, float RockerKx);
void  AllRocker_Init(float AccMax, float ts, float ZeroVal, float Rockermin, float Rockermax);
float RockerLimit(float RockerVal, float Zval, float min, float max);
#endif
