#ifndef __MESSAGETASKS_H
#define __MESSAGETASKS_H

#include "includes.h"

#define MINPWMVAL 1300
#define MAXPWMVAL 1700

#define MINMIDPWMVAL 1400
#define MAXMIDPWMVAL 1600

#define DEVHEARTBEAT_ID 0xF001
#define SOFTVERSION_ID  0xF00F
#define DEBUGMODE       0x01 //调试模式
#define TESTMODE        0x02 //推进器中值标定模式
#define NOMALMODE       0x03 //正常模式

void SendToPC(uint8_t id, uint8_t *data, uint8_t len);

void unpackCommanderData(uint8_t *buf); //控制指令
void unpackLedData(uint8_t *buf);       //探照灯
void unpackGimbalData(uint8_t *buf);    //云台
void unpackDebugData(uint8_t *buf);     //Debug PID
void MessageTask_Function(void const *argument);
void udp_msg(void);
//void udp_send(u8 *buff, u16 len);
void SaveToFlash(void);
void MessageSendLoop(void);
void UserSendSaveACK(uint8_t flag);

#pragma pack(1)
/*
* 调参结构体
*/
typedef struct
{
    float OutP;
    float OutI;
    float OutD;
    float InP;
    float InI;
    float InD;
} CloseLoopPID_t;

/*
* 控制指令结构体
*/
typedef struct
{
    int8_t  FlightMode;
    int16_t FrontBack;
    int16_t LeftRightMove;
    int16_t LeftRightTurn;
    int16_t UpDown;
    uint8_t throttle; //上位机设置油门档位
    int16_t setyaw;
    int32_t setdepth;
    int16_t setspeed;
    int16_t setheigh;
    int16_t light;          //灯
    int16_t ptz;            //云台
    int16_t manipulator[2]; //机械臂,两轴
} mtlink_all_CMD_t;

typedef union {
    struct
    {
        uint32_t err_leak : 1;
        uint32_t err_humi : 1;
        uint32_t err_temp : 1;
        uint32_t err_motor1 : 1;
        uint32_t err_motor2 : 1;
        uint32_t err_motor3 : 1;
        uint32_t err_motor4 : 1;
        uint32_t err_motor5 : 1;
        uint32_t err_motor6 : 1;
        uint32_t err_pressure : 1;
        uint32_t err_imu : 1;
        uint32_t err_reserve : 21;
    } b;
    uint32_t w;
} ErrorCodeParam_t;

/*
* 控制反馈结构体
*/
typedef struct
{
    short FB_rocker;
    short LR_rocker;
    short UD_rocker;
    short TURN_rocker;
    float yawref;
    float depthref;
    float rollref;
    short circle; //新添旋转圈数
} CtrlFeedBackParam_t;

typedef struct
{
    u16 motorpwm[6];
    u16 motorspeed[6];
} MotorParam_t;

typedef struct
{
    float val_1;      //设备1压力
    float dep_1;      //设备1深度
    float temp_1;     //设备1温度
    float depspeed_1; //下潜、上浮速度
    float val_2;      //设备2压力
    float dep_2;      //设备2深度
    float temp_2;     //设备2温度
    float depspeed_2; //下潜、上浮速度
    u8    valid_1;    //设备1数据有效标志
    u8    valid_2;    //设备2数据有效标志
} PressureParam_t;

typedef struct
{
    uint8_t sht35_valid; //数据 0为有效 1为无效
    float   temp;
    float   humi;
    uint8_t HP303B_valid;
    float   HP303B_temp;
    float   HP303B_pressure;
    float   adin8;    //凝露传感器
    float   adin9;    //电流
    float   adin13_1; //漏水检测1
    float   adin13_2; //漏水检测2

    uint8_t a;
    float   sysvoltage;

} SensorParam_t;

typedef struct
{
    float        roll;
    float        pitch;
    float        yaw;
    float        acc_x;
    float        acc_y;
    float        acc_z;
    float        gyro_x;
    float        gyro_y;
    float        gyro_z;
    unsigned int state;
    int          freq;
} IMUParam_t;

typedef struct
{
    float reserve[20];
} Reserve_t;

/*
* 上传状态信息结构体
*/
typedef struct
{
    uint8_t             ctrlmode;
    ErrorCodeParam_t    errorcode;
    SensorParam_t       sensors_data;
    PressureParam_t     pressure_data;
    IMUParam_t          imu_data;
    CtrlFeedBackParam_t ctrlfeedback;
    MotorParam_t        motor;
    Reserve_t           reve;
} AllInfoToPC_msg_t;

typedef struct
{
    u8    isRunMode;
    short FB_rocker;
    short LR_rocker;
    short UD_rocker;
    short TURN_rocker;
    float yaw;
    float pitch;
    float roll;
    float depth;
    float grayX;
    float grayY;
    float grayZ;
} ControlParam_t;

#pragma pack()

void UDP_ReceivedIRQ_Callback(uint8_t *buf, int len);

#endif
