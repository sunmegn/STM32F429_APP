#ifndef __MESSAGETASKS_H
#define __MESSAGETASKS_H

#include "includes.h"

#define MINPWMVAL 1300
#define MAXPWMVAL 1700

#define MINMIDPWMVAL 1400
#define MAXMIDPWMVAL 1600

#define DEVHEARTBEAT_ID 0xF001
#define DEBUGMODE       0x01 //����ģʽ
#define TESTMODE        0x02 //�ƽ�����ֵ�궨ģʽ
#define NOMALMODE       0x03 //����ģʽ

void SendToPC(uint8_t id, uint8_t *data, uint8_t len);

void unpackCommanderData(uint8_t *buf); //����ָ��
void unpackLedData(uint8_t *buf);       //̽�յ�
void unpackGimbalData(uint8_t *buf);    //��̨
void unpackDebugData(uint8_t *buf);     //Debug PID
void MessageTask_Function(void const *argument);
void udp_msg(void);
//void udp_send(u8 *buff, u16 len);
void SaveToFlash(void);
void MessageSendLoop(void);
void UserSendSaveACK(uint8_t flag);

#pragma pack(1)
/*
* ���νṹ��
*/
typedef struct
{
    u8    saveflag;    //�����־��1д�룬2���ģ�3��ȡ
    u8    outloopflag; //���⻷���Ʊ�־
    float OutP;
    float OutI;
    float OutD;
    float InP;
    float InI;
    float InD;
} CloseLoopPID_t;

typedef struct
{
    CloseLoopPID_t YAWPID;
    CloseLoopPID_t DeepPID;
    CloseLoopPID_t RollPID;
} AllPIDArg_typedef;

/*
* ����ָ��ṹ��
*/
typedef struct
{
    int8_t  FlightMode;
    int16_t FrontBack;
    int16_t LeftRightMove;
    int16_t LeftRightTurn;
    int16_t UpDown;
    uint8_t throttle; //��λ���������ŵ�λ
    int16_t setyaw;
    int32_t setdepth;
    int16_t setspeed;
    int16_t setheigh;
    int16_t light;          //��
    int16_t ptz;            //��̨
    int16_t manipulator[2]; //��е��,����
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
* ���Ʒ����ṹ��
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
    short circle; //������תȦ��
} CtrlFeedBackParam_t;

typedef struct
{
    u16 motorpwm[6];
    u16 motorspeed[6];
} MotorParam_t;

typedef struct
{
    float val_1;      //�豸1ѹ��
    float dep_1;      //�豸1���
    float temp_1;     //�豸1�¶�
    float depspeed_1; //��Ǳ���ϸ��ٶ�
    float val_2;      //�豸2ѹ��
    float dep_2;      //�豸2���
    float temp_2;     //�豸2�¶�
    float depspeed_2; //��Ǳ���ϸ��ٶ�
    u8    valid_1;    //�豸1������Ч��־
    u8    valid_2;    //�豸2������Ч��־
} PressureParam_t;

typedef struct
{
    uint8_t sht35_valid; //���� 0Ϊ��Ч 1Ϊ��Ч
    float   temp;
    float   humi;
    uint8_t HP303B_valid;
    float   HP303B_temp;
    float   HP303B_pressure;
    float   adin8;    //��¶������
    float   adin9;    //����
    float   adin13_1; //©ˮ���1
    float   adin13_2; //©ˮ���2

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
* �ϴ�״̬��Ϣ�ṹ��
*/
// typedef struct
// {
//     uint8_t             ctrlmode;
//     ErrorCodeParam_t    errorcode;
//     SensorParam_t       sensors_data;
//     PressureParam_t     pressure_data;
//     IMUParam_t          imu_data;
//     CtrlFeedBackParam_t ctrlfeedback;
//     MotorParam_t        motor;
//     Reserve_t           reve;
// } AllInfoToPC_msg_t;
/*
* �ϴ�״̬��Ϣ�ṹ��
*/
typedef struct
{
    uint8_t InitState;
    struct
    {
        //�˲��� ʮ������ ��ʾ
        uint16_t SHT35_code;
        uint16_t HB303_code;
        uint16_t Voltage_code;

        uint16_t Pressure_code;
        uint16_t Gyro_code;
        uint16_t Compass_code;
        uint16_t MPU6050_code;

        uint16_t PTZ_SW;
        uint16_t PTZ_CW;

        uint16_t Motor_code;
    } ErrorCode; //������  С���ڴ�ӡ��ʾ���ͼ

    struct
    {
        float SHT35_temp;      //�����¶ȣ�����
        float SHT35_humi;      //����ʪ��
        float HP303B_temp;     //�����¶ȣ�����
        float HP303B_pressure; //������ѹ

        float Current;     //����
        float Temp;        //��Դ���¶�
        float TDS1;        //©ˮ����1
        float TDS2;        //©ˮ����2
        float HD7;         //��¶ֵ
        float Voltage;     //��ѹ
        float MOS_Voltage; //MOS�˵�ѹ
    } SensorData;

    struct
    {
        float Pressure; //ˮ��ѹ��
        float Depth;    //���
        float Temp;     //ˮ��
        float Depspeed; //��Ǳ�ٶ�
    } PressureData;

    struct
    {
        float Angle_roll;  //����Ƕ�
        float Angle_pitch; //�����Ƕ�
        float Angle_yaw;   //����Ƕ�
    } IMUData;

    struct
    {
        uint8_t mode; //�˶�ģʽ

        short FB_rocker;   //ǰ������ֵ   ��ͼ
        short LR_rocker;   //��������ֵ   ��ͼ
        short UD_rocker;   //��Ǳ����ֵ   ��ͼ
        short TURN_rocker; //��ת����ֵ ��ͼ

        float yawref;   //���� �����趨ֵ ��ͼ
        float rollref;  //����趨�Ƕ�
        float pitchref; //
        float depthref; //���� ����趨ֵ ��ͼ

        float angle_yaw;  //��ǰ����   ��ͼ�ô���ֵ
        float angle_roll; //��ǰ�����
        float angle_pitch;
        float depth; //��ǰ��� ��ͼ�ô���ֵ

        float Acc_x; //X����ٶ�
        float Acc_y; //Y����ٶ�
        float Acc_z; //Z����ٶ�

        float gyro_yaw;   //��ǰ������ٶ�
        float gyro_roll;  //��ǰ������ٶ�
        float gyro_pitch; //
        float depspeed;   //��ǰ��Ǳ�ٶ�

        short circle; //�豸��תȦ��

        float Tx;
        float Ty;
        float Tz;

        float Tyaw;
        float Tpith;
        float Troll;
    } Ctrlfeedback;

    struct
    {
        u16 motorpwm[8];   //���pwmֵ
        u16 motorspeed[8]; //���ת��
    } MotorData;

    struct
    {
        float   getpos; //��̨��ǰ�Ƕ�
        uint8_t SW;     //��̨ ��ǰ״̬ ��ʮ��������ʾ��
    } HolderData;

    struct
    {
        int16_t getLinear;  //�����е�� צ�ӵ�ǰֵ
        int16_t getRotate;  //�����е�� ��ת��ǰֵ
        int16_t Arm_res[2]; //Ԥ������
    } ArmData;

    struct
    {
        float reserve[20]; //Ԥ��20��ͼ����  �ɻ�ͼ
    } Reserve;

    struct
    {
        uint16_t Taskruntime[17];
        uint8_t  SenserState[10];
        float    SenserFreq[10];
        float    SenserPackNum[10];
    } TaskAndSenser;

    struct
    {
        u8 MOSState;
        u8 DeckState;
        u8 CameraState;
        u8 MotorState;
        u8 SonarState;
    } PowerState;
} AllInfoToPC_Msg_t;

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
