/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:47
 * @LastEditors:Robosea
 * @LastEditTime:2020-03-24 20:04:34
 * @brief         : 该任务负责与上位机进行数据传输，主要负责数据的上传
 */
#include "messageTasks.h"
#include "Kalman_filtering.h"
#include "MTLink.h"
#include "global.h"
#include "pca9685.h"
#include "pwm.h"
#include "stmflash.h"
AllInfoToPC_msg_t RovInfo_msg; //用于上传至上位机的总信息

static IMUMsg_t      imu_data;       //惯导信息
static PowerMsg_t    power_data;     //电压电流信息
static PressureMsg_t pressure_data;  //压力信息
static TempHumiMsg_t temp_humi_data; //温度信息
extern CtrlPara_t    ctrlpara_data;  //控制参数结构体，包含定深定航，角速度，PID参数，PWM值
extern u8            motorcalflag;   //是否进行电机标定
extern uint16_t      para_pwm[6];    //电机标定PWM
u8                   safeflag          = 0;
MTLink_typedef       MTLink_UDP        = {0};  //单片机接收和上传上位机数据缓存结构体
QueueHandle_t        MTLinkUDPRxQueue  = NULL; //queue--
QueueHandle_t        DataManageQueue   = NULL; //queue--从上位机接收到的数据
QueueHandle_t        MTLinkUDPAskQueue = NULL; //queue--UDP应答

static mtlink_all_CMD_t myctrl_data; //上位机发送至单片机控制数据
ControlParam_t          ctrldata;    //控制数据

/**
 * @function_name: MTLinkUDP_Init
 * @brief: 初始化MTLink，创建队列
 * @param None
 * @return: None
 */
void MTLinkUDP_Init(void)
{
    do
    {
        MTLinkUDPRxQueue = xQueueCreate(10, sizeof(int));
    } while (MTLinkUDPRxQueue == NULL);
    do
    {
        MTLinkUDPAskQueue = xQueueCreate(10, sizeof(int));
    } while (MTLinkUDPAskQueue == NULL);
    do
    {
        DataManageQueue = xQueueCreate(1, sizeof(mtlink_all_CMD_t));
    } while (DataManageQueue == NULL);
}

//  0 :  当前航向
//  1 :  目标航向
//  2 :  输出PWM
//  3 :  输出PWM

//  0 :  当前深度
//  1 :  目标深度
//  2 :  当前PWM  UL
uint32_t                 Nowtick, Lasttick = 0;
extern TIM_HandleTypeDef htim7;

/***
 * @function_name:  MessageTask_Function
 * @brief: 消息任务函数
 * @param *argument
 * @return: None
 */
void MessageTask_Function(void const *argument)
{
    portTickType tick = xTaskGetTickCount();
    BaseType_t   err;
    static int   controlT = 0;
    MTLinkUDP_Init();
    int len;
    while (1)
    {
        run_time[1] = getRunTime(1);
#ifdef DEBUG

#else
        do_udp();
        xQueuePeek(TempHum_Message_Queue, &temp_humi_data, 1); //温湿度数据接收
        xQueuePeek(Battery_Message_Queue, &(power_data), 1);   //电池数据接收
        xQueuePeek(IMU_Message_Queue, &imu_data, 1);           //惯导数据接收
        xQueuePeek(Pressure_Message_Queue, &pressure_data, 1); //压传数据接
        if (DataManageQueue)
        {
            xQueuePeek(DataManageQueue, &myctrl_data, 1);

            /* ROV控制反馈消息 */
            ctrldata.isRunMode   = myctrl_data.FlightMode;    //控制模式
            ctrldata.FB_rocker   = myctrl_data.FrontBack;     //摇杆 - 前后运动
            ctrldata.LR_rocker   = myctrl_data.LeftRightMove; //摇杆 - 左右平移
            ctrldata.TURN_rocker = myctrl_data.LeftRightTurn; //摇杆 - 左右转向
            ctrldata.UD_rocker   = myctrl_data.UpDown;        //摇杆 - 上下运动
            ctrldata.yaw         = imu_data.yaw;              //航向角
            ctrldata.pitch       = imu_data.pitch;            //俯仰角
            ctrldata.roll        = imu_data.roll;             //横滚角
            ctrldata.grayX       = imu_data.gyrox;            //X轴 - 角速度
            ctrldata.grayY       = imu_data.gyroy;            //Y轴 - 角速度
            ctrldata.grayZ       = imu_data.gyroz;            //Z轴 - 角速度
            ctrldata.depth       = pressure_data.depth;       //压传 - 深度
        }

        //FIXME
        if (pressure_data.depth < 5) // 深度小于5cm
        {
            //g_runMode            = 0x01;
            ctrldata.FB_rocker   = ctrldata.FB_rocker * 0.5;
            ctrldata.LR_rocker   = ctrldata.LR_rocker * 0.5;
            ctrldata.UD_rocker   = ctrldata.UD_rocker * 0.5;
            ctrldata.TURN_rocker = ctrldata.TURN_rocker * 0.5;
        }

        if (controlT >= 2) //控制周期大于等于2
        {
            controlT = 0;
            if (Control_Message_Queue != NULL) //发给控制任务
            {
                err = xQueueSend(Control_Message_Queue, &ctrldata, 1); // 手柄控制
                if (err == errQUEUE_FULL)
                {
                    ; //printf("队列Key_Queue已满，数据发送失败!\r\n");
                }
            }
        }
        else
        {
            controlT++;
        }

        /* 传感器上传 */
        RovInfo_msg.imu_data.pitch          = imu_data.pitch;
        RovInfo_msg.imu_data.roll           = imu_data.roll;
        RovInfo_msg.imu_data.yaw            = -imu_data.yaw;
        RovInfo_msg.imu_data.gyro_x         = imu_data.gyrox;
        RovInfo_msg.imu_data.gyro_y         = imu_data.gyroy;
        RovInfo_msg.imu_data.gyro_z         = imu_data.gyroz;
        RovInfo_msg.imu_data.acc_x          = imu_data.accx;
        RovInfo_msg.imu_data.acc_y          = imu_data.accy;
        RovInfo_msg.imu_data.acc_z          = imu_data.accz;
        RovInfo_msg.imu_data.freq           = imu_data.Speed;
        RovInfo_msg.imu_data.state          = imu_data.valid;
        RovInfo_msg.pressure_data.dep_1     = pressure_data.depth;
        RovInfo_msg.pressure_data.temp_1    = pressure_data.Temperature;
        RovInfo_msg.sensors_data.temp       = temp_humi_data.temp;
        RovInfo_msg.sensors_data.humi       = temp_humi_data.humi;
        RovInfo_msg.sensors_data.sysvoltage = power_data.voltage; //FIXME 这里应该上传湿度值
        RovInfo_msg.sensors_data.adin9      = power_data.current;

        /* 反馈上传 */
        RovInfo_msg.ctrlmode            = g_runMode;
        RovInfo_msg.ctrlfeedback.yawref = ctrlpara_data.pid_head;

        RovInfo_msg.ctrlfeedback.depthref    = ctrlpara_data.pid_updown;
        RovInfo_msg.ctrlfeedback.FB_rocker   = ctrlpara_data.fb_rocker;
        RovInfo_msg.ctrlfeedback.LR_rocker   = ctrlpara_data.lr_rocker;
        RovInfo_msg.ctrlfeedback.TURN_rocker = ctrlpara_data.turn_rocker;
        RovInfo_msg.ctrlfeedback.UD_rocker   = ctrlpara_data.ud_rocker;
        RovInfo_msg.ctrlfeedback.rollref     = k_speed;

        /* 电机PWM反馈上传 */
        RovInfo_msg.motor.motorpwm[0] = ctrlpara_data.FL_val;
        RovInfo_msg.motor.motorpwm[1] = ctrlpara_data.FR_val;
        RovInfo_msg.motor.motorpwm[2] = ctrlpara_data.BL_val;
        RovInfo_msg.motor.motorpwm[3] = ctrlpara_data.BR_val;
        RovInfo_msg.motor.motorpwm[4] = ctrlpara_data.UL_val;
        RovInfo_msg.motor.motorpwm[5] = ctrlpara_data.UR_val;

        RovInfo_msg.motor.motorspeed[0] = (u16)run_time[0];
        RovInfo_msg.motor.motorspeed[1] = (u16)run_time[1];
        RovInfo_msg.motor.motorspeed[2] = (u16)run_time[2];
        RovInfo_msg.motor.motorspeed[3] = (u16)run_time[3];
        RovInfo_msg.motor.motorspeed[4] = (u16)run_time[4];
        RovInfo_msg.motor.motorspeed[5] = (u16)run_time[5];

        /* 上传 */
        MTLink_Encode(&MTLink_UDP, My_ID, HOST_ID, 0 /*不需要应答*/, DEVHEARTBEAT_ID, (uint8_t *)&RovInfo_msg, sizeof(RovInfo_msg), 10);

        /* 接收 */
        if (xQueueReceive(MTLinkUDPRxQueue, &len, 5) == pdTRUE)
        { //300ms超时

            //Lasttick = htim7.Instance->CNT;
            MTLink_Decode(&MTLink_UDP);
            //Nowtick = htim7.Instance->CNT - Lasttick;
        }

#endif
        vTaskDelayUntil(&tick, 20);
    }
}

/***
 * @function_name: SaveToFlash
 * @brief: 数据保存至Flash
 * @param {type}
 * @return: None
 */
// void SaveToFlash(void)
// {
// //    u8 error = 0;
//     STMFLASH_Erase_DATA(); //擦除数据存储区
//     osDelay(100);
//     STMFLASH_Write(PARAMADDR, para_pwm, 6);
//     osDelay(100);
//     STMFLASH_Read(PARAMADDR, Mid_pwm, 6);

// if (Mid_pwm[0] < MINMIDPWMVAL || Mid_pwm[0] > MAXMIDPWMVAL) //判断中值是否在合理区间
// {
//     Mid_pwm[0] = 1500;
//     error      = 1;
// }
// if (Mid_pwm[1] < MINMIDPWMVAL || Mid_pwm[1] > MAXMIDPWMVAL)
// {
//     Mid_pwm[1] = 1500;
//     error      = 1;
// }
// if (Mid_pwm[2] < MINMIDPWMVAL || Mid_pwm[2] > MAXMIDPWMVAL)
// {
//     Mid_pwm[2] = 1500;
//     error      = 1;
// }
// if (Mid_pwm[3] < MINMIDPWMVAL || Mid_pwm[3] > MAXMIDPWMVAL)
// {
//     Mid_pwm[3] = 1500;
//     error      = 1;
// }
// if (Mid_pwm[4] < MINMIDPWMVAL || Mid_pwm[4] > MAXMIDPWMVAL)
// {
//     Mid_pwm[4] = 1500;
//     error      = 1;
// }
// if (Mid_pwm[5] < MINMIDPWMVAL || Mid_pwm[5] > MAXMIDPWMVAL)
// {
//     Mid_pwm[5] = 1500;
//     error      = 1;
// }

// if (error == 1)
// {
//     UserSendSaveACK(0x00);
// }
// else if (error == 0)
// {
//     UserSendSaveACK(0x01);
// }
//}
/***
 * @function_name: UserSendSaveACK
 * @brief: 发送Ask
 * @param flag
 * @return: None
 */
// void UserSendSaveACK(uint8_t flag)
// {
//     SendToPC(MSG_SAVECAL, (uint8_t *)&(flag), 1);
// }

/***
 * @function_name: UDP_ReceivedIRQ_Callback
 * @brief: UDP收到数据后中断回调函数，将上位机发送来的数据存储至MTLink_UDP中
 * @param *buf:数据地址
 * @param len:数据长度
 * @return: None
 */
void UDP_ReceivedIRQ_Callback(uint8_t *buf, int len)
{
    int           nowlen                   = len;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if (MTLinkUDPRxQueue != NULL)
    {
        MTLinkFIFO_append(&MTLink_UDP, buf, len);
        xQueueSendToBackFromISR(MTLinkUDPRxQueue, &nowlen, &xHigherPriorityTaskWoken);
    }
}
