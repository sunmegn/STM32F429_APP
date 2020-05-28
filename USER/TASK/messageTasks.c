/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:47
 * @LastEditors:smake
 * @LastEditTime:2020-05-06 19:30:42
 * @brief         : 该任务负责与上位机进行数据传输，主要负责数据的上传
 */
#include "messageTasks.h"
#include "Kalman_filtering.h"
#include "MTLink.h"
#include "global.h"
#include "pca9685.h"
#include "pwm.h"
#include "stmflash.h"
#include "manipulaterD2.h"

AllInfoToPC_Msg_t RovInfo_msg = {0}; //用于上传至上位机的总信息

static IMUMsg_t          imu_data;       //惯导信息
static PowerMsg_t        power_data;     //电压电流信息
static PressureMsg_t     pressure_data;  //压力信息
static TempHumiMsg_t     temp_humi_data; //温度信息
extern CtrlPara_t        ctrlpara_data;  //控制参数结构体，包含定深定航，角速度，PID参数，PWM值
extern u8                motorcalflag;   //是否进行电机标定
extern uint16_t          para_pwm[6];    //电机标定PWM
extern Tottle_Crotypedef TottleCro;      //力，力矩信息
extern float             k_speed;        //卡尔曼滤波下潜速度
extern float             imu_yaw_real;   //罗盘角度真实值
u8                       safeflag          = 0;
MTLink_typedef           MTLink_UDP        = {0};  //单片机接收和上传上位机数据缓存结构体
QueueHandle_t            MTLinkUDPRxQueue  = NULL; //queue--
QueueHandle_t            DataManageQueue   = NULL; //queue--从上位机接收到的数据
QueueHandle_t            MTLinkUDPAskQueue = NULL; //queue--UDP应答

static mtlink_all_CMD_t myctrl_data; //上位机发送至单片机控制数据
ControlParam_t          ctrldata;    //控制数据
u32                     MessageTaskRunTimes = 0;
extern float            heading_ref, depth_ref, roll_ref;
extern QueueHandle_t    ManipulaterQueue;

extern manipulater_controlData_t motor_line;   //直线电机参数
extern manipulater_controlData_t motor_rotate; //旋转电机参数
extern union {
    float   float_val[2];
    uint8_t char_val[8];
} manipulater_position;

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
    portTickType tick     = xTaskGetTickCount();
    uint8_t      my_id[1] = {MY_ID};
    BaseType_t   err;
    static int   controlT = 0;
    MTLinkUDP_Init();
    MTLinkSetMY_ID(&MTLink_UDP, my_id, sizeof(my_id));
    int len = 100;
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

            xQueueSend(ManipulaterQueue, (uint8_t *)&myctrl_data.ARM2ASIS_linear, 5);
        }

        //FIXME
        // if (pressure_data.depth < 5) // 深度小于5cm
        // {
        //     g_runMode            = 0x01;
        //     ctrldata.FB_rocker   = ctrldata.FB_rocker * 0.5;
        //     ctrldata.LR_rocker   = ctrldata.LR_rocker * 0.5;
        //     ctrldata.UD_rocker   = ctrldata.UD_rocker * 0.5;
        //     ctrldata.TURN_rocker = ctrldata.TURN_rocker * 0.5;
        // }

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
        RovInfo_msg.Ctrlfeedback.mode = g_runMode;

        RovInfo_msg.Ctrlfeedback.FB_rocker   = ctrlpara_data.fb_rocker;
        RovInfo_msg.Ctrlfeedback.LR_rocker   = ctrlpara_data.lr_rocker;
        RovInfo_msg.Ctrlfeedback.TURN_rocker = ctrlpara_data.turn_rocker;
        RovInfo_msg.Ctrlfeedback.UD_rocker   = ctrlpara_data.ud_rocker;

        RovInfo_msg.Ctrlfeedback.yawref   = heading_ref;
        RovInfo_msg.Ctrlfeedback.rollref  = roll_ref;
        RovInfo_msg.Ctrlfeedback.pitchref = 0;
        RovInfo_msg.Ctrlfeedback.depthref = depth_ref;

        RovInfo_msg.Ctrlfeedback.circle = -imu_data.yaw / 360; //圈数
        imu_data.yaw                    = fmod(imu_data.yaw, 360.f);
        if (imu_data.yaw > 180)
        {
            imu_data.yaw -= 360.f;
        }
        else if (imu_data.yaw < -180)
        {
            imu_data.yaw += 360.f;
        }

        RovInfo_msg.Ctrlfeedback.angle_yaw   = -imu_data.yaw;
        RovInfo_msg.Ctrlfeedback.angle_roll  = imu_data.roll;
        RovInfo_msg.Ctrlfeedback.angle_pitch = imu_data.pitch;
        RovInfo_msg.Ctrlfeedback.depth       = pressure_data.depth;

        RovInfo_msg.Ctrlfeedback.Acc_x = imu_data.accx;
        RovInfo_msg.Ctrlfeedback.Acc_y = imu_data.accy;
        RovInfo_msg.Ctrlfeedback.Acc_z = imu_data.accz;

        //调试机械臂张合旋转角度时使用
        RovInfo_msg.Ctrlfeedback.gyro_yaw  = motor_line.send_position;
        RovInfo_msg.Ctrlfeedback.gyro_roll = motor_rotate.send_position;
        // RovInfo_msg.Ctrlfeedback.gyro_yaw   = imu_data.gyrox;
        // RovInfo_msg.Ctrlfeedback.gyro_roll  = imu_data.gyroy;
        RovInfo_msg.Ctrlfeedback.gyro_pitch = imu_data.gyroz;
        RovInfo_msg.Ctrlfeedback.depspeed   = k_speed;

        RovInfo_msg.Ctrlfeedback.Tx    = TottleCro.Tx;
        RovInfo_msg.Ctrlfeedback.Ty    = TottleCro.Ty;
        RovInfo_msg.Ctrlfeedback.Tz    = TottleCro.Tz;
        RovInfo_msg.Ctrlfeedback.Troll = TottleCro.Troll;
        RovInfo_msg.Ctrlfeedback.Tpith = TottleCro.Tpith;
        RovInfo_msg.Ctrlfeedback.Tyaw  = TottleCro.Tyaw;

        //Sensor
        RovInfo_msg.SensorData.SHT35_temp      = temp_humi_data.temp;
        RovInfo_msg.SensorData.SHT35_humi      = temp_humi_data.humi;
        RovInfo_msg.SensorData.HP303B_temp     = 0;
        RovInfo_msg.SensorData.HP303B_pressure = 0;
        RovInfo_msg.SensorData.Voltage         = power_data.voltage;
        RovInfo_msg.SensorData.Current         = power_data.current;
        RovInfo_msg.SensorData.Temp            = 0;
        RovInfo_msg.SensorData.TDS1            = 0;
        RovInfo_msg.SensorData.TDS2            = 0;
        RovInfo_msg.SensorData.HD7             = 0;
        RovInfo_msg.SensorData.MOS_Voltage     = 0;

        //IMU
        RovInfo_msg.IMUData.Angle_yaw   = -imu_data.yaw;
        RovInfo_msg.IMUData.Angle_pitch = imu_data.pitch;
        RovInfo_msg.IMUData.Angle_roll  = imu_data.roll;

        //Pressure
        RovInfo_msg.PressureData.Temp     = pressure_data.Temperature;
        RovInfo_msg.PressureData.Pressure = pressure_data.depth;
        RovInfo_msg.PressureData.Depth    = pressure_data.depth;
        RovInfo_msg.PressureData.Depspeed = k_speed;

        //机械臂
        RovInfo_msg.ArmData.getLinear = motor_line.send_position;
        RovInfo_msg.ArmData.getRotate = motor_rotate.send_position;
        //云台角度
        RovInfo_msg.HolderData.getpos = myctrl_data.ptz * 6 / 10;

        /* 上传 */
        MTLink_Encode(&MTLink_UDP, MY_ID, HOST_ID, 0 /*不需要应答*/, DEVHEARTBEAT_ID, (uint8_t *)&RovInfo_msg, sizeof(RovInfo_msg), 10);
        MessageTaskRunTimes++;
        /* 接收 */
        if (xQueueReceive(MTLinkUDPRxQueue, &len, 5) == pdTRUE)
        { //300ms超时

            Lasttick = htim7.Instance->CNT;
            MTLink_Decode(&MTLink_UDP);
            Nowtick = htim7.Instance->CNT - Lasttick;
        }

#endif
        vTaskDelayUntil(&tick, 20);
    }
}

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
