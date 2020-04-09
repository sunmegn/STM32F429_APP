/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:47
 * @LastEditors:Robosea
 * @LastEditTime:2020-03-24 20:04:34
 * @brief         : ������������λ���������ݴ��䣬��Ҫ�������ݵ��ϴ�
 */
#include "messageTasks.h"
#include "Kalman_filtering.h"
#include "MTLink.h"
#include "global.h"
#include "pca9685.h"
#include "pwm.h"
#include "stmflash.h"
AllInfoToPC_msg_t RovInfo_msg; //�����ϴ�����λ��������Ϣ

static IMUMsg_t      imu_data;       //�ߵ���Ϣ
static PowerMsg_t    power_data;     //��ѹ������Ϣ
static PressureMsg_t pressure_data;  //ѹ����Ϣ
static TempHumiMsg_t temp_humi_data; //�¶���Ϣ
extern CtrlPara_t    ctrlpara_data;  //���Ʋ����ṹ�壬��������������ٶȣ�PID������PWMֵ
extern u8            motorcalflag;   //�Ƿ���е���궨
extern uint16_t      para_pwm[6];    //����궨PWM
u8                   safeflag          = 0;
MTLink_typedef       MTLink_UDP        = {0};  //��Ƭ�����պ��ϴ���λ�����ݻ���ṹ��
QueueHandle_t        MTLinkUDPRxQueue  = NULL; //queue--
QueueHandle_t        DataManageQueue   = NULL; //queue--����λ�����յ�������
QueueHandle_t        MTLinkUDPAskQueue = NULL; //queue--UDPӦ��

static mtlink_all_CMD_t myctrl_data; //��λ����������Ƭ����������
ControlParam_t          ctrldata;    //��������

/**
 * @function_name: MTLinkUDP_Init
 * @brief: ��ʼ��MTLink����������
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

//  0 :  ��ǰ����
//  1 :  Ŀ�꺽��
//  2 :  ���PWM
//  3 :  ���PWM

//  0 :  ��ǰ���
//  1 :  Ŀ�����
//  2 :  ��ǰPWM  UL
uint32_t                 Nowtick, Lasttick = 0;
extern TIM_HandleTypeDef htim7;

/***
 * @function_name:  MessageTask_Function
 * @brief: ��Ϣ������
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
        xQueuePeek(TempHum_Message_Queue, &temp_humi_data, 1); //��ʪ�����ݽ���
        xQueuePeek(Battery_Message_Queue, &(power_data), 1);   //������ݽ���
        xQueuePeek(IMU_Message_Queue, &imu_data, 1);           //�ߵ����ݽ���
        xQueuePeek(Pressure_Message_Queue, &pressure_data, 1); //ѹ�����ݽ�
        if (DataManageQueue)
        {
            xQueuePeek(DataManageQueue, &myctrl_data, 1);

            /* ROV���Ʒ�����Ϣ */
            ctrldata.isRunMode   = myctrl_data.FlightMode;    //����ģʽ
            ctrldata.FB_rocker   = myctrl_data.FrontBack;     //ҡ�� - ǰ���˶�
            ctrldata.LR_rocker   = myctrl_data.LeftRightMove; //ҡ�� - ����ƽ��
            ctrldata.TURN_rocker = myctrl_data.LeftRightTurn; //ҡ�� - ����ת��
            ctrldata.UD_rocker   = myctrl_data.UpDown;        //ҡ�� - �����˶�
            ctrldata.yaw         = imu_data.yaw;              //�����
            ctrldata.pitch       = imu_data.pitch;            //������
            ctrldata.roll        = imu_data.roll;             //�����
            ctrldata.grayX       = imu_data.gyrox;            //X�� - ���ٶ�
            ctrldata.grayY       = imu_data.gyroy;            //Y�� - ���ٶ�
            ctrldata.grayZ       = imu_data.gyroz;            //Z�� - ���ٶ�
            ctrldata.depth       = pressure_data.depth;       //ѹ�� - ���
        }

        //FIXME
        if (pressure_data.depth < 5) // ���С��5cm
        {
            //g_runMode            = 0x01;
            ctrldata.FB_rocker   = ctrldata.FB_rocker * 0.5;
            ctrldata.LR_rocker   = ctrldata.LR_rocker * 0.5;
            ctrldata.UD_rocker   = ctrldata.UD_rocker * 0.5;
            ctrldata.TURN_rocker = ctrldata.TURN_rocker * 0.5;
        }

        if (controlT >= 2) //�������ڴ��ڵ���2
        {
            controlT = 0;
            if (Control_Message_Queue != NULL) //������������
            {
                err = xQueueSend(Control_Message_Queue, &ctrldata, 1); // �ֱ�����
                if (err == errQUEUE_FULL)
                {
                    ; //printf("����Key_Queue���������ݷ���ʧ��!\r\n");
                }
            }
        }
        else
        {
            controlT++;
        }

        /* �������ϴ� */
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
        RovInfo_msg.sensors_data.sysvoltage = power_data.voltage; //FIXME ����Ӧ���ϴ�ʪ��ֵ
        RovInfo_msg.sensors_data.adin9      = power_data.current;

        /* �����ϴ� */
        RovInfo_msg.ctrlmode            = g_runMode;
        RovInfo_msg.ctrlfeedback.yawref = ctrlpara_data.pid_head;

        RovInfo_msg.ctrlfeedback.depthref    = ctrlpara_data.pid_updown;
        RovInfo_msg.ctrlfeedback.FB_rocker   = ctrlpara_data.fb_rocker;
        RovInfo_msg.ctrlfeedback.LR_rocker   = ctrlpara_data.lr_rocker;
        RovInfo_msg.ctrlfeedback.TURN_rocker = ctrlpara_data.turn_rocker;
        RovInfo_msg.ctrlfeedback.UD_rocker   = ctrlpara_data.ud_rocker;
        RovInfo_msg.ctrlfeedback.rollref     = k_speed;

        /* ���PWM�����ϴ� */
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

        /* �ϴ� */
        MTLink_Encode(&MTLink_UDP, My_ID, HOST_ID, 0 /*����ҪӦ��*/, DEVHEARTBEAT_ID, (uint8_t *)&RovInfo_msg, sizeof(RovInfo_msg), 10);

        /* ���� */
        if (xQueueReceive(MTLinkUDPRxQueue, &len, 5) == pdTRUE)
        { //300ms��ʱ

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
 * @brief: ���ݱ�����Flash
 * @param {type}
 * @return: None
 */
// void SaveToFlash(void)
// {
// //    u8 error = 0;
//     STMFLASH_Erase_DATA(); //�������ݴ洢��
//     osDelay(100);
//     STMFLASH_Write(PARAMADDR, para_pwm, 6);
//     osDelay(100);
//     STMFLASH_Read(PARAMADDR, Mid_pwm, 6);

// if (Mid_pwm[0] < MINMIDPWMVAL || Mid_pwm[0] > MAXMIDPWMVAL) //�ж���ֵ�Ƿ��ں�������
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
 * @brief: ����Ask
 * @param flag
 * @return: None
 */
// void UserSendSaveACK(uint8_t flag)
// {
//     SendToPC(MSG_SAVECAL, (uint8_t *)&(flag), 1);
// }

/***
 * @function_name: UDP_ReceivedIRQ_Callback
 * @brief: UDP�յ����ݺ��жϻص�����������λ�������������ݴ洢��MTLink_UDP��
 * @param *buf:���ݵ�ַ
 * @param len:���ݳ���
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
