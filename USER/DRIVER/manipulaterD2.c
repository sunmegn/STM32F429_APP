/**
 * @author        :smake
 * @version       :v1.0.0
 * @Date          :2020-04-20 13:55:45
 * @LastEditors   :smake
 * @LastEditTime  :2020-06-07 01:45:46
 * @brief         :两轴机械臂驱动，机械臂使用485通信，通过指令发送函数可以向机械臂发送相应指令，接收函数对机械臂返回值进行处理。
 */
#include "manipulaterD2.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "global.h"
#include "queue.h"
#include "mathTool.h"

#define manipulater_D2_RS485_HEADER    0xA5
#define manipulater_D2_RS485_ACKHEADER 0xAC
//#define testmanipulater;

manipulater_controlData_t motor_line   = {0}; //直线电机参数
manipulater_controlData_t motor_rotate = {0}; //旋转电机参数
uint8_t                   crc8(uint8_t *data, uint16_t length);
uint8_t                   manipulater_D2_sendBuf[32];
uint8_t                   manipulater_D2_receiveBuf[32]; //机械臂返回值
float                     RsCmd_position_Limit_P_Mid  = 0.f;
float                     RsCmd_position_Limit_R_Mid  = 0.f;
float                     manipulater_throtal_R_devid = 0.01f; //手柄值除以该值并加上中值,将手柄[-100,100]映射到位置限制极限值，在Init函数中进行初始化
float                     manipulater_throtal_D_devid = 0.15f; //与夹爪执行周期联合使用控制夹爪张合速度
uint8_t                   RsCmd_Len;
uint8_t                   RsCmd_Crc;
uint8_t                   RsCmd_device_ID    = 0xFF;
float                     manipulater_period = 3; //机械臂的控制周期为命令周期的倍数
int                       manipulater_times  = 0; //机械臂控制指令次数
extern int                ctrl_cmd_SonarFlag;
uint8_t                   RotOrDir_Flag = 0;     //交替发送旋转电机，夹爪电机指令标志符
static manipulaterQueue_t manipulaterQueue_Data; //接收上位机发送的机械臂控制指令
QueueHandle_t             ManipulaterQueue = NULL;
union {
    float   float_val[2];
    uint8_t char_val[8];
} manipulater_position;

/**
 * @function_name:manipulater_D2_RS485Init
 * @brief:两轴机械臂初始化，设置模式，限位
 * @param None
 * @return bool
 */
int manipulater_D2_RS485Init(void)
{
    //与声呐共用一个485接口，已经在声呐初始化函数中进行初始化
    // RS485_RE(0);
    // HAL_UART_Receive_DMA(&huart3, usart3_rxbuf.RX_pData, sizeof(usart3_rxbuf.RX_pData));
    // __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    // __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    do
    {
        ManipulaterQueue = xQueueCreate(1, sizeof(manipulaterQueue_t));
    } while (ManipulaterQueue == NULL);

    //小环在外
    motor_line.runMode          = 0x01;
    motor_line.set_position_min = 0; //设置直线电机转动正向极限位置，安全起见，设置值比极值略小-6,6.5
    motor_line.set_position_max = 0;
    motor_line.set_velocity_min = 0;
    motor_line.set_velocity_max = 0;
    motor_line.set_current_min  = 0;
    motor_line.set_current_max  = 0;

    motor_rotate.runMode          = 0x03;
    motor_rotate.set_position_min = 30;
    motor_rotate.set_position_max = 330;
    motor_rotate.set_velocity_min = 0;
    motor_rotate.set_velocity_max = 0;
    motor_rotate.set_current_min  = 0;
    motor_rotate.set_current_max  = 0;

    RsCmd_position_Limit_R_Mid  = (motor_rotate.set_position_min + motor_rotate.set_position_max) / 2;
    manipulater_throtal_R_devid = (motor_rotate.set_position_max - motor_rotate.set_position_min) / 200; //手柄值除以该值并加上中值,将手柄[-100,100]映射到位置限制极限值

    // manipulater_D2_send(MOTOR_LINE_DRIVER_ID, RS_CMD_MODE, NOACK, (uint8_t *)&motor_line.runMode, RS_CMD_MODE_LEN);
    // vTaskDelay(100);
    // manipulater_D2_send(MOTOR_ROTATE_DRIVER_ID, RS_CMD_MODE, NOACK, (uint8_t *)&motor_rotate.runMode, RS_CMD_MODE_LEN);

#ifdef manipulater_To_Mid
    manipulater_D2_send(MOTOR_ROTATE_DRIVER_ID, RS_CMD_POSITION_SET, NOACK, (uint8_t *)&RsCmd_position_Limit_R_Mid, 4); //	机械臂初始使夹爪处于水平方向
#endif
    return 0;
}

void ManipulaterTaskFunction(void const *argument)
{
    portTickType tick                     = xTaskGetTickCount();
    uint8_t      motor_rotate_mode_set_ok = 0;
    int          ManipulaterTask_runTimes = 0;

    while (1)
    {
        //设置模式，判断模式
        if ((ManipulaterTask_runTimes % 10 == 0) && (motor_rotate_mode_set_ok == 0))
        {
            manipulater_D2_send(MOTOR_ROTATE_DRIVER_ID, RS_CMD_MODE, ACK, (uint8_t *)&motor_rotate.runMode, RS_CMD_MODE_LEN);
            if (ManipulaterTask_runTimes >= 100)
            {
                motor_rotate_mode_set_ok = 1;
            }
        }

        //FIXME由于485接收部分不完善，此方法无法使用
        //        if (motor_rotate.runMode == motor_rotate.receive_runMode)
        //        {
        //            //运行模式设置成功，可以发送指令运行
        //            motor_rotate_mode_set_ok = 1;
        //        }
        //        else
        //        {
        //            manipulater_D2_send(MOTOR_ROTATE_DRIVER_ID, RS_CMD_MODE, ACK, (uint8_t *)&motor_rotate.runMode, RS_CMD_MODE_LEN);
        //        }
        // manipulater_D2_send(MOTOR_LINE_DRIVER_ID, RS_CMD_MODE, NOACK, (uint8_t *)&motor_line.runMode, RS_CMD_MODE_LEN);

        if (ManipulaterQueue)
        {
            xQueueReceive(ManipulaterQueue, &manipulaterQueue_Data, 1);
            // if (manipulaterQueue_Data.SonarBUTTON != 1) //声呐关闭，启动机械臂
            // {
            manipulater_position.float_val[0] = -(float)manipulaterQueue_Data.line_motor_position;   //直线电机
            manipulater_position.float_val[1] = -(float)manipulaterQueue_Data.rotate_motor_position; //旋转电机

            if (manipulater_times >= manipulater_period)
            {
                if (RotOrDir_Flag == 0) //直线电机
                {
                    motor_line.send_current = 0.14f * manipulater_position.float_val[0];
                    manipulater_D2_send(MOTOR_LINE_DRIVER_ID, RS_CMD_CURRENT_SET, NOACK, (uint8_t *)&motor_line.send_current, RS_CMD_CURRENT_SET_LEN);
                    RotOrDir_Flag = 1;
                }
                else if (RotOrDir_Flag == 1 && motor_rotate_mode_set_ok) //旋转电机
                {
                    motor_rotate.send_position = RsCmd_position_Limit_R_Mid + ((float)manipulater_position.float_val[1] * manipulater_throtal_R_devid);
                    motor_rotate.send_position = ConstrainFloat(motor_rotate.send_position, motor_rotate.set_position_min, motor_rotate.set_position_max);
                    manipulater_D2_send(MOTOR_ROTATE_DRIVER_ID, RS_CMD_POSITION_SET, NOACK, (uint8_t *)&motor_rotate.send_position, 4);
                    RotOrDir_Flag = 0;
                }
                manipulater_times = 0;
            }
            manipulater_times++;
            // }
        }
        vTaskDelayUntil(&tick, 20);
        ManipulaterTask_runTimes++;
    }
}

// +---------------------------------------------------------------------------+
// |         |  Header |  Package Length |   ID  |  Obj|ACK<<8 |  Data |  CRC  |
// |---------|---------|-----------------|-------|-------------|----  -|-------|
// |  Length |    1    |       1         |   1   |      1      | Len-5 |   1   |
// |---------|---------|-----------------|-------|-------------|-------|-------|
// |   DATA  |   0xA5  |     5-255       | 0-255 |   1-127     |       |  CRC  |
// +---------------------------------------------------------------------------+
/**
 * @function_name:manipulater_D2_send
 * @brief:
 * @param :DID
 * @return:None
 */
int manipulater_D2_send(uint8_t DID, uint8_t obj, uint8_t ASK, uint8_t *send_data, uint8_t len)
{
    manipulater_D2_sendBuf[0] = 0xA5;
    manipulater_D2_sendBuf[1] = len + 5;
    manipulater_D2_sendBuf[2] = DID;
    obj |= (ASK << 7); //判断是否需要ASK
    manipulater_D2_sendBuf[3] = obj;
    memcpy(&manipulater_D2_sendBuf[4], send_data, len);
    manipulater_D2_sendBuf[len + 4] = crc8(manipulater_D2_sendBuf, manipulater_D2_sendBuf[1] - 1);

    HAL_GPIO_WritePin(GPIOD, USART3_RE_Pin, GPIO_PIN_SET);
    taskENTER_CRITICAL();
    HAL_UART_Transmit(&huart3, manipulater_D2_sendBuf, manipulater_D2_sendBuf[1], 5);
    taskEXIT_CRITICAL();
    vTaskDelay(4);
    HAL_GPIO_WritePin(GPIOD, USART3_RE_Pin, GPIO_PIN_RESET);

    return 0;
}

/**
 * @function_name:manipulater_D2_receive
 * @brief:解析2轴机械臂返回值
 * @param :manipulater_D2_receiveBuf
 * @return:None
 */
int manipulater_D2_receive(uint8_t *manipulater_D2_receiveBuf)
{
    if (manipulater_D2_receiveBuf[0] != manipulater_D2_RS485_ACKHEADER) //数据帧头不对
        return -2;
    RsCmd_Len = manipulater_D2_receiveBuf[1];
    RsCmd_Crc = manipulater_D2_receiveBuf[RsCmd_Len - 1];
    //
    if (0xf5 != RsCmd_Crc)                                               //0xF5,免校验,调试阶段用
        if (crc8(manipulater_D2_receiveBuf, RsCmd_Len - 1) != RsCmd_Crc) //crc校验未通过
            return -3;
    RsCmd_device_ID = manipulater_D2_receiveBuf[2];
    if (RsCmd_device_ID == MOTOR_LINE_DRIVER_ID)
    {
        switch (manipulater_D2_receiveBuf[3]) //判断返回命令值
        {
        case 0x01:
            /* 控制模式返回值 */
            memcpy(&motor_line.receive_runMode, &manipulater_D2_receiveBuf[4], 1);
            break;
        case 0x2:
            /* 设定速度时，电机返回值，长度为12byte，三个float，分别为当前位置，当前速度，当前电流 */
            memcpy(&motor_line.motor_position, &manipulater_D2_receiveBuf[4], 12); //由于位置，速度，电流已经按顺序排列好，故可以一次性进行复制
            break;
        case 0x03:
            /* 设定位置时，电机返回值，长度为12byte，三个float，分别为当前位置，当前速度，当前电流 */
            memcpy(&motor_line.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x04:
            /* 设定电流时，电机返回值，长度为12byte，三个float，分别为当前位置，当前速度，当前电流 */
            memcpy(&motor_line.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x05:
            /* 位置，速度，电流值查询返回值，长度为12byte，三个float，分别为当前位置，当前速度，当前电流 */
            memcpy(&motor_line.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x06:
            /* 温度查询返回值，一个float，为当前温度，单位为摄氏度 */
            memcpy(&motor_line.motor_temperature, &manipulater_D2_receiveBuf[4], 4);
            break;
        case 0x10:
            /* 设定位置边界时，电机返回值，长度为8byte，两个float，分别为设置的位置最大值，位置最小值*/
            memcpy(&motor_line.receive_position_max, &manipulater_D2_receiveBuf[4], 8);
            break;
        case 0x11:
            /* 设定速度边界时，电机返回值，长度为8byte，两个float，分别为设置的速度最大值，速度最小值*/
            memcpy(&motor_line.receive_velocity_max, &manipulater_D2_receiveBuf[4], 8);
            break;
        case 0x12:
            /* 设定电流边界时，电机返回值，长度为8byte，两个float，分别为设置电流最大值，电流最小值*/
            memcpy(&motor_line.receive_current_max, &manipulater_D2_receiveBuf[4], 8);
            break;

        default:
            break;
        }
    }
    else if (RsCmd_device_ID == MOTOR_ROTATE_DRIVER_ID)
    {
        switch (manipulater_D2_receiveBuf[3]) //判断返回命令值
        {
        case 0x01:
            /* 控制模式返回值 */
            memcpy(&motor_rotate.receive_runMode, &manipulater_D2_receiveBuf[4], 1);
        case 0x2:
            /* 设定速度时，电机返回值，长度为12byte，三个float，分别为当前位置，当前速度，当前电流 */
            memcpy(&motor_rotate.motor_position, &manipulater_D2_receiveBuf[4], 12); //由于位置，速度，电流已经按顺序排列好，故可以一次性进行复制
            break;
        case 0x03:
            /* 设定位置时，电机返回值，长度为12byte，三个float，分别为当前位置，当前速度，当前电流 */
            memcpy(&motor_rotate.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x04:
            /* 设定电流时，电机返回值，长度为12byte，三个float，分别为当前位置，当前速度，当前电流 */
            memcpy(&motor_rotate.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x05:
            /* 位置，速度，电流值查询返回值，长度为12byte，三个float，分别为当前位置，当前速度，当前电流 */
            memcpy(&motor_rotate.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x06:
            /* 温度查询返回值，一个float，为当前温度，单位为摄氏度 */
            memcpy(&motor_rotate.motor_temperature, &manipulater_D2_receiveBuf[4], 4);
            break;
        case 0x10:
            /* 设定位置边界时，电机返回值，长度为8byte，两个float，分别为设置的位置最大值，位置最小值*/
            memcpy(&motor_rotate.receive_position_max, &manipulater_D2_receiveBuf[4], 8);
            break;
        case 0x11:
            /* 设定速度边界时，电机返回值，长度为8byte，两个float，分别为设置的速度最大值，速度最小值*/
            memcpy(&motor_rotate.receive_velocity_max, &manipulater_D2_receiveBuf[4], 8);
            break;
        case 0x12:
            /* 设定电流边界时，电机返回值，长度为8byte，两个float，分别为设置电流最大值，电流最小值*/
            memcpy(&motor_rotate.receive_current_max, &manipulater_D2_receiveBuf[4], 8);
            break;

        default:
            break;
        }
    }
    return 0;
}

/******************************************************************************
* Name:    CRC-8               x8+x2+x+1
* Poly:    0x07
* Init:    0x00
* Refin:   False
* Refout:  False
* Xorout:  0x00
* Note:
*****************************************************************************/
uint8_t crc8(uint8_t *data, uint16_t length)
{
    uint8_t i;
    uint8_t crc = 0; // Initial value
    while (length--)
    {
        crc ^= *data++; // crc ^= *data; data++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}
