/**
 * @author        :smake
 * @version       :v1.0.0
 * @Date          :2020-04-20 13:55:45
 * @LastEditors   :smake
 * @LastEditTime  :2020-06-07 01:45:46
 * @brief         :�����е����������е��ʹ��485ͨ�ţ�ͨ��ָ��ͺ����������е�۷�����Ӧָ����պ����Ի�е�۷���ֵ���д���
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

manipulater_controlData_t motor_line   = {0}; //ֱ�ߵ������
manipulater_controlData_t motor_rotate = {0}; //��ת�������
uint8_t                   crc8(uint8_t *data, uint16_t length);
uint8_t                   manipulater_D2_sendBuf[32];
uint8_t                   manipulater_D2_receiveBuf[32]; //��е�۷���ֵ
float                     RsCmd_position_Limit_P_Mid  = 0.f;
float                     RsCmd_position_Limit_R_Mid  = 0.f;
float                     manipulater_throtal_R_devid = 0.01f; //�ֱ�ֵ���Ը�ֵ��������ֵ,���ֱ�[-100,100]ӳ�䵽λ�����Ƽ���ֵ����Init�����н��г�ʼ��
float                     manipulater_throtal_D_devid = 0.15f; //���צִ����������ʹ�ÿ��Ƽ�צ�ź��ٶ�
uint8_t                   RsCmd_Len;
uint8_t                   RsCmd_Crc;
uint8_t                   RsCmd_device_ID    = 0xFF;
float                     manipulater_period = 3; //��е�۵Ŀ�������Ϊ�������ڵı���
int                       manipulater_times  = 0; //��е�ۿ���ָ�����
extern int                ctrl_cmd_SonarFlag;
uint8_t                   RotOrDir_Flag = 0;     //���淢����ת�������צ���ָ���־��
static manipulaterQueue_t manipulaterQueue_Data; //������λ�����͵Ļ�е�ۿ���ָ��
QueueHandle_t             ManipulaterQueue = NULL;
union {
    float   float_val[2];
    uint8_t char_val[8];
} manipulater_position;

/**
 * @function_name:manipulater_D2_RS485Init
 * @brief:�����е�۳�ʼ��������ģʽ����λ
 * @param None
 * @return bool
 */
int manipulater_D2_RS485Init(void)
{
    //�����Ź���һ��485�ӿڣ��Ѿ������ų�ʼ�������н��г�ʼ��
    // RS485_RE(0);
    // HAL_UART_Receive_DMA(&huart3, usart3_rxbuf.RX_pData, sizeof(usart3_rxbuf.RX_pData));
    // __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    // __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    do
    {
        ManipulaterQueue = xQueueCreate(1, sizeof(manipulaterQueue_t));
    } while (ManipulaterQueue == NULL);

    //С������
    motor_line.runMode          = 0x01;
    motor_line.set_position_min = 0; //����ֱ�ߵ��ת��������λ�ã���ȫ���������ֵ�ȼ�ֵ��С-6,6.5
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
    manipulater_throtal_R_devid = (motor_rotate.set_position_max - motor_rotate.set_position_min) / 200; //�ֱ�ֵ���Ը�ֵ��������ֵ,���ֱ�[-100,100]ӳ�䵽λ�����Ƽ���ֵ

    // manipulater_D2_send(MOTOR_LINE_DRIVER_ID, RS_CMD_MODE, NOACK, (uint8_t *)&motor_line.runMode, RS_CMD_MODE_LEN);
    // vTaskDelay(100);
    // manipulater_D2_send(MOTOR_ROTATE_DRIVER_ID, RS_CMD_MODE, NOACK, (uint8_t *)&motor_rotate.runMode, RS_CMD_MODE_LEN);

#ifdef manipulater_To_Mid
    manipulater_D2_send(MOTOR_ROTATE_DRIVER_ID, RS_CMD_POSITION_SET, NOACK, (uint8_t *)&RsCmd_position_Limit_R_Mid, 4); //	��е�۳�ʼʹ��צ����ˮƽ����
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
        //����ģʽ���ж�ģʽ
        if ((ManipulaterTask_runTimes % 10 == 0) && (motor_rotate_mode_set_ok == 0))
        {
            manipulater_D2_send(MOTOR_ROTATE_DRIVER_ID, RS_CMD_MODE, ACK, (uint8_t *)&motor_rotate.runMode, RS_CMD_MODE_LEN);
            if (ManipulaterTask_runTimes >= 100)
            {
                motor_rotate_mode_set_ok = 1;
            }
        }

        //FIXME����485���ղ��ֲ����ƣ��˷����޷�ʹ��
        //        if (motor_rotate.runMode == motor_rotate.receive_runMode)
        //        {
        //            //����ģʽ���óɹ������Է���ָ������
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
            // if (manipulaterQueue_Data.SonarBUTTON != 1) //���Źرգ�������е��
            // {
            manipulater_position.float_val[0] = -(float)manipulaterQueue_Data.line_motor_position;   //ֱ�ߵ��
            manipulater_position.float_val[1] = -(float)manipulaterQueue_Data.rotate_motor_position; //��ת���

            if (manipulater_times >= manipulater_period)
            {
                if (RotOrDir_Flag == 0) //ֱ�ߵ��
                {
                    motor_line.send_current = 0.14f * manipulater_position.float_val[0];
                    manipulater_D2_send(MOTOR_LINE_DRIVER_ID, RS_CMD_CURRENT_SET, NOACK, (uint8_t *)&motor_line.send_current, RS_CMD_CURRENT_SET_LEN);
                    RotOrDir_Flag = 1;
                }
                else if (RotOrDir_Flag == 1 && motor_rotate_mode_set_ok) //��ת���
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
    obj |= (ASK << 7); //�ж��Ƿ���ҪASK
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
 * @brief:����2���е�۷���ֵ
 * @param :manipulater_D2_receiveBuf
 * @return:None
 */
int manipulater_D2_receive(uint8_t *manipulater_D2_receiveBuf)
{
    if (manipulater_D2_receiveBuf[0] != manipulater_D2_RS485_ACKHEADER) //����֡ͷ����
        return -2;
    RsCmd_Len = manipulater_D2_receiveBuf[1];
    RsCmd_Crc = manipulater_D2_receiveBuf[RsCmd_Len - 1];
    //
    if (0xf5 != RsCmd_Crc)                                               //0xF5,��У��,���Խ׶���
        if (crc8(manipulater_D2_receiveBuf, RsCmd_Len - 1) != RsCmd_Crc) //crcУ��δͨ��
            return -3;
    RsCmd_device_ID = manipulater_D2_receiveBuf[2];
    if (RsCmd_device_ID == MOTOR_LINE_DRIVER_ID)
    {
        switch (manipulater_D2_receiveBuf[3]) //�жϷ�������ֵ
        {
        case 0x01:
            /* ����ģʽ����ֵ */
            memcpy(&motor_line.receive_runMode, &manipulater_D2_receiveBuf[4], 1);
            break;
        case 0x2:
            /* �趨�ٶ�ʱ���������ֵ������Ϊ12byte������float���ֱ�Ϊ��ǰλ�ã���ǰ�ٶȣ���ǰ���� */
            memcpy(&motor_line.motor_position, &manipulater_D2_receiveBuf[4], 12); //����λ�ã��ٶȣ������Ѿ���˳�����кã��ʿ���һ���Խ��и���
            break;
        case 0x03:
            /* �趨λ��ʱ���������ֵ������Ϊ12byte������float���ֱ�Ϊ��ǰλ�ã���ǰ�ٶȣ���ǰ���� */
            memcpy(&motor_line.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x04:
            /* �趨����ʱ���������ֵ������Ϊ12byte������float���ֱ�Ϊ��ǰλ�ã���ǰ�ٶȣ���ǰ���� */
            memcpy(&motor_line.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x05:
            /* λ�ã��ٶȣ�����ֵ��ѯ����ֵ������Ϊ12byte������float���ֱ�Ϊ��ǰλ�ã���ǰ�ٶȣ���ǰ���� */
            memcpy(&motor_line.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x06:
            /* �¶Ȳ�ѯ����ֵ��һ��float��Ϊ��ǰ�¶ȣ���λΪ���϶� */
            memcpy(&motor_line.motor_temperature, &manipulater_D2_receiveBuf[4], 4);
            break;
        case 0x10:
            /* �趨λ�ñ߽�ʱ���������ֵ������Ϊ8byte������float���ֱ�Ϊ���õ�λ�����ֵ��λ����Сֵ*/
            memcpy(&motor_line.receive_position_max, &manipulater_D2_receiveBuf[4], 8);
            break;
        case 0x11:
            /* �趨�ٶȱ߽�ʱ���������ֵ������Ϊ8byte������float���ֱ�Ϊ���õ��ٶ����ֵ���ٶ���Сֵ*/
            memcpy(&motor_line.receive_velocity_max, &manipulater_D2_receiveBuf[4], 8);
            break;
        case 0x12:
            /* �趨�����߽�ʱ���������ֵ������Ϊ8byte������float���ֱ�Ϊ���õ������ֵ��������Сֵ*/
            memcpy(&motor_line.receive_current_max, &manipulater_D2_receiveBuf[4], 8);
            break;

        default:
            break;
        }
    }
    else if (RsCmd_device_ID == MOTOR_ROTATE_DRIVER_ID)
    {
        switch (manipulater_D2_receiveBuf[3]) //�жϷ�������ֵ
        {
        case 0x01:
            /* ����ģʽ����ֵ */
            memcpy(&motor_rotate.receive_runMode, &manipulater_D2_receiveBuf[4], 1);
        case 0x2:
            /* �趨�ٶ�ʱ���������ֵ������Ϊ12byte������float���ֱ�Ϊ��ǰλ�ã���ǰ�ٶȣ���ǰ���� */
            memcpy(&motor_rotate.motor_position, &manipulater_D2_receiveBuf[4], 12); //����λ�ã��ٶȣ������Ѿ���˳�����кã��ʿ���һ���Խ��и���
            break;
        case 0x03:
            /* �趨λ��ʱ���������ֵ������Ϊ12byte������float���ֱ�Ϊ��ǰλ�ã���ǰ�ٶȣ���ǰ���� */
            memcpy(&motor_rotate.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x04:
            /* �趨����ʱ���������ֵ������Ϊ12byte������float���ֱ�Ϊ��ǰλ�ã���ǰ�ٶȣ���ǰ���� */
            memcpy(&motor_rotate.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x05:
            /* λ�ã��ٶȣ�����ֵ��ѯ����ֵ������Ϊ12byte������float���ֱ�Ϊ��ǰλ�ã���ǰ�ٶȣ���ǰ���� */
            memcpy(&motor_rotate.motor_position, &manipulater_D2_receiveBuf[4], 12);
            break;
        case 0x06:
            /* �¶Ȳ�ѯ����ֵ��һ��float��Ϊ��ǰ�¶ȣ���λΪ���϶� */
            memcpy(&motor_rotate.motor_temperature, &manipulater_D2_receiveBuf[4], 4);
            break;
        case 0x10:
            /* �趨λ�ñ߽�ʱ���������ֵ������Ϊ8byte������float���ֱ�Ϊ���õ�λ�����ֵ��λ����Сֵ*/
            memcpy(&motor_rotate.receive_position_max, &manipulater_D2_receiveBuf[4], 8);
            break;
        case 0x11:
            /* �趨�ٶȱ߽�ʱ���������ֵ������Ϊ8byte������float���ֱ�Ϊ���õ��ٶ����ֵ���ٶ���Сֵ*/
            memcpy(&motor_rotate.receive_velocity_max, &manipulater_D2_receiveBuf[4], 8);
            break;
        case 0x12:
            /* �趨�����߽�ʱ���������ֵ������Ϊ8byte������float���ֱ�Ϊ���õ������ֵ��������Сֵ*/
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
