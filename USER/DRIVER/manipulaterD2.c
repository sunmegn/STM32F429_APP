/**
 * @author        :smake
 * @version       :v1.0.0
 * @Date          :2020-04-20 13:55:45
 * @LastEditors:smake
 * @LastEditTime:2020-04-20 20:37:50
 * @brief         :
 */
#include "manipulaterD2.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"

#define manipulater_D2_RS485_HEADER    0xA5
#define manipulater_D2_RS485_ACKHEADER 0xAC

uint8_t crc8(uint8_t *data, uint16_t length);
uint8_t RS485_sendBuf[32];
float RsCmd_position_Limit_P[2] = {-5.8, -0.2};//移动副
float RsCmd_position_Limit_R[2] = {-7.8, 9.8};//转动副

union control_data_def {
    float   float_val[8];
    uint8_t char_val[32]
};

/**
 * @function_name:manipulater_D2_RS485Init
 * @brief:两轴机械臂初始化，设置模式，限位
 * @param :None
 * @return:bool
 */
int manipulater_D2_RS485Init(void)
{
    //    float Position_limit_value[1]; //设定限位值
    //    Position_limit_value[0] = 0;
    //    Position_limit_value[1] = 0;
    //    manipulater_D2_send(0x01, 0x10, 0, Position_limit_value, sizeof(Position_limit_value));
    //    Position_limit_value[0] = 0;
    //    Position_limit_value[1] = 0;
    //    manipulater_D2_send(0x02, 0x10, 0, Position_limit_value, sizeof(Position_limit_value));
    // uint8_t manipulater_D2_setRunMode = 3; //设定运行模式为位置式
    // manipulater_D2_send(0x01, 0x01, 0, &manipulater_D2_setRunMode, sizeof(manipulater_D2_setRunMode));
    // manipulater_D2_send(0x02, 0x01, 0, &manipulater_D2_setRunMode, sizeof(manipulater_D2_setRunMode));
    return 0;
}
/**
 * @function_name:manipulater_D2_send
 * @brief:
 * @param :DID
 * @return:None
 */
int manipulater_D2_send(uint8_t DID, uint8_t obj, uint8_t ASK, uint8_t *send_data, uint8_t len)
{
    RS485_sendBuf[0] = 0xA5;
    RS485_sendBuf[1] = len + 5;
    RS485_sendBuf[2] = DID;
    obj              = obj | (ASK << 8); //判断是否需要ASK
    RS485_sendBuf[3] = obj;
    memcpy(&RS485_sendBuf[4], send_data, len);
    RS485_sendBuf[len + 4] = crc8(RS485_sendBuf, RS485_sendBuf[1]-1);

//	HAL_GPIO_WritePin(GPIOD, USART3_RE_Pin|POWER_KEY_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart3, RS485_sendBuf, RS485_sendBuf[1], 5);
//	HAL_GPIO_WritePin(GPIOD, USART3_RE_Pin|POWER_KEY_Pin, GPIO_PIN_RESET);

    return 1;
}

/**
 * @function_name:manipulater_D2_receive
 * @brief:
 * @param :manipulater_D2_receiveBuf
 * @return:None
 */
//int manipulater_D2_receive(manipulater_D2_receiveBuf_def *manipulater_D2_receiveBuf)
//{
//    if (manipulater_D2_receiveBuf[0] != manipulater_D2_RS485_HEADER) //数据帧头不对
//        return -2;
//    RsCmd_Len = manipulater_D2_receiveBuf[1];
//    RsCmd_Crc = manipulater_D2_receiveBuf[RsCmd_Len - 1];

//    if (0xf5 != RsCmd_Crc)                                               //0xF5,免校验,调试阶段用
//        if (crc8(manipulater_D2_receiveBuf, RsCmd_Len - 1) != RsCmd_Crc) //crc校验未通过
//            return -3;

//    RsCmd_device_ID = manipulater_D2_receiveBuf[2];

//    int Id_Index = 0;
//    if (Local_device_ID[0] == RsCmd_device_ID) //判断是否为本机设备ID，究竟是那个设备
//    {
//        Id_Index = 0;
//    }
//    else if (Local_device_ID[1] == RsCmd_device_ID)
//    {
//        Id_Index = 1;
//    }
//    else
//        return 2;
//}

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
