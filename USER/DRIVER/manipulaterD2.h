/**
 * @author        :smake
 * @version       :v1.0.0
 * @Date          :2020-04-20 11:21:14
 * @LastEditors:smake
 * @LastEditTime:2020-04-20 21:04:33
 * @brief         :
 */

#ifndef __maiupulaterD2_H_
#define __maiupulaterD2_H_
#include "types.h"

struct manipulater_D2_receiveBuf_def
{
    uint8_t len;
    uint8_t DID;
    float   RsAck_velocity[2];
    float   RsAck_position[2];
    float   RsAck_current[2];
    float   RsAck_temperature[2];
    float   RsAck_voltage[2];
    uint8_t crc8_val;
} ;

// #define manipulater_D2_setRunMode       0x01;
// #define manipulater_D2_setSpeed         0x02;
// #define manipulater_D2_setPosition      0x03;
// #define manipulater_D2_setCurrent       0x04;
// #define manipulater_D2_LSCnow           0x05;
// #define manipulater_D2_temperature      0x06;
// #define manipulater_D2_setPositionLimit 0x10;
// #define manipulater_D2_setSpeedLimit    0x11;
// #define manipulater_D2_setCurrentLimit  0x12;
//Command_ID定义
#define RS_CMD_MODE            0x01 //控制模式设置
#define RS_CMD_VELOCITY_SET    0x02 //速度指令设置
#define RS_CMD_POSITION_SET    0x03 //位置指令设置
#define RS_CMD_CURRENT_SET     0x04 //电流指令设置
#define RS_CMD_POS_SPD_GET     0x05 //位置速度电流当前值查询
#define RS_CMD_TEMPERATURE     0x06 //温度查询
#define RS_CMD_VELOCITY_ANSWER 0x82 //三条应答指令为设置指令+0x80
#define RS_CMD_POSITION_ANSWER 0x83
#define RS_CMD_CURRENT_ANSWER  0x84
#define RS_CMD_VOLTAGE         0x66 //电压查询
#define RS_CMD_POSITION_LIMITS 0x10 //位置边界限制
#define RS_CMD_VELOCITY_LIMITS 0x11 //速度边界边界限制
#define RS_CMD_CURRENT_LIMITS  0x12 //电流边界限制


int   manipulater_D2_RS485Init(void);
int   manipulater_D2_send(uint8_t DID, uint8_t obj, uint8_t ASK, uint8_t *send_data, uint8_t len);
//int manipulater_D2_receive(manipulater_D2_receiveBuf_def *manipulater_D2_receiveBuf)
#endif