/**
 * @author        :smake
 * @version       :v1.0.0
 * @Date          :2020-04-20 11:21:14
 * @LastEditors:smake
 * @LastEditTime:2020-04-27 17:04:36
 * @brief         :
 */

#ifndef __MANIPULATERD2_H_
#define __MANIPULATERD2_H_
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
};

//Command_ID定义
#define RS_CMD_MODE                0x01 //控制模式设置
#define RS_CMD_MODE_LEN            0x01 //发送控制模式数据长度
#define RS_CMD_VELOCITY_SET        0x02 //速度指令设置
#define RS_CMD_VELOCITY_SET_LEN    0x04 //速度指令设置数据长度
#define RS_CMD_POSITION_SET        0x03 //位置指令设置
#define RS_CMD_POSITION_SET_LEN    0x04 //位置指令设置数据长度
#define RS_CMD_CURRENT_SET         0x04 //电流指令设置
#define RS_CMD_CURRENT_SET_LEN     0x04 //电流指令设置数据长度
#define RS_CMD_POS_SPD_GET         0x05 //位置速度电流当前值查询
#define RS_CMD_POS_SPD_GET_LEN     0x00 //位置速度电流当前值查询数据长度
#define RS_CMD_TEMPERATURE         0x06 //温度查询
#define RS_CMD_TEMPERATURE_LEN     0x04 //温度查询数据长度
#define RS_CMD_VELOCITY_ANSWER     0x82 //三条应答指令为设置指令+0x80
#define RS_CMD_VELOCITY_ANSWER_LEN 0x04 //三条应答指令数据长度
#define RS_CMD_POSITION_ANSWER     0x83
#define RS_CMD_POSITION_ANSWER_LEN 0x04
#define RS_CMD_CURRENT_ANSWER      0x84
#define RS_CMD_CURRENT_ANSWER_LEN  0x04
#define RS_CMD_VOLTAGE             0x66 //电压查询
// #define RS_CMD_VOLTAGE_LEN         0x00 //电压查询数据长度
#define RS_CMD_POSITION_LIMITS     0x10 //位置边界限制
#define RS_CMD_POSITION_LIMITS_LEN 0x08 //位置边界限制数据长度
#define RS_CMD_VELOCITY_LIMITS     0x11 //速度边界边界限制
#define RS_CMD_VELOCITY_LIMITS_LEN 0x04 //速度边界边界限制数据长度
#define RS_CMD_CURRENT_LIMITS      0x12 //电流边界限制
#define RS_CMD_CURRENT_LIMITS_LEN  0x04 //电流边界限制数据长度

#define NOACK                  0
#define ACK                    1
#define MOTOR_LINE_DRIVER_ID   0x01
#define MOTOR_ROTATE_DRIVER_ID 0x02

#pragma pack(1)
typedef struct
{
    uint8_t driver_id;            //电机ID
    uint8_t runMode;              //电机模式，0：静默模式；1：电流模式；2：速度模式；3：位置模式
    float   send_position;        //发送电机位置
    float   send_velocity;        //发送电机速度
    float   send_current;         //发送电机电流值
    float   set_position_max;     //设置电机位置最大值
    float   set_position_min;     //设置电机位置最小值
    float   set_velocity_max;     //设置电机速度最大值
    float   set_velocity_min;     //设置电机速度最小值
    float   set_current_max;      //设置电机电流最大值 A
    float   set_current_min;      //设置电机电流最小值 A
    float   receive_position_max; //
    float   receive_position_min; //
    float   receive_velocity_max; //
    float   receive_velocity_min; //
    float   receive_current_max;  //
    float   receive_current_min;  //
    float   motor_temperature;    //获取电机当前温度值 ℃
    float   motor_position;       //获取电机当前位置值
    float   motor_velocity;       //获取电机当前速度值
    float   motor_current;        //获取电机当前电流值
} manipulater_controlData_t;

typedef struct
{
    int16_t line_motor_position;
    int16_t rotate_motor_position;
    uint8_t SonarBUTTON;

} manipulaterQueue_t;

#pragma pack()

int manipulater_D2_RS485Init(void);
 void ManipulaterTaskFunction(void const *argument);
int manipulater_D2_send(uint8_t DID, uint8_t obj, uint8_t ASK, uint8_t *send_data, uint8_t len);
int manipulater_D2_receive(uint8_t *manipulater_D2_receiveBuf);
#endif
