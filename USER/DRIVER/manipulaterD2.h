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
//Command_ID����
#define RS_CMD_MODE            0x01 //����ģʽ����
#define RS_CMD_VELOCITY_SET    0x02 //�ٶ�ָ������
#define RS_CMD_POSITION_SET    0x03 //λ��ָ������
#define RS_CMD_CURRENT_SET     0x04 //����ָ������
#define RS_CMD_POS_SPD_GET     0x05 //λ���ٶȵ�����ǰֵ��ѯ
#define RS_CMD_TEMPERATURE     0x06 //�¶Ȳ�ѯ
#define RS_CMD_VELOCITY_ANSWER 0x82 //����Ӧ��ָ��Ϊ����ָ��+0x80
#define RS_CMD_POSITION_ANSWER 0x83
#define RS_CMD_CURRENT_ANSWER  0x84
#define RS_CMD_VOLTAGE         0x66 //��ѹ��ѯ
#define RS_CMD_POSITION_LIMITS 0x10 //λ�ñ߽�����
#define RS_CMD_VELOCITY_LIMITS 0x11 //�ٶȱ߽�߽�����
#define RS_CMD_CURRENT_LIMITS  0x12 //�����߽�����


int   manipulater_D2_RS485Init(void);
int   manipulater_D2_send(uint8_t DID, uint8_t obj, uint8_t ASK, uint8_t *send_data, uint8_t len);
//int manipulater_D2_receive(manipulater_D2_receiveBuf_def *manipulater_D2_receiveBuf)
#endif