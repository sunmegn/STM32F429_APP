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

//Command_ID����
#define RS_CMD_MODE                0x01 //����ģʽ����
#define RS_CMD_MODE_LEN            0x01 //���Ϳ���ģʽ���ݳ���
#define RS_CMD_VELOCITY_SET        0x02 //�ٶ�ָ������
#define RS_CMD_VELOCITY_SET_LEN    0x04 //�ٶ�ָ���������ݳ���
#define RS_CMD_POSITION_SET        0x03 //λ��ָ������
#define RS_CMD_POSITION_SET_LEN    0x04 //λ��ָ���������ݳ���
#define RS_CMD_CURRENT_SET         0x04 //����ָ������
#define RS_CMD_CURRENT_SET_LEN     0x04 //����ָ���������ݳ���
#define RS_CMD_POS_SPD_GET         0x05 //λ���ٶȵ�����ǰֵ��ѯ
#define RS_CMD_POS_SPD_GET_LEN     0x00 //λ���ٶȵ�����ǰֵ��ѯ���ݳ���
#define RS_CMD_TEMPERATURE         0x06 //�¶Ȳ�ѯ
#define RS_CMD_TEMPERATURE_LEN     0x04 //�¶Ȳ�ѯ���ݳ���
#define RS_CMD_VELOCITY_ANSWER     0x82 //����Ӧ��ָ��Ϊ����ָ��+0x80
#define RS_CMD_VELOCITY_ANSWER_LEN 0x04 //����Ӧ��ָ�����ݳ���
#define RS_CMD_POSITION_ANSWER     0x83
#define RS_CMD_POSITION_ANSWER_LEN 0x04
#define RS_CMD_CURRENT_ANSWER      0x84
#define RS_CMD_CURRENT_ANSWER_LEN  0x04
#define RS_CMD_VOLTAGE             0x66 //��ѹ��ѯ
// #define RS_CMD_VOLTAGE_LEN         0x00 //��ѹ��ѯ���ݳ���
#define RS_CMD_POSITION_LIMITS     0x10 //λ�ñ߽�����
#define RS_CMD_POSITION_LIMITS_LEN 0x08 //λ�ñ߽��������ݳ���
#define RS_CMD_VELOCITY_LIMITS     0x11 //�ٶȱ߽�߽�����
#define RS_CMD_VELOCITY_LIMITS_LEN 0x04 //�ٶȱ߽�߽��������ݳ���
#define RS_CMD_CURRENT_LIMITS      0x12 //�����߽�����
#define RS_CMD_CURRENT_LIMITS_LEN  0x04 //�����߽��������ݳ���

#define NOACK                  0
#define ACK                    1
#define MOTOR_LINE_DRIVER_ID   0x01
#define MOTOR_ROTATE_DRIVER_ID 0x02

#pragma pack(1)
typedef struct
{
    uint8_t driver_id;            //���ID
    uint8_t runMode;              //���ģʽ��0����Ĭģʽ��1������ģʽ��2���ٶ�ģʽ��3��λ��ģʽ
    float   send_position;        //���͵��λ��
    float   send_velocity;        //���͵���ٶ�
    float   send_current;         //���͵������ֵ
    float   set_position_max;     //���õ��λ�����ֵ
    float   set_position_min;     //���õ��λ����Сֵ
    float   set_velocity_max;     //���õ���ٶ����ֵ
    float   set_velocity_min;     //���õ���ٶ���Сֵ
    float   set_current_max;      //���õ���������ֵ A
    float   set_current_min;      //���õ��������Сֵ A
    float   receive_position_max; //
    float   receive_position_min; //
    float   receive_velocity_max; //
    float   receive_velocity_min; //
    float   receive_current_max;  //
    float   receive_current_min;  //
    float   motor_temperature;    //��ȡ�����ǰ�¶�ֵ ��
    float   motor_position;       //��ȡ�����ǰλ��ֵ
    float   motor_velocity;       //��ȡ�����ǰ�ٶ�ֵ
    float   motor_current;        //��ȡ�����ǰ����ֵ
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
