#ifndef __DATA_PARSE_H
#define __DATA_PARSE_H

#include "includes.h"
#include "stdbool.h"
/****************remote message id*****************/
#define MSG_TEMP_HUMI_ID2   0x10
#define MSG_TEMP_HUMI_ID    0x00
#define MSG_PRESSURE_ID     0x01
#define MSG_COMPASS_ID      0x02
#define MSG_POWER_ID        0x03
#define MSG_CMD_ID          0x04
#define MSG_GIMBAL_ID       0x05
#define MSG_LIGHT_ID        0x06
#define MSG_PID_ID          0x07
#define MSG_STATE_ID        0x08
#define MSG_TEST_ID         0x09
#define MSG_PHONE_ID  			0X22
#define MSG_GPS_ID          0x20
#define MSG_BLANCE_EN_ID    0X21
#define MSG_MANIPULATOR     0x22
#define MSG_HEARTBEAT_ID    0X40
#define MSG_CONPARA_ID      0x50

#define MSG_CALIBRATION     0x30
#define MSG_SAVECAL         0x31


#define MAX_SEND_FREQ     100

 
typedef enum
{
	HEART_DIE = 0,
	HEART_ALIVE
}HeartAlive_t;
typedef struct
{
	u8 HeartBeat_State;  // 0:死亡 1:活着
	int HeartBeatCounter;
	HeartAlive_t Temp_FishLostState;
  HeartAlive_t Last_FishLostState;
}HeartBeat_t;

typedef struct
{
	u16 HeartBeat_PackNum;  
	u8 state;
}HeartBeatMsg_t;

typedef struct 
{
    u16 max_count;
	  u16 temp_count;
	  bool  loss_flag;
}FeedCounter_t;


//Pi - stm32 断开检测
void Pi_Counter_Init(void);
void Pi_LossCounter(void);
void Pi_FeedCounter(void);
bool Pi_CounterCheck(void);
void Pi_HeartBeatCounter(void);
extern HeartBeat_t HeartBeat_State;
//电子罗盘 断开检测 
void Compass_Counter_Init(void);
void Compass_LossCounter(void);
void Compass_FeedCounter(void);
bool Compass_CounterCheck(void);

extern uint8_t UserSendFlag[0xFF];	             //发送标志
extern uint8_t UserSendFreq[0xFF];	             //发送频率
extern uint8_t UserSortResult[0xFF];            //发送频率排序
extern uint8_t UserSendList[MAX_SEND_FREQ];     //发送列表
//PC - stm32 解析函数
extern void SendToPC(uint8_t id,uint8_t* data,uint8_t len);
void MessageInit(void);
void PC_Stm32_unpackData(uint8_t *buf,u16 len);
//void Mavlink_Data_Parse(uint8_t* buf,uint16_t len);

#endif
