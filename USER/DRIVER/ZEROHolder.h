#ifndef _ZEROHOLDER_H
#define _ZEROHOLDER_H

#include "main.h"
#include "global.h"

#include "can.h"

/*宏定义*/
#define CAN1_RX_LEN 8

#define NotReadyToSwitchOn 0x00
#define SwitchOnDisabled   0x40
#define ReadyToSwitchOn    0x21
#define SwitchOn           0x33
#define OperationEnabled   0x37
#define Fault              0x08

#define CWDisableVol       0x10
#define CWShutdown         0x06
#define CWSwitchon         0x07
#define CWSOandENOperation 0x1f

#define ENCODEVAL 524288 //2的19次幂



/*结构体*/
typedef struct
{
    u8  RX_flag : 1; //IDLE receive flag
    u16 Size;        //receive length
    u16 cob_id;
    u8  RX_pData[CAN1_RX_LEN]; //DMA receive buffer
    u8  RealBuff[CAN1_RX_LEN];

    uint8_t vaild;
} CAN1Rec_t;

/*函数声明*/
void              HolderCAN1Queues_Init(void);
void              Holder_CAN_Init(void);
HAL_StatusTypeDef HolderCAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t ID, uint8_t *buf, int len, uint32_t Timeout);
void              HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);
void              CAN1_Test_Demo(void);

uint8_t  ZEROCANOpen_Tsdo(uint16_t id, uint16_t obj, uint8_t subobj, uint32_t data, uint8_t datalen);
uint8_t  HolderCANOpen_Rsdo_RX(CAN1Rec_t *CAN1Rec, uint16_t cob_id, uint16_t len, uint8_t *buf);
uint8_t  ZEROCANOpen_Rsdo(uint16_t id, uint16_t obj, uint8_t subobj, CAN1Rec_t *Recf, uint16_t time);
uint8_t  ZERO_Set(uint8_t id, uint16_t obj, uint8_t subobj, uint32_t data, uint8_t datalen, CAN1Rec_t *Recf, uint16_t timeout, uint8_t sendtimes);
uint8_t  ZERO_GetStat(uint8_t id, uint16_t timeout, uint8_t sendtimes);
uint8_t  ZERO_ChangeStat(uint8_t id, uint8_t CW, uint16_t timeout);
uint8_t  ZERO_Init(uint16_t delay);
uint8_t  ZERO_GetDriverState(uint16_t CW);
uint8_t  ZERO_Set_Pos(u32 pos);
uint8_t  ZERO_Set_Angle(int16_t angle, uint32_t midpos, int16_t maxang, int16_t minang);
uint32_t ZERO_Get_Pos(void);
int16_t  ZERO_Get_Angle(uint32_t midpos, int16_t maxang, int16_t minang);
uint8_t  ZERO_SaveSaftyPos(uint32_t mid, int16_t addmax, int16_t addmin);

void ZEROHolder_Test_Demo(void);

extern QueueHandle_t CAN1RxQueue;
extern QueueHandle_t CAN1TxQueue;

/*全局定义*/

#endif
