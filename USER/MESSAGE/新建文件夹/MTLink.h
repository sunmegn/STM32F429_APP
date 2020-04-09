#ifndef __MTLINK_H_
#define __MTLINK_H_

#include "main.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"

#include "global.h"
#define MTLINKMAXPLEN 1024       //最大包长
#define DECODENUM     1024 * 100 //解码数据容量

typedef struct
{
    uint8_t  STX;
    uint16_t Len;
    uint8_t  RPC;
    uint8_t  PSTATE;
    uint8_t  CNT;
    uint8_t  SID;
    uint8_t  DID;
    uint16_t Object;
    uint8_t *PayLoad;
    uint8_t  CRCL;
    uint8_t  CRCH;
    uint8_t  TxFIFO[MTLINKMAXPLEN + 10];     // +20
    uint8_t  RxFIFO[MTLINKMAXPLEN * 2 + 40]; //*2
    uint8_t  AskFIFO[20];
    uint8_t  Decodebuf[DECODENUM]; //
    uint8_t *RxFIFOp;
    uint8_t  My_ID[5];
} MTLink_typedef;
extern MTLink_typedef MTLink_UDP;
extern QueueHandle_t  MTLinkUDPAskQueue;

/******************************************************************/
bool MTLink_WaitRespands(MTLink_typedef *m, uint32_t waittime_ms);
void SendToRespandsBuf(MTLink_typedef *m, uint8_t *buf, int len);
bool MTLink_SendBuffer(uint8_t DID, uint8_t *buf, int len);
/******************************************************************/
void MTLinkPrint(MTLink_typedef *m, uint8_t SID, uint8_t DID, char *str, int len, uint32_t timeout);
bool MTLink_Decode(MTLink_typedef *m);
bool MTLink_Encode(MTLink_typedef *m, uint8_t SID, uint8_t DID, uint8_t ASK, uint16_t obj, uint8_t *buf, int len, uint32_t waittime_ms);
void MTLinkFIFO_append(MTLink_typedef *m, uint8_t *pushbuf, int pushlen);
void MTLinkSetMY_ID(MTLink_typedef *m, uint8_t *my_id, int idnum);
void Pressure_CheckCRC(uint8_t *buf, int len, uint8_t *CRC_H, uint8_t *CRC_L);
bool MTLink_RespandsDecode(MTLink_typedef *m);
bool MTLinkDispose(uint8_t SID, uint8_t DID, uint16_t obj, uint8_t *buf, int len);

extern QueueHandle_t MTLinkUDPRxQueue;

#endif
