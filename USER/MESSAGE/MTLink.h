#ifndef __MTLINK_H_
#define __MTLINK_H_

#include "main.h"

//#include "global.h"
#include "udp_demo.h"
#define MTLINKMAXPLEN 1024

#define HOST_ID 1
#define FREE_ID 255
#define My_ID   2

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
    uint8_t  TxFIFO[MTLINKMAXPLEN + 20];     // +20
    uint8_t  RxFIFO[MTLINKMAXPLEN * 2 + 40]; //*2
    uint8_t  AskFIFO[20];
    uint8_t  Decodebuf[1024 * 20]; //
    uint8_t *RxFIFOp;
} MTLink_typedef;

extern MTLink_typedef MTLink_UDP; //在messageTask函数中定义用于上传至上位机的数据

bool MTLink_Decode(MTLink_typedef *m);
bool MTLink_Encode(MTLink_typedef *m, uint8_t SID, uint8_t DID, uint8_t ASK, uint16_t obj, uint8_t *buf, int len, uint32_t waittime_ms);
void MTLinkFIFO_append(MTLink_typedef *m, uint8_t *pushbuf, int pushlen);

void Pressure_CheckCRC(uint8_t *buf, int len, uint8_t *CRC_H, uint8_t *CRC_L);
bool MTLink_RespandsDecode(MTLink_typedef *m);
bool MTLinkDispose(uint8_t SID, uint16_t obj, uint8_t *buf, int len);
void MTLinkPrint(MTLink_typedef *m, uint8_t SID, uint8_t DID, char *str, int len, uint32_t timeout);

extern QueueHandle_t MTLinkUDPRxQueue;
extern QueueHandle_t MTLinkUDPAskQueue;

#endif
