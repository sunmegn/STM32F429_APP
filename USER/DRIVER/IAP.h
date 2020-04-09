#ifndef __IAP_H__
#define __IAP_H__
#include "stmflash.h"


//��ת��ַ
#define FLASH_APP_ADDR       ADDR_FLASH_SECTOR_4       
//Flag��ַ
#define JUMP_FLAG_ADDER		ADDR_FLASH_SECTOR_12  //
typedef  void (*pFunction)(void);

extern pFunction JumpToApplication;
extern uint32_t JumpFlag;


void Jump_To_Bootloader(void);
#endif
