#include "IAP.h"


pFunction JumpToApplication;
uint32_t  JumpFlag __attribute__((at(JUMP_FLAG_ADDER)));

void Jump_To_Bootloader(void)
{
    u16 flag = 0;

    __set_FAULTMASK(1);
    //STMFLASH_Write(JUMP_FLAG_ADDER, &flag, 0x01);
    NVIC_SystemReset();
}
