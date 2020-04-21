#include "IAP.h"

pFunction JumpToApplication;
uint32_t  JumpFlag __attribute__((at(UPDATA_FLAG_ADDR)));

void Jump_To_Bootloader(void)
{
    STMFLASH_Erase_Sector(UPDATA_FLAG_SECTOR);
    FLASH_WaitForLastOperation(FLASH_WAITETIME);
    STMFLASH_Write_Byte(UPDATA_FLAG_ADDR, 0x00); //跳转到bootloader
    HAL_Delay(20);

    if (STMFLASH_Read_Byte(UPDATA_FLAG_SECTOR) == 0)
    {
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
    else
    {
        //MTLinkPrint(&MTLink_UDP, MY_ID, HOST_ID, "跳转Bootloader失败!", sizeof("跳转Bootloader失败!"), 300);
    }
}
