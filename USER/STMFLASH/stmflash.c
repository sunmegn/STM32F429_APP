#include "stmflash.h"
 
//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
u16 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu16*)faddr; 
}  

void STMFLASH_Erase_DATA(void)
{
	FLASH_EraseInitTypeDef pEraseInit;
	u32 error;
	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	pEraseInit.NbSectors = 1;
	pEraseInit.Sector = FLASH_SECTOR_23;
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_2;//按半字擦除
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&pEraseInit, &error);
	HAL_FLASH_Lock();

}

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u32 NumToWrite)	
{ 
	unsigned int i = 0;
	HAL_FLASH_Unlock();//解锁flash
	
	for(i = 0;i < NumToWrite;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]);
		WriteAddr+=2;
	}
	
	HAL_FLASH_Lock();//锁定flash
}

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=2;//偏移4个字节.	
	}
}










