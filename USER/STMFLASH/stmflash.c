#include "stmflash.h"
 
//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
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
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_2;//�����ֲ���
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&pEraseInit, &error);
	HAL_FLASH_Lock();

}

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u32 NumToWrite)	
{ 
	unsigned int i = 0;
	HAL_FLASH_Unlock();//����flash
	
	for(i = 0;i < NumToWrite;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]);
		WriteAddr+=2;
	}
	
	HAL_FLASH_Lock();//����flash
}

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=2;//ƫ��4���ֽ�.	
	}
}










