/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2020-03-02 16:44:29
 * @LastEditors:Robosea
 * @LastEditTime:2020-03-03 10:26:53
 * @brief         :
 */
#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "includes.h"
#include "stm32f4xx_hal_flash_ex.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//STM32�ڲ�FLASH��д ��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

typedef volatile unsigned int vu32;
typedef volatile unsigned int vu16;
//FLASH��ʼ��ַ
/*************************************
*0x08000000
*bootloader
*0x08010000
*APP
*0x08050000
*NULL
*0x08070000
*0000-0004 bootloader ���±�־
*0x081E0000 �ƽ�����ֵ�洢��
*/
//#define FLASH_BASE_ADDR  0x08010000
//#define UPDATA_FLAG_ADDR 0x08100000  //ʹ������12  16kB�ռ�洢����
#define PARAMADDR 0x081E0000 //��Ƭ�����һ��FLASH��ַ128K

#define ADDR_FLASH_SECTOR_4  ((uint32_t)0x08010000) //����4��ʼ��ַ, 64 Kbytes
#define ADDR_FLASH_SECTOR_12 ((uint32_t)0x08100000) //����12��ʼ��ַ, 16 Kbytes
#define FLASH_WAITETIME  50000      // FLASH�ȴ���ʱʱ��
enum
{
    UPDATA = 0,
    APPRUN,

};

//u16 STMFLASH_ReadWord(u32 faddr);		  	//������
void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u32 NumToWrite); //��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u32 NumToRead);    //��ָ����ַ��ʼ����ָ�����ȵ�����
void STMFLASH_Erase_DATA(u8 SectorNum);
uint32_t STMFLASH_Read_Byte(uint32_t faddr);
void STMFLASH_Erase_Sector(uint8_t The_sector_num);
void STMFLASH_Erase(void);
void STMFLASH_Write_Byte(uint32_t WriteAddr, uint8_t data);
#endif
