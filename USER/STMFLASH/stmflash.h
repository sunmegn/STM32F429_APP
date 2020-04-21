/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2020-03-02 16:44:29
 * @LastEditors:smake
 * @LastEditTime:2020-04-19 00:29:06
 * @brief         :
 */
#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "includes.h"
#include "stm32f4xx_hal_flash_ex.h"
typedef volatile unsigned int vu32;
typedef volatile unsigned int vu16;
//FLASH起始地址
/*************************************
*0x08000000
*bootloader
*0x08010000
*APP
*0x08050000
*NULL
*0x08070000
*0000-0004 bootloader 更新标志
*0x081E0000 推进器中值存储区
*/
//#define FLASH_BASE_ADDR  0x08010000
//#define UPDATA_FLAG_ADDR 0x08100000  //使用扇区12  16kB空间存储数据
#define PARAMADDR            0x081E0000             //单片机最后一块FLASH地址128K
#define ADDR_FLASH_SECTOR_4  ((uint32_t)0x08010000) //扇区4起始地址, 64 Kbytes
#define ADDR_FLASH_SECTOR_12 ((uint32_t)0x08100000) //扇区12起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_13 ((uint32_t)0x08104000) //扇区13起始地址, 16 Kbytes
#define FLASH_WAITETIME      50000                  // FLASH等待超时时间
enum
{
    UPDATA = 0,
    APPRUN,
};
//u16 STMFLASH_ReadWord(u32 faddr);		  	//读出字
void     STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u32 NumToWrite); //从指定地址开始写入指定长度的数据
void     STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u32 NumToRead);    //从指定地址开始读出指定长度的数据
void     STMFLASH_Erase_DATA(u8 SectorNum);
uint32_t STMFLASH_Read_Byte(uint32_t faddr);
void     STMFLASH_Erase_Sector(uint8_t The_sector_num);
void     STMFLASH_Erase(void);
void     STMFLASH_Write_Byte(uint32_t WriteAddr, uint8_t data);
#endif
