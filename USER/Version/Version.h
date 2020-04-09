#ifndef __VERSION_H
#define __VERSION_H 
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
/*********************
	驱动功能说明：
	功能测试Demo:  
*********************/ 
typedef struct
{
	char VersionName[20];	//项目名称 
	char num[10];			//版本号 
	char Name[20];			//软件负责人  
	char Date[10];			//软件版本日期 
	char time[10];			//软件版本时间 
}AUV_Version_Typedef;
void LoadVersion(AUV_Version_Typedef *ver,float num,char* name,int years,int month,int date,int Hours,int minute);
void PrintVersion(AUV_Version_Typedef *ver);

extern AUV_Version_Typedef MyVersion;
#endif
