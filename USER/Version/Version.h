#ifndef __VERSION_H
#define __VERSION_H 
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
/*********************
	��������˵����
	���ܲ���Demo:  
*********************/ 
typedef struct
{
	char VersionName[20];	//��Ŀ���� 
	char num[10];			//�汾�� 
	char Name[20];			//���������  
	char Date[10];			//����汾���� 
	char time[10];			//����汾ʱ�� 
}AUV_Version_Typedef;
void LoadVersion(AUV_Version_Typedef *ver,float num,char* name,int years,int month,int date,int Hours,int minute);
void PrintVersion(AUV_Version_Typedef *ver);

extern AUV_Version_Typedef MyVersion;
#endif
