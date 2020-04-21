#include "Version.h" 

AUV_Version_Typedef MyVersion;

void LoadVersion(AUV_Version_Typedef *ver,float num,char* name,int years,int month,int date,int Hours,int minute)
{
	sprintf(ver->VersionName,"20Kg ROV-Version:");
	sprintf(ver->num,"%0.1f",num);
	sprintf(ver->Name,"%s",name);
	sprintf(ver->Date,"%d-%d-%d",years,month,date);
	sprintf(ver->time,"%d:%d",Hours,minute);
}


void PrintVersion(AUV_Version_Typedef *ver)
{
	printf("%s\r\n",ver->VersionName);
	printf("VersionNum:%s\r\n",ver->num);
	printf("VersionName:%s\r\n",ver->Name);
	printf("VersionDate:%s\r\n",ver->Date);
	printf("VersionTime:%s\r\n",ver->time);
}
/*int main(void)
{
	char sbuf[100];
	LoadVersion(&MyVersion,1.0,"TianChen",2019,5,8,1,23);
	PrintVersion(&MyVersion);
	return 1;
}*/
