#include "imu.h"
/**
  ******************************************************************************
  * @file    imu.c
  * @author  LiuYang
  * @brief   原子C3 串口读取惯导数据
  *
  *
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */
#define G           9.8
#define IMUFreq     50
#define YawValue(x) (x >= 360) ? (x - 360) : ((x < 0) ? (x + 360) : (x))

#define SETMEASUREMODE      1
#define SWITCHTOCONFIG      2
#define SETSELFTESTMODE     3
#define SAVETOFLASH         4
#define GETSELFTESTMODE     5
#define SETSELFTESTMODE1    6
#define GETALLSTATUS        7
#define RESET               8
#define WAKEUPHOST          9
#define GETPACKETUPDATERATE 10
//u8 WakeUpHostMode[8]  		= {0x41,0x78,0xff,0x01,0x01,0x00,0xc6,0x6d};
u8 WakeUpHostAck[8] = {0x41, 0x78, 0xff, 0x01, 0x81, 0x00, 0x46, 0x6d};

//u8 ImuSystemRest[8]  		= {0x41,0x78,0xff,0x01,0x04,0x00,0xc3,0x6d};
//static u8 SetEngineFuslonMode[9]	= {0x41,0x78,0xff,0x04,0x09,0x01,0x00,0xCA,0x6D};
//static u8 EngineFuslonModeAck[9] 	= {0x41,0x78,0xff,0x04,0x89,0x01,0x00,0x4A,0x6D};
u8 GetAllStatus[8]       = {0x41, 0x78, 0xff, 0x02, 0x06, 0x00, 0xC2, 0x6D}; //0xE7 --> 0xC2
u8 SwitchToConfigMode[8] = {0x41, 0x78, 0xff, 0x01, 0x02, 0x00, 0xc5, 0x6d};
//static u8 SwitchToConfigAck[8] 	= {0x41,0x78,0xff,0x01,0x82,0x00,0x46,0x6d};
u8 SwitchToMeasureMode[8] = {0x41, 0x78, 0xff, 0x01, 0x03, 0x00, 0xc4, 0x6d};
//static u8 SwitchToMeasureAck[8]	= {0x41,0x78,0xff,0x01,0x83,0x00,0x44,0x6d};
u8 ConfigurationWriteToFlash[8]    = {0x41, 0x78, 0xff, 0x01, 0x08, 0x00, 0xcf, 0x6d};
u8 ConfigurationWriteToFlashACK[8] = {0x41, 0x78, 0xFF, 0x01, 0x88, 0x00, 0x4F, 0x6D};
//static u8 SETPacketUpdateRate[10] = {0x41,0x78,0xff,0x04,0x10,0x02,0x32,0x00,0xe2,0x6d};//50Hz
//static u8 SETPacketUpdateRateACK[10] = {0x41,0x78,0xff,0x04,0x90,0x02,0x32,0x00,0x62,0x6d};//50Hz

//static u8 GetGroscopeScale[10]={0x41,0x78,0xFF,0x03,0x07,0x6D};
u8 GetPacketUpdateRate[8] = {0x41, 0x78, 0xFF, 0x04, 0x11, 0x00, 0xD3, 0x6D};
u8 SetSelTestMode[9]      = {0x41, 0x78, 0xFF, 0x04, 0x54, 0x01, 0x11, 0x86, 0x6D}; //设置成自自检不通过也输出
u8 SetSelTestMode1[9]     = {0x41, 0x78, 0xFF, 0x04, 0x54, 0x01, 0x01, 0x96, 0x6D}; //设置自检不通过不输出
u8 SetSelTestModeACK[9]   = {0x41, 0x78, 0xFF, 0x04, 0xD4, 0x01, 0x05, 0x12, 0x6D};
u8 GetSelTestMode[8]      = {0x41, 0x78, 0xFF, 0x04, 0x55, 0x00, 0x97, 0x6D};
u8 GetSelTestModeRES[9]   = {0x41, 0x78, 0xFF, 0x04, 0xD5, 0x01, 0x00, 0x92, 0x6D};
u8 RestMode[8]            = {0x41, 0x78, 0xFF, 0x01, 0x04, 0x00, 0xC3, 0x6D};
/********************************************
IMU坐标系： 芯片点方向
			  -   +        俯仰角：pitch 前后方向
				^Y 
				|          
						   -
			  z *	--> x	  侧倾角：Roll 左右方向

						   +
			
		 偏航角：Yaw(向左为正，向右为负)
*********************************************/
enum
{
    IMU_ID      = 0,
    IMU_ID1     = 1,
    IMU_ADDR    = 2,
    IMU_CLASSID = 3,
    IMU_MSGID   = 4,
    IMU_PLDLEN  = 5,
    IMU_PLD     = 6,
    IMU_CRC     = 7,
    IMU_ID2     = 8,
};
u8             is_imustatus            = 2;
static u8      SaberDataPacket_Head[6] = {0x41, 0x78, 0xFF, 0x06, 0x81, 0x53};
IMUAccPkg      accRawData;
IMU_UsartRec_t imu_rec;
IMUGyroPkg     gyroRawData;
IMUEulerPkg    eulerRawData;
u8             Atom_BCC(u8 *addr, unsigned short len)
{
    u8 *Datapoint;
    Datapoint                 = addr;
    u8             XorData    = 0;
    unsigned short DataLength = len;
    while (DataLength--)
    {
        XorData = XorData ^ *Datapoint;
        Datapoint++;
    }
    return XorData;
}

IMU_UsartRec_t imu_rec;

void IMU_Init(void)
{
    HAL_GPIO_WritePin(SABER_RST_GPIO_Port, SABER_RST_Pin, GPIO_PIN_RESET);
    delay_ms(100);
    HAL_GPIO_WritePin(SABER_RST_GPIO_Port, SABER_RST_Pin, GPIO_PIN_SET);

    HAL_UART_Receive_DMA(&IMU_USART, imu_rec.RX_pData, sizeof(imu_rec.RX_pData));
    __HAL_UART_CLEAR_IDLEFLAG(&IMU_USART);
    __HAL_UART_ENABLE_IT(&IMU_USART, UART_IT_IDLE);

}

//0x8801  Kalman Acc
//0x8C01  Kalman Gyro
//0x9001  Kalman Mag
//0xB000  Quatuernion 四元数
//0xB001  Euler Angle 欧拉角 姿态角

void SaberImu_unpack(u8 *buf, u16 len)
{
    static u8 state   = IMU_ID;
    static u8 datacnt = 0;
    u8        IMURawData[200];
    u8        i        = 0;
    u8        class_id = 0;
    u8        msg_id   = 0;
    u8        pld_len  = 0;
    u8        XorData  = 0;
    u8        cnt      = 0;
    u16       Pid      = 0;
    u8        pldnum   = 0;

    for (i = 0; i < len; i++)
    {
        if (datacnt >= 198)
        {
            datacnt = 0;
            state   = IMU_ID;
        }
        if (state == IMU_ID && buf[i] == 0x41)
        {
            /////
            memset(IMURawData, 0, 200);
            datacnt             = 0;
            IMURawData[datacnt] = buf[i];
            /////
            state    = IMU_ID1;
            XorData  = 0;
            Pid      = 0;
            pld_len  = 0;
            msg_id   = 0;
            class_id = 0;
            cnt      = 0;
            XorData  = XorData ^ IMURawData[datacnt];
            datacnt++;
        }
        else if (state == IMU_ID1 && buf[i] == 0x78)
        {
            state               = IMU_ADDR;
            IMURawData[datacnt] = buf[i];
            XorData             = XorData ^ IMURawData[datacnt];
            datacnt++;
        }
        else if (state == IMU_ADDR && buf[i] == 0xFF)
        {
            state               = IMU_CLASSID;
            IMURawData[datacnt] = buf[i];
            XorData             = XorData ^ IMURawData[datacnt];
            datacnt++;
        }
        else if (state == IMU_CLASSID)
        {
            if (buf[i] == 0x01 || buf[i] == 0x02 || buf[i] == 0x03 || buf[i] == 0x06)
            {
                state               = IMU_MSGID;
                IMURawData[datacnt] = buf[i];
                class_id            = IMURawData[datacnt];
                XorData             = XorData ^ IMURawData[datacnt];
                datacnt++;
            }
            else
            {
                datacnt = 0;
                state   = IMU_ID;
            }
        }
        else if (state == IMU_MSGID)
        {
            if (buf[i] == 0x01 || buf[i] == 0x81 || buf[i] == 0x82 || buf[i] == 0x86)
            {
                state               = IMU_PLDLEN;
                IMURawData[datacnt] = buf[i];
                msg_id              = IMURawData[datacnt];
                XorData             = XorData ^ IMURawData[datacnt];
                datacnt++;
            }
            else
            {
                datacnt = 0;
                state   = IMU_ID;
            }
        }
        else if (state == IMU_PLDLEN)
        {
            IMURawData[datacnt] = buf[i];
            pld_len             = IMURawData[datacnt];
            XorData             = XorData ^ IMURawData[datacnt];
            if (pld_len)
                state = IMU_PLD;
            else
                state = IMU_CRC;
            datacnt++;
        }
        else if (state == IMU_PLD)
        {

            IMURawData[datacnt] = buf[i];
            XorData             = XorData ^ IMURawData[datacnt];
            datacnt++;
            cnt++;
            if (cnt >= pld_len)
            {
                state = IMU_CRC;
                cnt   = 0;
            }
        }
        else if (state == IMU_CRC) //&& XorData == buf[i]
        {
            IMURawData[datacnt] = buf[i];
            if (XorData == IMURawData[datacnt])
            {
                datacnt++;
                state = IMU_ID2;
            }
            else
            {
                datacnt = 0;
                state   = IMU_ID;
            }
        }
        else if (state == IMU_ID2) //&&buf[i] == 0x6D
        {
            IMURawData[datacnt] = buf[i];
            if (IMURawData[datacnt] == 0x6D)
            {
                if (class_id == 0x01 && msg_id == 0x82)
                {
                    is_imustatus = 3;
                }
                else if (class_id == 0x01 && msg_id == 0x83)
                {
                    is_imustatus = 0;
                }
                else if (class_id == 0x02 && msg_id == 0x86)
                {
                    is_imustatus       = 4;
                    pldnum             = 6;
                    eulerRawData.valid = (IMURawData[pldnum + 1] >> 4) & 0x0F;
                }
                else if (class_id == 0x06 && msg_id == 0x81)
                {
                    pldnum = 6;
                    Pid    = IMURawData[pldnum] | (IMURawData[pldnum + 1] << 8);
                    if (Pid == 0x8801) //Kalman Acc
                    {
                        memcpy(&accRawData, &IMURawData[pldnum + 3], 12);
                    }
                    Pid = (IMURawData[pldnum + 16] << 8) | IMURawData[pldnum + 15];
                    if (Pid == 0x8C00) //Kalman Gyro
                    {
                        memcpy(&gyroRawData, &IMURawData[pldnum + 18], 12);
                    }
                    Pid = (IMURawData[pldnum + 31] << 8) | IMURawData[pldnum + 30];
                    if (Pid == 0x9001) //Kalman Mag
                    {
                        //memcpy(&IMUEulerPkg,&buf[pldnum+33],12);
                    }
                    Pid = (IMURawData[pldnum + 46] << 8) | IMURawData[pldnum + 45];
                    if (Pid == 0xB000) //Quatuernion 四元数
                    {
                        //memcpy(&IMUEulerPkg,&buf[pldnum+48],16);
                    }
                    Pid = (IMURawData[pldnum + 65] << 8) | IMURawData[pldnum + 64];
                    if (Pid == 0xB001) //Euler Angle 欧拉角 姿态角
                    {
                        memcpy(&eulerRawData, &IMURawData[pldnum + 67], 12);
                    }
                }
                state    = IMU_ID;
                XorData  = 0;
                pld_len  = 0;
                msg_id   = 0;
                class_id = 0;
                cnt      = 0;
                datacnt  = 0;
            }
            else
            {
                datacnt = 0;
                state   = IMU_ID;
            }
        }
    }
}
extern DMA_HandleTypeDef hdma_uart8_rx;

void HAL_UART8_Receive_IDLE(UART_HandleTypeDef *huart)
{

    if ((__HAL_UART_GET_FLAG(&IMU_USART, UART_FLAG_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&IMU_USART);
        HAL_UART_DMAStop(&IMU_USART); //DMAmuxChannel

        imu_rec.RX_Size = IMU_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx);
        imu_rec.RX_flag = 1;
        SaberImu_unpack(imu_rec.RX_pData, imu_rec.RX_Size);
        //				SaberImu_unpack(WakeUpHostMode,8);
        memset(imu_rec.RX_pData, '\0', IMU_RX_LEN);
        HAL_UART_Receive_DMA(&IMU_USART, imu_rec.RX_pData, IMU_RX_LEN);
    }
}
void IMU_GetData(IMUMsg_t *imu)
{
    imu->pitch = -eulerRawData.roll;
    imu->roll  = eulerRawData.pitch;
    imu->yaw   = eulerRawData.yaw;
    imu->gyrox = gyroRawData.gyroX;
    imu->gyroy = gyroRawData.gyroY;
    imu->gyroz = gyroRawData.gyroZ;
    imu->accx  = accRawData.Accx;
    imu->accy  = accRawData.Accy;
    imu->accz  = accRawData.Accz;
    imu->valid = eulerRawData.valid;
}

void IMU_Getrpy(Data_GYRO_HandleType *imurpy)
{
    imurpy->gyroX = -gyroRawData.gyroY;
    imurpy->gyroY = gyroRawData.gyroX;
    imurpy->gyroZ = gyroRawData.gyroZ;
}

u8 imu_send_cmd = 0;
//u8 pBuf[20];

//int ret = 0;
//u8 crc;
//int  AtomCmd_Compose_Send(u8 *addr, u16 len)
//{
//	int index = 0;

//	//Header
//	pBuf[index++] = addr[0];
//	pBuf[index++] = addr[1];
//	pBuf[index++] = addr[2];
//	pBuf[index++] = addr[3];
//	pBuf[index++] = addr[4];
//	pBuf[index++] = addr[5] & 0xff;

//	//Payload
//	for (int i = 0; i < addr[5]; i++)
//		pBuf[index++] = addr[6+i];

//	//BCC
//	crc = Atom_BCC((u8*)pBuf, 6 + addr[5]);
//	pBuf[index++] = crc & 0xff;

//	//Footer
//	pBuf[index++] = 'm';

//	return ret;
//}
void IMU_Test_Demo(void)
{
    HAL_GPIO_WritePin(SABER_RST_GPIO_Port, SABER_RST_Pin, GPIO_PIN_RESET);
    delay_ms(100);
    HAL_GPIO_WritePin(SABER_RST_GPIO_Port, SABER_RST_Pin, GPIO_PIN_SET);

    HAL_UART_Receive_DMA(&IMU_USART, imu_rec.RX_pData, IMU_RX_LEN);
    __HAL_UART_CLEAR_IDLEFLAG(&IMU_USART);
    __HAL_UART_ENABLE_IT(&IMU_USART, UART_IT_IDLE);

    while (1)
    {
        if (imu_send_cmd == SETMEASUREMODE)
        {
            imu_send_cmd = 0;
            HAL_UART_Transmit(&IMU_USART, SwitchToMeasureMode, sizeof(SwitchToMeasureMode), 0x100);
        }
        else if (imu_send_cmd == SWITCHTOCONFIG)
        {
            imu_send_cmd = 0;

            //AtomCmd_Compose_Send(SwitchToConfigMode);
            HAL_UART_Transmit(&IMU_USART, SwitchToConfigMode, sizeof(SwitchToConfigMode), 0x100);
        }
        else if (imu_send_cmd == SETSELFTESTMODE)
        {
            imu_send_cmd = 0;

            //			AtomCmd_Compose_Send(SetSelTestMode,9);

            HAL_UART_Transmit(&IMU_USART, SetSelTestMode, sizeof(SetSelTestMode), 0x100);
        }
        else if (imu_send_cmd == SAVETOFLASH)
        {
            imu_send_cmd = 0;

            //AtomCmd_Compose_Send(ConfigurationWriteToFlash);
            HAL_UART_Transmit(&IMU_USART, ConfigurationWriteToFlash, sizeof(ConfigurationWriteToFlash), 0x100);
        }
        else if (imu_send_cmd == GETSELFTESTMODE)
        {
            imu_send_cmd = 0;
            HAL_UART_Transmit(&IMU_USART, GetSelTestMode, sizeof(GetSelTestMode), 0x100);
            //AtomCmd_Compose_Send(GetSelTestMode);
        }
        else if (imu_send_cmd == SETSELFTESTMODE1)
        {
            imu_send_cmd = 0;
            //			AtomCmd_Compose_Send(SetSelTestMode1,9);
            HAL_UART_Transmit(&IMU_USART, SetSelTestMode1, sizeof(SetSelTestMode1), 0x100);
        }
        else if (imu_send_cmd == GETALLSTATUS)
        {
            imu_send_cmd = 0;
            //			AtomCmd_Compose_Send(GetAllStatus,8);
            HAL_UART_Transmit(&IMU_USART, GetAllStatus, sizeof(GetAllStatus), 0x100);
        }
        else if (imu_send_cmd == RESET)
        {
            imu_send_cmd = 0;
            //			AtomCmd_Compose_Send(RestMode,8);
            HAL_UART_Transmit(&IMU_USART, RestMode, sizeof(RestMode), 0x100);
        }
        else if (imu_send_cmd == WAKEUPHOST)
        {
            imu_send_cmd = 0;
            //			AtomCmd_Compose_Send(WakeUpHostAck,8);
            HAL_UART_Transmit(&IMU_USART, WakeUpHostAck, sizeof(WakeUpHostAck), 0x100);
        }
        else if (imu_send_cmd == GETPACKETUPDATERATE)
        {
            imu_send_cmd = 0;
            //			AtomCmd_Compose_Send(GetPacketUpdateRate,8);
            HAL_UART_Transmit(&IMU_USART, GetPacketUpdateRate, sizeof(GetPacketUpdateRate), 0x100);
        }
        HAL_Delay(500);
    }
}
