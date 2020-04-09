#include "data_parse.h"

HeartBeat_t HeartBeat_State;
FeedCounter_t Pi_Counter;


uint8_t UserSendFlag[0xFF];	             //发送标志
uint8_t UserSendFreq[0xFF];	             //发送频率
uint8_t UserSortResult[0xFF];            //发送频率排序
uint8_t UserSendList[MAX_SEND_FREQ];     //发送列表

HeartBeat_t HeartBeat_State;
FeedCounter_t Pi_Counter;
 
//Pi = stm32 看门狗
void Pi_Counter_Init(void)
{
	Pi_Counter.max_count = 500;
	Pi_Counter.temp_count = 0;
	Pi_Counter.loss_flag = false;
}
void Pi_LossCounter(void)
{
	if(Pi_Counter.temp_count<Pi_Counter.max_count){
		Pi_Counter.temp_count++;
	}
}
void Pi_HeartBeatCounter(void)
{
	 if(HeartBeat_State.HeartBeatCounter>500){
		 HeartBeat_State.HeartBeat_State = 0;           //心跳停止
		 HeartBeat_State.Last_FishLostState = HeartBeat_State.Temp_FishLostState;
		 HeartBeat_State.Temp_FishLostState = HEART_DIE;//丢失状态
	 }else{HeartBeat_State.HeartBeatCounter++;}
}
void Pi_FeedCounter(void)
{
	Pi_Counter.temp_count=0;
}
bool Pi_CounterCheck(void)
{
	return (Pi_Counter.temp_count == Pi_Counter.max_count);
}

/**********************************************************************************************************
*函 数 名: SendFreqSort
*功能说明: 根据消息发送频率来给消息ID排序
*形    参:.排序结果数组指针 发送频率数组指针
*返 回 值: 无
**********************************************************************************************************/
static void SendFreqSort(uint8_t* sortResult, uint8_t* sendFreq)
{
    uint8_t i = 0, j = 0;
    uint8_t temp;

    //先初始化消息ID排序序列
    for(i = 0; i<0xFF; i++)
        sortResult[i] = i;

    //开始按照发送频率来给消息ID排序
    for(i=0; i<0xFF; i++)
    {
        for(j=i+1; j<0xFF; j++)
        {
            if(sendFreq[sortResult[j]] > sendFreq[sortResult[i]])
            {
                temp = sortResult[i];
                sortResult[i] = sortResult[j];
                sortResult[j] = temp;
            }
        }
    }

    //除了第一个，其它改为倒序，目的是为了让所有数据帧发送尽可能均匀
    uint8_t validNum = 0;
    for(i=0; i<0xFF; i++)
    {
        if(sendFreq[sortResult[i]] != 0)
            validNum++;
    }

    validNum -= 1;

    for(i=1; i<=validNum/2; i++)
    {
        temp = sortResult[i];
        sortResult[i] = sortResult[validNum + 1 - i];
        sortResult[validNum + 1 - i] = temp;
    }
}

/**********************************************************************************************************
*函 数 名: SendListCreate
*功能说明: 根据各消息帧的发送频率自动生成发送列表
*形    参: 发送频率数组指针 排序结果数组指针 发送列表数组指针
*返 回 值: 无
**********************************************************************************************************/
static void SendListCreate(uint8_t* sendFreq, uint8_t* sortResult, uint8_t* sendList)
{
    uint8_t sendNum = 0;
    uint8_t i, j;
    static float interval;
    uint8_t random;

    //判断总发送量是否超出最大发送频率，若超过则退出该函数
    for(i=0; i<0xFF; i++)
    {
        if(sendFreq[sortResult[i]] == 0)
            break;

        sendNum += sendFreq[sortResult[i]];
    }
    if(sendNum > MAX_SEND_FREQ)
        return;

    //清空发送列表
    for(i=0; i<MAX_SEND_FREQ; i++)
        sendList[i] = 0;

    //开始生成发送列表
    for(i=0; i<0xFF; i++)
    {
        if(sendFreq[sortResult[i]] == 0)
            return;

        //发送间隔
        interval = (float)MAX_SEND_FREQ / sendFreq[sortResult[i]];
        //生成随机数，作为该帧数据在列表中的排序起始点，这样可以尽量使各帧数据分布均匀
        random   = GetRandom() % MAX_SEND_FREQ;

        for(j=0; j<sendFreq[sortResult[i]]; j++)
        {
            for(uint8_t k=0; k<MAX_SEND_FREQ-j*interval; k++)
            {
                if(sendList[(int16_t)(j*interval+k+random) % MAX_SEND_FREQ] == 0)
                {
                    sendList[(int16_t)(j*interval+k+random) % MAX_SEND_FREQ] = sortResult[i];
                    break;
                }
            }
        }
    }
}

void MessageInit(void)
{
	
    /*初始化各帧的发送频率,最大频率不超过MAX_SEND_FREQ*/
    UserSendFreq[MSG_TEMP_HUMI_ID2]        = 1;
    UserSendFreq[MSG_PRESSURE_ID]          = 20;
    UserSendFreq[MSG_COMPASS_ID]           = 20;
    UserSendFreq[MSG_POWER_ID]             = 1;
	  //UserSendFreq[MSG_GPS_ID]             = 1;
		UserSendFreq[MSG_CONPARA_ID]           = 21;
	  UserSendFreq[MSG_HEARTBEAT_ID]         = 1;
    //生成发送列表
    SendFreqSort(UserSortResult, UserSendFreq);
    SendListCreate(UserSendFreq, UserSortResult, UserSendList);

//    MavParamSetDefault();	
	
	
}
 

//stm32 -> Pi      数据上传函数    		         
uint8_t send_buf[30]={0};
void SendToPC(uint8_t id,uint8_t* data,uint8_t len)// 
{
    static uint8_t count = 0;
	  send_buf[0] = 0xfe;
    send_buf[1] = 7+len;
    send_buf[2] = count;
    send_buf[3] = id;
    for(uint8_t i=0;i<len;i++)
    {
        send_buf[4+i] = *(data+i);
    }
    send_buf[len+6] = 0x0a;
		udp_send(send_buf,send_buf[1]);
}
