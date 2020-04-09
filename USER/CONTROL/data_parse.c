#include "data_parse.h"

HeartBeat_t HeartBeat_State;
FeedCounter_t Pi_Counter;


uint8_t UserSendFlag[0xFF];	             //���ͱ�־
uint8_t UserSendFreq[0xFF];	             //����Ƶ��
uint8_t UserSortResult[0xFF];            //����Ƶ������
uint8_t UserSendList[MAX_SEND_FREQ];     //�����б�

HeartBeat_t HeartBeat_State;
FeedCounter_t Pi_Counter;
 
//Pi = stm32 ���Ź�
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
		 HeartBeat_State.HeartBeat_State = 0;           //����ֹͣ
		 HeartBeat_State.Last_FishLostState = HeartBeat_State.Temp_FishLostState;
		 HeartBeat_State.Temp_FishLostState = HEART_DIE;//��ʧ״̬
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
*�� �� ��: SendFreqSort
*����˵��: ������Ϣ����Ƶ��������ϢID����
*��    ��:.����������ָ�� ����Ƶ������ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void SendFreqSort(uint8_t* sortResult, uint8_t* sendFreq)
{
    uint8_t i = 0, j = 0;
    uint8_t temp;

    //�ȳ�ʼ����ϢID��������
    for(i = 0; i<0xFF; i++)
        sortResult[i] = i;

    //��ʼ���շ���Ƶ��������ϢID����
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

    //���˵�һ����������Ϊ����Ŀ����Ϊ������������֡���;����ܾ���
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
*�� �� ��: SendListCreate
*����˵��: ���ݸ���Ϣ֡�ķ���Ƶ���Զ����ɷ����б�
*��    ��: ����Ƶ������ָ�� ����������ָ�� �����б�����ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void SendListCreate(uint8_t* sendFreq, uint8_t* sortResult, uint8_t* sendList)
{
    uint8_t sendNum = 0;
    uint8_t i, j;
    static float interval;
    uint8_t random;

    //�ж��ܷ������Ƿ񳬳������Ƶ�ʣ����������˳��ú���
    for(i=0; i<0xFF; i++)
    {
        if(sendFreq[sortResult[i]] == 0)
            break;

        sendNum += sendFreq[sortResult[i]];
    }
    if(sendNum > MAX_SEND_FREQ)
        return;

    //��շ����б�
    for(i=0; i<MAX_SEND_FREQ; i++)
        sendList[i] = 0;

    //��ʼ���ɷ����б�
    for(i=0; i<0xFF; i++)
    {
        if(sendFreq[sortResult[i]] == 0)
            return;

        //���ͼ��
        interval = (float)MAX_SEND_FREQ / sendFreq[sortResult[i]];
        //�������������Ϊ��֡�������б��е�������ʼ�㣬�������Ծ���ʹ��֡���ݷֲ�����
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
	
    /*��ʼ����֡�ķ���Ƶ��,���Ƶ�ʲ�����MAX_SEND_FREQ*/
    UserSendFreq[MSG_TEMP_HUMI_ID2]        = 1;
    UserSendFreq[MSG_PRESSURE_ID]          = 20;
    UserSendFreq[MSG_COMPASS_ID]           = 20;
    UserSendFreq[MSG_POWER_ID]             = 1;
	  //UserSendFreq[MSG_GPS_ID]             = 1;
		UserSendFreq[MSG_CONPARA_ID]           = 21;
	  UserSendFreq[MSG_HEARTBEAT_ID]         = 1;
    //���ɷ����б�
    SendFreqSort(UserSortResult, UserSendFreq);
    SendListCreate(UserSendFreq, UserSortResult, UserSendList);

//    MavParamSetDefault();	
	
	
}
 

//stm32 -> Pi      �����ϴ�����    		         
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
