#include "ZEROHolder.h"

#include "Object.h"
#include "global.h"

/*
	�������̨����
	CAN  ������1000KHz  Э��CANOpen
	���CANOpen  Ӧ��Ϊ��̨ ����������Ͷ�ʵʱ��Ҫ�󲻸� ����ѡ���SDO�ķ�ʽ
*/

QueueHandle_t CAN1RxQueue = NULL;
QueueHandle_t CAN1TxQueue = NULL;
/**
  * @funNm  
  * @brief  
  * @param	
  * @retval  
*/
static void CAN1RxQueues_Init(void)
{
    do
    {
        CAN1RxQueue = xQueueCreate(1, sizeof(CAN1Rec_t));
    } while (CAN1RxQueue == NULL);
}

static void CAN1TxQueues_Init(void)
{
    do
    {
        CAN1TxQueue = xQueueCreate(1, sizeof(uint8_t));
    } while (CAN1TxQueue == NULL);
}

void HolderCAN1Queues_Init(void)
{
    CAN1RxQueues_Init();
    CAN1TxQueues_Init();
}

/**
  * @funNm  
  * @brief  CAN�˲���
  * @param	
  * @retval  
*/
static void HOlderCANFilterConfig_Scale16_ALL_Pass(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank   = 0;
    sFilterConfig.FilterMode   = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale  = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow  = 0;

    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow  = 0;

    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    sFilterConfig.FilterBank           = 0;
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @funNm  
  * @brief  CAN��ʼ��
  * @param	
  * @retval  
*/
void Holder_CAN_Init(void)
{
    HOlderCANFilterConfig_Scale16_ALL_Pass();                          //�˲���ȫ������
    HAL_CAN_Start(&hcan1);                                             //����CAN
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //�����жϹ�������
}

/**
  * @funNm  
  * @brief  CAN����
  * @param	hcan������ʹ��can
  * @param	ID�����͵�can id
  * @param  buf�����͵�����
  * @param  len�����͵����ݣ�buf������
  * @param  Timeout���趨��ʱʱ��
  * @retval ���ط���״̬
*/
HAL_StatusTypeDef HolderCAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t ID, uint8_t *buf, int len, uint32_t Timeout)
{
    CAN_TxHeaderTypeDef txMessage;

    int      TxLen = len;
    uint32_t TxMailbox;
    uint8_t *p        = buf;
    uint32_t Basetime = HAL_GetTick();
    uint32_t nowtime  = Basetime;

    txMessage.RTR                = CAN_RTR_DATA;
    txMessage.ExtId              = 0;
    txMessage.StdId              = ID;
    txMessage.IDE                = CAN_ID_STD; //��׼֡
    txMessage.TransmitGlobalTime = DISABLE;
    do
    {
        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 2 && (nowtime - Basetime) < Timeout)
        { //�ж����������
            nowtime = HAL_GetTick();
        }
        if ((nowtime - Basetime) >= Timeout)
        {
            return HAL_ERROR;
        }
        if (TxLen > 8)
        {
            txMessage.DLC = 8;
            HAL_CAN_AddTxMessage(hcan, &txMessage, p, &TxMailbox);
            TxLen -= 8;
            p += 8;
        }
        else
        {
            txMessage.DLC = TxLen;
            HAL_CAN_AddTxMessage(hcan, &txMessage, p, &TxMailbox);
            TxLen = 0;
        }
        nowtime = HAL_GetTick();
    } while (TxLen > 0 && (nowtime - Basetime) < Timeout);
    if ((nowtime - Basetime) >= Timeout)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
  * @funNm  
  * @brief  CAN���ջص�
  * @param	
  * @retval  
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    CAN_RxHeaderTypeDef RxMessage;

    uint8_t    RxData[8];
    BaseType_t xHigherPriorityTaskWoken = NULL;

    CAN1Rec_t CAN1Rec;

    if (CanHandle == &hcan1)
    {
        if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxMessage, RxData) != HAL_OK)
        {
            /* Reception Error */
            Error_Handler();
        }
        else
        {
            //��̨RSDO���ݽ��մ���
            HolderCANOpen_Rsdo_RX(&CAN1Rec, RxMessage.StdId, RxMessage.DLC, RxData);

            CAN1Rec.cob_id = RxMessage.StdId;
            CAN1Rec.Size   = RxMessage.DLC;
            memcpy(CAN1Rec.RX_pData, RxData, 8);
            if (CAN1RxQueue)
                xQueueSendFromISR(CAN1RxQueue, &CAN1Rec, &xHigherPriorityTaskWoken);
        }
    }
}

/**
  * @funNm  
  * @brief  CAN�շ�����
  * @param	
  * @retval  
*/
u8        testcan1sendbuffer[8] = {0, 1, 2, 3, 4, 5, 6, 7};
CAN1Rec_t testCAN1Rec;
void      CAN1_Test_Demo(void)
{
    HolderCAN_Transmit(&hcan1, 1, testcan1sendbuffer, 8, 10);
    xQueueReceive(CAN1RxQueue, &testCAN1Rec, 10);
}

/**
  * @funNm  
  * @brief  �����CANOpenЭ���Tsdo ����
  * @param	id���豸ID
  * @param	obj������
  * @param	subobj��������
  * @param	data������
  * @param	datalen�����ݣ�data������  ���Ϊ4 bety
  * @retval ����TSDO����״̬
*/
uint8_t ZEROCANOpen_Tsdo(uint16_t id, uint16_t obj, uint8_t subobj, uint32_t data, uint8_t datalen)
{
    uint8_t state  = 1;
    uint8_t buf[8] = {0};

    if (datalen == 0) //[0] �������
        buf[0] = 0x40;
    else if (datalen == 1)
        buf[0] = 0x2f;
    else if (datalen == 2)
        buf[0] = 0x2b;
    else if (datalen == 3)
        buf[0] = 0x27;
    else if (datalen == 4)
        buf[0] = 0x23;
    else
        return 0x04; //���ݱ������

    buf[1] = (uint8_t)obj; //[1][2] ����
    buf[2] = (uint8_t)(obj >> 8);
    buf[3] = (uint8_t)(subobj); //[3]������
    buf[4] = (uint8_t)(data);   //[4][5][6][7]���� С����ǰ
    buf[5] = (uint8_t)(data >> 8);
    buf[6] = (uint8_t)(data >> 16);
    buf[7] = (uint8_t)(data >> 24);

    state = HolderCAN_Transmit(&hcan1, 0x600 + id, buf, 8, 10); //can1����
    if (state == HAL_OK)
    {
        if (CAN1TxQueue) //���Ͷ��� ��������� ׼�����շ�������
            xQueueOverwrite(CAN1TxQueue, &state);
        return state; //���ط���״̬
    }
    else
        return state; //���ط���״̬
}

/**
  * @funNm  
  * @brief  ��̨Rsdo���մ���
  * @param	
  * @retval  
*/
uint8_t HolderCANOpen_Rsdo_RX(CAN1Rec_t *CAN1Rec, uint16_t cob_id, uint16_t len, uint8_t *buf)
{
    BaseType_t xHigherPriorityTaskWoken = NULL;

    uint8_t state;
    if (CAN1TxQueue)
    {
        //��ȡ��������ɵ��ź� ���Խ���������
        if (xQueueReceiveFromISR(CAN1TxQueue, &state, &xHigherPriorityTaskWoken))
        {
            CAN1Rec->cob_id = cob_id;
            CAN1Rec->Size   = len;
            memcpy(CAN1Rec->RX_pData, buf, 8);
            //���յ�����
            if (CAN1RxQueue)
                xQueueSendFromISR(CAN1RxQueue, CAN1Rec, &xHigherPriorityTaskWoken);
            return 1;
        }
        return 0;
    }
    return 0;
}

/**
  * @funNm  
  * @brief  RSDO ���ݽ���
  * @param	id���豸ID
  * @param	obj������
  * @param	subobj��������
  * @param	Recf������������ ����ṹ��
  * @param	time�����ն���ʱ��
  * @retval ���ش���״̬
*/
uint8_t ZEROCANOpen_Rsdo(uint16_t id, uint16_t obj, uint8_t subobj, CAN1Rec_t *Recf, uint16_t time)
{
    if (xQueueReceive(CAN1RxQueue, Recf, time) == pdPASS)
    {
        if (0x580 + id == Recf->cob_id) //COB_ID
        {
            if (((uint8_t)obj == Recf->RX_pData[1]) && ((uint8_t)(obj >> 8) == Recf->RX_pData[2]) && (subobj == Recf->RX_pData[3])) //������2�ֽڣ�   ��������1�ֽڣ�
            {
                memset(Recf->RealBuff, '\0', CAN1_RX_LEN);
                switch (Recf->RX_pData[0]) //������
                {
                case 0x43: //�ĸ��ֽ�ȫ����Ч
                    memcpy(Recf->RealBuff, Recf->RX_pData + 4, 4);
                    break;
                case 0x47: //�����ֽ���Ч
                    memcpy(Recf->RealBuff, Recf->RX_pData + 4, 3);
                    break;
                case 0x4b: //�����ֽ���Ч
                    memcpy(Recf->RealBuff, Recf->RX_pData + 4, 2);
                    break;
                case 0x4f: //һ���ֽ���Ч
                    memcpy(Recf->RealBuff, Recf->RX_pData + 4, 1);
                    break;
                case 0x60: //0�ֽ�
                    break;
                default:         //����
                    return 0x26; //��������
                }
                Recf->cob_id = 0;
                memset(Recf->RX_pData, '\0', 8);
                return 0x0f;
            }
            else
            {
                Recf->cob_id = 0;
                memset(Recf->RX_pData, '\0', 8);
                return 0x23; //У��ʧ��
            }
        }
        else
        {
            Recf->cob_id = 0;
            memset(Recf->RX_pData, '\0', 8);
            return 0x24; //�豸ID����
        }
    }
    Recf->cob_id = 0;
    memset(Recf->RX_pData, '\0', 8);
    return 0x00; //δ���ܵ�����
}

/**
  * @funNm  
  * @brief  �����ָ�����ݷ���
  * @param	id���豸ID ��ǰROVͳһ�޸�Ϊ 1
  * @param	obj������
  * @param	subobj��������
  * @param	data��ָ������
  * @param	datalen�����ݣ�data������ ���Ϊ 4
  * @param	Recf������ ����ṹ��
  * @param	timeout    ��ʱʱ��
  * @param	sendtimes  ���ʹ���
  * @retval ����״̬
*/
uint8_t ZERO_Set(uint8_t id, uint16_t obj, uint8_t subobj, uint32_t data, uint8_t datalen, CAN1Rec_t *Recf, uint16_t timeout, uint8_t sendtimes)
{
    uint32_t Basetime = HAL_GetTick();
    uint32_t nowtime  = Basetime;
    uint8_t  state    = 0;

    do
    {
        osDelay(1);
        state = ZEROCANOpen_Tsdo(id, obj, subobj, data, datalen);     //����ָ��
        if (state == 0)                                               //�жϷ��ͳɹ�
            state = ZEROCANOpen_Rsdo(id, obj, subobj, Recf, timeout); //���ܽ���
        nowtime = HAL_GetTick();
    } while (state != 0x0f && (nowtime - Basetime) < (timeout * sendtimes));

    Recf->vaild = state;
    return state;
}

/**
  * @funNm  GetStat
  * @brief  ��ȡ״̬
  * @param	id���豸ID
  * @param	timeout   ��ʱʱ��
  * @param	sendtimes  ���ʹ���
  * @retval �����豸״̬
*/
uint8_t ZERO_GetStat(uint8_t id, uint16_t timeout, uint8_t sendtimes)
{
    uint32_t         Basetime = HAL_GetTick();
    uint32_t         nowtime  = Basetime;
    uint8_t          state    = 0;
    uint8_t          SW       = 0;
    static CAN1Rec_t CAN1Recf;

    do
    {
        osDelay(1);
        state = ZEROCANOpen_Tsdo(id, 0x6041, 0x00, 0, 0); //����״̬����
        if (state == 0)
            state = ZEROCANOpen_Rsdo(id, 0x6041, 0x00, &CAN1Recf, timeout); //���ص�ǰ״̬
        nowtime = HAL_GetTick();
    } while (state != 0x0f && (nowtime - Basetime) < (timeout * sendtimes));
    if ((nowtime - Basetime) >= (timeout * sendtimes))
    {
        return state;
    }
    else
    {
        SW = CAN1Recf.RealBuff[0];
        return SW;
    }
}

/**
  * @funNm  
  * @brief  ״̬�л� ״̬��
  * @param	id���豸ID
  * @param	CW������ ���ݿ����ֺ͵�ǰ״̬ �Զ��л���Ŀ��״̬
  * @param	timeout   ��ʱʱ��
  * @retval ���ص�ǰ״̬
*/
uint8_t ZERO_ChangeStat(uint8_t id, uint8_t CW, uint16_t timeout)
{
    uint8_t          SW = 0;
    static CAN1Rec_t CAN1Recf;
    static uint8_t   Faultflagcount = 0;

    SW   = ZERO_GetStat(id, timeout, 1); //��ȡ��ǰ״̬
    g_SW = SW;
    g_CW = CW;

    if ((SW & 0Xff) == SwitchOnDisabled) //�޹���
    {
        ZERO_Set(id, 0x6040, 0x00, 0x0006, 2, &CAN1Recf, timeout, 1);
        osDelay(500);
    }
    else if ((SW & 0Xff) == ReadyToSwitchOn) //׼����
    {
        if ((CW & 0Xff) == CWDisableVol)
        {
            ZERO_Set(id, 0x6040, 0x00, 0x0000, 2, &CAN1Recf, timeout, 1);
        }
        else
        {
            ZERO_Set(id, 0x6040, 0x00, 0x0007, 2, &CAN1Recf, timeout, 1);
        }
    }
    else if ((SW & 0Xff) == SwitchOn) //�ȴ�ʹ��
    {
        if ((CW & 0Xff) == CWDisableVol)
        {
            ZERO_Set(id, 0x6040, 0x00, 0x0000, 2, &CAN1Recf, timeout, 1);
        }
        else if ((CW & 0Xff) == CWShutdown)
        {
            ZERO_Set(id, 0x6040, 0x00, 0x0006, 2, &CAN1Recf, timeout, 1);
            osDelay(500);
        }
        else
        {
            ZERO_Set(id, 0x6040, 0x00, 0x001f, 2, &CAN1Recf, timeout, 1);
        }
    }
    else if ((SW & 0Xff) == OperationEnabled) //����
    {
        if ((CW & 0Xff) == CWDisableVol)
        {
            ZERO_Set(id, 0x6040, 0x00, 0x0000, 2, &CAN1Recf, timeout, 1);
        }
        else if ((CW & 0Xff) == CWShutdown)
        {
            ZERO_Set(id, 0x6040, 0x00, 0x0006, 2, &CAN1Recf, timeout, 1);
            osDelay(500);
        }
        else if ((CW & 0Xff) == CWSwitchon)
        {
            ZERO_Set(id, 0x6040, 0x00, 0x0007, 2, &CAN1Recf, timeout, 1);
        }
        else
        {
        }
    }
    else if ((SW & 0X08) == Fault) //����
    {
        ZERO_Set(id, 0x6040, 0x00, 0x0080, 2, &CAN1Recf, timeout, 1);
        ZERO_Set(id, 0x6040, 0x00, 0x0006, 2, &CAN1Recf, timeout, 1);
        osDelay(500);
        ZERO_Set(id, 0x6040, 0x00, 0x0007, 2, &CAN1Recf, timeout, 1);
        ZERO_Set(id, 0x6040, 0x00, 0x001f, 2, &CAN1Recf, timeout, 1);
    }
    else
    {
    }
    return SW;
}

/**
  * @funNm  
  * @brief  ��ʼ�������  ʹ���ʹ��
  * @param	delay����ʼǰ���ӳ�ʱ�� ������ϵ���׼��ʱ��
  * @retval 
*/
uint8_t ZERO_Init(uint16_t delay)
{
    static CAN1Rec_t CAN1Recf;

    osDelay(delay); //�ȴ����׼����  �ɸ���ʵ���������

    ZERO_Set(1, 0x603F, 0x00, 0, 0, &CAN1Recf, 50, 4);      //��ѯ����
    ZERO_Set(1, 0x6040, 0x00, 0x0080, 2, &CAN1Recf, 50, 4); //�������
    ZERO_Set(1, 0x6060, 0x00, 0x07, 1, &CAN1Recf, 50, 4);   //����ģʽ �岹ģʽ

    ZERO_Set(1, 0x6040, 0x00, 0x0006, 2, &CAN1Recf, 50, 4); //ֹͣ
    osDelay(500);                                           //��ʱ500���� ԭ������л�ʹ�ܺ�ֹͣ״̬ʱ ��ʱ�ᵼ�±�բ�޷���
    ZERO_Set(1, 0x6040, 0x00, 0x0007, 2, &CAN1Recf, 50, 4); //׼����
    ZERO_Set(1, 0x6040, 0x00, 0x001f, 2, &CAN1Recf, 50, 4); //ʹ��

    if (ZERO_GetStat(1, 50, 4) == 0x37) //�ж�״̬���Ƿ�Ϊ����״̬  1������״̬��ʼ���ɹ�  2��������״̬��ʼ��ʧ��   ����Ϊ������ڴ���
        return 0x0f;
    else
        return CAN1Recf.vaild;
}

/**
  * @funNm    
  * @brief  ��ȡ���л������״̬ 
  * @param	CW��������
  * @retval  
*/
uint8_t ZERO_GetDriverState(uint16_t CW)
{
    uint8_t SW = 0;

    SW = ZERO_ChangeStat(1, CW, 50);

    return SW;
}

/**
  * @funNm   
  * @brief   �����˶�λ�ã����ԣ�
  * @param	 pos��Ŀ��λ�ã���λcnt��
  * @retval  
*/
uint8_t ZERO_Set_Pos(u32 pos)
{
    uint8_t          sat = 0;
    static CAN1Rec_t CAN1Recf;

    sat = ZERO_Set(1, 0x60C1, 0x01, pos, 4, &CAN1Recf, 10, 1); //����Ŀ��λ��

    return sat;
}

/**
  * @funNm   
  * @brief   �����˶��Ƕȣ���ԣ�
  * @param	 angle���˶�����ԽǶȣ���λ�ȣ� 
  * @param	 midpos�����õ���λ�ã���λcnt�� ��ԽǶȵĲο�λ��
  * @param	 maxang������˶��Ƕ�
  * @param	 minang����С�˶��Ƕ�
  * @retval  
*/
uint8_t ZERO_Set_Angle(int16_t angle, uint32_t midpos, int16_t maxang, int16_t minang)
{
    uint8_t          sat = 0;
    static CAN1Rec_t CAN1Recf;
    uint32_t         pos = 0;

    pos = CONSTRAIN(angle, maxang, minang) * ENCODEVAL * 1.0 / 360 + midpos;
    sat = ZERO_Set(1, 0x60C1, 0x01, pos, 4, &CAN1Recf, 10, 1); //����Ŀ��λ��

    return sat;
}

/**
  * @funNm  
  * @brief  ��ȡ��ǰλ�ã����ԣ�
  * @param	
  * @retval ����λ�ã���λcnt��
*/
uint32_t ZERO_Get_Pos(void)
{
    uint8_t          sat = 0;
    static u32andu8  posval;
    static CAN1Rec_t CAN1Recf;

    sat = ZERO_Set(1, 0x6064, 0x00, 0, 0, &CAN1Recf, 10, 1); //��ȡ��ǰλ��
    if (sat)
    {
        memcpy(posval.numbuff, CAN1Recf.RealBuff, 4);
        return posval.num;
    }
    else
        return posval.num;
}

/**
  * @funNm   
  * @brief   ��ȡ��ǰ�Ƕȣ���ԣ�
  * @param	 midpos����λ�ã���λcnt�� ��ԽǶȵĲο�λ��
  * @param	 maxang������˶��Ƕ�
  * @param	 minang����С�˶��Ƕ�
  * @retval  ������ԽǶȣ���λ�ȣ�
*/
int16_t ZERO_Get_Angle(uint32_t midpos, int16_t maxang, int16_t minang)
{
    uint8_t          sat = 0;
    static u32andu8  posval;
    static CAN1Rec_t CAN1Recf;
    static float     angle = 0;

    sat = ZERO_Set(1, 0x6064, 0x00, 0, 0, &CAN1Recf, 10, 1); //��ȡ��ǰλ��
    if (sat)
    {
        memcpy(posval.numbuff, CAN1Recf.RealBuff, 4);
        angle = (float)((long)(posval.num) - midpos) / (float)ENCODEVAL * 360.0; //�Ƕ�ת��

        return angle * (posval.num / (posval.num + 0.000001));
    }
    else
        return angle * (posval.num / (posval.num + 0.000001));
}

/**
  * @funNm   
  * @brief   ���ð�ȫλ��
  * @param	 mid����ֵλ�� �ǶȲο���
  * @param	 addmax�����Ƕ�
  * @param	 addmin����С�Ƕ�
  * @retval  ����״̬
*/
uint8_t ZERO_SaveSaftyPos(uint32_t mid, int16_t addmax, int16_t addmin)
{
    uint8_t          sat = 0;
    static CAN1Rec_t CAN1Recf;

    sat = ZERO_Set(1, 0x607D, 0x01, addmin * ENCODEVAL * 1.0 / 360 + mid, 4, &CAN1Recf, 10, 1); //�������λ�ñ�����Сλ��
    sat = ZERO_Set(1, 0x607D, 0x02, addmax * ENCODEVAL * 1.0 / 360 + mid, 4, &CAN1Recf, 10, 1); //�������λ�ñ������λ��

    uint8_t buf[2] = {0x00, 0xE8}; //��������  0x640+id  00 0E
    HolderCAN_Transmit(&hcan1, 0x641, buf, 2, 10);

    return sat;
}

/**
  * @funNm  
  * @brief  
  * @param	
  * @retval  
*/
u32  testGetCnt = 0;
u32  testSetCnt = 262144;
void ZEROHolder_Test_Demo(void)
{
}
