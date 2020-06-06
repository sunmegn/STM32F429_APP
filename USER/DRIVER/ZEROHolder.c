#include "ZEROHolder.h"

#include "Object.h"
#include "global.h"

/*
	零差电机云台驱动
	CAN  波特率1000KHz  协议CANOpen
	零差CANOpen  应用为云台 数据量不大和对实时性要求不高 所以选择的SDO的方式
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
  * @brief  CAN滤波器
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
  * @brief  CAN初始化
  * @param	
  * @retval  
*/
void Holder_CAN_Init(void)
{
    HOlderCANFilterConfig_Scale16_ALL_Pass();                          //滤波器全部接收
    HAL_CAN_Start(&hcan1);                                             //开启CAN
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //开启中断挂起允许
}

/**
  * @funNm  
  * @brief  CAN发送
  * @param	hcan：发送使用can
  * @param	ID：发送的can id
  * @param  buf：发送的数据
  * @param  len：发送的数据（buf）长度
  * @param  Timeout：设定超时时间
  * @retval 返回发送状态
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
    txMessage.IDE                = CAN_ID_STD; //标准帧
    txMessage.TransmitGlobalTime = DISABLE;
    do
    {
        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 2 && (nowtime - Basetime) < Timeout)
        { //有多个空余邮箱
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
  * @brief  CAN接收回调
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
            //云台RSDO数据接收处理
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
  * @brief  CAN收发测试
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
  * @brief  零差电机CANOpen协议的Tsdo 发送
  * @param	id：设备ID
  * @param	obj：索引
  * @param	subobj：子索引
  * @param	data：数据
  * @param	datalen：数据（data）长度  最大为4 bety
  * @retval 返回TSDO发送状态
*/
uint8_t ZEROCANOpen_Tsdo(uint16_t id, uint16_t obj, uint8_t subobj, uint32_t data, uint8_t datalen)
{
    uint8_t state  = 1;
    uint8_t buf[8] = {0};

    if (datalen == 0) //[0] 命令代码
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
        return 0x04; //数据编码错误

    buf[1] = (uint8_t)obj; //[1][2] 索引
    buf[2] = (uint8_t)(obj >> 8);
    buf[3] = (uint8_t)(subobj); //[3]子索引
    buf[4] = (uint8_t)(data);   //[4][5][6][7]数据 小端在前
    buf[5] = (uint8_t)(data >> 8);
    buf[6] = (uint8_t)(data >> 16);
    buf[7] = (uint8_t)(data >> 24);

    state = HolderCAN_Transmit(&hcan1, 0x600 + id, buf, 8, 10); //can1发送
    if (state == HAL_OK)
    {
        if (CAN1TxQueue) //发送队列 代表发送完成 准备接收返回数据
            xQueueOverwrite(CAN1TxQueue, &state);
        return state; //返回发送状态
    }
    else
        return state; //返回发送状态
}

/**
  * @funNm  
  * @brief  云台Rsdo接收处理
  * @param	
  * @retval  
*/
uint8_t HolderCANOpen_Rsdo_RX(CAN1Rec_t *CAN1Rec, uint16_t cob_id, uint16_t len, uint8_t *buf)
{
    BaseType_t xHigherPriorityTaskWoken = NULL;

    uint8_t state;
    if (CAN1TxQueue)
    {
        //获取到接收完成的信号 可以接收数据了
        if (xQueueReceiveFromISR(CAN1TxQueue, &state, &xHigherPriorityTaskWoken))
        {
            CAN1Rec->cob_id = cob_id;
            CAN1Rec->Size   = len;
            memcpy(CAN1Rec->RX_pData, buf, 8);
            //接收到数据
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
  * @brief  RSDO 数据解析
  * @param	id：设备ID
  * @param	obj：索引
  * @param	subobj：子索引
  * @param	Recf：待处理数据 缓存结构体
  * @param	time：接收堵塞时间
  * @retval 返回处理状态
*/
uint8_t ZEROCANOpen_Rsdo(uint16_t id, uint16_t obj, uint8_t subobj, CAN1Rec_t *Recf, uint16_t time)
{
    if (xQueueReceive(CAN1RxQueue, Recf, time) == pdPASS)
    {
        if (0x580 + id == Recf->cob_id) //COB_ID
        {
            if (((uint8_t)obj == Recf->RX_pData[1]) && ((uint8_t)(obj >> 8) == Recf->RX_pData[2]) && (subobj == Recf->RX_pData[3])) //索引（2字节）   子索引（1字节）
            {
                memset(Recf->RealBuff, '\0', CAN1_RX_LEN);
                switch (Recf->RX_pData[0]) //命令字
                {
                case 0x43: //四个字节全部有效
                    memcpy(Recf->RealBuff, Recf->RX_pData + 4, 4);
                    break;
                case 0x47: //三个字节有效
                    memcpy(Recf->RealBuff, Recf->RX_pData + 4, 3);
                    break;
                case 0x4b: //两个字节有效
                    memcpy(Recf->RealBuff, Recf->RX_pData + 4, 2);
                    break;
                case 0x4f: //一个字节有效
                    memcpy(Recf->RealBuff, Recf->RX_pData + 4, 1);
                    break;
                case 0x60: //0字节
                    break;
                default:         //错误
                    return 0x26; //其他错误
                }
                Recf->cob_id = 0;
                memset(Recf->RX_pData, '\0', 8);
                return 0x0f;
            }
            else
            {
                Recf->cob_id = 0;
                memset(Recf->RX_pData, '\0', 8);
                return 0x23; //校验失败
            }
        }
        else
        {
            Recf->cob_id = 0;
            memset(Recf->RX_pData, '\0', 8);
            return 0x24; //设备ID错误
        }
    }
    Recf->cob_id = 0;
    memset(Recf->RX_pData, '\0', 8);
    return 0x00; //未接受到数据
}

/**
  * @funNm  
  * @brief  零差电机指令数据发送
  * @param	id：设备ID 当前ROV统一修改为 1
  * @param	obj：索引
  * @param	subobj：子索引
  * @param	data：指令数据
  * @param	datalen：数据（data）长度 最大为 4
  * @param	Recf：数据 缓存结构体
  * @param	timeout    超时时间
  * @param	sendtimes  发送次数
  * @retval 返回状态
*/
uint8_t ZERO_Set(uint8_t id, uint16_t obj, uint8_t subobj, uint32_t data, uint8_t datalen, CAN1Rec_t *Recf, uint16_t timeout, uint8_t sendtimes)
{
    uint32_t Basetime = HAL_GetTick();
    uint32_t nowtime  = Basetime;
    uint8_t  state    = 0;

    do
    {
        osDelay(1);
        state = ZEROCANOpen_Tsdo(id, obj, subobj, data, datalen);     //发送指令
        if (state == 0)                                               //判断发送成功
            state = ZEROCANOpen_Rsdo(id, obj, subobj, Recf, timeout); //接受解析
        nowtime = HAL_GetTick();
    } while (state != 0x0f && (nowtime - Basetime) < (timeout * sendtimes));

    Recf->vaild = state;
    return state;
}

/**
  * @funNm  GetStat
  * @brief  获取状态
  * @param	id：设备ID
  * @param	timeout   超时时间
  * @param	sendtimes  发送次数
  * @retval 返回设备状态
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
        state = ZEROCANOpen_Tsdo(id, 0x6041, 0x00, 0, 0); //发送状态请求
        if (state == 0)
            state = ZEROCANOpen_Rsdo(id, 0x6041, 0x00, &CAN1Recf, timeout); //返回当前状态
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
  * @brief  状态切换 状态机
  * @param	id：设备ID
  * @param	CW：控制 根据控制字和当前状态 自动切换至目标状态
  * @param	timeout   超时时间
  * @retval 返回当前状态
*/
uint8_t ZERO_ChangeStat(uint8_t id, uint8_t CW, uint16_t timeout)
{
    uint8_t          SW = 0;
    static CAN1Rec_t CAN1Recf;
    static uint8_t   Faultflagcount = 0;

    SW   = ZERO_GetStat(id, timeout, 1); //获取当前状态
    g_SW = SW;
    g_CW = CW;

    if ((SW & 0Xff) == SwitchOnDisabled) //无故障
    {
        ZERO_Set(id, 0x6040, 0x00, 0x0006, 2, &CAN1Recf, timeout, 1);
        osDelay(500);
    }
    else if ((SW & 0Xff) == ReadyToSwitchOn) //准备好
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
    else if ((SW & 0Xff) == SwitchOn) //等待使能
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
    else if ((SW & 0Xff) == OperationEnabled) //运行
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
    else if ((SW & 0X08) == Fault) //故障
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
  * @brief  初始化零差电机  使电机使能
  * @param	delay：初始前的延长时间 给电机上电后的准备时间
  * @retval 
*/
uint8_t ZERO_Init(uint16_t delay)
{
    static CAN1Rec_t CAN1Recf;

    osDelay(delay); //等待电机准备好  可根据实际情况更改

    ZERO_Set(1, 0x603F, 0x00, 0, 0, &CAN1Recf, 50, 4);      //查询错误
    ZERO_Set(1, 0x6040, 0x00, 0x0080, 2, &CAN1Recf, 50, 4); //清除错误
    ZERO_Set(1, 0x6060, 0x00, 0x07, 1, &CAN1Recf, 50, 4);   //设置模式 插补模式

    ZERO_Set(1, 0x6040, 0x00, 0x0006, 2, &CAN1Recf, 50, 4); //停止
    osDelay(500);                                           //延时500毫秒 原因快速切换使能和停止状态时 有时会导致抱闸无法打开
    ZERO_Set(1, 0x6040, 0x00, 0x0007, 2, &CAN1Recf, 50, 4); //准备好
    ZERO_Set(1, 0x6040, 0x00, 0x001f, 2, &CAN1Recf, 50, 4); //使能

    if (ZERO_GetStat(1, 50, 4) == 0x37) //判断状态字是否为运行状态  1：运行状态初始化成功  2：非运行状态初始化失败   可能为电机存在错误
        return 0x0f;
    else
        return CAN1Recf.vaild;
}

/**
  * @funNm    
  * @brief  获取并切换零差电机状态 
  * @param	CW：控制字
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
  * @brief   设置运动位置（绝对）
  * @param	 pos：目标位置（单位cnt）
  * @retval  
*/
uint8_t ZERO_Set_Pos(u32 pos)
{
    uint8_t          sat = 0;
    static CAN1Rec_t CAN1Recf;

    sat = ZERO_Set(1, 0x60C1, 0x01, pos, 4, &CAN1Recf, 10, 1); //设置目标位置

    return sat;
}

/**
  * @funNm   
  * @brief   设置运动角度（相对）
  * @param	 angle：运动的相对角度（单位度） 
  * @param	 midpos：设置的中位置（单位cnt） 相对角度的参考位置
  * @param	 maxang：最大运动角度
  * @param	 minang：最小运动角度
  * @retval  
*/
uint8_t ZERO_Set_Angle(int16_t angle, uint32_t midpos, int16_t maxang, int16_t minang)
{
    uint8_t          sat = 0;
    static CAN1Rec_t CAN1Recf;
    uint32_t         pos = 0;

    pos = CONSTRAIN(angle, maxang, minang) * ENCODEVAL * 1.0 / 360 + midpos;
    sat = ZERO_Set(1, 0x60C1, 0x01, pos, 4, &CAN1Recf, 10, 1); //设置目标位置

    return sat;
}

/**
  * @funNm  
  * @brief  获取当前位置（绝对）
  * @param	
  * @retval 返回位置（单位cnt）
*/
uint32_t ZERO_Get_Pos(void)
{
    uint8_t          sat = 0;
    static u32andu8  posval;
    static CAN1Rec_t CAN1Recf;

    sat = ZERO_Set(1, 0x6064, 0x00, 0, 0, &CAN1Recf, 10, 1); //获取当前位置
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
  * @brief   获取当前角度（相对）
  * @param	 midpos：中位置（单位cnt） 相对角度的参考位置
  * @param	 maxang：最大运动角度
  * @param	 minang：最小运动角度
  * @retval  返回相对角度（单位度）
*/
int16_t ZERO_Get_Angle(uint32_t midpos, int16_t maxang, int16_t minang)
{
    uint8_t          sat = 0;
    static u32andu8  posval;
    static CAN1Rec_t CAN1Recf;
    static float     angle = 0;

    sat = ZERO_Set(1, 0x6064, 0x00, 0, 0, &CAN1Recf, 10, 1); //获取当前位置
    if (sat)
    {
        memcpy(posval.numbuff, CAN1Recf.RealBuff, 4);
        angle = (float)((long)(posval.num) - midpos) / (float)ENCODEVAL * 360.0; //角度转换

        return angle * (posval.num / (posval.num + 0.000001));
    }
    else
        return angle * (posval.num / (posval.num + 0.000001));
}

/**
  * @funNm   
  * @brief   设置安全位置
  * @param	 mid：中值位置 角度参考点
  * @param	 addmax：最大角度
  * @param	 addmin：最小角度
  * @retval  返回状态
*/
uint8_t ZERO_SaveSaftyPos(uint32_t mid, int16_t addmax, int16_t addmin)
{
    uint8_t          sat = 0;
    static CAN1Rec_t CAN1Recf;

    sat = ZERO_Set(1, 0x607D, 0x01, addmin * ENCODEVAL * 1.0 / 360 + mid, 4, &CAN1Recf, 10, 1); //设置软件位置保护最小位置
    sat = ZERO_Set(1, 0x607D, 0x02, addmax * ENCODEVAL * 1.0 / 360 + mid, 4, &CAN1Recf, 10, 1); //设置软件位置保护最大位置

    uint8_t buf[2] = {0x00, 0xE8}; //保存命令  0x640+id  00 0E
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
