#include "MyCAN.h"
#include "CanNetWork.h"
#include "stm32f4xx_hal_can.h"

void CANFilterConfig_Scale16_ALL_Pass(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank   = 0;                     //?????2
    sFilterConfig.FilterMode   = CAN_FILTERMODE_IDMASK; //???????
    sFilterConfig.FilterScale  = CAN_FILTERSCALE_16BIT; //???32??
    sFilterConfig.FilterIdHigh = 0;                     //????????StdIdArray[]???????,????StdIdArray[0]?????
    sFilterConfig.FilterIdLow  = 0;

    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow  = 0; //??????

    sFilterConfig.FilterFIFOAssignment = 0; //???????????FIFO0?
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    sFilterConfig.FilterBank           = 0;
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
static void CANFilterConfig_Scale16_IdList(int ID1, int ID2, int ID3, int ID4, int bank)
{
    CAN_FilterTypeDef sFilterConfig;
    uint32_t          StdId1 = ID1; //????4???CAN ID????
    uint32_t          StdId2 = ID2;
    uint32_t          StdId3 = ID3;
    uint32_t          StdId4 = ID4;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST; //??????
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_16BIT; //?????16?
    sFilterConfig.FilterIdHigh         = StdId1 << 5;           //4???CAN ID?????4????
    sFilterConfig.FilterIdLow          = StdId2 << 5;
    sFilterConfig.FilterMaskIdHigh     = StdId3 << 5;
    sFilterConfig.FilterMaskIdLow      = StdId4 << 5;
    sFilterConfig.FilterFIFOAssignment = 0; //?????????FIFO0?
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.FilterBank           = bank;
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
static void CANFilterConfig_Scale16_IdListFIFO2(int ID1, int ID2, int ID3, int ID4, int bank)
{
    CAN_FilterTypeDef sFilterConfig;
    uint32_t          StdId1 = ID1; //????4???CAN ID????
    uint32_t          StdId2 = ID2;
    uint32_t          StdId3 = ID3;
    uint32_t          StdId4 = ID4;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST; //??????
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_16BIT; //?????16?
    sFilterConfig.FilterIdHigh         = StdId1 << 5;           //4???CAN ID?????4????
    sFilterConfig.FilterIdLow          = StdId2 << 5;
    sFilterConfig.FilterMaskIdHigh     = StdId3 << 5;
    sFilterConfig.FilterMaskIdLow      = StdId4 << 5;
    sFilterConfig.FilterFIFOAssignment = 1; //?????????FIFO0?
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.FilterBank           = bank;
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void CANopen_Init(void)
{
#ifdef USE_SALVE_RPDO
    //RPDO
    CANFilterConfig_Scale16_IdList(COBID_RPDO + Net1_ID, COBID_RPDO + Net2_ID, COBID_RPDO + Net3_ID, COBID_RPDO + Net4_ID, 0);
    CANFilterConfig_Scale16_IdList(COBID_RPDO + Net5_ID, COBID_RPDO + Net6_ID, COBID_RPDO + Net7_ID, COBID_RPDO + Net8_ID, 1);
#else
    //TPDO
    CANFilterConfig_Scale16_IdList(COBID_TPDO + Net1_ID, COBID_TPDO + Net2_ID, COBID_TPDO + Net3_ID, COBID_TPDO + Net4_ID, 0);
    CANFilterConfig_Scale16_IdList(COBID_TPDO + Net5_ID, COBID_TPDO + Net6_ID, COBID_TPDO + Net7_ID, COBID_TPDO + Net8_ID, 1);
#endif

    //TCMD
    CANFilterConfig_Scale16_IdList(COBID_TCMD + Net1_ID, COBID_TCMD + Net2_ID, COBID_TCMD + Net3_ID, COBID_TCMD + Net4_ID, 2);
    CANFilterConfig_Scale16_IdList(COBID_TCMD + Net5_ID, COBID_TCMD + Net6_ID, COBID_TCMD + Net7_ID, COBID_TCMD + Net8_ID, 3);
    //RCMD
    CANFilterConfig_Scale16_IdList(COBID_RCMD + Net1_ID, COBID_RCMD + Net2_ID, COBID_RCMD + Net3_ID, COBID_RCMD + Net4_ID, 4);
    CANFilterConfig_Scale16_IdList(COBID_RCMD + Net5_ID, COBID_RCMD + Net6_ID, COBID_RCMD + Net7_ID, COBID_RCMD + Net8_ID, 5);
    //epdo
    CANFilterConfig_Scale16_IdList(COBID_EPDO + Net1_ID, COBID_EPDO + Net2_ID, COBID_EPDO + Net3_ID, COBID_EPDO + Net4_ID, 6);
    CANFilterConfig_Scale16_IdList(COBID_EPDO + Net5_ID, COBID_EPDO + Net6_ID, COBID_EPDO + Net7_ID, COBID_EPDO + Net8_ID, 7);

    //SDOASK
    CANFilterConfig_Scale16_IdList(COBID_SDO_ASK + Net1_ID, COBID_SDO_ASK + Net2_ID, COBID_SDO_ASK + Net3_ID, COBID_SDO_ASK + Net4_ID, 8);
    CANFilterConfig_Scale16_IdList(COBID_SDO_ASK + Net5_ID, COBID_SDO_ASK + Net6_ID, COBID_SDO_ASK + Net7_ID, COBID_SDO_ASK + Net8_ID, 9);

    //SDORES
    CANFilterConfig_Scale16_IdList(COBID_SDO_Res + Net1_ID, COBID_SDO_Res + Net2_ID, COBID_SDO_Res + Net3_ID, COBID_SDO_Res + Net4_ID, 10);
    CANFilterConfig_Scale16_IdList(COBID_SDO_Res + Net5_ID, COBID_SDO_Res + Net6_ID, COBID_SDO_Res + Net7_ID, COBID_SDO_Res + Net8_ID, 11);
    //My_ID
    CANFilterConfig_Scale16_IdList(COBID_RPDO + MY_ID, COBID_TCMD + MY_ID, COBID_RCMD + MY_ID, COBID_EPDO + MY_ID, 12);
    CANFilterConfig_Scale16_IdList(COBID_SDO_ASK + MY_ID, COBID_SDO_Res + MY_ID, COBID_TPDO + MY_ID, COBID_SYNC, 13);

    HAL_CAN_Start(&hcan1);
    //	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    CANTransmitThreadCreate(3);
    CANSDOThreadCreate(1);
    CANCMDThreadCreate(2);
    CANRPDOThreadCreate(2);
    CANSYNCThreadCreate(2);
    CANEPDOThreadCreate(2);
    CANRxThreadCreate(2);
}

HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t ID, uint8_t *buf, int len, uint32_t Timeout)
{
    CAN_TxHeaderTypeDef txMessage;
    int                 TxLen = len;
    uint32_t            TxMailbox;
    uint32_t            Basetime = HAL_GetTick();
    uint32_t            nowtime  = Basetime;
    uint8_t *           p        = buf;
    //txMessage.RTR                = CAN_RTR_DATA;
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

QueueHandle_t   CANTransmitQueue = NULL;
CAN_TxRxtypeDef CanSend;
osThreadId      CANSendHandle;
void            CANTransmitThreadCreate(int num)
{
    osThreadDef(CANSend, CanTransmitThread, num, 0, 512);
    CANSendHandle = osThreadCreate(osThread(CANSend), NULL);
    while (CANTransmitQueue == NULL)
    {
        CANTransmitQueue = xQueueCreate(40, sizeof(CAN_TxRxtypeDef));
    }
}
void CanTransmitThread(void const *argument) //CAN发送线程管理
{
    osDelay(10);
    while (1)
    {
        if (xQueueReceive(CANTransmitQueue, &CanSend, portMAX_DELAY) == pdTRUE)
        {
            CAN1_Send(&hcan1, &CanSend);
        }
    }
}

HAL_StatusTypeDef CAN1_TransmitForFreeRTOS(uint32_t ID, uint8_t *buf, int len, uint32_t Timeout)
{
    CAN_TxRxtypeDef Mycansend;
    Mycansend.COB_ID = ID;
    Mycansend.len    = len;
    int i;
    for (i = 0; i < len; i++)
    {
        Mycansend.buf[i] = buf[i];
    }
    xQueueSendToBack(CANTransmitQueue, &Mycansend, Timeout);
    return HAL_OK;
}

HAL_StatusTypeDef CAN1_Send(CAN_HandleTypeDef *hcan, CAN_TxRxtypeDef *CanTransmit)
{
    static CAN_TxHeaderTypeDef txMessage;
    uint32_t                   TxMailbox;
    //txMessage.RTR                = CAN_RTR_DATA;
    txMessage.ExtId              = 0;
    txMessage.StdId              = CanTransmit->COB_ID;
    txMessage.IDE                = CAN_ID_STD; //标准帧
    txMessage.TransmitGlobalTime = DISABLE;
    txMessage.DLC                = CanTransmit->len;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 1)
        ; //有多个空余邮箱
    HAL_CAN_AddTxMessage(hcan, &txMessage, CanTransmit->buf, &TxMailbox);
    return HAL_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t             RxData[8];
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxMessage, RxData) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }
    if (CanHandle == &hcan1)
    {
        CANOPEN_Rx_Process(RxMessage.StdId, RxData, RxMessage.DLC);
    }
    //	else if(CanHandle == &hcan2){
    //		CAN2_RxInterputCallback(RxMessage.ExtId,RxMessage.StdId,RxData,RxMessage.DLC);
    //	}
}

osThreadId CANRxHandle;
void       CANRxThreadCreate(int num)
{
    osThreadDef(CANRx, CanRxThread, num, 0, 1024);
    CANRxHandle = osThreadCreate(osThread(CANRx), NULL);
}
void CanRxThread(void const *argument) //CAN发送线程管理
{
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t             RxData[8];
    osDelay(10);
    portTickType tick = xTaskGetTickCount();
    while (1)
    {
        while (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage, RxData) == HAL_OK)
        {
            CANOPEN_Rx_Process(RxMessage.StdId, RxData, RxMessage.DLC);
            osDelay(1);
        }
        //		vTaskDelayUntil(&tick,100);
        osDelay(2);
    }
}
