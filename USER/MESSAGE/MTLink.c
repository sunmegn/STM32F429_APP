/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2020-02-17 16:38:08
 * @LastEditors:Robosea
 * @LastEditTime:2020-02-23 18:05:17
 * @FilePath      :\ROV_F429_APP-10-10\USER\MESSAGE\MTLink.c
 * @brief         :
 */
#include "MTLink.h"
#include "math.h"
#include "messageTasks.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

/***********************��Ҫ�޸�*********************************/
//����Ŀ��ID�޸ķ��ͺ���
bool MTLink_SendBuffer(uint8_t DID, uint8_t *buf, int len)
{
    switch (DID)
    {
    case FREE_ID:
        break;
    case HOST_ID:
        udp_send(buf, len);
        break;
    default:
        break;
    }
    return true;
}

void SendToRespandsBuf(MTLink_typedef *m, uint8_t *buf, int len)
{
    memcpy(m->AskFIFO, buf, len);
    int pkglen = len;
    if (m == &MTLink_UDP)
    {                                                     //��UDP��MTLink
        xQueueSendToBack(MTLinkUDPAskQueue, &pkglen, 10); //������Ϣ
    }
}

bool MTLink_WaitRespands(MTLink_typedef *m, uint32_t waittime_ms)
{
    uint32_t Basetime = HAL_GetTick();
    uint32_t nowtime  = Basetime;
    bool     state    = false;
    int      len;
    if (m == &MTLink_UDP)
    { //��UDP��MTLink
        do
        {
            if (xQueueReceive(MTLinkUDPAskQueue, &len, waittime_ms / 5) == pdTRUE)
            {
                state = MTLink_RespandsDecode(m);
            }
            nowtime = HAL_GetTick();
        } while (state == false && (nowtime - Basetime) <= waittime_ms);
    }
    return state;
}
/********************************************************************************/

bool MTLink_RespandsDecode(MTLink_typedef *m)
{
    uint8_t *phead = m->AskFIFO, CRCL = 0xff, CRCH = 0xff, DID;
    uint32_t i, pkglen = 0, nowobj;
    bool     state = false;
    for (i = 0; i < (sizeof(m->AskFIFO) - 11); i++)
    { //��֡ͷ
        if (m->AskFIFO[i] == m->STX)
        {
            phead = m->AskFIFO + i;
            memcpy(&pkglen, phead + 1, 2);
            if ((i + pkglen + 12) <= sizeof(m->AskFIFO))
            {
                Pressure_CheckCRC(phead, pkglen + 10, &CRCH, &CRCL);
                if (CRCL == *(phead + pkglen + 10) && CRCH == *(phead + pkglen + 11))
                { //У��ͨ��
                    memcpy(&nowobj, phead + 8, 2);
                    DID = *(phead + 7);
                    if (*(phead + 3) == m->RPC && DID == m->SID && nowobj == m->Object)
                    { //˵����������Ӧ��
                        memset(phead, 0, pkglen + 12);
                        return true;
                    }
                    else if (DID != m->SID)
                    { //ת��
                        MTLink_SendBuffer(DID, phead, pkglen + 12);
                    }
                    memset(phead, 0, pkglen + 12);
                    i = i + pkglen + 12;
                }
            }
        }
    }
    return false;
}

bool MTLink_LongPackageFirstEncode(MTLink_typedef *m, uint8_t SID, uint8_t DID, uint8_t SorR, uint8_t RPC, uint16_t obj, uint8_t *buf, int len, uint32_t waittime_ms)
{
    uint8_t SL, ES, ACK;
    bool    state = false;
    SL            = 1; //��������
    ACK           = 1; //��ҪӦ��
    ES            = 1; //��һ��
    m->STX        = 0xFE;
    m->TxFIFO[0]  = m->STX;
    m->Len        = len;
    memcpy(&m->TxFIFO[1], &m->Len, 2);
    m->RPC       = RPC;
    m->TxFIFO[3] = m->RPC;
    m->PSTATE    = ((SL & 0X01) << 4) + ((ES & 0X03) << 2) + ((ACK & 0X01) << 1) + (SorR & 0x01);
    m->TxFIFO[4] = m->PSTATE;
    m->CNT++;
    m->TxFIFO[5] = m->CNT;
    m->SID       = SID;
    m->TxFIFO[6] = m->SID;
    m->DID       = DID;
    m->TxFIFO[7] = m->DID;
    m->Object    = obj;
    memcpy(&m->TxFIFO[8], &m->Object, 2);
    memcpy(&m->TxFIFO[10], buf, m->Len);
    m->PayLoad = &m->TxFIFO[10];
    Pressure_CheckCRC(m->TxFIFO, 10 + m->Len, &m->CRCH, &m->CRCL);
    m->TxFIFO[11 + m->Len - 1] = m->CRCL;
    m->TxFIFO[12 + m->Len - 1] = m->CRCH;
    unsigned int retry_count   = 0;
    do
    {
        MTLink_SendBuffer(m->DID, m->TxFIFO, m->Len + 12);
        m->CNT++;
        m->TxFIFO[5] = m->CNT;
        if (MTLink_WaitRespands(m, waittime_ms) == true)
        { //wait AsK timeout_ms
            return true;
        }
        else
        {
            state = false;
        }
        retry_count++;
    } while (state == false && retry_count < 3);
    return state;
}

bool MTLink_LongPackageNextEncode(MTLink_typedef *m, uint8_t SID, uint8_t DID, uint8_t SorR, uint8_t RPC, uint16_t obj, uint8_t *buf, int len, uint32_t waittime_ms)
{
    uint8_t SL, ES, ACK;
    bool    state = false;
    SL            = 1; //��������
    ACK           = 1; //��ҪӦ��
    ES            = 0; //��1+n��
    m->STX        = 0xFE;
    m->TxFIFO[0]  = m->STX;
    m->Len        = len;
    memcpy(&m->TxFIFO[1], &m->Len, 2);
    m->RPC       = RPC;
    m->TxFIFO[3] = m->RPC;
    m->PSTATE    = ((SL & 0X01) << 4) + ((ES & 0X03) << 2) + ((ACK & 0X01) << 1) + (SorR & 0x01);
    m->TxFIFO[4] = m->PSTATE;
    m->CNT++;
    m->TxFIFO[5] = m->CNT;
    m->SID       = SID;
    m->TxFIFO[6] = m->SID;
    m->DID       = DID;
    m->TxFIFO[7] = m->DID;
    m->Object    = obj;
    memcpy(&m->TxFIFO[8], &m->Object, 2);
    memcpy(&m->TxFIFO[10], buf, m->Len);
    m->PayLoad = &m->TxFIFO[10];
    Pressure_CheckCRC(m->TxFIFO, 10 + m->Len, &m->CRCH, &m->CRCL);
    m->TxFIFO[11 + m->Len - 1] = m->CRCL;
    m->TxFIFO[12 + m->Len - 1] = m->CRCH;
    unsigned int retry_count   = 0;
    do
    {
        MTLink_SendBuffer(m->DID, m->TxFIFO, m->Len + 12);
        m->CNT++;
        m->TxFIFO[5] = m->CNT;
        if (MTLink_WaitRespands(m, waittime_ms) == true)
        { //wait AsK timeout_ms
            return true;
        }
        else
        {
            state = false;
        }
        retry_count++;
    } while (state == false && retry_count < 3);
    return state;
}

bool MTLink_LongPackageEndEncode(MTLink_typedef *m, uint8_t SID, uint8_t DID, uint8_t SorR, uint8_t RPC, uint16_t obj, uint8_t *buf, int len, uint32_t waittime_ms)
{
    uint8_t SL, ES, ACK;
    bool    state = false;
    SL            = 1;    //��������
    ACK           = 1;    //��ҪӦ��
    ES            = 0x03; //���һ��
    m->STX        = 0xFE;
    m->TxFIFO[0]  = m->STX;
    m->Len        = len;
    memcpy(&m->TxFIFO[1], &m->Len, 2);
    m->RPC       = RPC;
    m->TxFIFO[3] = m->RPC;
    m->PSTATE    = ((SL & 0X01) << 4) + ((ES & 0X03) << 2) + ((ACK & 0X01) << 1) + (SorR & 0x01);
    m->TxFIFO[4] = m->PSTATE;
    m->CNT++;
    m->TxFIFO[5] = m->CNT;
    m->SID       = SID;
    m->TxFIFO[6] = m->SID;
    m->DID       = DID;
    m->TxFIFO[7] = m->DID;
    m->Object    = obj;
    memcpy(&m->TxFIFO[8], &m->Object, 2);
    memcpy(&m->TxFIFO[10], buf, m->Len);
    m->PayLoad = &m->TxFIFO[10];
    Pressure_CheckCRC(m->TxFIFO, 10 + m->Len, &m->CRCH, &m->CRCL);
    m->TxFIFO[11 + m->Len - 1] = m->CRCL;
    m->TxFIFO[12 + m->Len - 1] = m->CRCH;
    unsigned int retry_count   = 0;
    do
    {
        MTLink_SendBuffer(m->DID, m->TxFIFO, m->Len + 12);
        m->CNT++;
        m->TxFIFO[5] = m->CNT;
        if (MTLink_WaitRespands(m, waittime_ms) == true)
        { //wait AsK timeout_ms
            return true;
        }
        else
        {
            state = false;
        }
        retry_count++;
    } while (state == false && retry_count < 3);
    return state;
}

/**
 * @function_name:MTLink_LongPackageEncode
 * @brief:MTLinkЭ�鳤���������
 * @param *m
 * @param SID:ԴID
 * @param DID:Ŀ��ID
 * @param ASK:�Ƿ���ҪӦ��
 * @param SorR:1Ϊ�������ݣ�0Ϊ��ȡ����
 * @param obj:ָ���
 * @param *buf:���ݵ�ַ
 * @param len�����ݳ���
 * @param waittime_ms: �ȴ�ʱ��
 * @return:bool
 */
bool MTLink_LongPackageEncode(MTLink_typedef *m, uint8_t SID, uint8_t DID, uint8_t ASK, uint8_t SorR, uint16_t obj, uint8_t *buf, int len, uint32_t waittime_ms)
{
    bool     state  = false;
    uint32_t Nowlen = len, Send_Progress = 0;
    uint8_t  nowrpc = (len / MTLINKMAXPLEN) - 1; //ʣ�������
    if ((len / MTLINKMAXPLEN) > 255)
    {
        return false;
    }
    if ((len % MTLINKMAXPLEN) > 0)
    {
        nowrpc += 1;
    }
    /*send pkglength*******************************/
    if (MTLink_LongPackageFirstEncode(m, SID, DID, SorR, nowrpc, obj, buf + len - Nowlen, MTLINKMAXPLEN, waittime_ms) == false)
    {
        return false;
    }
    Nowlen -= MTLINKMAXPLEN;
    /*send buffer**********************************/
    while (Nowlen > MTLINKMAXPLEN)
    {
        nowrpc--;
        if (MTLink_LongPackageNextEncode(m, SID, DID, SorR, nowrpc, obj, buf + len - Nowlen, MTLINKMAXPLEN, waittime_ms) == false)
        {
            return false;
        }
        Send_Progress = (len - Nowlen) * 100 / len; //Downloading
        Nowlen -= MTLINKMAXPLEN;
        //		printf("Sending%d\r\n",Send_Progress);
    };
    /*send end buffer*****************************/
    nowrpc = 0;
    if (MTLink_LongPackageEndEncode(m, SID, DID, SorR, nowrpc, obj, buf + len - Nowlen, Nowlen, waittime_ms) == false)
    {
        return false;
    }
    Send_Progress = 100;
    //	printf("Sending%d\r\n",Send_Progress);
    return true;
}

/**
 * @function_name:MTLink_ShortPackageEncode
 * @brief:MTLinkЭ��̰��������
 * @param *m
 * @param SID:ԴID
 * @param DID:Ŀ��ID
 * @param ASK:�Ƿ���ҪӦ��
 * @param SorR:1Ϊ�������ݣ�0Ϊ��ȡ����
 * @param obj:ָ���
 * @param *buf:���ݵ�ַ
 * @param len�����ݳ���
 * @param waittime_ms: �ȴ�ʱ��
 * @return:bool
 */
bool MTLink_ShortPackageEncode(MTLink_typedef *m, uint8_t SID, uint8_t DID, uint8_t ASK, uint8_t SorR, uint16_t obj, uint8_t *buf, int len, uint32_t waittime_ms)
{
    uint8_t SL, ES, ACK;
    bool    state = false;
    SL            = 0;   //�̰�����
    ACK           = ASK; //��ҪӦ��
    ES            = 0;   //��1+n��
    m->STX        = 0xFE;
    m->TxFIFO[0]  = m->STX;
    m->Len        = len;
    memcpy(&m->TxFIFO[1], &m->Len, 2);
    m->RPC       = 0; //�̰�
    m->TxFIFO[3] = m->RPC;
    m->PSTATE    = ((SL & 0X01) << 4) + ((ES & 0X03) << 2) + ((ACK & 0X01) << 1) + (SorR & 0x01);
    m->TxFIFO[4] = m->PSTATE;
    m->CNT++;
    m->TxFIFO[5] = m->CNT;
    m->SID       = SID;
    m->TxFIFO[6] = m->SID;
    m->DID       = DID;
    m->TxFIFO[7] = m->DID;
    m->Object    = obj;
    memcpy(&m->TxFIFO[8], &m->Object, 2);
    memcpy(&m->TxFIFO[10], buf, m->Len);
    m->PayLoad = &m->TxFIFO[10];
    Pressure_CheckCRC(m->TxFIFO, 10 + m->Len, &m->CRCH, &m->CRCL);
    m->TxFIFO[11 + m->Len - 1] = m->CRCL;
    m->TxFIFO[12 + m->Len - 1] = m->CRCH;
    unsigned int retry_count   = 0;
    if (ACK == 1)
    {
        do
        {
            MTLink_SendBuffer(m->DID, m->TxFIFO, m->Len + 12);
            m->CNT++;
            m->TxFIFO[5] = m->CNT;
            if (MTLink_WaitRespands(m, waittime_ms) == true)
            { //wait AsK timeout_ms
                return true;
            }
            else
            {
                state = false;
            }
            retry_count++;
        } while (state == false && retry_count < 3);
    }
    else
    {
        MTLink_SendBuffer(m->DID, m->TxFIFO, m->Len + 12);
        return true;
    }
    return state;
}

/**
 * @function_name:MTLink_Encode
 * @brief:MTLink���ݷ������
 * @param  *m:���ݵ�ַ
 * @return:bool
 */
bool MTLink_Encode(MTLink_typedef *m, uint8_t SID, uint8_t DID, uint8_t ASK, uint16_t obj, uint8_t *buf, int len, uint32_t waittime_ms)
{
    if (len > MTLINKMAXPLEN * 255)
    {
        return false;
    }
    if (len > MTLINKMAXPLEN)
    {
        return MTLink_LongPackageEncode(m, SID, DID, 1, 1, obj, buf, len, waittime_ms);
    }
    else
    {
        return MTLink_ShortPackageEncode(m, SID, DID, ASK, 1, obj, buf, len, waittime_ms);
    }
    return false;
}

bool MTLink_Ack(MTLink_typedef *m, uint8_t RPC, uint16_t obj, uint8_t SID, uint8_t DID)
{
    uint8_t SL, ES, ACK;
    bool    state = false;
    SL            = 0; //�̰�����
    ACK           = 0; //
    ES            = 0; //��1+n��
    m->STX        = 0xFE;
    m->TxFIFO[0]  = m->STX;
    m->Len        = 0;
    memcpy(&m->TxFIFO[1], &m->Len, 2);
    m->RPC       = RPC; //�̰�
    m->TxFIFO[3] = m->RPC;
    m->PSTATE    = 0x80 + ((SL & 0X01) << 4) + ((ES & 0X03) << 2) + ((ACK & 0X01) << 1) + (1 & 0x01);
    m->TxFIFO[4] = m->PSTATE;
    m->CNT++;
    m->TxFIFO[5] = m->CNT;
    m->SID       = SID;
    m->TxFIFO[6] = m->SID;
    m->DID       = DID;
    m->TxFIFO[7] = m->DID;
    m->Object    = obj;
    memcpy(&m->TxFIFO[8], &m->Object, 2);
    m->PayLoad = NULL;
    Pressure_CheckCRC(m->TxFIFO, 10 + m->Len, &m->CRCH, &m->CRCL);
    m->TxFIFO[11 + m->Len - 1] = m->CRCL;
    m->TxFIFO[12 + m->Len - 1] = m->CRCH;
    unsigned int retry_count   = 0;
    MTLink_SendBuffer(m->DID, m->TxFIFO, m->Len + 12);
    return true;
}

/**
 * @function_name:MTLink_LongPackageRx
 * @brief:MTLink��������,��MTLink_Decode����
 * @param *m:����
 * @param SID:Դ��ַ
 * @param obj:����
 * @param RPC:ʣ�����
 * @param ES:���ݰ�״̬��1Ϊ��һ����0Ϊ��������е����ݰ���3Ϊ���һ��
 * @param *buf:���ݵ�ַ
 * @param len:����
 * @return:None
 */
bool MTLink_LongPackageRx(MTLink_typedef *m, uint8_t SID, uint16_t obj, uint8_t RPC, uint8_t ES, uint8_t *buf, int len)
{
    static bool     NowPKGState = false;
    static uint8_t *Longbufp, nowSID, Nextrpc;
    static uint16_t Nowobj;
    static uint32_t NowLen;
    uint8_t *       p0 = m->Decodebuf;
    switch (ES)
    {
    case 1:                      //��һ��
        Longbufp = m->Decodebuf; //����
        memcpy(Longbufp, buf, len);
        Longbufp += len;
        Nextrpc     = RPC - 1;
        NowLen      = len;
        Nowobj      = obj;
        nowSID      = SID;
        NowPKGState = true;
        break;
    case 0: //���̰�
        if (Nextrpc == RPC && nowSID == SID && Nowobj == obj && NowPKGState == true)
        {
            NowLen += len;
            memcpy(Longbufp, buf, len);
            Longbufp += len;
            Nextrpc--;
        }
        else
        {
            NowPKGState = false;
        }
        break;
    case 3: //���һ��
        if (NowPKGState == true)
        {
            NowLen += len;
            memcpy(Longbufp, buf, len);
            MTLinkDispose(SID, obj, p0, NowLen);
            NowPKGState = false;
        }
        break;
    default:
        NowPKGState = false;
        break;
    }
    return true;
}

/**
 * @function_name:MTLink_Decode
 * @brief:MTLink���ݽ������
 * @param  *m:���ݵ�ַ
 * @return:bool
 */
bool MTLink_Decode(MTLink_typedef *m)
{
    uint8_t *phead = m->RxFIFO, CRCL = 0xff, CRCH = 0xff, DID, SID, ACK = 0, SorR = 0, SL = 0, ES = 0, RPC, Res = 0;
    uint16_t i, nowobj;
    uint32_t pkglen = 0;
    bool     state  = false;
    for (i = 0; i < (m->RxFIFOp - m->RxFIFO - 11); i++)
    { //��֡ͷ
        if (*(m->RxFIFO + i) == 0xfe)
        {
            phead = m->RxFIFO + i;
            memcpy(&pkglen, phead + 1, 2);
            if ((i + pkglen + 12) <= sizeof(m->RxFIFO))
            {
                Pressure_CheckCRC(phead, pkglen + 10, &CRCH, &CRCL);
                if (CRCL == *(phead + pkglen + 10) && CRCH == *(phead + pkglen + 11))
                { //У��ͨ��
                    memcpy(&nowobj, phead + 8, 2);
                    DID  = *(phead + 7);
                    SID  = *(phead + 6);
                    ACK  = (*(phead + 4) >> 1) & 0X01;
                    SL   = (*(phead + 4) >> 4) & 0X01;
                    ES   = (*(phead + 4) >> 2) & 0X03;
                    SorR = (*(phead + 4)) & 0X01;
                    Res  = (*(phead + 4) >> 7) & 0X01;
                    RPC  = *(phead + 3);
                    if (DID == My_ID || DID == 0xff) //�������λ�������������������н��պͽ��룬�������ת��
                    {                                //˵����������
                        if (Res == 1)
                        {
                            SendToRespandsBuf(m, phead, 12 + pkglen); //ת����ASKbuf
                        }
                        if (ACK == 1)
                        { //��Ҫ�ظ�
                            MTLink_Ack(m, RPC, nowobj, My_ID, SID);
                        }
                        if (SL == 1)
                        { //��������
                            MTLink_LongPackageRx(m, SID, nowobj, RPC, ES, phead + 10, pkglen);
                        }
                        else
                        {
                            MTLinkDispose(SID, nowobj, phead + 10, pkglen); //����
                        }
                    }
                    else
                    { //ת��
                        MTLink_SendBuffer(DID, phead, pkglen + 12);
                    }
                    state = true;
                    memset(phead, 0, pkglen + 12);
                    i = i + pkglen + 11;
                }
            }
        }
    }
    if (state == true)
    {
        m->RxFIFOp = m->RxFIFO;
    }
    return state;
}

/**
 * @function_name:Pressure_CheckCRC
 * @brief:CRC���
 * @param {type} 
 * @return:None
 */
void Pressure_CheckCRC(uint8_t *buf, int len, uint8_t *CRC_H, uint8_t *CRC_L)
{
    uint16_t i, j, tmp, CRC16;
    CRC16 = 0xffff;
    for (i = 0; i < len; i++)
    {
        CRC16 = *buf ^ CRC16;
        for (j = 0; j < 8; j++)
        {
            tmp   = CRC16 & 0x0001;
            CRC16 = CRC16 >> 1;
            if (tmp)
                CRC16 = CRC16 ^ 0xA001;
        }
        buf++;
    }
    CRC_H[0] = CRC16 >> 8;
    CRC_L[0] = CRC16 & 0xff;
}

/**
 * @function_name:MTLinkFIFO_append
 * @brief:MTLink����
 * @param {type} 
 * @return:None
 */
void MTLinkFIFO_append(MTLink_typedef *m, uint8_t *pushbuf, int pushlen)
{
    if (m->RxFIFOp == NULL)
    { //��һ��
        m->RxFIFOp = m->RxFIFO;
    }
    int nowlen = (m->RxFIFOp - m->RxFIFO);
    if ((nowlen + pushlen) > sizeof(m->RxFIFO))
    {
        memset(m->RxFIFO, 0, sizeof(m->RxFIFO)); //����������
        m->RxFIFOp = m->RxFIFO;
        nowlen     = 0;
        if (pushlen > sizeof(m->RxFIFO))
        {
            memcpy(m->RxFIFOp, pushbuf + (pushlen - sizeof(m->RxFIFO)), sizeof(m->RxFIFO));
            m->RxFIFOp += sizeof(m->RxFIFO);
        }
    }
    memcpy(m->RxFIFOp, pushbuf, pushlen);
    m->RxFIFOp += pushlen;
}

void MTLinkPrint(MTLink_typedef *m, uint8_t SID, uint8_t DID, char *str, int len, uint32_t timeout)
{
    MTLink_Encode(&MTLink_UDP, SID, DID, 0, 0x0000, (uint8_t *)str, len, timeout);
}