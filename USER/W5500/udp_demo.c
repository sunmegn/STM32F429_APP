/**
******************************************************************************
* @file   		udp_demo.c
* @author  		WIZnet Software Team 
* @version 		V1.0
* @date    		2015-02-14
* @brief   		UDP��ʾ����
******************************************************************************
**/
#include "udp_demo.h"

void udp_send(u8 *buff, u16 len) //udp ����
{
    sendto(SOCK_UDPS, buff, len, remote_ip, remote_port);
}
/**
*@brief		UDP���Գ���
*@param		��
*@return	��
*/
static uint16 len = 0;
static uint8  buff[1024];

void do_udp(void)
{

    switch (getSn_SR(SOCK_UDPS)) /*��ȡsocket��״̬*/
    {
    case SOCK_CLOSED:                                /*socket���ڹر�״̬*/
        socket(SOCK_UDPS, Sn_MR_UDP, local_port, 0); /*��ʼ��socket*/
        break;

    case SOCK_UDP: /*socket��ʼ�����*/
        if (getSn_IR(SOCK_UDPS) & Sn_IR_RECV)
        {
            setSn_IR(SOCK_UDPS, Sn_IR_RECV); /*������ж�*/
        }
        if ((len = getSn_RX_RSR(SOCK_UDPS)) > 7) /*���յ�����*/
        {
            memset(buff, 0, sizeof(buff));
            recvfrom(SOCK_UDPS, buff, len, remote_ip, &remote_port); /*W5500���ռ����������������*/
            UDP_ReceivedIRQ_Callback(buff, len - 8);
        }

        break;
    }
}
