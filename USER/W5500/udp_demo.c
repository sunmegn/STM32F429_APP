/**
******************************************************************************
* @file   		udp_demo.c
* @author  		WIZnet Software Team 
* @version 		V1.0
* @date    		2015-02-14
* @brief   		UDP演示函数
******************************************************************************
**/
#include "udp_demo.h"

void udp_send(u8 *buff, u16 len) //udp 发送
{
    sendto(SOCK_UDPS, buff, len, remote_ip, remote_port);
}
/**
*@brief		UDP测试程序
*@param		无
*@return	无
*/
static uint16 len = 0;
static uint8  buff[1024];

void do_udp(void)
{

    switch (getSn_SR(SOCK_UDPS)) /*获取socket的状态*/
    {
    case SOCK_CLOSED:                                /*socket处于关闭状态*/
        socket(SOCK_UDPS, Sn_MR_UDP, local_port, 0); /*初始化socket*/
        break;

    case SOCK_UDP: /*socket初始化完成*/
        if (getSn_IR(SOCK_UDPS) & Sn_IR_RECV)
        {
            setSn_IR(SOCK_UDPS, Sn_IR_RECV); /*清接收中断*/
        }
        if ((len = getSn_RX_RSR(SOCK_UDPS)) > 7) /*接收到数据*/
        {
            memset(buff, 0, sizeof(buff));
            recvfrom(SOCK_UDPS, buff, len, remote_ip, &remote_port); /*W5500接收计算机发送来的数据*/
            UDP_ReceivedIRQ_Callback(buff, len - 8);
        }

        break;
    }
}
