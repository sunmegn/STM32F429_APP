#ifndef __UDP_DEMO_H
#define __UDP_DEMO_H

#include "includes.h"

extern uint16 udp_port;/*定义UDP的一个端口并初始化*/
void udp_send(u8 *buff, u16 len);//udp 发送
void do_udp(void);
#endif 


