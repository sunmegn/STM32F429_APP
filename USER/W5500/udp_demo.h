#ifndef __UDP_DEMO_H
#define __UDP_DEMO_H

#include "includes.h"

extern uint16 udp_port;/*����UDP��һ���˿ڲ���ʼ��*/
void udp_send(u8 *buff, u16 len);//udp ����
void do_udp(void);
#endif 


