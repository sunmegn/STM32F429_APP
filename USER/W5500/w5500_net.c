#include "w5500_net.h"

void W5500_Init(void)
{
	//gpio_for_w5500_config();						/*初始化MCU相关引脚*/
  reset_w5500();                     /* W5500硬件复位 */
	set_w5500_mac();										/*配置MAC地址*/
	set_w5500_ip();											/*配置IP地址*/
	
	
	socket_buf_init(txsize, rxsize);		/*初始化8个Socket的发送接收缓存大小*/
	
  //printf(" W5500可以和电脑的UDP端口通讯 \n");
	//printf(" W5500的本地端口为:%d \n",local_port);
	//printf(" 远端端口为:%d \n",remote_port);
	//printf(" 连接成功后，PC机发送数据给W5500，W5500将返回对应数据 \n");
	//测试使用
//		while(1) 														/*循环执行的函数*/ 
//		{
//	    do_udp();                         /*UDP 数据回环测试*/
//		}
//		
}
void W5500_Test_Demo(void)
{
	//gpio_for_w5500_config();						/*初始化MCU相关引脚*/
  reset_w5500();                     /* W5500硬件复位 */
	set_w5500_mac();										/*配置MAC地址*/
	set_w5500_ip();											/*配置IP地址*/
	
	
	socket_buf_init(txsize, rxsize);		/*初始化8个Socket的发送接收缓存大小*/
	
  //printf(" W5500可以和电脑的UDP端口通讯 \n");
	//printf(" W5500的本地端口为:%d \n",local_port);
	//printf(" 远端端口为:%d \n",remote_port);
	//printf(" 连接成功后，PC机发送数据给W5500，W5500将返回对应数据 \n");
	//测试使用
		while(1) 														/*循环执行的函数*/ 
		{
	    do_udp();                         /*UDP 数据回环测试*/
		}
			
	
	
}
