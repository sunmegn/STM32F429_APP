#include "w5500_net.h"

void W5500_Init(void)
{
	//gpio_for_w5500_config();						/*��ʼ��MCU�������*/
  reset_w5500();                     /* W5500Ӳ����λ */
	set_w5500_mac();										/*����MAC��ַ*/
	set_w5500_ip();											/*����IP��ַ*/
	
	
	socket_buf_init(txsize, rxsize);		/*��ʼ��8��Socket�ķ��ͽ��ջ����С*/
	
  //printf(" W5500���Ժ͵��Ե�UDP�˿�ͨѶ \n");
	//printf(" W5500�ı��ض˿�Ϊ:%d \n",local_port);
	//printf(" Զ�˶˿�Ϊ:%d \n",remote_port);
	//printf(" ���ӳɹ���PC���������ݸ�W5500��W5500�����ض�Ӧ���� \n");
	//����ʹ��
//		while(1) 														/*ѭ��ִ�еĺ���*/ 
//		{
//	    do_udp();                         /*UDP ���ݻػ�����*/
//		}
//		
}
void W5500_Test_Demo(void)
{
	//gpio_for_w5500_config();						/*��ʼ��MCU�������*/
  reset_w5500();                     /* W5500Ӳ����λ */
	set_w5500_mac();										/*����MAC��ַ*/
	set_w5500_ip();											/*����IP��ַ*/
	
	
	socket_buf_init(txsize, rxsize);		/*��ʼ��8��Socket�ķ��ͽ��ջ����С*/
	
  //printf(" W5500���Ժ͵��Ե�UDP�˿�ͨѶ \n");
	//printf(" W5500�ı��ض˿�Ϊ:%d \n",local_port);
	//printf(" Զ�˶˿�Ϊ:%d \n",remote_port);
	//printf(" ���ӳɹ���PC���������ݸ�W5500��W5500�����ض�Ӧ���� \n");
	//����ʹ��
		while(1) 														/*ѭ��ִ�еĺ���*/ 
		{
	    do_udp();                         /*UDP ���ݻػ�����*/
		}
			
	
	
}
