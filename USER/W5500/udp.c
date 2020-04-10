#include "udp.h"

#include "wizchip_conf.h"
#include "socket.h"

#include "messageTasks.h"

#define SPI_I2S_FLAG_RXNE               ((uint16_t)0x0001)
#define SPI_I2S_FLAG_TXE                ((uint16_t)0x0002)

uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
uint8_t DstIP[4]={192,168,1,255};
uint16_t	DstPort=8000;
uint16_t	SrcPort=5001;
#define SOCK_TCPS        0
#define DATA_BUF_SIZE   1500

/* Private macro -------------------------------------------------------------*/
uint8_t gDATABUF[DATA_BUF_SIZE];

// Default Network Configuration
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc,0x11, 0x11, 0x11},
                            .ip = {192, 168, 1, 10},
                            .sn = {255,255,255,0},
                            .gw = {192, 168, 1, 1},
                            .dns = {8,8,8,8},
                            .dhcp = NETINFO_STATIC };

/**
  * @brief  �����ٽ���
  * @retval None
  */
void SPI_CrisEnter(void)
{
	__set_PRIMASK(1);
}

/**
  * @brief  �˳��ٽ���
  * @retval None
  */
void SPI_CrisExit(void)
{
	__set_PRIMASK(0);
}

/**
  * @brief  Ƭѡ�ź�����͵�ƽ
  * @retval None
  */
void SPI_CS_Select(void)
{
	W5500_CS(0);
}

/**
  * @brief  Ƭѡ�ź�����ߵ�ƽ
  * @retval None
  */
void SPI_CS_Deselect(void)
{
	W5500_CS(1);
}

/**
  * @funNm  
  * @brief   
  * @param	
  * @retval 
*/  
uint8_t SPI3_SendByte(uint8_t byte)
{
	uint8_t d_read,d_send=byte;
	
	if(HAL_SPI_TransmitReceive(&W5500_SPI,&d_send,&d_read,1,10)!=HAL_OK)//0xFFFFFF
		d_read=0XFF;
  
	return d_read; 
}

/**
  * @funNm  
  * @brief   
  * @param	
  * @retval 
*/  
uint8_t SPI_ReadByte(void)
{
	return SPI3_SendByte(0x00);
}

/**
  * @funNm  
  * @brief   
  * @param	
  * @retval 
*/  
void SPI_WriteByte(uint8_t TxData)
{
	SPI3_SendByte(TxData);	
}

/**
  * @funNm  
  * @brief   
  * @param	
  * @retval 
*/  
void network_init(void)
{	
	uint8_t tmpstr[6];
	
	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
	ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO);
	ctlwizchip(CW_GET_ID,(void*)tmpstr);
}

/**
  * @funNm  
  * @brief   
  * @param	
  * @retval 
*/   
void Udp_Init(void)
{
	uint8_t tmp;
	
	W5500_RESET(0);
	HAL_Delay(800);
	
	W5500_RESET(1);
	HAL_Delay(800);
	
	reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit);	//ע���ٽ�������
	reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);//ע��SPIƬѡ�źź���
	reg_wizchip_spi_cbfunc(SPI_ReadByte, SPI_WriteByte);	//ע���д����
	
	/* WIZCHIP SOCKET Buffer initialize */
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1)
	{
//		 printf("WIZCHIP Initialized fail.\r\n");
		 while(1);
	}

	/* PHY link status check */
	do{
		if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1)
		{
//			printf("Unknown PHY Link stauts.\r\n");
		}
	}while(tmp == PHY_LINK_OFF);

	/* Network initialization */
	network_init();
}

/**
  * @funNm  
  * @brief   
  * @param	
  * @retval 
*/  
void do_udp(void)
{
	int16_t ret = 0;
	uint16_t dstport = 0;
	uint8_t dstip[4]={192,168,1,255};
	
	switch(getSn_SR(0))																						// ��ȡsocket0��״̬
	{
		case SOCK_UDP:	                                                                                // Socket���ڳ�ʼ�����(��)״̬
			
			osDelay(5);
			if(getSn_IR(0) & Sn_IR_RECV)
			{
				setSn_IR(0, Sn_IR_RECV);															    // Sn_IR��RECVλ��1
			}
			
			// ���ݻػ����Գ������ݴ�Զ����λ������W5500��W5500���յ����ݺ��ٻظ�Զ����λ��
			if((ret=getSn_RX_RSR(0))>0)
			{ 
				memset(gDATABUF,0,ret+1);
				recvfrom(0,gDATABUF, ret, dstip,&dstport);			                                     // W5500��������Զ����λ�������ݣ���ͨ��SPI���͸�MCU
				UDP_ReceivedIRQ_Callback(gDATABUF, ret-8);		
				
			//	sendto(0,gDATABUF,ret-8, DstIP, DstPort);		  		                          		// ���յ����ݺ��ٻظ�Զ����λ����������ݻػ�
			}
			break;
			
		case SOCK_CLOSED:	                                                                            // Socket���ڹر�״̬
			
			socket(0,Sn_MR_UDP,SrcPort,0x00);												           // ��Socket0��������ΪUDPģʽ����һ�����ض˿�
			break;
	}
}

/**
  * @funNm  
  * @brief   
  * @param	
  * @retval 
*/  
void udp_send(uint8_t *buf, uint16_t len)
{
	sendto(0,buf,len, DstIP, DstPort);
}

/**
  * @funNm  
  * @brief   
  * @param	
  * @retval 
*/  
int fputc(int ch,FILE *f)
{
	udp_send((uint8_t*)&ch, 1);
	return ch;
}







