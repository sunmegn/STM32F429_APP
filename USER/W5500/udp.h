#ifndef __UDP_H_
#define __UDP_H_

#include "main.h"
#include "spi.h"


#define W5500_SPI     	hspi1
#define W5500_CS(x)		(x?HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET):HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET))
#define W5500_RESET(x)  (x?HAL_GPIO_WritePin(W5500_RESET_GPIO_Port, W5500_RESET_Pin, GPIO_PIN_SET):HAL_GPIO_WritePin(W5500_RESET_GPIO_Port, W5500_RESET_Pin, GPIO_PIN_RESET))

void udp_send(uint8_t *buf, uint16_t len);
void do_udp(void);
void Udp_Init(void);
#endif 


