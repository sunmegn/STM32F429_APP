#ifndef _TYPE_H_
#define _TYPE_H_

#include <string.h>
#include <stdio.h>

#include "main.h"
#include "spi.h"
//#define W5500_SPI     		              hspi1
//#define W5500_SPI_CS_Pin                W5500_SPI3_CS_Pin
//#define W5500_SPI_CS_GPIO_Port          W5500_SPI3_CS_GPIO_Port
//#define W5500_SPI_SCK_Pin               W5500_SPI_SCK_Pin
//#define W5500_SPI_SCK_GPIO_Port         W5500_SPI_SCK_GPIO_Port
//#define W5500_SPI_MISO_Pin              W5500_SPI_MISO_Pin
//#define W5500_SPI_MISO_GPIO_Port        W5500_SPI_MISO_GPIO_Port
//#define W5500_SPI_MOSI_Pin              W5500_SPI_MOSI_Pin
//#define W5500_SPI_MOSI_GPIO_Port        W5500_SPI_MOSI_GPIO_Port

#define	MAX_SOCK_NUM		8	/**< Maxmium number of socket  */

typedef char int8;

typedef volatile char vint8;

typedef unsigned char uint8;

typedef volatile unsigned char vuint8;

typedef int int16;

typedef unsigned short uint16;

typedef long int32;

typedef unsigned long uint32;

typedef uint8			u_char;		/**< 8-bit value */
typedef uint8 			SOCKET;
typedef uint16			u_short;	/**< 16-bit value */
typedef uint16			u_int;		/**< 16-bit value */
typedef uint32			u_long;		/**< 32-bit value */

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

typedef union _un_l2cval 
{
	u_long	lVal;
	u_char	cVal[4];
}un_l2cval;

typedef union _un_i2cval 
{
	u_int	iVal;
	u_char	cVal[2];
}un_i2cval;

#endif		/* _TYPE_H_ */
