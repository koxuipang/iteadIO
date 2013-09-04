/*************************************************************************
	> File Name: io.h
	> Author: Jyzhiyu
	> Mail: jyzhiyu@gmail.com 
	> Created Time: 2013年08月13日 星期二 13时00分57秒
 ************************************************************************/

#ifndef IO

#define IO

#include <regmap.h>
#include <pinmap.h>

// for gpio_8_bit_bus
typedef struct{
	unsigned char D0;
	unsigned char D1;
	unsigned char D2;
	unsigned char D3;
	unsigned char D4;
	unsigned char D5;
	unsigned char D6;
	unsigned char D7;
}STRUCT_8BITS_BUS;

// for gpio_16_bit_bus
typedef struct{
	unsigned char D0;
	unsigned char D1;
	unsigned char D2;
	unsigned char D3;
	unsigned char D4;
	unsigned char D5;
	unsigned char D6;
	unsigned char D7;
	unsigned char D8;
	unsigned char D9;
	unsigned char D10;
	unsigned char D11;
	unsigned char D12;
	unsigned char D13;
	unsigned char D14;
	unsigned char D15;
}STRUCT_16BITS_BUS;


#define INPUT 			0
#define OUTPUT 			1
#define	PWM_OUTPUT		2
#define	PULLUP		 	3
#define	PULLDOWN		4
#define	PULLOFF			5
#define	CHECK		 	6
#define	GPIO_CLOCK		7

#define UART0 			10
#define UART2 			11
#define UART3 			12
#define UART4			13
#define UART7 			14

#define _A10_GPIO_BASE   	(0x01C20800)
#define _A10_PWM_BASE    	(0x01C20e00)
#define _GPIO_BASE		(0x01C20000)
#define _GPIO_PWM		(0x01c20000)

#define _LOW			0 	
#define _HIGH			1
#define fd_nu 			1
#define speed_nu 		2
#define SPI0 			0
#define SPI1			1

#define	_MAX_PINS		144

#define I2C_BLOCK_MAX     	32
#define I2C_READ          	1
#define I2C_WRITE         	0
#define I2C_BYTE_DATA     	2
#define I2C_WORD_DATA	  	3
#define I2C1              	1
#define I2C2              	2

#define _MMAP_SIZE		4096
#define _MMAP_MASK		(_MMAP_SIZE - 1)
#define __BLOCK_SIZE 		512

/**************************for digital and time function***************************************/
extern int 					pinMode(int pin, int mode);
extern int 					digitalWrite(int pin,int value);
extern int 					digitalRead	(int pin);
extern void         		delay(unsigned int howLong) ;
extern void         		delayMicroseconds(unsigned int howLong) ;
extern unsigned int 		millis(void) ;
extern unsigned int 		micros(void) ;

/*************************for 8/16 bit BUS**************************************************/ 
extern int 					Set8BitsBUS(STRUCT_8BITS_BUS *bus,char d0,char d1,char d2,char d3,char d4,char d5,char d6,char d7,char mode);
extern int 					digitalWrite8(STRUCT_8BITS_BUS *bus,char data);
extern unsigned char 		digitalRead8(STRUCT_8BITS_BUS *bus);
extern int 					Set16BitsBUS(STRUCT_16BITS_BUS *bus,char d0,char d1,char d2,char d3, char d4,char d5,char d6,char d7,char d8,char d9,char d10,char d11,char d12,char d13,char d14,char d15,char mode);
extern int 					digitalWrite16(STRUCT_16BITS_BUS *bus,short int data);
extern unsigned short int 	digitalRead16(STRUCT_16BITS_BUS *bus);

/*************************for serial bus****************************************************/
extern int 	 				SerialBegin(int part_data, int baud);
extern int   				SerialEnd(int fd);
extern int   				SerialFlush(int fd);
extern int   				SerialWrite(int fd, unsigned char c);
extern int   				SerialPrint(int fd, char *s);
extern int   				SerialPrintln(int fd, char *s);
extern int   				SerialAvailable(int fd);
extern int   				SerialRead(int fd);

/*************************for I2c bus******************************************************/
extern char 				WireRead8(int device,char dev_offset);
extern short int 			WireRead16(int device,short int dev_offset);
extern int 					WireWrite8(int device, char dev_offset,unsigned char volue);
extern int 					WireWrite16(int device, short int dev_offset,short int volue);
extern int	 				WireBeginTransmission(int device, unsigned char dev_addr);
extern int 					WireBegin(int device);
extern int 					WireEndTransmission(int device);
/*************************for SPI bus******************************************************/
extern int 					SPIBegin(int device);
extern int 					SetBitOrder(int device,int LH_mode);
extern int 					SetDataMode(int device, int RT_mode);
extern int 					SetClockDivider(int device,int DRI);
extern char 				SPITransfer(int device,char data);
extern int 			 		SPIEnd(int device);

#endif


