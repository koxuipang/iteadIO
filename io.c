/*************************************************************************
> File Name: io.c
> Author: jyzhiyu
> Mail: jyzhiyu@gmail.com 
> Created Time: 2013骞?8鏈?0鏃?鏄熸湡鍏?17鏃?0鍒?2绉?
************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>

//#define CUBIEBOARD
#define ITEADPLUS

#include <io.h>
#include <pinmap.h>

#define _A10_GPIO_BASE   	(0x01C20800)
#define _A10_PWM_BASE    	(0x01C20e00)
#define _GPIO_BASE			(0x01C20000)
#define _GPIO_PWM			(0x01c20000)

#define _LOW				0 	
#define _HIGH				1
#define fd_nu 				1
#define speed_nu 			2
#define SPI0 				0
#define SPI1				1

#define	_MAX_PINS			144

#define I2C_BLOCK_MAX     	32
#define I2C_READ          	1
#define I2C_WRITE         	0
#define I2C_BYTE_DATA     	2
#define I2C_WORD_DATA	  	3
#define I2C1              	1
#define I2C2              	2

#define _MMAP_SIZE			4096
#define _MMAP_MASK			(_MMAP_SIZE - 1)
#define __BLOCK_SIZE 		512

static int g_i2c_fd_offset_array[2][3]={0};

static int marks[_MAX_PINS];
static int range[_MAX_PINS];

static volatile uint32_t *_REG_TIMER;
static volatile uint32_t *_REG_GPIO;
static volatile uint32_t *_REG_CLK;
static volatile uint32_t *_REG_PWM;
static volatile uint32_t *_gpio;
static volatile uint32_t *_pwm;

static uint64_t epochMilli, epochMicro ;
static int 	g_pin_mode = -1;
static int 	g_soft_pwm_pin=0;
int 		g_fd_array[6]={};
static int 	g_SPI_data_array[3][2]={};

union i2c_data
{
        uint8_t  byte;
        uint16_t word;
        uint8_t  block[I2C_BLOCK_MAX + 2];
};
struct i2c_ioctl_data
{
        char read_write;
        uint8_t command;
        int size;
        union i2c_data *data;
};
void 	*pwm_thread(void *argk);
int 	set_high_pri (int pri);
/*******************************************************************************
*>创建人 :庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 寄存器级别操作函数
*>备注: 
********************************************************************************/

uint32_t *get_memory_addr(uint32_t phy_addr)
{
	uint32_t *_phy_to_memory;
	uint32_t fd;
	
	fd = open ("/dev/mem", O_RDWR | O_SYNC);
	if(fd == -1)
	{
		printf("get_memory is failed\n");
	}
	_phy_to_memory = (uint32_t *)mmap(0, __BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, phy_addr);
	return _phy_to_memory;
}
uint32_t Readreg32(uint32_t _addr)
{
	static volatile uint32_t *_addr_target;
	int _fd = 0;
	uint32_t _value = 0;


	_fd = open("/dev/mem", O_RDWR|O_NDELAY);
	if(-1 == _fd)
	{
		printf("Open /dev/mem failed!\n");
		return -1;
	}
	
	uint32_t _mmap_base = (_addr & ~_MMAP_MASK);
	uint32_t _mmap_seek = ((_addr - _mmap_base)>>2);

	_addr_target = (uint32_t *)mmap(NULL, _MMAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, _mmap_base);

	if(0 == _addr_target)
	{
		close(_fd);
		printf("mmap failed:fd(%d),addr(0x%x),size(%d)\n", _fd, _addr, _MMAP_SIZE);
		return -1;
	}
	else
	{
		_value =*(_addr_target + _mmap_seek);
        #ifdef CORE_DEBUG
            printf("Readreg32: Addr = 0x%x, Value = 0x%x", (_addr_target + _mmap_seek), _value);
        #endif
		close(_fd);
		munmap((uint32_t *)_addr_target, _MMAP_SIZE);
		return (_value);
	}
}

uint32_t Writereg32(uint32_t _addr, uint32_t _value)

{
	static volatile uint32_t *_addr_target;
	int _fd = 0;


	_fd = open("/dev/mem", O_RDWR|O_NDELAY);
	
	if(-1 == _fd)
	{
		printf("Open /dev/mem failed!\n");
		return -1;
	}
	
	uint32_t _mmap_base = (_addr & ~_MMAP_MASK);
	uint32_t _mmap_seek = ((_addr - _mmap_base)>>2);

	_addr_target = (uint32_t *)mmap(NULL, _MMAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, _mmap_base);

	if(0 == _addr_target)
	{
		close(_fd);
		printf("mmap failed:fd(%d),addr(0x%x),size(%d)\n", _fd, _addr, _MMAP_SIZE);
		return -1;
	}
	else
	{
		*(_addr_target + _mmap_seek) = _value;
        #ifdef CORE_DEBUG
            printf("Writereg32: Addr = 0x%x, Value = 0x%x", (_addr_target + _mmap_seek), *(_addr_target + _mmap_seek));
        #endif
		close(_fd);
		munmap((uint32_t *)_addr_target, _MMAP_SIZE);
		return (_value);
	}
}

/*******************************************************************************
*>创建人 :庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述:设置管脚模式
*>备注: 模式: INPUT、OUTPUT、PWM_OUTPUT、PULLUP、PULLDOWN、PULLOFF、CHECK
********************************************************************************/
int pinMode(int _pin, int _mode)
{
	int fd,_pin_save_phy;

	_pin_save_phy = _pin;

	if(g_soft_pwm_pin == _pin)
		g_soft_pwm_pin = -1;
	
	_pin = _GPIO_MAP[_pin];

	if (-1 == _pin)
	{
		printf("ERR: This _pin is not availbale!\n");
		return -1;
	}

	_gpio 	= get_memory_addr(_GPIO_BASE);
	_gpio 	+= 0x200;
	
	_pwm 	= get_memory_addr(_GPIO_PWM);
	_pwm 	+= 0x380;

	switch(_mode)
	{
		case INPUT:
		{
			int _Bank 	= _pin >> 5;
			int _Index 	= _pin - (_Bank << 5);
			int _Offset = ((_Index - ((_Index >> 3) << 3)) << 2);
			uint32_t _Phyaddr 	= _A10_GPIO_BASE + (_Bank * 36) + ((_Index >> 3) << 2);
			uint32_t _RegValue 	= 0;

			_RegValue = Readreg32(_Phyaddr);
	
			_RegValue &= ~(7<<_Offset);
			Writereg32(_Phyaddr, _RegValue);
			_RegValue = Readreg32(_Phyaddr);

			g_pin_mode = INPUT;
			break;
		}
		case OUTPUT:
		{
			int _Bank 	= _pin >> 5;
			int _Index 	= _pin - (_Bank << 5);
			int _Offset = ((_Index - ((_Index >> 3) << 3)) << 2);
			uint32_t _Phyaddr 	= _A10_GPIO_BASE + (_Bank * 36) + ((_Index >> 3) << 2);
			uint32_t _RegValue 	= 0;

			_RegValue = Readreg32(_Phyaddr);
			
			_RegValue &= ~(7 << _Offset);
			_RegValue |= (1 << _Offset);
			Writereg32(_Phyaddr,_RegValue);
			_RegValue = Readreg32(_Phyaddr);

			g_pin_mode = OUTPUT;
			break;
		} 
		case PWM_OUTPUT:
		{
			pinMode(_pin_save_phy,OUTPUT);
			digitalWrite(_pin_save_phy, _LOW);

			g_soft_pwm_pin = _pin_save_phy;
			
			marks[_pin_save_phy] = 0;
			range[_pin_save_phy] = 255;

			pthread_t soft_pwm_struct;
			pthread_create(&soft_pwm_struct, NULL, pwm_thread, NULL);

			g_pin_mode = PWM_OUTPUT;
			break;
		}
		case PULLUP:
		{

			int pud 	= 1;
			int bank 	= _pin >> 5;
			int index 	= _pin - (bank << 5);
			int sub 	= index >> 4;
			int sub_index = index - 16*sub;
			uint32_t phyaddr = _A10_GPIO_BASE + (bank * 36) + 0x1c + sub; 
			uint32_t _RegValue 	= 0;
			
			pud &= 3 ;
			
			_RegValue = Readreg32(phyaddr);
			_RegValue &= ~(3 << (sub_index << 1));
			_RegValue |= (pud << (sub_index << 1));
			
			printf("phyaddr is 0x%x,regvalue is 0x%x\n",phyaddr,_RegValue);
			Writereg32(phyaddr,_RegValue);
			_RegValue = Readreg32(phyaddr);
			printf("read regvalue is 0x%x\n",_RegValue);

			g_pin_mode = PULLUP;
			break;
			
		}
		case PULLDOWN:
		{
			int pud 	= 2;
			int bank 	= _pin >> 5;
			int index 	= _pin - (bank << 5);
			int sub 	= index >> 4;
			int sub_index = index - 16*sub;
			uint32_t phyaddr = _A10_GPIO_BASE + (bank * 36) + 0x1c + sub; 
			uint32_t _RegValue 	= 0;
			
			pud &= 3 ;
			
			_RegValue = Readreg32(phyaddr);
			_RegValue &= ~(3 << (sub_index << 1));
			_RegValue |= (pud << (sub_index << 1));
			
			
			printf("phyaddr is 0x%x,regvalue is 0x%x\n",phyaddr,_RegValue);
			Writereg32(phyaddr,_RegValue);
			_RegValue = Readreg32(phyaddr);
			printf("read regvalue is 0x%x\n",_RegValue);

			g_pin_mode = PULLDOWN;
			break;
		}
		case PULLOFF:
		{
			int pud 	= 0;
			int bank 	= _pin >> 5;
			int index 	= _pin - (bank << 5);
			int sub 	= index >> 4;
			int sub_index = index - 16*sub;
			uint32_t phyaddr = _A10_GPIO_BASE + (bank * 36) + 0x1c + sub; 
			uint32_t _RegValue 	= 0;
			
			pud &= 3 ;
			
			_RegValue = Readreg32(phyaddr);
			_RegValue &= ~(3 << (sub_index << 1));
			_RegValue |= (pud << (sub_index << 1));
			
			
			printf("phyaddr is 0x%x,regvalue is 0x%x\n",phyaddr,_RegValue);
			Writereg32(phyaddr,_RegValue);
			_RegValue = Readreg32(phyaddr);
			printf("read regvalue is 0x%x\n",_RegValue);
 
			g_pin_mode = PULLOFF;
			break;
		}
		case CHECK:
		{
			return g_pin_mode;
		} 
		default:
			break;
	}
}
/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: pwm  线程执行函数
*>备注: 
********************************************************************************/

void *pwm_thread(void *argk)
{
	int _mark, _space ;

	set_high_pri(50);

	for (;;)
	{
		_mark  = marks[g_soft_pwm_pin];
		_space = range[g_soft_pwm_pin]-_mark;

		if (_mark != 0)
		digitalWrite(g_soft_pwm_pin, _HIGH);
		delayMicroseconds (_mark * 20);

		if (_space != 0)
		digitalWrite(g_soft_pwm_pin, _LOW);
		delayMicroseconds(_space * 20);
		if(g_soft_pwm_pin == -1)
			break;
	}
}
/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 调度配置函数
*>备注: 
********************************************************************************/
int set_high_pri (int pri)
{
  struct sched_param sched ;

  memset (&sched, 0, sizeof(sched)) ;

  if (pri > sched_get_priority_max (SCHED_RR))
    pri = sched_get_priority_max (SCHED_RR) ;

  sched.sched_priority = pri ;
  return sched_setscheduler (0, SCHED_RR, &sched) ;
}
/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述:  
*>备注: Attempt to set a high priority schedulling for the running program
********************************************************************************/
void analogWrite(int pin, int value)
{
	pin &= 255;

	if (value < 0)
	value = 0;
	else if (value > range [pin])
	value = range [pin];

	marks [pin] = value;
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 写数据到指定端口
*>备注: pin: 指定端口 value: HIGH/LOW
********************************************************************************/
int digitalWrite(int _pin,int _value)
{

	_pin = _GPIO_MAP[_pin];

	uint32_t regval = 0;
	int bank = _pin >> 5;
	int index = _pin - (bank << 5);
	uint32_t _offset = (((bank * 0x24) + 0x10) >> 2); 		

	if(-1 == _pin)
	{
		printf("invalid _pin,please check it over.\n");
		return -1;
	}

	if((*(_gpio + _offset) & (1 << (index & 31))) != _value)
	{
		if(_value == _LOW)
		{
			*(_gpio + _offset) = *(_gpio + _offset) & ~(1 << (index & 31));  
		} 
		else 
		{
			*(_gpio + _offset) = *(_gpio + _offset) | (1 << (index & 31));
		}

		return 0;
	}
	
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人
*>日期:
*>描述: 从指定端口读取数据
*>备注: 返回1/0
********************************************************************************/
int digitalRead(int _pin)
{
	_pin = _GPIO_MAP[_pin];

	uint32_t regval = 0;
	int bank 	= _pin >> 5;
	int index 	= _pin - (bank << 5);
	uint32_t _offset = (((bank * 0x24) + 0x10) >> 2); 

	if ((*(_gpio + _offset) & (1 << (index & 31))) != 0)
		return _HIGH;
	else
		return _LOW;	
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人: 
*>日期:
*>描述: 延时函数
*>备注: 毫秒级别
********************************************************************************/
void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人
*>日期:
*>描述: 延时函数
*>备注: 微妙实现
********************************************************************************/
void delayMicrosecondsHard (unsigned int howLong)
{
	struct timeval tNow, tLong, tEnd ;

	gettimeofday (&tNow, NULL) ;
	tLong.tv_sec  = howLong / 1000000 ;
	tLong.tv_usec = howLong % 1000000 ;
	timeradd (&tNow, &tLong, &tEnd) ;

	while (timercmp (&tNow, &tEnd, <))
	gettimeofday (&tNow, NULL) ;
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人: 
*>日期:
*>描述: 延时函数
*>备注: 微妙级别
********************************************************************************/
void delayMicroseconds (unsigned int howLong)
{
	struct timespec sleeper ;

	if (howLong ==   0)
		return ;
	else if (howLong  < 100)
		delayMicrosecondsHard (howLong) ;
	else
	{
		sleeper.tv_sec  = 0;
		sleeper.tv_nsec = (long)(howLong * 1000);
		nanosleep (&sleeper, NULL);
	}
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人
*>日期:
*>描述: 读取系统运行时间
*>备注: 毫秒级别
********************************************************************************/
unsigned int millis (void)
{
	struct timeval tv;
	uint64_t now;

	gettimeofday(&tv, NULL) ;
	now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000);

	return (uint32_t)(now - epochMilli);
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人
*>日期:
*>描述: 读取系统运行时间
*>备注: 微妙级别
********************************************************************************/
unsigned int micros (void)
{
	struct timeval tv;
	uint64_t now;

	gettimeofday (&tv, NULL);
	now  = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec;

	return (uint32_t)(now - epochMicro);
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人
*>日期:
*>描述: 设置8位总线
*>备注: bus : 总线结构体 d0~d7 : 用户引脚 mode: 引脚模式
********************************************************************************/
int Set8BitsBUS(STRUCT_8BITS_BUS *bus,char d0,char d1,char d2,char d3,char d4,char d5,char d6,char d7,char mode)
{
	int statue;

	bus->D0 = d0;
	bus->D1 = d1;
	bus->D2 = d2;
	bus->D3 = d3;
	bus->D4 = d4;
	bus->D5 = d5;
	bus->D6 = d6;
	bus->D7 = d7;
	
	pinMode(d0,mode);
	pinMode(d1,mode);
	pinMode(d2,mode);
	pinMode(d3,mode);
	pinMode(d4,mode);
	pinMode(d5,mode);
	pinMode(d6,mode);
	statue = pinMode(d7,mode);

	if(statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}

}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人
*>日期:
*>描述: 8位数据总线写数据
*>备注: bus用户设定的8位总线，data: 需要输出的字符数据
********************************************************************************/
int digitalWrite8(STRUCT_8BITS_BUS *bus,char data)
{
	int statue = 0;

	(data & 0x01)?digitalWrite(bus->D0,1):digitalWrite(bus->D0,0);
	(data & 0x02)?digitalWrite(bus->D1,1):digitalWrite(bus->D1,0);
	(data & 0x04)?digitalWrite(bus->D2,1):digitalWrite(bus->D2,0);
	(data & 0x08)?digitalWrite(bus->D3,1):digitalWrite(bus->D3,0);
	(data & 0x10)?digitalWrite(bus->D4,1):digitalWrite(bus->D4,0);
	(data & 0x20)?digitalWrite(bus->D5,1):digitalWrite(bus->D5,0);
	(data & 0x40)?digitalWrite(bus->D6,1):digitalWrite(bus->D6,0);
	(data & 0x80)?(statue=digitalWrite(bus->D7,1)):(statue=digitalWrite(bus->D7,0));

	return statue;
	
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人
*>日期:
*>描述: 8位数据总线读取数据
*>备注: bus:用户要读取的数据
********************************************************************************/
unsigned char digitalRead8(STRUCT_8BITS_BUS *bus)
{
	unsigned char data=0x00; 

	digitalRead(bus->D0)?(data |= 0x01):(data |= 0x00);
	digitalRead(bus->D1)?(data |= 0x02):(data |= 0x00);
	digitalRead(bus->D2)?(data |= 0x04):(data |= 0x00);
	digitalRead(bus->D3)?(data |= 0x08):(data |= 0x00);
	digitalRead(bus->D4)?(data |= 0x10):(data |= 0x00);
	digitalRead(bus->D5)?(data |= 0x20):(data |= 0x00);
	digitalRead(bus->D6)?(data |= 0x40):(data |= 0x00);
	digitalRead(bus->D7)?(data |= 0x80):(data |= 0x00);

	return data;
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人
*>日期:
*>描述: 设置16位总线
*>备注:  bus:总线结构体d0~d15 : 总线引脚 mode : 引脚模式
********************************************************************************/
int Set16BitsBUS(STRUCT_16BITS_BUS *bus,char d0,char d1,char d2,char d3,char d4,char d5,char d6,char d7,char d8,char d9,char d10,char d11,char d12,char d13,char d14,char d15,char mode)
{
	int statue;

	bus->D0 = d0;
	bus->D1 = d1;
	bus->D2 = d2;
	bus->D3 = d3;
	bus->D4 = d4;
	bus->D5 = d5;
	bus->D6 = d6;
	bus->D7 = d7;
	bus->D8 = d8;
	bus->D9 = d9;
	bus->D10 = d10;
	bus->D11 = d11;
	bus->D12 = d12;
	bus->D13 = d13;
	bus->D14 = d14;
	bus->D15 = d15;

	pinMode(d0,mode);
	pinMode(d1,mode);
	pinMode(d2,mode);
	pinMode(d3,mode);
	pinMode(d4,mode);
	pinMode(d5,mode);
	pinMode(d6,mode);
	pinMode(d7,mode);
	pinMode(d8,mode);
	pinMode(d9,mode);
	pinMode(d10,mode);
	pinMode(d11,mode);
	pinMode(d12,mode);
	pinMode(d13,mode);
	pinMode(d14,mode);
	statue = pinMode(d15,mode);
	
	if(statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人
*>日期:
*>描述: 16位总线写数据
*>备注: bus: 总线data:16位的数据
********************************************************************************/
int digitalWrite16(STRUCT_16BITS_BUS *bus,short int data)
{
	int statue = 0;

	(data & 0x0001)?digitalWrite(bus->D0,1):digitalWrite(bus->D0,0);
	(data & 0x0002)?digitalWrite(bus->D1,1):digitalWrite(bus->D1,0);
	(data & 0x0004)?digitalWrite(bus->D2,1):digitalWrite(bus->D2,0);
	(data & 0x0008)?digitalWrite(bus->D3,1):digitalWrite(bus->D3,0);
	(data & 0x0010)?digitalWrite(bus->D4,1):digitalWrite(bus->D4,0);
	(data & 0x0020)?digitalWrite(bus->D5,1):digitalWrite(bus->D5,0);
	(data & 0x0040)?digitalWrite(bus->D6,1):digitalWrite(bus->D6,0);
	(data & 0x0080)?digitalWrite(bus->D7,1):digitalWrite(bus->D7,0);
	(data & 0x0100)?digitalWrite(bus->D8,1):digitalWrite(bus->D8,0);
	(data & 0x0200)?digitalWrite(bus->D9,1):digitalWrite(bus->D9,0);
	(data & 0x0400)?digitalWrite(bus->D10,1):digitalWrite(bus->D10,0);
	(data & 0x0800)?digitalWrite(bus->D11,1):digitalWrite(bus->D11,0);
	(data & 0x1000)?digitalWrite(bus->D12,1):digitalWrite(bus->D12,0);
	(data & 0x2000)?digitalWrite(bus->D13,1):digitalWrite(bus->D13,0);
	(data & 0x4000)?digitalWrite(bus->D14,1):digitalWrite(bus->D14,0);
	(data & 0x8000)?(statue=digitalWrite(bus->D15,1)):(statue=digitalWrite(bus->D15,0));

	return statue;
}

/*******************************************************************************
*>创建人:  	庞鹏鹏
*>日期:		2013/8/27
*>修改人
*>日期:
*>描述: 读取16位总线数据
*>备注: bus:总线 返回16位的数据
********************************************************************************/
unsigned short int digitalRead16(STRUCT_16BITS_BUS *bus)
{
	unsigned short int data=0x0000; 

	digitalRead(bus->D0)?(data |= 0x0001):(data |= 0x0000);
	digitalRead(bus->D1)?(data |= 0x0002):(data |= 0x0000);
	digitalRead(bus->D2)?(data |= 0x0004):(data |= 0x0000);
	digitalRead(bus->D3)?(data |= 0x0008):(data |= 0x0000);
	digitalRead(bus->D4)?(data |= 0x0010):(data |= 0x0000);
	digitalRead(bus->D5)?(data |= 0x0020):(data |= 0x0000);
	digitalRead(bus->D6)?(data |= 0x0040):(data |= 0x0000);
	digitalRead(bus->D7)?(data |= 0x0080):(data |= 0x0000);
	digitalRead(bus->D8)?(data |= 0x0100):(data |= 0x0000);
	digitalRead(bus->D9)?(data |= 0x0200):(data |= 0x0000);
	digitalRead(bus->D10)?(data |= 0x0400):(data |= 0x0000);
	digitalRead(bus->D11)?(data |= 0x0800):(data |= 0x0000);
	digitalRead(bus->D12)?(data |= 0x1000):(data |= 0x0000);
	digitalRead(bus->D13)?(data |= 0x2000):(data |= 0x0000);
	digitalRead(bus->D14)?(data |= 0x4000):(data |= 0x0000);
	digitalRead(bus->D15)?(data |= 0x8000):(data |= 0x0000);

	return data;

}

/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 串口启动函数
*>备注: part_data: 串口选择，baud:波特率
********************************************************************************/

int SerialBegin (int part_data, int baud)
{
  struct termios _options;
  speed_t _myBaud;
  int     _status,_fd;
  char *device;

	switch(part_data)
	{
		case UART0:
			
		break;
		case UART2:
			device = "/dev/ttyS1";
		break;
		case UART3:
			device = "/dev/ttyS2";
		break;
		case UART4:
			device = "/dev/ttyS3";
		break;
		case UART7:
			device = "/dev/ttyS4";
		break;
		default:
		break;
	}

	switch (baud)
	{
		case   9600:	_myBaud =   B9600 ; break ;
		case  19200:	_myBaud =  B19200 ; break ;
		case  38400:	_myBaud =  B38400 ; break ;
		case  57600:	_myBaud =  B57600 ; break ;
		case 115200:	_myBaud = B115200 ; break ;
		default:
		return -2;
	}

  if ((_fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;

  fcntl (_fd, F_SETFL, O_RDWR) ;

	switch(part_data)
	{
		case UART0:
			g_fd_array[0] = _fd;
		break;
		case UART2:
			g_fd_array[1] = _fd;
		break;
		case UART3:
			g_fd_array[2] = _fd;
		break;
		case UART4:
			g_fd_array[3] = _fd;
		break;
		case UART7:
			g_fd_array[4] = _fd;
		break;
		default:
		break;
	}

	tcgetattr (_fd, &_options) ;

    cfmakeraw   (&_options) ;
    cfsetispeed (&_options, _myBaud) ;
    cfsetospeed (&_options, _myBaud) ;

    _options.c_cflag |= (CLOCAL | CREAD) ;
    _options.c_cflag &= ~PARENB ;
    _options.c_cflag &= ~CSTOPB ;
    _options.c_cflag &= ~CSIZE ;
    _options.c_cflag |= CS8 ;
    _options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    _options.c_oflag &= ~OPOST ;

    _options.c_cc [VMIN]  =   0 ;
    _options.c_cc [VTIME] = 100 ;

	tcsetattr (_fd, TCSANOW | TCSAFLUSH, &_options) ;

	ioctl (_fd,TIOCMGET,&_status);

	_status |= TIOCM_DTR ;
	_status |= TIOCM_RTS ;

	ioctl (_fd, TIOCMSET, &_status);
	usleep (10000) ;	

	return _fd;
}

/*******************************************************************************
*>创建人: 庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 清空缓存函数
*>备注: 
********************************************************************************/

int SerialFlush (int fd)
{
	char _statue;
	int  _sfd;

	switch(fd)
	{
		case UART0:
			_sfd = g_fd_array[0];
		break;
		case UART2:
			_sfd = g_fd_array[1];
		break;
		case UART3:
			_sfd = g_fd_array[2];
		break;
		case UART4:
			_sfd = g_fd_array[3];
		break;
		case UART7:
			_sfd = g_fd_array[4];
		break;
		default:
		break;
	}

 	_statue = tcflush(_sfd, TCIOFLUSH);

	if(_statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 串口结束函数
*>备注: 
********************************************************************************/

int SerialEnd (int fd)
{
	int _statue,_sfd;
	
	switch(fd)
	{
		case UART0:
			_sfd = g_fd_array[0];
		break;
		case UART2:
			_sfd = g_fd_array[1];
		break;
		case UART3:
			_sfd = g_fd_array[2];
		break;
		case UART4:
			_sfd = g_fd_array[3];
		break;
		case UART7:
			_sfd = g_fd_array[4];
		break;
		default:
		break;
	}

  	_statue = close(_sfd) ;
	
	if(_statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 向某一特定串口写8位数据
*>备注: fd串口，c需要发送的8位数据
********************************************************************************/

int SerialWrite (int fd, unsigned char c)
{
	int _statue,_sfd;
	
	switch(fd)
	{
		case UART0:
			_sfd = g_fd_array[0];
		break;
		case UART2:
			_sfd = g_fd_array[1];
		break;
		case UART3:
			_sfd = g_fd_array[2];
		break;
		case UART4:
			_sfd = g_fd_array[3];
		break;
		case UART7:
			_sfd = g_fd_array[4];
		break;
		default:
		break;
	}
	
  	_statue = write (_sfd, &c, 1);
	
	if(_statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 通过某一串口输出字符串
*>备注: fd: 某一串口，s: 需要输出的字符串
********************************************************************************/

int SerialPrint (int fd, char *s)
{
	int _statue,_sfd;
	
	switch(fd)
	{
		case UART0:
			_sfd = g_fd_array[0];
		break;
		case UART2:
			_sfd = g_fd_array[1];
		break;
		case UART3:
			_sfd = g_fd_array[2];
		break;
		case UART4:
			_sfd = g_fd_array[3];
		break;
		case UART7:
			_sfd = g_fd_array[4];
		break;
		default:
		break;
	}
  	_statue = write (_sfd, s, strlen (s));
	
	if(_statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 通过某一串口输出字符串(带有换行功能)
*>备注: fd: 某一串口，s: 需要输出的字符串
********************************************************************************/
int SerialPrintln (int fd, char *s)
{
	int _statue,_sfd;

	switch(fd)
	{
		case UART0:
			_sfd = g_fd_array[0];
		break;
		case UART2:
			_sfd = g_fd_array[1];
		break;
		case UART3:
			_sfd = g_fd_array[2];
		break;
		case UART4:
			_sfd = g_fd_array[3];
		break;
		case UART7:
			_sfd = g_fd_array[4];
		break;
		default:
		break;
	}
	
  	write (_sfd, s, strlen (s));
	write (_sfd,"\r\n",strlen("\r\n"));
	
	if(_statue < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 查询是否接收到数据
*>备注: 
********************************************************************************/

int SerialAvailable (int fd)
{
  	int result,_sfd;

	switch(fd)
	{
		case UART0:
			_sfd = g_fd_array[0];
		break;
		case UART2:
			_sfd = g_fd_array[1];
		break;
		case UART3:
			_sfd = g_fd_array[2];
		break;
		case UART4:
			_sfd = g_fd_array[3];
		break;
		case UART7:
			_sfd = g_fd_array[4];
		break;
		default:
		break;
	}
	
  	if (ioctl (_sfd, FIONREAD, &result) == -1)
    	return -1;

	return result;
}

/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/27
*>修改人:
*>日期:
*>描述: 从某一特定的端口读取数据
*>备注: 
********************************************************************************/

int SerialRead (int fd)
{
  	uint8_t x;
	int _sfd;

	switch(fd)
	{
		case UART0:
			_sfd = g_fd_array[0];
		break;
		case UART2:
			_sfd = g_fd_array[1];
		break;
		case UART3:
			_sfd = g_fd_array[2];
		break;
		case UART4:
			_sfd = g_fd_array[3];
		break;
		case UART7:
			_sfd = g_fd_array[4];
		break;
		default:
		break;
	}

  	if (read (_sfd, &x, 1) != 1)
    	return -1 ;

  	return ((int)x)&0xFF;
}

/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/31
*>修改人:
*>日期:
*>描述:  I2C进程通讯函数
*>备注: 
********************************************************************************/

static inline int i2c_access (int fd, char rw, uint8_t command, int size,union i2c_data *data)
{
        struct i2c_ioctl_data args;

        args.read_write = rw;
        args.command    = command;
        args.size       = size;
        args.data       = data;

        return ioctl(fd,I2C_SMBUS,&args);
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/31
*>修改人:
*>日期:
*>描述:  I2C8位读取函数
*>备注: 返回8位数据
********************************************************************************/
char WireRead8(int device,char dev_offset)
{
        union i2c_data data;

        if (i2c_access(g_i2c_fd_offset_array[device][1], I2C_READ, dev_offset, I2C_BYTE_DATA, &data))
			return -1 ;
        else
			return data.byte & 0xFF;
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/31
*>修改人:
*>日期:
*>描述:  I2C16位读取函数
*>备注: 返回16位数据
********************************************************************************/
short int WireRead16 (int device,short int dev_offset)
{  
		union i2c_data data;  

		if (i2c_access (g_i2c_fd_offset_array[device][1], I2C_READ,dev_offset,I2C_WORD_DATA, &data))    
			return -1 ;  
		else    
			return 	data.byte & 0xFF ;
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/31
*>修改人:
*>日期:
*>描述:  I2C 8位写入函数
*>备注: 
********************************************************************************/
int WireWrite8(int device, char dev_offset,unsigned char volue)
{
        union i2c_data data;
        data.byte = volue;
     
		return i2c_access(g_i2c_fd_offset_array[device][1],I2C_WRITE,dev_offset,I2C_BYTE_DATA,&data);
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/31
*>修改人:
*>日期:
*>描述:  I2C 16位写入函数
*>备注: 
********************************************************************************/
int WireWrite16(int device, short int dev_offset,short int volue)
{
        union i2c_data data;
        data.byte = volue;
     
		return i2c_access(g_i2c_fd_offset_array[device][1],I2C_WRITE,dev_offset,I2C_WORD_DATA,&data);
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/31
*>修改人:
*>日期:
*>描述:  I2C和从设备建立通讯函数
*>备注: 
********************************************************************************/
int WireBeginTransmission(int device, unsigned char dev_addr)
{
        ioctl(g_i2c_fd_offset_array[device][1],I2C_SLAVE,dev_addr);
        return 1;
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/31
*>修改人:
*>日期:
*>描述:  I2C启动函数
*>备注: 
********************************************************************************/
int WireBegin(int device)
{
        int fd;

        switch(device)
        {
	        case I2C1:
                fd=open("/dev/i2c-1", O_RDWR);
                g_i2c_fd_offset_array[I2C1][1]=fd;
                break;
	        case I2C2:
                fd=open("/dev/i2c-2", O_RDWR);
                g_i2c_fd_offset_array[I2C2][1]=fd;
                break;
	        default:break;
        }


        if(fd<0)
        {
                return -1;
        }
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/31
*>修改人:
*>日期:
*>描述:  I2C关闭函数
*>备注: 
********************************************************************************/
int WireEndTransmission(int device)
{
        close(g_i2c_fd_offset_array[device][1]);
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/8/31
*>修改人:
*>日期:
*>描述:  SPI开始函数
*>备注: 
********************************************************************************/
int SPIBegin(int device)
{
	int fd=0;

	switch(device)
	{
		case SPI0:fd = open ("/dev/spidev0.0", O_RDWR);g_SPI_data_array[device][fd_nu]=fd;break;
		case SPI1:fd = open ("/dev/spidev0.1", O_RDWR);g_SPI_data_array[device][fd_nu]=fd;break;
		default: break;
	}
	if(fd<0) return -1;
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/09/02
*>修改人:
*>日期:
*>描述:  SPI大小端设置函数
*>备注: 
********************************************************************************/
int SetBitOrder(int device,int LH_mode)
{
	 if(ioctl(g_SPI_data_array[device][fd_nu],SPI_IOC_WR_LSB_FIRST, &LH_mode)<0) return -1;
	 if(ioctl(g_SPI_data_array[device][fd_nu],SPI_IOC_RD_LSB_FIRST, &LH_mode)<0) return -1;
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/09/02
*>修改人:
*>日期:
*>描述:  SPI传输模式设置函数
*>备注: 
********************************************************************************/
int SetDataMode(int device, int RT_mode)
{
	if(ioctl(g_SPI_data_array[device][fd_nu], SPI_IOC_WR_MODE, &RT_mode)<0) return -1;
	if(ioctl(g_SPI_data_array[device][fd_nu], SPI_IOC_RD_MODE, &RT_mode)<0) return -1;
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/09/02
*>修改人:
*>日期:
*>描述:  SPI时钟设置函数
*>备注: 
********************************************************************************/
int SetClockDivider(int device,int DRI)
{
	switch(device)
	{
		case SPI0 : g_SPI_data_array[device][speed_nu]=DRI;break;
		case SPI1 : g_SPI_data_array[device][speed_nu]=DRI;break;
		default:break;
	}
	if(ioctl(g_SPI_data_array[device][fd_nu], SPI_IOC_WR_MAX_SPEED_HZ,&DRI)<0) return -1;
	if(ioctl(g_SPI_data_array[device][fd_nu], SPI_IOC_RD_MAX_SPEED_HZ,&DRI)<0) return -1;
	
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/09/02
*>修改人:
*>日期:
*>描述:  SPI传输函数
*>备注: 
********************************************************************************/
char SPITransfer(int device,char data)
{
	 char w_data[2]={};
	 char r_data[2]={};
	 char r_data_send;

	 w_data[0]=data;
	 
	 write(g_SPI_data_array[0][1],&w_data,1);
	 read(g_SPI_data_array[0][1],&r_data,1);

	 r_data_send=r_data[0];
	 return r_data_send;
	 
}
/*******************************************************************************
*>创建人:庞鹏鹏
*>日期:2013/09/02
*>修改人:
*>日期:
*>描述: SPI关闭函数 
*>备注: 
********************************************************************************/
int SPIEnd(int device)
{
	if(close(g_SPI_data_array[device][fd_nu])<0) return -1;
}
	




