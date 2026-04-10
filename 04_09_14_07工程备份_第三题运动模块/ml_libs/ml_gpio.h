#ifndef _ml_gpio_h_
#define _ml_gpio_h_
#include "headfile.h"

typedef enum
{
		GPIO_A = 0x00,
	  GPIO_B = 0x01,
	  GPIO_C = 0x02,
}GPIOn_enum;    //IO口枚举类型

typedef enum
{
		Pin_0 = 0x00,
		Pin_1 = 0x01,
		Pin_2 = 0x02,
		Pin_3 = 0x03,
		Pin_4 = 0x04,
		Pin_5 = 0x05,
		Pin_6 = 0x06,
		Pin_7 = 0x07,
		Pin_8 = 0x08,
		Pin_9 = 0x09,
		Pin_10 = 0x0A,
		Pin_11 = 0x0B,
		Pin_12 = 0x0C,
		Pin_13 = 0x0D,
		Pin_14 = 0x0E,
		Pin_15 = 0x0F,  
}Pinx_enum;

typedef enum
{
		OUT_PP   =  0x00,     //默认推挽输出
	  AF_PP    =  0x03,     //复用推挽输出
	  OUT_OD   =  0x02,     //开漏输出
		IU       =  0x01,   //上拉输入
	  ID       =  0x04, //下拉输入
	  IF       =  0x00,  //浮空输入
	  AIN      =  0x1E,  //模拟输入
}GPIO_MODE_enum;  //定义IO口模式

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;

void gpio_init(GPIOn_enum GPIOn,Pinx_enum Pinx,GPIO_MODE_enum mode);
void gpio_set(GPIOn_enum GPIOn,Pinx_enum Pinx,uint8_t mode);
uint8_t gpio_get(GPIOn_enum GPIOn,Pinx_enum Pinx);


#endif
