#include "stm32f10x.h"                  // Device header
#include "ml_delay.h"
#include "ml_gpio.h"



void Key_Init(void)
{	
	
	gpio_init(GPIO_B,Pin_12,IU);
	gpio_init(GPIO_B,Pin_13,IU);
	gpio_init(GPIO_B,Pin_14,IU);
//	gpio_init(GPIO_B,Pin_15,IU);
   //设置为下拉输入
}

int mission_get(void)
{   int mission=0;
	if( gpio_get(GPIO_B,Pin_12)==0)
	{   delay_ms(50);
		while(gpio_get(GPIO_B,Pin_12)==0);
		delay_ms(50);
		mission=1;
		
	}
	else if(gpio_get(GPIO_B,Pin_13)==0)
	{
		delay_ms(50);
		while(gpio_get(GPIO_B,Pin_13)==0);
		delay_ms(50);
		mission=2;
	}
	else if(gpio_get(GPIO_B,Pin_14)==0)
	{
	    delay_ms(50);
		while(gpio_get(GPIO_B,Pin_14)==0);
		delay_ms(50);
		mission=3;
	}
//	else if(gpio_get(GPIO_B,Pin_15)==0)
//	{
//		delay_ms(50);
//		while(gpio_get(GPIO_B,Pin_15)==0);
//		delay_ms(50);
//		mission=4;
//	}
	return mission;
}
