#include "stm32f10x.h"                  
#include "ml_delay.h"               

void LED_Init(void)
{
	gpio_init(GPIO_B,Pin_15,		OUT_PP);
}

void Enable_LED(void)
  {
  gpio_set(GPIO_B,Pin_15,1);
	  delay_ms(300);
  gpio_set(GPIO_B,Pin_15,0);

	}
