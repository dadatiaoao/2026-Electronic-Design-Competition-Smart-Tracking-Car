#include "headfile.h"

void Buzzer_Init(void)
{
	gpio_init(GPIO_C,Pin_15,OUT_PP);
	gpio_set(GPIO_C, Pin_15, 1);
}
	
void Enable_Buzzer(void)
  {
		gpio_set(GPIO_C,Pin_15,0);
		delay_ms(100);
		gpio_set(GPIO_C,Pin_15,1);
	}