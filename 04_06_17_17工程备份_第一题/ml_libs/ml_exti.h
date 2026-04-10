#ifndef _ml_exti_h
#define _ml_exti_h
#include "headfile.h"

typedef enum
{
	EXTI_PA0,
	EXTI_PB0,
	EXTI_PC0,
	EXTI_PA1,
	EXTI_PB1,
	EXTI_PC1,
	EXTI_PA2,
	EXTI_PB2,
	EXTI_PC2,
	EXTI_PA3,
	EXTI_PB3,
	EXTI_PC3,
	EXTI_PA4,
	EXTI_PB4,
	EXTI_PC4,
	EXTI_PA5,
	EXTI_PB5,
	EXTI_PC5,
	EXTI_PA6,
	EXTI_PB6,
	EXTI_PC6,
	EXTI_PA7,
	EXTI_PB7,
	EXTI_PC7,
	EXTI_PB9,

}EXTI_Pnx_enum;

typedef enum
{
	RISING,       
	FALLING,      
}EXTI_Trigger_enum;

void exti_init(EXTI_Pnx_enum pin,EXTI_Trigger_enum trigger,uint8_t priority);

#endif
