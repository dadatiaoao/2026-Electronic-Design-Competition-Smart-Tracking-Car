#ifndef _tim_h_
#define _tim_h_

#include "headfile.h"


typedef enum      
{
		TIM_2  =  0x00,
	  TIM_3  =  0x01,
	  TIM_4  =  0x02,
}TIMn_enum;     

extern TIM_TypeDef *tim_index[3];

void tim_interrupt_ms_init(TIMn_enum timn,int time_ms,uint8_t priority);

#endif
