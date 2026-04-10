#ifndef _pwm_h_
#define _pwm_h_
#include "headfile.h"

#define MAX_DUTY  50000

typedef enum
{
		TIM2_CH1  =  0x00, // PA0
	  TIM2_CH2  =  0x01, // PA1
	  TIM2_CH3  =  0x02, // PA2
	  TIM2_CH4  =  0x03, // PA3
	  TIM3_CH1  =  0x04, // PA6
	  TIM3_CH2  =  0x05, // PA7
	  TIM3_CH3  =  0x06, // PB0
	  TIM3_CH4  =  0x07, // PB1
	  TIM4_CH1  =  0x08, // PB6
	  TIM4_CH2  =  0x09, // PB7
	  TIM4_CH3  =  0x0a, // PB8
	  TIM4_CH4  =  0x0b, // PB9
}TIMn_CHn_enum;  //枚举定义定时器通道

void pwm_pin_init(TIMn_CHn_enum timn_chn);
void pwm_init(TIMn_enum timn,TIMn_CHn_enum timn_chn,int fre);
void pwm_update(TIMn_enum timn,TIMn_CHn_enum timn_chn,uint16_t duty);

#endif
