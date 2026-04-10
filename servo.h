#ifndef _servo_h_
#define _servo_h_
#include "headfile.h"

// 舵机通道枚举（基于现有TIMn_CHn_enum）
typedef enum {
    //SERVO_CH1 = TIM4_CH1,  // PB6
    //SERVO_CH2 = TIM4_CH2,  // PB7（推荐使用）
    //SERVO_CH3 = TIM4_CH3,  // PB8
    SERVO_CH4 = TIM4_CH4   // PB9
} ServoChannel_t;

// 函数声明
void servo_init(ServoChannel_t channel);
void servo_set_angle(ServoChannel_t channel, uint8_t angle);
void servo_set_pulse(ServoChannel_t channel, float pulse_ms);

#endif