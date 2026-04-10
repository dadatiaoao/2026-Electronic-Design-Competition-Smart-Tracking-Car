#ifndef _motor_h
#define _motor_h
#include "headfile.h"

void motor_init(void);
void motorA_duty(int duty);
void motorB_duty(int duty);
void encoder_init(void);

extern volatile int length;
extern int Encoder_count1, Encoder_count2;
extern int speed_now;
extern uint8_t motorA_dir, motorB_dir;

#endif
