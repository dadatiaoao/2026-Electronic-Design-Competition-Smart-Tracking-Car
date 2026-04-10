#ifndef  _ml_nvic_h_
#define  _ml_nvic_h_
#include "headfile.h"

void NVIC_Group_Config(uint8_t group);
void NVIC_init(uint8_t PreemptionPriority,uint8_t nvic_channel);


#endif
