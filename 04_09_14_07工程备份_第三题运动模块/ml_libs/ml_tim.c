#include "headfile.h"

// 定义一个数组，用于存储TIM2、TIM3和TIM4的寄存器地址
TIM_TypeDef *tim_index[3] = { TIM2 , TIM3 , TIM4 };

//-------------------------------------------------------------------------------------------------------------------
// @brief		定时器中断初始化（同时初始化定时器中断）
// @param	  timn		选择定时器（所选定时器参考ml_tim.h中的枚举定义）
// @param	  time_ms  定时器中断的时间间隔(ms)
// @param	  priority 中断优先级(0~15，数值越小优先级越高)
// @return		void  
// 示例用法:	tim_interrupt_ms_init(TIM_2,1000,0);
//-------------------------------------------------------------------------------------------------------------------
void tim_interrupt_ms_init(TIMn_enum timn,int time_ms,uint8_t priority)
{	
    // 使能定时器的时钟
    RCC->APB1ENR |= 1<<timn;  

    // 设置自动重载值，实现指定的定时时间
    tim_index[timn]->ARR = 10*time_ms-1; 
    // 设置预分频器的值
    tim_index[timn]->PSC = 7200-1;       
    // 使能定时器
    tim_index[timn]->CR1 |= 0x01;        
    // 使能定时器中断
    tim_index[timn]->DIER |= 0x01;       

    // 初始化中断向量表，设置中断优先级
    NVIC_init(priority,timn+28);         
}
