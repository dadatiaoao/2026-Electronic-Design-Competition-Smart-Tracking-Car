#include "headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// @brief		PWM引脚初始化
// @param	  timn_chn		选择定时器通道
// @return		void  
// Sample usage:	pwm_pin_init(timn_chn);  示例用法，传入定时器通道进行初始化
//-------------------------------------------------------------------------------------------------------------------
void pwm_pin_init(TIMn_CHn_enum timn_chn)
{
    // 根据不同的定时器通道选择不同的GPIO引脚进行初始化
    switch(timn_chn)
    {
        case TIM2_CH1:
            // 初始化GPIOA的Pin_0引脚为复用推挽输出模式，用于TIM2_CH1通道的PWM输出
            gpio_init(GPIO_A,Pin_0,AF_PP);
            break;
        case TIM2_CH2:
            // 初始化GPIOA的Pin_1引脚为复用推挽输出模式，用于TIM2_CH2通道的PWM输出
            gpio_init(GPIO_A,Pin_1,AF_PP);
            break;
        case TIM2_CH3:
            // 初始化GPIOA的Pin_2引脚为复用推挽输出模式，用于TIM2_CH3通道的PWM输出
            gpio_init(GPIO_A,Pin_2,AF_PP);
            break;
        case TIM2_CH4:
            // 初始化GPIOA的Pin_3引脚为复用推挽输出模式，用于TIM2_CH4通道的PWM输出
            gpio_init(GPIO_A,Pin_3,AF_PP);
            break;
        case TIM3_CH1:
            // 初始化GPIOA的Pin_6引脚为复用推挽输出模式，用于TIM3_CH1通道的PWM输出
            gpio_init(GPIO_A,Pin_6,AF_PP);
            break;
        case TIM3_CH2:
            // 初始化GPIOA的Pin_7引脚为复用推挽输出模式，用于TIM3_CH2通道的PWM输出
            gpio_init(GPIO_A,Pin_7,AF_PP);
            break;
        case TIM3_CH3:
            // 初始化GPIOB的Pin_0引脚为复用推挽输出模式，用于TIM3_CH3通道的PWM输出
            gpio_init(GPIO_B,Pin_0,AF_PP);
            break;
        case TIM3_CH4:
            // 初始化GPIOB的Pin_1引脚为复用推挽输出模式，用于TIM3_CH4通道的PWM输出
            gpio_init(GPIO_B,Pin_1,AF_PP);
            break;
        case TIM4_CH1:
            // 初始化GPIOB的Pin_6引脚为复用推挽输出模式，用于TIM4_CH1通道的PWM输出
            gpio_init(GPIO_B,Pin_6,AF_PP);
            break;
        case TIM4_CH2:
            // 初始化GPIOB的Pin_7引脚为复用推挽输出模式，用于TIM4_CH2通道的PWM输出
            gpio_init(GPIO_B,Pin_7,AF_PP);
            break;
        case TIM4_CH3:
            // 初始化GPIOB的Pin_8引脚为复用推挽输出模式，用于TIM4_CH3通道的PWM输出
            gpio_init(GPIO_B,Pin_8,AF_PP);
            break;
        case TIM4_CH4:
            // 初始化GPIOB的Pin_9引脚为复用推挽输出模式，用于TIM4_CH4通道的PWM输出
            gpio_init(GPIO_B,Pin_9,AF_PP);
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		PWM初始化
// @param	  timn		选择定时器（可选择的定时器参考ml_tim.h中的枚举定义）
// @param	  timn_chn		选择定时器通道（可选择的通道参考ml_pwm.h中的枚举定义）
// @param	  fre		输出频率，频率需要大于15Hz
// @return		void  
// Sample usage:	pwm_init(TIM_2,TIM2_CH1,50);     
//-------------------------------------------------------------------------------------------------------------------
void pwm_init(TIMn_enum timn,TIMn_CHn_enum timn_chn,int fre)
{ 
    // 计算通道号，通过取模和加1操作得到1 - 4的通道号
    uint8_t ch = timn_chn%4+1;
    // 使能对应定时器的时钟，通过左移操作将相应位置1
    RCC->APB1ENR |= 1<<timn;
    // 设置自动重装载寄存器的值，根据输入频率计算得到
    tim_index[timn]->ARR = 1000000/fre-1; 
    // 设置预分频器的值为72 - 1
    tim_index[timn]->PSC = 72-1;       
    // 调用引脚初始化函数，对所选通道对应的引脚进行初始化
    pwm_pin_init(timn_chn);

    // 根据不同的通道号进行不同的配置
    switch(ch)
    {
        case 1:
            // 设置通道1为PWM1模式，通过左移6位并按位或操作实现
            tim_index[timn]->CCMR1 |= 6<<4;	
            // 使能通道1的预装载寄存器
            tim_index[timn]->CCMR1 |= 1<<3;       
            // 初始化通道1的捕获比较寄存器的值为0
            tim_index[timn]->CCR1 = 0;            
            break;
        case 2:
            // 设置通道2为PWM1模式，通过左移12位并按位或操作实现
            tim_index[timn]->CCMR1 |= 6<<12;	
            // 使能通道2的预装载寄存器
            tim_index[timn]->CCMR1 |= 1<<11;		
            // 初始化通道2的捕获比较寄存器的值为0
            tim_index[timn]->CCR2 = 0;            
            break;
        case 3:
            // 设置通道3为PWM1模式，通过左移4位并按位或操作实现
            tim_index[timn]->CCMR2 |= 6<<4;	
            // 使能通道3的预装载寄存器
            tim_index[timn]->CCMR2 |= 1<<3;		
            // 初始化通道3的捕获比较寄存器的值为0
            tim_index[timn]->CCR3 = 0;            
            break;		
        case 4:
            // 设置通道4为PWM1模式，通过左移12位并按位或操作实现
            tim_index[timn]->CCMR2 |= 6<<12;	
            // 使能通道4的预装载寄存器
            tim_index[timn]->CCMR2 |= 1<<11;		
            // 初始化通道4的捕获比较寄存器的值为0
            tim_index[timn]->CCR4 = 0;            
            break;				
    }
    // 使能所选通道的PWM输出，通过左移操作将相应位置1
    tim_index[timn]->CCER |= 0x01<<(4*(ch-1)); 
    // 使能自动重装载预装载寄存器和计数器
    tim_index[timn]->CR1 |= 0x81;        
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		PWM占空比更新
// @param	  timn		选择定时器（可选择的定时器参考ml_tim.h中的枚举定义）
// @param	  timn_chn		选择定时器通道（可选择的通道参考ml_pwm.h中的枚举定义）
// @param	  duty    占空比数值
// @return		void  
// Sample usage:	pwm_update(TIM_2,TIM2_CH1,0);     
//-------------------------------------------------------------------------------------------------------------------
void pwm_update(TIMn_enum timn,TIMn_CHn_enum timn_chn,uint16_t duty)
{
    // 计算通道号，通过取模和加1操作得到1 - 4的通道号
    uint8_t ch = timn_chn%4+1;
    // 如果占空比大于等于最大占空比，则将占空比设置为最大占空比
    if(duty>=MAX_DUTY)
        duty=MAX_DUTY;
    // 获取自动重装载寄存器的值
    uint16_t temp = tim_index[timn]->ARR;   
    // 根据不同的通道号更新相应通道的捕获比较寄存器的值
    switch(ch)
    {
        case 1:
            // 根据占空比和自动重装载寄存器的值计算新的捕获比较寄存器的值
            tim_index[timn]->CCR1 = (uint16_t)((float)duty/MAX_DUTY*(temp+1));               
            break;
        case 2:	
            // 根据占空比和自动重装载寄存器的值计算新的捕获比较寄存器的值
            tim_index[timn]->CCR2 = (uint16_t)((float)duty/MAX_DUTY*(temp+1));               
            break;
        case 3:	
            // 根据占空比和自动重装载寄存器的值计算新的捕获比较寄存器的值
            tim_index[timn]->CCR3 = (uint16_t)((float)duty/MAX_DUTY*(temp+1));               
            break;		
        case 4:
            // 根据占空比和自动重装载寄存器的值计算新的捕获比较寄存器的值
            tim_index[timn]->CCR4 = (uint16_t)((float)duty/MAX_DUTY*(temp+1));               
            break;					
    }
}