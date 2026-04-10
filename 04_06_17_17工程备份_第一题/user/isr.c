#include "stm32f10x.h"                  // STM32F10x系列单片机头文件
#include "headfile.h"

// 定时器2中断服务函数
void TIM2_IRQHandler(void)
{
    if(TIM2->SR & 1)  // 检查更新中断标志
    {
        // 清除中断标志
        TIM2->SR &= ~1; 
    }
}

// 定时器3中断服务函数（常用作系统时基或周期性任务）
void TIM3_IRQHandler(void)
{
    if(TIM3->SR & 1)  // 检查更新中断标志
    {
        // 执行PID控制算法（用户自定义函数）
        pid_control(basicspeed);
        // 清除中断标志
        TIM3->SR &= ~1; 
    }
}

// 定时器4中断服务函数
void TIM4_IRQHandler(void)
{
    if(TIM4->SR & 1)  // 检查更新中断标志
    {
        // 清除中断标志
        TIM4->SR &= ~1; 
    }
}

extern volatile int uart_received_data; 
// 串口1中断服务函数
void USART1_IRQHandler(void)
{
    if (USART1->SR & 0x20)  // 检查接收中断标志
    {
		uart_received_data =uart_getbyte(UART_1);
        // 清除中断标志
        USART1->SR &= ~0x20;   
    }
}

// 串口2中断服务函数
void USART2_IRQHandler(void)
{
    if (USART2->SR & 0x20)  // 检查接收中断标志
    {
        auto_vision_rx_byte(uart_getbyte(UART_2));
        // 清除中断标志
        USART2->SR &= ~0x20;   
    }
}

// 串口3中断服务函数
void USART3_IRQHandler(void)
{
    if (USART3->SR & 0x20)  // 检查接收中断标志
    {
        // 清除中断标志
        USART3->SR &= ~0x20;   
    }
}

// 外部中断0服务函数（PA0/PB0/PC0）
void EXTI0_IRQHandler(void) 
{
    if(EXTI->PR & (1<<0))  // 检查中断标志
    {
        // 清除中断标志
        EXTI->PR = 1<<0; 
    }
}

// 外部中断1服务函数（PA1/PB1/PC1）
void EXTI1_IRQHandler(void) 
{
    if(EXTI->PR & (1<<1))  // 检查中断标志
    {
        // 清除中断标志
        EXTI->PR = 1<<1; 
    }
}

// 外部中断2服务函数（PA2/PB2/PC2）
void EXTI2_IRQHandler(void) 
{
    if(EXTI->PR & (1<<2))  // 检查中断标志
    {
        // 编码器A计数逻辑（根据PA3状态增减计数值）
        if(gpio_get(GPIO_A, Pin_3))  // 检测PA3引脚电平
		{ Encoder_count1++;
		length++;}
        else
            Encoder_count1--;
        // 清除中断标志
        EXTI->PR = 1<<2; 
    }
}

// 外部中断3服务函数（PA3/PB3/PC3）
void EXTI3_IRQHandler(void) 
{
    if(EXTI->PR & (1<<3))  // 检查中断标志
    {  
		
        // 清除中断标志
        EXTI->PR = 1<<3; 
    }
}

// 外部中断4服务函数（PA4/PB4/PC4）
void EXTI4_IRQHandler(void) 
{
    if(EXTI->PR & (1<<4))  // 检查中断标志
    {
        // 编码器B计数逻辑（根据PA5状态增减计数值）
        if(gpio_get(GPIO_A, Pin_5))  // 检测PA5引脚电平
           { Encoder_count2++;}
        else
            Encoder_count2--;
        // 清除中断标志
        EXTI->PR = 1<<4; 
    }
}

// 外部中断5-9服务函数
void EXTI9_5_IRQHandler(void)
{
    // 处理EXTI5中断
    if(EXTI->PR & (1<<5))   
    {
        EXTI->PR = 1<<5; 
    }
    // 处理EXTI6中断
    if(EXTI->PR & (1<<6))   
    {
        EXTI->PR = 1<<6; 
    }
    // 处理EXTI7中断
    if(EXTI->PR & (1<<7))   
    {
      MPU6050_GetData();  // 获取MPU6050传感器数据
      // 积分计算姿态角（示例代码）
		
			// 陀螺仪角度
			yaw_gyro += (float)gz / 16.4 * 0.005;
			//加速度计角度
			yaw_acc = atan2((float)ay,ax) * 57.296;
			//卡尔曼
			yaw_Kalman = Kalman_Filter(&KF_Yaw, yaw_acc, (float)gz / 16.4);
        // 清除中断标志
        EXTI->PR = 1<<7; 
    }
    // 处理EXTI8中断
    if(EXTI->PR & (1<<8))   
    {
        EXTI->PR = 1<<8; 
    }
    // 处理EXTI9中断
    if(EXTI->PR & (1<<9))   
    {  
	
        EXTI->PR = 1<<9; 
    }
}