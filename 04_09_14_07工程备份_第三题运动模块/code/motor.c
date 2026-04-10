#include "motor.h"

// 定义电机 A 的转动方向，1 表示正转，0 表示反转
uint8_t motorA_dir = 1; 
// 定义电机 B 的转动方向，1 表示正转，0 表示反转
uint8_t motorB_dir = 1; 

// 编码器 1 的计数值，用于记录电机 A 相关编码器的脉冲数
int Encoder_count1 = 0;
// 编码器 2 的计数值，用于记录电机 B 相关编码器的脉冲数
int Encoder_count2 = 0;
volatile int length;
int speed_now;

/**
 * @brief 电机初始化函数
 * @details 该函数用于初始化电机的 PWM 输出和控制引脚。通过调用 pwm_init 函数初始化定时器和通道，
 *          并设置 PWM 频率为 1000Hz。同时，将电机控制引脚初始化为推挽输出模式，以便后续控制电机的转动。
 */
void motor_init()
{
    // 初始化定时器 2 的通道 1 用于电机 A 的 PWM 输出，PWM 频率为 1000Hz
    pwm_init(TIM_2,TIM2_CH1,1000);   
    gpio_init(GPIO_A,Pin_6,OUT_PP);
    gpio_init(GPIO_A,Pin_7,OUT_PP);
    pwm_init(TIM_2,TIM2_CH2,1000);   
    gpio_init(GPIO_B,Pin_0,OUT_PP);
    gpio_init(GPIO_B,Pin_1,OUT_PP);
}

/**
 * @brief 设置电机 A 的占空比
 * @param duty 要设置的 PWM 占空比
 * @details 该函数用于更新电机 A 的 PWM 占空比，同时根据 motorA_dir 的值设置控制引脚的电平，
 *          从而控制电机 A 的转动方向和速度。
 */
void motorA_duty(int duty)
{
    pwm_update(TIM_2,TIM2_CH1,duty);  
    gpio_set(GPIO_A,Pin_6,motorA_dir);
    gpio_set(GPIO_A,Pin_7,!motorA_dir);
}

/**
 * @brief 设置电机 B 的占空比
 * @param duty 要设置的 PWM 占空比
 * @details 该函数用于更新电机 B 的 PWM 占空比，同时根据 motorB_dir 的值设置控制引脚的电平，
 *          从而控制电机 B 的转动方向和速度。
 */
void motorB_duty(int duty)
{
    pwm_update(TIM_2,TIM2_CH2,duty);  
    gpio_set(GPIO_B,Pin_0,motorB_dir);
    gpio_set(GPIO_B,Pin_1,!motorB_dir);
}

/**
 * @brief 编码器初始化函数
 * @details 该函数用于初始化编码器相关的外部中断和 GPIO 引脚。通过调用 exti_init 函数初始化外部中断，
 *          并设置触发方式为下降沿触发。同时，将编码器信号输入引脚初始化为上拉输入模式，以便读取编码器的脉冲信号。
 */
void encoder_init()
{
    // 初始化外部中断 EXTI_PA2，下降沿触发，中断优先级为 0，用于编码器 1 的信号
    exti_init(EXTI_PA2,FALLING,0);
    // 将 GPIOA 的 Pin_3 引脚初始化为上拉输入模式，用于接收编码器 1 的信号
    gpio_init(GPIO_A,Pin_3,IU);

    // 初始化外部中断 EXTI_PA4，下降沿触发，中断优先级为 0，用于编码器 2 的信号检测
    exti_init(EXTI_PA4,FALLING,0);
    // 将 GPIOA 的 Pin_5 引脚初始化为上拉输入模式，用于接收编码器 2 的信号
    gpio_init(GPIO_A,Pin_5,IU);
}                     