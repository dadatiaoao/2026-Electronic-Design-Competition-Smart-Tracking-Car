#include "servo.h"

// 舵机参数
#define SERVO_FREQUENCY 50     // 50Hz（20ms周期）
#define PULSE_MIN_MS    0.5f   // 0度对应脉宽
#define PULSE_MAX_MS    2.5f   // 180度对应脉宽
#define PULSE_RANGE_MS  (PULSE_MAX_MS - PULSE_MIN_MS)  // 2.0ms

/**
 * @brief 舵机初始化
 * @param channel 舵机通道（SERVO_CH1~SERVO_CH4）
 */
void servo_init(ServoChannel_t channel)
{
    // 初始化PWM，频率50Hz
    pwm_init(TIM_4, (TIMn_CHn_enum)channel, SERVO_FREQUENCY);
    // 设置初始位置为0度
    servo_set_angle(channel, 0);
}

/**
 * @brief 设置舵机角度
 * @param channel 舵机通道
 * @param angle 角度值（0-180度）
 */
void servo_set_angle(ServoChannel_t channel, uint8_t angle)
{
    // 角度限制在0-180度
    if (angle > 180) angle = 180;

    // 角度转换为脉宽（毫秒）
    float pulse_ms = PULSE_MIN_MS + (angle * PULSE_RANGE_MS / 180.0f);

    // 调用脉宽设置函数
    servo_set_pulse(channel, pulse_ms);
}

/**
 * @brief 设置舵机脉宽
 * @param channel 舵机通道
 * @param pulse_ms 脉宽值（0.5-2.5ms）
 */
void servo_set_pulse(ServoChannel_t channel, float pulse_ms)
{
    // 脉宽限制
    if (pulse_ms < PULSE_MIN_MS) pulse_ms = PULSE_MIN_MS;
    if (pulse_ms > PULSE_MAX_MS) pulse_ms = PULSE_MAX_MS;

    // 计算占空比
    // 周期20ms = 20000us
    // 占空比 = 脉宽/周期 * MAX_DUTY
    // MAX_DUTY在ml_pwm.h中定义为50000
    uint32_t duty = (uint32_t)((pulse_ms / 20.0f) * 50000.0f);

    // 更新PWM输出
    pwm_update(TIM_4, (TIMn_CHn_enum)channel, duty);
}