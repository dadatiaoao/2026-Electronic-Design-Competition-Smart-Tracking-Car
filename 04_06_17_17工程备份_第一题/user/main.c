#include "headfile.h"

// 全局变量声明
int mission = 0;
int uart_received_data = 0;
uint16_t cnt = 0;

// PID控制相关外部变量（在pid.c中定义）
extern pid_t motorA;
extern pid_t motorB;
extern pid_t angle;
extern volatile float jiaodu;
extern volatile int basicspeed;
extern volatile int xunji;

// 循迹路程累计相关外部变量（在pid.c中定义）
extern volatile float tracking_distance_cm;
extern volatile float distance_threshold_cm;
extern volatile uint8_t distance_threshold_reached;
extern volatile uint8_t tracking_distance_enabled;
extern float pulse_to_cm_factor;

// 路程点控制相关外部变量（在pid.c中定义）
extern distance_point_t distance_points[];  // 路程点数组
extern uint8_t distance_point_count;        // 路程点数量
extern volatile uint8_t final_stop_triggered; // 最终停止已触发标志

// MPU6050相关外部变量（在ml_mpu6050.c中定义）
extern int16_t ax, ay, az, gx, gy, gz;
extern float yaw_gyro, yaw_acc, yaw_Kalman;

/**
 * @brief 主函数 - 黑线循迹小车控制系统
 * @details 初始化所有外设后，小车自动检测黑线并循迹
 *          控制逻辑在TIM3中断（10ms周期）中执行
 */
int main(void)
{
    // 1. 系统外设初始化
    Buzzer_Init();              // 蜂鸣器初始化
		OLED_Init();                // OLED显示屏初始化
    motor_init();               // 电机PWM和方向控制初始化
    encoder_init();             // 编码器外部中断初始化
    Key_Init();                 // 按键初始化
    LED_Init();                 // LED指示灯初始化
    uart_init(UART_1, 115200, 0x00);  // 串口1初始化，115200波特率
    gray_init();                // 8路灰度传感器初始化
    I2C_Init();                 // I2C总线初始化
    MPU6050_Init();             // MPU6050六轴传感器初始化

    // 2. PID控制器参数初始化
    // 电机A速度环PID（增量式PID）
    pid_init(&motorA, DELTA_PID, 100.0f, 0.0f, 5.0f);
    // 电机B速度环PID（增量式PID）
    pid_init(&motorB, DELTA_PID, 100.0f, 0.0f, 5.0f);
    // 角度环PID（位置式PID） - 用于车身角度控制
    pid_init(&angle, POSITION_PID, 2.0f, 0.0f, 0.1f);

    // 3. 中断配置
    exti_init(EXTI_PB7, RISING, 0);        // 外部中断PB7，上升沿触发
    tim_interrupt_ms_init(TIM_3, 10, 0);   // 定时器3中断，10ms周期，优先级0

    // 4. 系统初始状态设置
    xunji = 1;      // 使能循迹模式（1=开启循迹，0=关闭循迹）
    jiaodu = 0;     // 初始目标角度为0度（直行）
    basicspeed = 130; // 先降速提稳，后续可按赛道再上调

    // 循迹路程阈值设置
    distance_threshold_cm = 457.2f;     // 默认阈值
    tracking_distance_cm = 0.0f;        // 清零累计路程
    distance_threshold_reached = 0;     // 重置阈值到达标志（1=已到达）
    tracking_distance_enabled = 0;      // 初始未使能累计
    pulse_to_cm_factor = 0.0776f;       // 默认转换因子（需校准）0.0567f

    // 重置路程点状态
    for (uint8_t i = 0; i < distance_point_count; i++) {
        distance_points[i].triggered = 0;
    }
    final_stop_triggered = 0;

    // 5. 系统启动提示
    Enable_Buzzer();    // 蜂鸣器响一声表示系统启动
    Enable_LED();       // LED闪烁一次表示系统启动
    delay_ms(500);      // 短暂延时
		
    // 6. 主循环 - 黑线循迹模式
		static uint32_t last_display = 0;
		static uint32_t angle_print_counter = 0;
		const uint32_t ANGLE_PRINT_INTERVAL = 50; // 50 * 10ms = 500ms
		printf("启动");
    while (1)
    {
				last_display++;
				if (last_display > 1000) {  // 每3秒显示一次
						//printf("当前距离: %.1fcm\n", tracking_distance_cm);
						last_display = 0;
				}

        // 角度输出控制（每500ms输出一次）
        angle_print_counter++;
        if (angle_print_counter >= ANGLE_PRINT_INTERVAL) {
            // 计算Z轴角速度（度/秒）
						float gyro_z_deg = (float)gz / 16.4f;

            // 输出角速度和卡尔曼滤波角度
            printf("GyroZ: %.2f deg/s, \n Angle: %.2f deg\n", gyro_z_deg, yaw_Kalman);

            angle_print_counter = 0; // 重置计数器
        }

        // 注意：主要控制逻辑在TIM3中断服务程序中执行
        // TIM3每10ms中断一次，调用pid_control()函数
        // pid_control()中会根据xunji标志调用track()进行循迹

        // 路程点检测与控制逻辑
        // 检查路程点触发
        for (uint8_t i = 0; i < distance_point_count; i++) {
            if (distance_points[i].triggered == 1) {
                Enable_Buzzer();  // 触发蜂鸣器
								Enable_LED();
                distance_points[i].triggered = 2;  // 标记为已处理，避免重复触发
                printf("到达路程点: %.1fcm\n", distance_points[i].distance_cm);
            }
        }

        // 检查最终停止
        if (distance_threshold_reached && !final_stop_triggered) {
            final_stop_triggered = 1;
            xunji = 0;      // 停止循迹模式
            basicspeed = 0; // 停止电机基础速度
            Enable_Buzzer(); // 最终停止提示
            delay_ms(200);  // 稍长提示音
            printf("到达最终阈值: %.1fcm，停止小车\n", distance_threshold_cm);
        }

        // 简单延时，防止CPU占用率过高
        delay_ms(10);
    }
}

/**
 * @brief 系统工作流程说明：
 *
 * 1. 系统启动流程：
 *    - 初始化所有外设（电机、传感器、显示、通信等）
 *    - 配置PID控制器参数
 *    - 设置中断（定时器中断10ms）
 *    - 启动提示（蜂鸣器+LED）
 *    - 进入主循环
 *
 * 2. 循迹控制流程（在TIM3中断中）：
 *    - 每10ms执行一次pid_control()
 *    - pid_control()函数中：
 *        a. 读取MPU6050角度数据（如果使用角度环）
 *        b. 如果xunji=1，调用track()函数：
 *           根据8路灰度传感器状态调整左右电机速度差
 *        c. 计算速度环PID（编码器反馈）
 *        d. 输出PWM控制电机
 *
 * 3. 传感器布局与循迹逻辑：
 *    - 8路灰度传感器从左到右：D1, D2, D3, D4, D5, D6, D7, D8
 *    - 传感器值：1=检测到黑线，0=未检测到黑线
 *    - track()函数（在gray_track.c中）根据传感器组合调整电机速度
 *    - 典型情况：
 *        * 中间传感器（D4,D5）检测到黑线：直行（左右电机速度相同）
 *        * 左侧传感器检测到黑线：右转（左电机加速，右电机减速）
 *        * 右侧传感器检测到黑线：左转（右电机加速，左电机减速）
 *
 * 4. 关键参数调整：
 *    - basicspeed：基础前进速度，影响循迹速度
 *    - PID参数：在pid_init()调用中调整
 *    - 速度差参数：在gray_track.c的track()函数中调整
 *    - 循迹灵敏度：通过gray_track.c中的传感器阈值调整
 *
 * 5. 扩展功能预留：
 *    - 按键选择不同循迹模式
 *    - 串口指令控制
 *    - OLED状态显示
 *    - MPU6050角度辅助控制
 */