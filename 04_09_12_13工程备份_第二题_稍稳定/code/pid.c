#include "headfile.h"

// 定义三个 PID 结构体变量，分别用于电机 A、电机 B 和角度控制
pid_t motorA;
pid_t motorB;
pid_t angle;
volatile float jiaodu=0;
volatile int basicspeed;
volatile int xunji=1;

volatile uint8_t auto_run_enable = 1;
volatile uint8_t route_finished = 0;

// 循迹路程累计相关变量
volatile float tracking_distance_cm = 0.0f;        // 累计循迹路程（厘米）
volatile float distance_threshold_cm = 345.3f;     // 停止阈值（厘米），可调
volatile uint8_t distance_threshold_reached = 0;   // 阈值达到标志（1=已到达）
volatile uint8_t tracking_distance_enabled = 0;    // 路程累计使能标志（1=正在累计）
float pulse_to_cm_factor = 0.0776f;                // 脉冲到厘米转换因子（默认值）

// 路程点配置数组（可自定义修改）
distance_point_t distance_points[] = {
    {72.6f, 0},   //
    {168.8f, 0},  // 
    {246.3f, 0},  //
    {342.6f, 0},  // 
    // 可根据需要添加更多点
};

uint8_t distance_point_count = sizeof(distance_points) / sizeof(distance_points[0]);
volatile uint8_t final_stop_triggered = 0;  // 最终停止已触发标志

static uint8_t route_mode = 1;
static uint8_t route_index = 1;
static volatile uint8_t prompt_pending = 0;
static volatile route_point_t prompt_point = POINT_A;
static volatile vision_event_t prompt_event = EVENT_NONE;

static char vision_line[40];
static uint8_t vision_line_len = 0;

static const route_point_t route_one[5] = {POINT_A, POINT_B, POINT_C, POINT_D, POINT_A};
static const route_point_t route_two[5] = {POINT_A, POINT_C, POINT_B, POINT_D, POINT_A};

static char route_point_char(route_point_t point)
{
    switch(point)
    {
        case POINT_A: return 'A';
        case POINT_B: return 'B';
        case POINT_C: return 'C';
        case POINT_D: return 'D';
        default: return '?';
    }
}

static vision_event_t parse_event_token(const char *token)
{
    if(token == NULL)
    {
        return EVENT_NONE;
    }

    if(strstr(token, "SCISS") != NULL || strstr(token, "JIAN") != NULL)
    {
        return EVENT_SCISSORS;
    }
    if(strstr(token, "HAMMER") != NULL || strstr(token, "CHUI") != NULL)
    {
        return EVENT_HAMMER;
    }
    if(strstr(token, "LIGHT") != NULL || strstr(token, "DAHUO") != NULL)
    {
        return EVENT_LIGHTER;
    }

    return EVENT_NONE;
}

const char *route_point_name(route_point_t point)
{
    switch(point)
    {
        case POINT_A: return "A";
        case POINT_B: return "B";
        case POINT_C: return "C";
        case POINT_D: return "D";
        default: return "?";
    }
}

const char *vision_event_name(vision_event_t event)
{
    switch(event)
    {
        case EVENT_SCISSORS: return "SCISSORS";
        case EVENT_HAMMER: return "HAMMER";
        case EVENT_LIGHTER: return "LIGHTER";
        default: return "NONE";
    }
}

static const route_point_t *active_route_table(void)
{
    return (route_mode == 2) ? route_two : route_one;
}

static void route_finish(void)
{
    route_finished = 1;
    auto_run_enable = 0;
    basicspeed = 0;
    xunji = 0;
    motor_target_set(0, 0);
    motorA.out = 0;
    motorB.out = 0;
    motorA_duty(0);
    motorB_duty(0);
}

static void route_apply_speed(uint8_t step)
{
    switch(step)
    {
        case 1:
            basicspeed = 120;
            break;
        case 2:
            basicspeed = 110;
            break;
        case 3:
            basicspeed = 110;
            break;
        default:
            basicspeed = 90;
            break;
    }
}

static void route_on_point(route_point_t point, vision_event_t event)
{
    const route_point_t *route = active_route_table();

    if(route_finished)
    {
        return;
    }

    if(point != route[route_index])
    {
        return;
    }

    prompt_point = point;
    prompt_event = event;
    prompt_pending = 1;

    if(route_index >= 4)
    {
        route_finish();
        return;
    }

    route_index++;
    route_apply_speed(route_index);
}

static void vision_parse_frame(char *frame)
{
    char *comma;
    route_point_t point;
    vision_event_t event;

    if(frame == NULL)
    {
        return;
    }

    while(*frame == '#' || *frame == '$' || *frame == ' ')
    {
        frame++;
    }

    comma = strchr(frame, ',');
    if(comma == NULL)
    {
        return;
    }

    *comma = '\0';
    point = POINT_A;
    if(frame[0] == 'B' || frame[0] == 'b')
    {
        point = POINT_B;
    }
    else if(frame[0] == 'C' || frame[0] == 'c')
    {
        point = POINT_C;
    }
    else if(frame[0] == 'D' || frame[0] == 'd')
    {
        point = POINT_D;
    }

    event = parse_event_token(comma + 1);
    route_on_point(point, event);
}

void auto_route_select(int mode)
{
    route_mode = (mode == 2) ? 2 : 1;
    route_index = 1;
    auto_run_enable = 1;
    route_finished = 0;
    prompt_pending = 1;
    prompt_point = POINT_A;
    prompt_event = EVENT_NONE;
    basicspeed = 120;
    jiaodu = 0;
    xunji = 1;
}

void auto_vision_rx_byte(uint8_t byte)
{
    if(byte == '\r')
    {
        return;
    }

    if(byte == '\n')
    {
        vision_line[vision_line_len] = '\0';
        vision_parse_frame(vision_line);
        vision_line_len = 0;
        return;
    }

    if(vision_line_len < sizeof(vision_line) - 1)
    {
        vision_line[vision_line_len++] = (char)byte;
    }
    else
    {
        vision_line_len = 0;
    }
}

void auto_prompt_service(void)
{
    char voice_cmd[48];
    char local_point;
    vision_event_t local_event;

    if(!prompt_pending)
    {
        return;
    }

    __disable_irq();
    local_point = route_point_char(prompt_point);
    local_event = prompt_event;
    prompt_pending = 0;
    __enable_irq();

    Enable_LED();
    Enable_Buzzer();

    sprintf(voice_cmd, "POINT=%c,EVENT=%s\r\n", local_point, vision_event_name(local_event));
    uart_sendstr(UART_3, voice_cmd);
    printf("PROMPT:%c,%s\r\n", local_point, vision_event_name(local_event));
}

/**
 * @brief 检查路程点是否到达
 * @details 在pid_control()函数中调用，检查是否到达预设的路程点
 */
static void check_distance_points(void) {
    if (!tracking_distance_enabled || distance_threshold_reached) {
        return;
    }

    // 检查各个路程点：仅从“未触发(0)”切换到“待处理(1)”一次
    for (uint8_t i = 0; i < distance_point_count; i++) {
        if (distance_points[i].triggered == 0 &&
            tracking_distance_cm >= distance_points[i].distance_cm) {
            distance_points[i].triggered = 1;  // 标记为待主循环处理
        }
    }
}

/**
 * @brief PID 初始化函数，用于初始化 PID 结构体的参数
 * @param pid 指向要初始化的 PID 结构体的指针
 * @param mode PID 控制模式，如增量式或位置式
 * @param p 比例系数
 * @param i 积分系数
 * @param d 微分系数
 * @details 该函数将传入的 PID 控制模式、比例系数、积分系数和微分系数赋值给 PID 结构体的相应成员
 */
void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
    pid->pid_mode = mode;  // 设置 PID 控制模式
    pid->p = p;            // 设置比例系数
    pid->i = i;            // 设置积分系数
    pid->d = d;            // 设置微分系数
}

/**
 * @brief 电机目标速度设置函数，用于设置电机 A 和电机 B 的目标速度
 * @param spe1 电机 A 的目标速度
 * @param spe2 电机 B 的目标速度
 * @details 该函数根据传入的目标速度的正负，设置电机的转动方向，并将目标速度的绝对值赋值给 PID 结构体的目标值成员
 */
  void motor_target_set(int spe1, int spe2)
{
    if(spe1 >= 0)
    {
        motorA_dir = 1;  // 电机 A 正转
        motorA.target = spe1;  // 设置电机 A 的目标速度
    }
    else
    {
        motorA_dir = 0;  // 电机 A 反转
        motorA.target = -spe1;  // 设置电机 A 的目标速度为绝对值
    }

    if(spe2 >= 0)
    {
        motorB_dir = 1;  // 电机 B 正转
        motorB.target = spe2;  // 设置电机 B 的目标速度
    }
    else
    {
        motorB_dir = 0;  // 电机 B 反转
        motorB.target = -spe2;  // 设置电机 B 的目标速度为绝对值
    }
}

/**
 * @brief PID 控制主函数，实现角度和电机速度的 PID 控制
 * @details 该函数首先进行角度的 PID 控制，然后根据角度控制的输出设置电机的目标速度，再进行电机速度的 PID 控制，最后对 PID 输出进行限幅并应用到电机上
 */
void pid_control(int basicspeed)
{
  if(!auto_run_enable)
  {
      motor_target_set(0, 0);
      motorA.out = 0;
      motorB.out = 0;
      motorA_duty(0);
      motorB_duty(0);
      return;
  }

  // 如果基础速度为0，停止电机
  if(basicspeed == 0)
  {
      motor_target_set(0, 0);
			motorA.out = 0;
      motorB.out = 0;
      motorA_duty(0);
      motorB_duty(0);
      return;
  }

  // 注意：阈值达到逻辑已移至main.c主循环中处理
  // 这里不再主动停止电机，只统计路程
 
	angle.target = jiaodu;
  angle.now = yaw_gyro;
	//angle.now = yaw_Kalman;
  pid_cal(&angle);
  motor_target_set(basicspeed-angle.out,basicspeed+angle.out);
	if(xunji==1)
	{
			gray_read();//先读传感器再循迹
			track();
	}
	if(motorA_dir){motorA.now = Encoder_count1;}else{motorA.now = -Encoder_count1;}
  if(motorB_dir){motorB.now = Encoder_count2;}else{motorB.now = -Encoder_count2;}

    // 路程累计逻辑：行驶时统一累计，和循迹开关解耦，便于上层FSM在直行/循迹阶段都可用
  if (!distance_threshold_reached) {
                tracking_distance_enabled = 1;
                // 计算本次10ms间隔的平均脉冲数（取绝对值，忽略方向）
                float avg_pulses = (fabsf(Encoder_count1) + fabsf(Encoder_count2)) / 2.0f;
                // 转换为距离并累加
                tracking_distance_cm += avg_pulses * pulse_to_cm_factor;
                // 检查是否达到阈值
                if (tracking_distance_cm >= distance_threshold_cm) {
                        distance_threshold_reached = 1;
                        // 注意：停止逻辑已移至main.c主循环中处理
                }
                // 保留兼容：若上层仍在使用预设路程点，继续更新触发标志
                check_distance_points();
        }

   Encoder_count1 = 0;
   Encoder_count2 = 0;
   pid_cal(&motorA);
   pid_cal(&motorB);
   pidout_limit(&motorA);
   pidout_limit(&motorB);
	// 4. 将 PID 计算得到的输出应用到电机上，控制电机的占空比
   motorA_duty(motorA.out);
   motorB_duty(motorB.out);

}
/**
 * @brief PID 计算函数，根据不同的 PID 控制模式进行计算
 * @param pid 指向要进行计算的 PID 结构体的指针
 * @details 该函数根据 PID 结构体的控制模式（增量式或位置式），计算比例项、积分项和微分项，并更新 PID 输出
 */
void pid_cal(pid_t *pid)
{
    // 计算当前误差
    pid->error[0] = pid->target - pid->now;

    // 根据 PID 控制模式进行计算
    if(pid->pid_mode == DELTA_PID)  // 增量式 PID
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);// 计算比例项，当前误差与上一次误差的差值乘以比例系数
		pid->iout = pid->i * pid->error[0]; // 计算积分项，当前误差乘以积分系数
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]); // 计算微分项，当前误差减去两倍的上一次误差加上上上次误差，再乘以微分系数
        pid->out += pid->pout + pid->iout + pid->dout; // 更新 PID 输出，累加增量
    }
    else if(pid->pid_mode == POSITION_PID)  // 位置式 PID
    {
        pid->pout = pid->p * pid->error[0];// 计算比例项，当前误差乘以比例系数
        pid->iout += pid->i * pid->error[0];// 计算积分项，累加当前误差乘以积分系数
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);// 计算微分项，当前误差与上一次误差的差值乘以微分系数
        pid->out = pid->pout + pid->iout + pid->dout;   // 更新 PID 输出
    }

    // 记录上一次和上上次的误差
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

/**
 * @brief PID 输出限幅函数，确保 PID 输出在合理范围内
 * @param pid 指向要进行限幅的 PID 结构体的指针
 * @details 该函数将 PID 输出限制在 0 到 MAX_DUTY 之间，避免输出过大或过小
 */
void pidout_limit(pid_t *pid)
{
    // 若输出大于等于最大占空比，则将输出设置为最大占空比
    if(pid->out>=MAX_DUTY)	 
        pid->out=MAX_DUTY;
    // 若输出小于等于 0，则将输出设置为 0
    if(pid->out<=0)	
        pid->out=0;
}

uint8_t rotate_to_angle(float target_angle, int rotate_speed,
                        uint8_t wait_completion, uint32_t timeout_ms,
                        uint8_t beep_on_finish)
{
    // 旋转控制参数
    #define ANGLE_ERROR_THRESH  2.0f      // 角度误差阈值（度）
    #define LOOP_DELAY_MS       10        // 等待循环延时（毫秒）

    float current_angle, angle_error;
    uint32_t timeout_counter = 0;
    uint32_t max_loops = 0;
    uint8_t rotate_completed = 0;

    // 1. 保存当前角度，计算绝对目标角度
    current_angle = yaw_gyro;
    jiaodu = current_angle + target_angle;  // 目标角度 = 当前角度 + 旋转角度

    // 2. 设置旋转速度（较小的速度实现原地旋转）
    basicspeed = rotate_speed;

    printf("[旋转开始] 当前:%.2f°, 目标:%.2f°, 旋转:%.2f°, 速度:%d\n",
           current_angle, jiaodu, target_angle, rotate_speed);

    // 3. 如果不等待完成，直接返回
    if (!wait_completion) {
        printf("[旋转设置完成] 不等待，立即返回\n");
        return 1;
    }

    // 4. 计算最大等待循环数
    max_loops = timeout_ms / LOOP_DELAY_MS;
    if (max_loops == 0) max_loops = 1;

    // 5. 等待旋转完成循环
    while (!rotate_completed) {
        xunji = 0;
				// 计算当前角度误差
        angle_error = fabs(jiaodu - yaw_gyro);

        // 检查是否达到目标精度
        if (angle_error < ANGLE_ERROR_THRESH) {
            rotate_completed = 1;
            printf("[旋转完成] 当前:%.2f°, 目标:%.2f°, 误差:%.2f°, 用时:%lums\n",
                   yaw_gyro, jiaodu, angle_error, timeout_counter * LOOP_DELAY_MS);
            break;
        }

        // 检查超时
        timeout_counter++;
        if (timeout_counter >= max_loops) {
            rotate_completed = 1;  // 强制标记完成，避免死锁
            printf("[旋转超时] 当前:%.2f°, 目标:%.2f°, 误差:%.2f°, 超时:%lums\n",
                   yaw_gyro, jiaodu, angle_error, timeout_ms);
            break;
        }

        // 调试输出（每30次循环输出一次）
        static uint32_t debug_counter = 0;
        debug_counter++;
        if (debug_counter % 30 == 0) {
            printf("[旋转中] 误差:%.2f°, 进度:%lu/%lu\n",
                   angle_error, timeout_counter, max_loops);
        }

        // 延时，防止CPU占用过高
        delay_ms(LOOP_DELAY_MS);
    }

    // 旋转完成后，将目标角度设置为当前角度，消除残留误差
    jiaodu = yaw_gyro;
    // 重置角度PID的历史误差和输出，防止残留影响直线行走
    angle.error[0] = 0;
    angle.error[1] = 0;
    angle.error[2] = 0;
    angle.iout = 0;
    angle.out = 0;

    // 6. 旋转完成提示
    if (beep_on_finish) {
        Enable_Buzzer();
        delay_ms(200);
    }

    // 7. 返回旋转状态（1=成功，0=超时）
    return (angle_error < ANGLE_ERROR_THRESH) ? 1 : 0;
}

uint8_t angle_aligned(float current_angle, float reference_angle, float tolerance) {
    float error = current_angle - reference_angle;

    // 处理角度环绕（-180到180度）
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    return (fabs(error) < tolerance);
}
