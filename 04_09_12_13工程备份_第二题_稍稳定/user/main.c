#include "headfile.h"
#include "../code/gray_track.h"

volatile int uart_received_data = 0;

/*
 * 题二：8字路线状态机版本
 * 说明：
 * 1) 10ms控制闭环仍在TIM3中断内运行（pid_control）。
 * 2) 本文件只在主循环中做状态切换，不在中断中加入重计算。
 * 3) 通过 tracking_distance_cm 统一判定各段到点，节点动作里清零里程。
 */

/* ==================== 模块1：几何常量与宏定义 ==================== */
#define DIST_DIAGONAL_CM        102.0f//对角线
#define DIST_ARC_CM             83.0f//弧线//弧线长度修正值79.1，原125.6
#define ANGLE_DIAGONAL          50.0f//旋转角度

#define SPEED_STRAIGHT          150//直行速度
#define SPEED_TRACK             130//循迹速度
#define SPEED_TURN              90//转弯速度

/* 角度微调接口（赛道实测后只改这里） */
#define TRIM_TURN_AC            +15.0f
#define TRIM_ALIGN_CB           -5.0f
#define TRIM_TURN_BD            +5.0f
#define TRIM_ALIGN_DA           -10.0f

/* 若旋转方向相反，改下面4个符号即可（+1或-1） */
#define SIGN_TURN_AC            (-1.0f)
#define SIGN_ALIGN_CB           (+1.0f)
#define SIGN_TURN_BD            (+1.0f)
#define SIGN_ALIGN_DA           (-1.0f)

/* 安全参数 */
#define TURN_TIMEOUT_MS         2600U
#define FINISH_STOP_MS          500U
#define DIST_REACH_EPS_CM       1.0f

/* 角度对齐条件 */
#define ANGLE_ALIGN_TOLERANCE   6.0f          // 角度对齐容差（度）

/* ==================== 模块2：状态机枚举定义 ==================== */
typedef enum
{
    STATE_INIT = 0,
    STATE_TURN_AC,      /* A点向内旋转，准备A->C对角直行 */
    STATE_GO_C,         /* A->C 直行 */
    STATE_ALIGN_CB,     /* C点对齐，准备进入右侧半弧循迹 */
    STATE_TRACK_CB,     /* C->B 半弧循迹 */
    STATE_TURN_BD,      /* B点向内旋转，准备B->D对角直行 */
    STATE_GO_D,         /* B->D 直行 */
    STATE_ALIGN_DA,     /* D点对齐，准备进入左侧半弧循迹 */
    STATE_TRACK_DA,     /* D->A 半弧循迹 */
    STATE_FINISH,
    STATE_IDLE
} Task2_State_t;

static Task2_State_t g_task2_state = STATE_INIT;
static float g_initial_yaw = 0.0f;           // 初始角度记录

/* ==================== 外部变量声明 ==================== */
extern pid_t motorA;
extern pid_t motorB;
extern pid_t angle;
extern volatile float jiaodu;
extern volatile int basicspeed;
extern volatile int xunji;

extern volatile float tracking_distance_cm;
extern volatile float distance_threshold_cm;
extern volatile uint8_t distance_threshold_reached;
extern volatile uint8_t tracking_distance_enabled;
extern float pulse_to_cm_factor;

/* ==================== 模块3：节点触发与声光提示函数 ==================== */
void node_action(void)
{
    Enable_Buzzer();
    Enable_LED();

    __disable_irq();
    tracking_distance_cm = 0.0f;
    distance_threshold_reached = 0;
    __enable_irq();
}

/* 小工具：读取里程（与ISR并发，做一次快照） */
static float distance_cm_snapshot(void)
{
    float d;
    __disable_irq();
    d = tracking_distance_cm;
    __enable_irq();
    return d;
}

/* ==================== 模块4：轨迹检测辅助函数 ==================== */
/* 检测是否有传感器检测到黑线（至少一个传感器为1） */
static uint8_t detect_track(void)
{
    uint8_t detected;
    /* 由于xunji可能为0，需要主动读取传感器 */
    __disable_irq();
    gray_read();
    /* 检查D1-D8中是否有检测到黑线 */
    detected = (D1 || D2 || D3 || D4 || D5 || D6 || D7 || D8);
    __enable_irq();
    return detected;
}

/* ==================== 模块5：核心状态机流转控制 ==================== */
void Task2_Process(void)
{
    float dist_cm = distance_cm_snapshot();

    switch (g_task2_state)
    {
        case STATE_INIT:
            /* 起点A，车头朝B；先静态清零里程，再进入首段转向 */
            xunji = 0;
            basicspeed = 0;
            jiaodu = yaw_Kalman;
            g_initial_yaw = yaw_Kalman;  // 记录初始角度
            node_action();
            g_task2_state = STATE_TURN_AC;
            printf("[T2] INIT -> TURN_AC, 初始角度=%.2f\r\n", g_initial_yaw);
            break;

        case STATE_TURN_AC://转向对角线
            xunji = 0;
            basicspeed = SPEED_STRAIGHT;
            (void)rotate_to_angle(SIGN_TURN_AC * ANGLE_DIAGONAL + TRIM_TURN_AC,
                                  SPEED_TURN, 1, TURN_TIMEOUT_MS, 0);
            node_action();
            g_task2_state = STATE_GO_C;
            printf("[T2] TURN_AC -> GO_C\r\n");
            break;

        case STATE_GO_C://直走
            xunji = 0;
            basicspeed = SPEED_STRAIGHT;
            if (dist_cm >= (DIST_DIAGONAL_CM - DIST_REACH_EPS_CM) || detect_track())
            {
                node_action();
                g_task2_state = STATE_ALIGN_CB;
                printf("[T2] Reach C, GO_C -> ALIGN_CB (trigger: %s)\r\n",
                       dist_cm >= (DIST_DIAGONAL_CM - DIST_REACH_EPS_CM) ? "distance" : "track detected");
            }
            break;

        case STATE_ALIGN_CB://转向轨迹
            xunji = 0;
            basicspeed = SPEED_TRACK;
            (void)rotate_to_angle(SIGN_ALIGN_CB * ANGLE_DIAGONAL + TRIM_ALIGN_CB,
                                  SPEED_TURN, 1, TURN_TIMEOUT_MS, 0);
            node_action();
            xunji = 1;
            basicspeed = SPEED_TRACK;
            g_task2_state = STATE_TRACK_CB;
            printf("[T2] ALIGN_CB -> TRACK_CB\r\n");
            break;

        case STATE_TRACK_CB://循迹
            xunji = 1;
            basicspeed = SPEED_TRACK;
            if (dist_cm >= (DIST_ARC_CM - DIST_REACH_EPS_CM) &&
                angle_aligned(yaw_Kalman, g_initial_yaw, ANGLE_ALIGN_TOLERANCE))
            {
                node_action();
                xunji = 0;
                g_task2_state = STATE_TURN_BD;
                printf("[T2] Reach B, 角度对齐, TRACK_CB -> TURN_BD\r\n");
            }
            break;

        case STATE_TURN_BD://转向对角线
            xunji = 0;
            basicspeed = SPEED_STRAIGHT;
            (void)rotate_to_angle(SIGN_TURN_BD * ANGLE_DIAGONAL + TRIM_TURN_BD,
                                  SPEED_TURN, 1, TURN_TIMEOUT_MS, 0);
            node_action();
            g_task2_state = STATE_GO_D;
            printf("[T2] TURN_BD -> GO_D\r\n");
            break;

        case STATE_GO_D://直走
            xunji = 0;
            basicspeed = SPEED_STRAIGHT;
            if (dist_cm +10>= (DIST_DIAGONAL_CM - DIST_REACH_EPS_CM) || detect_track())
            {
                node_action();
                g_task2_state = STATE_ALIGN_DA;
                printf("[T2] Reach D, GO_D -> ALIGN_DA (trigger: %s)\r\n",
                       dist_cm +10>= (DIST_DIAGONAL_CM - DIST_REACH_EPS_CM) ? "distance" : "track detected");
            }
            break;

        case STATE_ALIGN_DA://转向轨迹
            xunji = 0;
            basicspeed = SPEED_TRACK;
            (void)rotate_to_angle(SIGN_ALIGN_DA * ANGLE_DIAGONAL + TRIM_ALIGN_DA,
                                  SPEED_TURN, 1, TURN_TIMEOUT_MS, 0);
            node_action();
            xunji = 1;
            basicspeed = SPEED_TRACK;
            g_task2_state = STATE_TRACK_DA;
            printf("[T2] ALIGN_DA -> TRACK_DA\r\n");
            break;

        case STATE_TRACK_DA://循迹
            xunji = 1;
            basicspeed = SPEED_TRACK;
            if (dist_cm >= (DIST_ARC_CM - DIST_REACH_EPS_CM) &&
                angle_aligned(yaw_Kalman, g_initial_yaw, ANGLE_ALIGN_TOLERANCE))
            {
                node_action();
                g_task2_state = STATE_FINISH;
                printf("[T2] Reach A, 角度对齐, TRACK_DA -> FINISH\r\n");
            }
            break;

        case STATE_FINISH://停止
            xunji = 0;
            basicspeed = 0;
            motor_target_set(0, 0);
            motorA_duty(0);
            motorB_duty(0);
            delay_ms(FINISH_STOP_MS);
            g_task2_state = STATE_IDLE;
            printf("[T2] FINISH -> IDLE\r\n");
            break;

        case STATE_IDLE:
        default:
            xunji = 0;
            basicspeed = 0;
            break;
    }
}

/* ==================== 模块5：主函数集成（精简框架） ==================== */
int main(void)
{
    /* 1) 硬件初始化（沿用现工程初始化顺序） */
    Buzzer_Init();
    OLED_Init();
    motor_init();
    encoder_init();
    Key_Init();
    LED_Init();
    uart_init(UART_1, 115200, 0x00);
    gray_init();
    I2C_Init();
    MPU6050_Init();

    /* 2) PID参数初始化 */
    pid_init(&motorA, DELTA_PID, 100.0f, 0.0f, 5.0f);
    pid_init(&motorB, DELTA_PID, 100.0f, 0.0f, 5.0f);
    pid_init(&angle, POSITION_PID, 2.0f, 0.0f, 0.1f);

    /* 3) 中断初始化（10ms控制周期） */
    exti_init(EXTI_PB7, RISING, 0);
    tim_interrupt_ms_init(TIM_3, 10, 0);

    /* 4) 初始姿态与里程配置 */
    xunji = 0;
    basicspeed = 0;
    jiaodu = yaw_gyro;

    tracking_distance_cm = 0.0f;
    tracking_distance_enabled = 1;
    distance_threshold_reached = 0;
    distance_threshold_cm = 10000.0f; /* 题二由FSM自行判断，不依赖全局阈值停机 */
    pulse_to_cm_factor = 0.0776f;

    node_action();
    printf("[T2] Start 8-route FSM\r\n");

    while (1)
    {
        Task2_Process();
        delay_ms(10);
    }
}
