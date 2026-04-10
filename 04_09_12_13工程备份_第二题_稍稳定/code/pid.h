#ifndef __PID_h_
#define __PID_h_
#include "headfile.h"

enum
{
  POSITION_PID = 0, 
  DELTA_PID,         
};

typedef enum
{
	POINT_A = 0,
	POINT_B,
	POINT_C,
	POINT_D,
}route_point_t;

typedef enum
{
	EVENT_NONE = 0,
	EVENT_SCISSORS,
	EVENT_HAMMER,
	EVENT_LIGHTER,
}vision_event_t;

typedef struct
{
	float target;	
	float now;
	float error[3];		
	float p,i,d;
	float pout, dout, iout;
	float out;   
	
	uint32_t pid_mode;

}pid_t;

void pid_cal(pid_t *pid);
void  pid_control(int basicspeed);

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void motor_target_set(int spe1, int spe2);
void pidout_limit(pid_t *pid);

void auto_route_select(int mode);
void auto_vision_rx_byte(uint8_t byte);
void auto_prompt_service(void);
const char *route_point_name(route_point_t point);
const char *vision_event_name(vision_event_t event);
uint8_t rotate_to_angle(float target_angle, int rotate_speed, uint8_t wait_completion, uint32_t timeout_ms, uint8_t beep_on_finish);
uint8_t angle_aligned(float current_angle, float reference_angle, float tolerance);

extern int control_mode;  // 默认模式（角度环+速度环）

extern pid_t motorA;
extern pid_t motorB;
extern pid_t angle;
extern volatile float jiaodu;
extern volatile int basicspeed;
extern volatile int xunji;

extern volatile uint8_t auto_run_enable;
extern volatile uint8_t route_finished;

// 循迹路程累计相关变量声明
extern volatile float tracking_distance_cm;
extern volatile float distance_threshold_cm;
extern volatile uint8_t distance_threshold_reached;
extern volatile uint8_t tracking_distance_enabled;
extern float pulse_to_cm_factor;

// 路程点配置结构
typedef struct {
    float distance_cm;      // 触发距离（厘米）
	uint8_t triggered;      // 触发状态（0=未触发，1=待处理，2=已处理）
} distance_point_t;

// 路程点控制相关变量声明
extern distance_point_t distance_points[];  // 路程点数组
extern uint8_t distance_point_count;        // 路程点数量
extern volatile uint8_t final_stop_triggered; // 最终停止已触发标志
#endif
