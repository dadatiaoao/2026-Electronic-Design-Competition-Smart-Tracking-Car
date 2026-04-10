#ifndef _filter_h
#define _filter_h
#include "math.h"

typedef struct
{
	float Q_angle; // 过程噪声-角度分量
	float Q_bias;  // 过程噪声-温漂分量
	float R;       // 测量噪声(加速度计/磁力计)
	float P[2][2]; // 误差协方差矩阵
	float dt;      // 积分时间
	float K1, K2;  // 卡尔曼增益

	
	float Angle;     // 角度估计值
	float Gyro_bias; // 陀螺仪静差估计值

}KF_t;

float Mahony_Filter(float gyro, float acc);
float Kalman_Filter(KF_t *kf, float obsValue, float ut);

extern KF_t KF_Yaw, KF_Roll, KF_Pitch;

#endif
