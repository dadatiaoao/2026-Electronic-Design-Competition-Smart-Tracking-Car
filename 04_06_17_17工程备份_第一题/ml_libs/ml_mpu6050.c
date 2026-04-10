#include "ml_mpu6050.h"

int16_t ax, ay, az, gx, gy, gz;
float roll_gyro, pitch_gyro, yaw_gyro;
float roll_acc, pitch_acc, yaw_acc;
float roll_Kalman, pitch_Kalman, yaw_Kalman;


void MPU6050_Write(uint8_t addr, uint8_t dat)
{
	I2C_Start();
	I2C_SendByte(MPU6050_ADDR);
	I2C_WaitAck();
	I2C_SendByte(addr);
	I2C_WaitAck();
	I2C_SendByte(dat);
	I2C_WaitAck();
	I2C_Stop();
}

uint8_t MPU6050_Read(uint8_t addr)
{
	I2C_Start();
	I2C_SendByte(MPU6050_ADDR);
	I2C_WaitAck();
	I2C_SendByte(addr);
	I2C_WaitAck();
	I2C_Stop();
	
	I2C_Start();
	I2C_SendByte(MPU6050_ADDR | 0x01);
	I2C_WaitAck();
	uint8_t dat = I2C_ReceiveByte();
	I2C_NotSendAck();
	I2C_Stop();
	
	return dat;
}

void MPU6050_Init()
{
    // 配置电源管理寄存器1：选择PLL时钟源（参考Y轴陀螺仪）
    MPU6050_Write(PWR_MGMT_1, 0x02);
    // 配置采样率分频寄存器：设置采样率为200Hz（计算公式：2000Hz/(1+0x27)=50Hz？需确认）
    MPU6050_Write(SMPLRT_DIV, 0x27);
    // 配置数字低通滤波器：禁用DLPF（截止频率256Hz）
    MPU6050_Write(CONFIG, 0x00);
    // 配置陀螺仪量程：±2000°/s
    MPU6050_Write(GYRO_CONFIG, 0x18);
    // 配置加速度计量程：±2g
    MPU6050_Write(ACCEL_CONFIG, 0x00);
    // 配置中断使能：使能数据就绪中断
    MPU6050_Write(INT_ENABLE, 0x01);
}

void MPU6050_GetData()
{
	uint8_t data_h, data_l;
	data_h = MPU6050_Read(ACCEL_XOUT_H);
	data_l = MPU6050_Read(ACCEL_XOUT_L);
	ax = data_l | (data_h << 8);

	data_h = MPU6050_Read(ACCEL_YOUT_H);
	data_l = MPU6050_Read(ACCEL_YOUT_L);
	ay = data_l | (data_h << 8);

	data_h = MPU6050_Read(ACCEL_ZOUT_H);
	data_l = MPU6050_Read(ACCEL_ZOUT_L);
	az = data_l | (data_h << 8);
	
	data_h = MPU6050_Read(GYRO_XOUT_H);
	data_l = MPU6050_Read(GYRO_XOUT_L);
	gx = data_l | (data_h << 8);

	data_h = MPU6050_Read(GYRO_YOUT_H);
	data_l = MPU6050_Read(GYRO_YOUT_L);
	gy = data_l | (data_h << 8);
	
	data_h = MPU6050_Read(GYRO_ZOUT_H);
	data_l = MPU6050_Read(GYRO_ZOUT_L);
	gz = data_l | (data_h << 8);
//	
//	gz-=-20;
//    gz -= -24;
     gz -= 13;
}



