#include "ml_hmc5883l.h"

int16_t hmc_x, hmc_y, hmc_z;
float yaw_hmc;

void HMC5883L_Write(uint8_t addr, uint8_t dat)
{
	I2C_Start();
	I2C_SendByte(HMC5883L_ADDR);
	I2C_WaitAck();
	I2C_SendByte(addr);
	I2C_WaitAck();
	I2C_SendByte(dat);
	I2C_WaitAck();
	I2C_Stop();
}

uint8_t HMC5883L_Read(uint8_t addr)
{
	I2C_Start();
	I2C_SendByte(HMC5883L_ADDR);
	I2C_WaitAck();
	I2C_SendByte(addr);
	I2C_WaitAck();
	I2C_Stop();
	
	I2C_Start();
	I2C_SendByte(HMC5883L_ADDR | 0x01);
	I2C_WaitAck();
	uint8_t dat = I2C_ReceiveByte();
	I2C_NotSendAck();
	I2C_Stop();
	
	return dat;
}

void HMC5883L_Init()
{
	HMC5883L_Write(HMC5883L_CRA, 0xf8);  // 最大输出速率（75HZ）
	HMC5883L_Write(HMC5883L_CRB, 0x20);  // 默认增益
	HMC5883L_Write(HMC5883L_MR, 0x00);   // 连续测量
}

void HMC5883L_GetData()
{
	uint8_t data_h, data_l;
	data_h = HMC5883L_Read(HMC5883L_DOXMR);
	data_l = HMC5883L_Read(HMC5883L_DOXLR);
	hmc_x = data_l | (data_h << 8);

	data_h = HMC5883L_Read(HMC5883L_DOYMR);
	data_l = HMC5883L_Read(HMC5883L_DOYLR);
	hmc_y = data_l | (data_h << 8);

	data_h = HMC5883L_Read(HMC5883L_DOZMR);
	data_l = HMC5883L_Read(HMC5883L_DOZLR);
	hmc_z = data_l | (data_h << 8);
}




	


