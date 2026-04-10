#ifndef _hmc5883l_h
#define _hmc5883l_h
#include "ml_i2c.h"
#include "math.h"

#define HMC5883L_ADDR 0x3c
#define HMC5883L_CRA 0x00
#define HMC5883L_CRB 0x01
#define HMC5883L_MR 0x02
#define HMC5883L_DOXMR 0x03
#define HMC5883L_DOXLR 0x04
#define HMC5883L_DOZMR 0x05
#define HMC5883L_DOZLR 0x06
#define HMC5883L_DOYMR 0x07
#define HMC5883L_DOYLR 0x08
#define HMC5883L_SR 0x09
#define HMC5883L_IRA 0x0A
#define HMC5883L_IRB 0x0B
#define HMC5883L_IRC 0x0C

void HMC5883L_Write(uint8_t addr, uint8_t dat);
uint8_t HMC5883L_Read(uint8_t addr);
void HMC5883L_Init(void);
void HMC5883L_GetData(void);

extern int16_t hmc_x, hmc_y, hmc_z;
extern float yaw_hmc;

#endif

