#ifndef __OLED_H
#define __OLED_H
#include "headfile.h"

/* 
   软件模拟I2C协议
	 修改下面三个参数 
	 自定义OLED的SCL和SDA引脚
	 需将对应引脚配置成开漏输出	 
*/
//#define OLED_GPIO     GPIO_B
//#define OLED_SCL_Pin  Pin_13
//#define OLED_SDA_Pin  Pin_14


void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_SetCursor(uint8_t Y, uint8_t X);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t int_Length, uint8_t float_Length);

#endif
