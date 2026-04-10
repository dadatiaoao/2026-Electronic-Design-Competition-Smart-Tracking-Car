#ifndef _ml_adc_h
#define _ml_adc_h
#include "headfile.h"

typedef enum
{
	ADC_1,
	ADC_2,
}ADCx_enum;

typedef enum
{
	ADC_Channel_0,  //PA0
	ADC_Channel_1,  //PA1
	ADC_Channel_2,  //PA2
	ADC_Channel_3,  //PA3
	ADC_Channel_4,  //PA4
	ADC_Channel_5,  //PA5
	ADC_Channel_6,  //PA6
	ADC_Channel_7,  //PA7
	ADC_Channel_8,  //PB0
	ADC_Channel_9,  //PB1
	ADC_Channel_10, //PC0
	ADC_Channel_11, //PC1
	ADC_Channel_12, //PC2
	ADC_Channel_13, //PC3
	ADC_Channel_14, //PC4
	ADC_Channel_15, //PC5
}ADCINx_enum;

void adc_pin_init(ADCINx_enum adc_channel);
void adc_init(ADCx_enum adc,ADCINx_enum adc_channel);
uint16_t adc_get(ADCx_enum adc,ADCINx_enum adc_channel);


#endif
