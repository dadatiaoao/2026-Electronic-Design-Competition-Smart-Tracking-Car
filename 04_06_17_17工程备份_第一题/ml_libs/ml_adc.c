#include "headfile.h"

// 定义一个指向ADC_TypeDef类型的指针数组，包含ADC1和ADC2
ADC_TypeDef *adc_index[3] = {ADC1, ADC2};

// 函数功能：ADC引脚初始化
// 参数：adc_channel - 选择的ADC通道
// 返回值：无
// 示例用法：adc_pin_init(adc_channel);  // 初始化指定的ADC通道引脚
void adc_pin_init(ADCINx_enum adc_channel)
{
    switch (adc_channel)
    {
        case ADC_Channel_0: { gpio_init(GPIO_A, Pin_0, AIN); break; }
        case ADC_Channel_1: { gpio_init(GPIO_A, Pin_1, AIN); break; }
        case ADC_Channel_2: { gpio_init(GPIO_A, Pin_2, AIN); break; }
        case ADC_Channel_3: { gpio_init(GPIO_A, Pin_3, AIN); break; }
        case ADC_Channel_4: { gpio_init(GPIO_A, Pin_4, AIN); break; }
        case ADC_Channel_5: { gpio_init(GPIO_A, Pin_5, AIN); break; }
        case ADC_Channel_6: { gpio_init(GPIO_A, Pin_6, AIN); break; }
        case ADC_Channel_7: { gpio_init(GPIO_A, Pin_7, AIN); break; }
        case ADC_Channel_8: { gpio_init(GPIO_B, Pin_0, AIN); break; }
        case ADC_Channel_9: { gpio_init(GPIO_B, Pin_1, AIN); break; }
        case ADC_Channel_10: { gpio_init(GPIO_C, Pin_0, AIN); break; }
        case ADC_Channel_11: { gpio_init(GPIO_C, Pin_1, AIN); break; }
        case ADC_Channel_12: { gpio_init(GPIO_C, Pin_2, AIN); break; }
        case ADC_Channel_13: { gpio_init(GPIO_C, Pin_3, AIN); break; }
        case ADC_Channel_14: { gpio_init(GPIO_C, Pin_4, AIN); break; }
        case ADC_Channel_15: { gpio_init(GPIO_C, Pin_5, AIN); break; }
    }
}

// 函数功能：ADC初始化
// 参数：adc - 选择的ADC
//       adc_channel - 选择的ADC通道
// 返回值：无
// 示例用法：adc_init(ADC_1, ADC_Channel_1);
void adc_init(ADCx_enum adc, ADCINx_enum adc_channel)
{
    // 初始化ADC引脚
    adc_pin_init(adc_channel);

    // 使能相应ADC的时钟
    RCC->APB2ENR |= 1 << (9 + adc); 
    // 复位ADC时钟
    RCC->APB2RSTR |= 1 << (9 + adc); 
    // 释放复位
    RCC->APB2RSTR &= ~(1 << (9 + adc)); 

    // 清除ADC分频位
    RCC->CFGR &= ~(3 << 14); 
    // 设置ADC分频为6，ADC频率为72/6 = 12MHz
    RCC->CFGR |= 1 << 15;  

    // 清除ADC扫描模式
    adc_index[adc]->CR1 &= ~(0x0f << 16); 
    // 禁止扫描模式
    adc_index[adc]->CR1 &= ~(1 << 8); 
    // 禁止连续转换模式
    adc_index[adc]->CR2 &= ~(1 << 1); 

    // 清除规则通道转换触发位
    adc_index[adc]->CR2 &= ~(7 << 17); 
    // 设置为软件触发
    adc_index[adc]->CR2 |= 7 << 17; 
    // 使能软件触发
    adc_index[adc]->CR2 |= 1 << 20; 

    // 禁止右对齐
    adc_index[adc]->CR2 &= ~(1 << 11); 

    // 只设置一个转换通道
    adc_index[adc]->SQR1 &= 0xff0fffff; 

    if (adc_channel >= 10)
    {
        // 清除通道采样时间位
        adc_index[adc]->SMPR1 &= ~(7 << (3 * (adc_channel - 10))); 
        // 设置通道采样时间
        adc_index[adc]->SMPR1 |= 7 << (3 * (adc_channel - 10)); 
    }
    else
    {
        // 清除通道采样时间位
        adc_index[adc]->SMPR2 &= ~(7 << (3 * adc_channel)); 
        // 设置通道采样时间
        adc_index[adc]->SMPR2 |= 7 << (3 * adc_channel); 
    }

    // 使能ADC转换
    adc_index[adc]->CR2 |= 1 << 0; 

    // 使能复位校准
    adc_index[adc]->CR2 |= 1 << 3; 
    // 等待复位校准完成
    while (adc_index[adc]->CR2 & (1 << 3)); 
    // 使能AD校准
    adc_index[adc]->CR2 |= 1 << 2; 
    // 等待AD校准完成
    while (adc_index[adc]->CR2 & (1 << 2)); 
}

// 函数功能：获取ADC转换值
// 参数：adc - 选择的ADC
//       adc_channel - 选择的ADC通道
// 返回值：uint16_t类型的ADC转换结果
// 示例用法：uint16_t value = adc_get(ADC_1, ADC_Channel_1);
uint16_t adc_get(ADCx_enum adc, ADCINx_enum adc_channel)
{
    // 选择转换通道
    adc_index[adc]->SQR3 = adc_channel; 
    // 启动规则通道转换
    adc_index[adc]->CR2 |= 1 << 22; 
    // 等待转换完成
    while (!(adc_index[adc]->SR & (1 << 1))); 

    // 返回ADC转换结果
    return adc_index[adc]->DR; 
}