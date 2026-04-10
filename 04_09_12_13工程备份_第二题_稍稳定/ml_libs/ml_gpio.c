#include "headfile.h"

// 定义一个数组，用于存储 GPIOA、GPIOB 和 GPIOC 的指针
GPIO_TypeDef *gpio_index[3] = { GPIOA , GPIOB , GPIOC };

//-------------------------------------------------------------------------------------------------------------------
// @brief		GPIO 初始化
// @param	  GPIOn	  选择的 GPIO 端口
// @param	  Pinx    选择的引脚
// @param	  mode    选择的工作模式
// @return		void  
// Sample usage:		gpio_init(GPIO_A,Pin_0,OUT_PP);
//-------------------------------------------------------------------------------------------------------------------
void gpio_init(GPIOn_enum GPIOn,Pinx_enum Pinx,GPIO_MODE_enum mode)
{
    uint32_t temp;
    
    // 使能相应 GPIO 端口的时钟，4 左移 GPIOn 位
    RCC->APB2ENR |= 4<<GPIOn;         
    
    // 如果引脚编号小于 8
    if(Pinx<8)
    {
        // 清除 CRL 寄存器中对应引脚的 4 位配置
        gpio_index[GPIOn]->CRL &= ~((0x0000000F)<<(Pinx*4));     
        // 如果是推挽输出、复用推挽输出或开漏输出模式
        if(mode == OUT_PP || mode == AF_PP || mode == OUT_OD)
            // 设置 CRL 寄存器中对应引脚的配置为推挽输出（默认输出高电平）
            gpio_index[GPIOn]->CRL |= ((0x00000003)|(1<<mode))<<(Pinx*4);        
        // 如果是上拉输入或下拉输入模式
        else if(mode == IU || mode == ID)
        {
            // 设置 CRL 寄存器中对应引脚为浮空/上拉-下拉输入模式
            gpio_index[GPIOn]->CRL |= 0x00000008<<(Pinx*4);        
            // 如果是上拉输入模式
            if(mode == IU)
            {
                // 清除 ODR 寄存器中对应引脚的位
                temp = gpio_index[GPIOn]->ODR&(~(1<<Pinx));          
                // 设置 ODR 寄存器中对应引脚的位为 1
                gpio_index[GPIOn]->ODR = temp|(1<<Pinx);             
            }
            // 如果是下拉输入模式
            if(mode == ID)
                // 清除 ODR 寄存器中对应引脚的位
                gpio_index[GPIOn]->ODR &= ~(1<<Pinx);                
            }
        // 其他模式
        else
            // 设置 CRL 寄存器中对应引脚的配置
            gpio_index[GPIOn]->CRL |= (0x00000004<<mode)<<(Pinx*4);								
    }
    // 如果引脚编号大于等于 8
    else
    {
        // 清除 CRH 寄存器中对应引脚的 4 位配置
        gpio_index[GPIOn]->CRH &= ~((0x0000000F)<<((Pinx-8)*4));     
        // 如果是推挽输出、复用推挽输出或开漏输出模式
        if(mode == OUT_PP || mode == AF_PP || mode == OUT_OD)
            // 设置 CRH 寄存器中对应引脚的配置为推挽输出（默认输出高电平）
            gpio_index[GPIOn]->CRH |= ((0x00000003)|(1<<mode))<<((Pinx-8)*4);        
        // 如果是上拉输入或下拉输入模式
        else if(mode == IU || mode == ID)
        {
            // 设置 CRH 寄存器中对应引脚为浮空/上拉-下拉输入模式
            gpio_index[GPIOn]->CRH |= 0x00000008<<((Pinx-8)*4);        
            // 如果是上拉输入模式
            if(mode == IU)
            {
                // 清除 ODR 寄存器中对应引脚的位
                temp = GPIOA->ODR&(~(1<<Pinx));          
                // 设置 ODR 寄存器中对应引脚的位为 1
                gpio_index[GPIOn]->ODR = temp|(1<<Pinx);             
            }
            // 如果是下拉输入模式
            if(mode == ID)
                // 清除 ODR 寄存器中对应引脚的位
                gpio_index[GPIOn]->ODR &= ~(1<<Pinx);                
            }
        // 其他模式
        else
            // 设置 CRH 寄存器中对应引脚的配置
            gpio_index[GPIOn]->CRH |= (0x00000004<<mode)<<((Pinx-8)*4);	
    }
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		设置 GPIO 引脚电平
// @param	  GPIOn	  选择的 GPIO 端口
// @param	  Pinx    选择的引脚
// @param	  mode    置 1 为高电平 置 0 为低电平
// @return		void  
// Sample usage:			gpio_set(GPIO_A,Pin_0,1);
//-------------------------------------------------------------------------------------------------------------------
void gpio_set(GPIOn_enum GPIOn,Pinx_enum Pinx,uint8_t mode)
{
    uint32_t temp;
    
    // 如果 mode 为 1
    if(mode)
    {
        // 清除 ODR 寄存器中对应引脚的位
        temp = gpio_index[GPIOn]->ODR&(~(1<<Pinx));   
        // 设置 ODR 寄存器中对应引脚的位为 1
        gpio_index[GPIOn]->ODR = temp|(1<<Pinx);      
    }
    // 如果 mode 为 0
    else
        // 清除 ODR 寄存器中对应引脚的位
        gpio_index[GPIOn]->ODR &= ~(1<<Pinx);         
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		获取 GPIO 引脚电平
// @param	  GPIOn	  选择的 GPIO 端口
// @param	  Pinx    选择的引脚
// @return		uint8_t  
// Sample usage:			gpio_get(GPIO_A,Pin_4)
//-------------------------------------------------------------------------------------------------------------------
uint8_t gpio_get(GPIOn_enum GPIOn,Pinx_enum Pinx)
{
    uint32_t temp;
    
    // 读取 IDR 寄存器中对应引脚的位
    temp = gpio_index[GPIOn]->IDR&(1<<Pinx);   
    // 如果 temp 不为 0，说明选择的引脚为 1，即高电平
    if(temp)                          
        return 1;
    // 如果 temp 为 0，说明选择的引脚为 0，即低电平
    else                              
        return 0;
}