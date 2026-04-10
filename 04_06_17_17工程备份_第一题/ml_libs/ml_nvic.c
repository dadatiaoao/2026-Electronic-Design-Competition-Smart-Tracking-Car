#include "headfile.h"

//-------------------------------------------------------
// 中断分组配置函数，用于设置 NVIC 中断分组，可设置抢占优先级和响应优先级
//-------------------------------------------------------

// 配置 NVIC 中断分组的函数
void NVIC_Group_Config(uint8_t group)
{
    uint32_t temp,temp1;
    // 计算要写入 AIRCR 寄存器 8 - 10 位的值，用于设置中断分组，通过取反 group 并与 0x07 按位与后左移 8 位得到
    temp1 = ((~group)&0x07)<<8; 
    // 获取 SCB->AIRCR 寄存器的原始值
    temp = SCB->AIRCR;          
    // 清除 SCB->AIRCR 寄存器中 8 - 10 位的值，其他位保持不变
    temp &= 0x0000F8FF;         
    // 将计算好的分组值写入到 temp 中对应的 8 - 10 位
    temp |= temp1;              
    // 写入钥匙值 0x05FA0000 到 temp 中，这是写入 SCB->AIRCR 寄存器的必要步骤
    temp |= 0x05FA0000;         
    // 将最终的 temp 值写入到 SCB->AIRCR 寄存器，完成中断分组配置
    SCB->AIRCR = temp;
}

// 初始化 NVIC 中断的函数
void NVIC_init(uint8_t PreemptionPriority,uint8_t nvic_channel)
{
    uint8_t temp;
    
    // 默认选择中断分组 4，即 4 位抢占优先级 + 0 位响应优先级
    NVIC_Group_Config(4);                                  
    // 保留 NVIC->IP 寄存器中对应中断通道的低 1 位，清除其他位，用于保留原始响应优先级相关位
    temp = NVIC->IP[nvic_channel]&0x01;                    
    // 将传入的抢占优先级左移 4 位后与保留的低 1 位按位或，然后写入到 NVIC->IP 寄存器对应中断通道，完成抢占优先级设置
    NVIC->IP[nvic_channel] = temp|(PreemptionPriority<<4); 
    // 使能对应中断通道，通过计算 nvic_channel 对应的 ISER 寄存器索引和位偏移来设置相应的使能位
    NVIC->ISER[nvic_channel/32] = 1<<nvic_channel%32;      
}