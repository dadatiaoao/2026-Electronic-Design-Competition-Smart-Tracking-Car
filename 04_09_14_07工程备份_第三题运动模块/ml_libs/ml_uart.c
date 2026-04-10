#include "headfile.h"

// 定义APB1总线时钟频率为36MHz（USART2/3使用）
#define PCLK1    36000000  
// 定义APB2总线时钟频率为72MHz（USART1使用）
#define PCLK2    72000000  

// USART寄存器基地址数组，索引对应UART_1/UART_2/UART_3枚举值
USART_TypeDef *uart_index[3] = { USART1, USART2, USART3 };

//-----------------------------------------------------------------------------
// 启用printf重定向至USART1（需要初始化UART1）
#if 1
#pragma import(__use_no_semihosting)  // 禁用半主机模式依赖
// 简化版文件结构体定义（仅支持基本输出）
struct __FILE { 
    int handle;  // 文件句柄（此处无需详细实现）
}; 
FILE __stdout;   // 标准输出文件指针

// 系统退出函数空实现（避免半主机依赖）
void _sys_exit(int x) { 
    (void)x;  // 消除未使用参数警告
} 

// 重定向fputc至USART1，使printf输出到串口
int fputc(int ch, FILE *f) {
    while((USART1->SR & 0x40) == 0);  // 等待发送缓冲区空（TXE标志）
    USART1->DR = (uint8_t)ch;         // 写入数据寄存器
    return ch;
}
#endif 
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// 函数名：uart_pin_init
// 功能：初始化指定UART的TX/RX引脚
// 参数：
//   uartn：UART编号（UART_1/UART_2/UART_3）
// 说明：
//   - TX引脚配置为复用推挽输出（AF_PP）
//   - RX引脚配置为浮空输入（ID）
//-----------------------------------------------------------------------------
void uart_pin_init(UARTn_enum uartn) {
    switch(uartn) {
        case UART_1:  // USART1: PA9(TX), PA10(RX)
            gpio_init(GPIO_A, Pin_9, AF_PP);
            gpio_init(GPIO_A, Pin_10, ID);
            break;
        case UART_2:  // USART2: PA2(TX), PA3(RX)
            gpio_init(GPIO_A, Pin_2, AF_PP);
            gpio_init(GPIO_A, Pin_3, ID);
            break;
        case UART_3:  // USART3: PB10(TX), PB11(RX)
            gpio_init(GPIO_B, Pin_10, AF_PP);
            gpio_init(GPIO_B, Pin_11, ID);
            break;
    }
}

//-----------------------------------------------------------------------------
// 函数名：uart_baud_config
// 功能：配置UART波特率
// 参数：
//   uartn：UART编号
//   baud：目标波特率（如9600, 115200）
// 说明：
//   - USART1使用APB2时钟（72MHz），其他使用APB1（36MHz）
//   - 计算公式：BRR = (PCLK / (16 * baud)) 的整数和小数部分组合
//-----------------------------------------------------------------------------
void uart_baud_config(UARTn_enum uartn, int baud) {
    uint32_t mantissa;    // BRR整数部分
    float fraction;       // BRR小数部分（需转换为4位二进制）
    int clock_freq;       // 当前UART时钟频率

    // 选择时钟源
    clock_freq = (uartn == UART_1) ? PCLK2 : PCLK1;

    // 计算波特率分频值
    mantissa = clock_freq / (16 * baud);               // 整数部分
    fraction = (float)clock_freq / (16.0 * baud) - mantissa;  // 小数部分
    fraction = (fraction * 16);                        // 转换为4位值（0-15）

    // 组合并写入波特率寄存器（BRR）
    uart_index[uartn]->BRR = (mantissa << 4) | (uint32_t)fraction;
}

//-----------------------------------------------------------------------------
// 函数名：uart_init
// 功能：完整初始化UART外设
// 参数：
//   uartn：UART编号
//   baud：波特率
//   priority：中断优先级（0-15，数值越小优先级越高）
// 说明：
//   - 启用对应APB总线时钟
//   - 配置GPIO引脚
//   - 设置波特率
//   - 使能UART、发送器、接收器及接收中断
//   - 配置NVIC中断优先级
//-----------------------------------------------------------------------------
void uart_init(UARTn_enum uartn, int baud, uint8_t priority) {
    // 启用总线时钟
    if(uartn == UART_1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // APB2时钟使能
    } else {
        // USART2/3位于APB1，位偏移计算（USART2_EN=17, USART3_EN=18）
        RCC->APB1ENR |= (1 << (uartn + 16));   
    }

    uart_pin_init(uartn);      // 初始化GPIO
    uart_baud_config(uartn, baud); // 配置波特率

    // 使能UART功能（UE=1），启用发送(TE)和接收(RE)，接收中断(RIE)
    uart_index[uartn]->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    // 配置NVIC中断（优先级和中断通道）
    // USART1全局中断号37，USART2=38，USART3=39（需确认具体MCU）
    NVIC_init(priority, uartn + 37); 
}

//-----------------------------------------------------------------------------
// 函数名：uart_sendbyte
// 功能：通过UART发送单个字节（阻塞式）
// 参数：
//   uartn：UART编号
//   Byte：待发送数据字节
//-----------------------------------------------------------------------------
void uart_sendbyte(UARTn_enum uartn, uint8_t Byte) {
    // 等待发送缓冲区空（TXE标志置位）
    while((uart_index[uartn]->SR & USART_SR_TXE) == 0);
    uart_index[uartn]->DR = Byte;  // 写入数据启动发送
}

//-----------------------------------------------------------------------------
// 函数名：uart_sendstr
// 功能：发送字符串（直至空终止符）
// 参数：
//   uartn：UART编号
//   str：待发送C字符串指针
//-----------------------------------------------------------------------------
void uart_sendstr(UARTn_enum uartn, char* str) {
    while(*str != '\0') {
        uart_sendbyte(uartn, (uint8_t)*str++);
    }
}

//-----------------------------------------------------------------------------
// 函数名：uart_getbyte
// 功能：从UART接收一个字节（需在中断服务例程中调用）
// 参数：
//   uartn：UART编号
// 返回值：接收到的数据字节
// 说明：
//   - 通常在RXNE（接收中断）触发后读取数据
//-----------------------------------------------------------------------------
uint8_t uart_getbyte(UARTn_enum uartn) {
    return (uint8_t)(uart_index[uartn]->DR);  // 读取数据寄存器
}