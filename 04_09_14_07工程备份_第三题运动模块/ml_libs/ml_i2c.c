#include "ml_i2c.h"

// 定义宏，用于设置SDA引脚的输出电平
#define SDA_Output(x)  gpio_set(I2C_GPIO, I2C_SDA_GPIO_Pin, x)
// 定义宏，用于设置SCL引脚的输出电平
#define SCL_Output(x)  gpio_set(I2C_GPIO, I2C_SCL_GPIO_Pin, x)
// 定义宏，用于读取SDA引脚的输入电平
#define SDA_Input() gpio_get(I2C_GPIO, I2C_SDA_GPIO_Pin)

// 初始化I2C总线
void I2C_Init()
{
    // 初始化SCL引脚为开漏输出模式
    gpio_init(I2C_GPIO, I2C_SCL_GPIO_Pin, OUT_OD);
    // 初始化SDA引脚为开漏输出模式
    gpio_init(I2C_GPIO, I2C_SDA_GPIO_Pin, OUT_OD);

    // 将SDA引脚置高
    SDA_Output(1);
    // 将SCL引脚置高
    SCL_Output(1);
}

// 产生I2C起始信号
void I2C_Start()
{
    // 先将SDA和SCL引脚都置高
    SDA_Output(1);
    SCL_Output(1);
    // 在SCL为高电平时，将SDA拉低，产生起始信号
    SDA_Output(0);
    // 拉低SCL，准备后续的数据传输
    SCL_Output(0);
}

// 产生I2C停止信号
void I2C_Stop()
{
    // 先将SDA拉低
    SDA_Output(0);
    // 将SCL置高
    SCL_Output(1);
    // 在SCL为高电平时，将SDA拉高，产生停止信号
    SDA_Output(1);
}

// 发送一个字节的数据
void I2C_SendByte(uint8_t byte)
{
    for(int i = 0; i < 8; i++)
    {
        // 判断当前位是0还是1
        if(byte & (0x80>>i))
            // 如果是1，将SDA置高
            SDA_Output(1);
        else
            // 如果是0，将SDA置低
            SDA_Output(0);
        // 拉高SCL，发送当前位的数据
        SCL_Output(1);
        // 拉低SCL，准备发送下一位数据
        SCL_Output(0);
    }
}

// 接收一个字节的数据
uint8_t I2C_ReceiveByte()
{
    uint8_t byte = 0;
    // 将SDA引脚设置为输入模式（拉高）
    SDA_Output(1);
    for(int i = 0; i < 8; i++)
    {
        // 拉高SCL，准备读取数据
        SCL_Output(1);
        // 读取SDA引脚的电平
        if(SDA_Input())
            // 如果为高电平，将对应位设置为1
            byte |= (0x80>>i);
        // 拉低SCL，准备读取下一位数据
        SCL_Output(0);
    }

    return byte;
}

// 发送应答信号
void I2C_SendAck()
{
    // 将SDA拉低，表示应答
    SDA_Output(0);
    // 拉高SCL，发送应答信号
    SCL_Output(1);
    // 拉低SCL，结束应答信号的发送
    SCL_Output(0);
}

// 发送非应答信号
void I2C_NotSendAck()
{
    // 将SDA拉高，表示非应答
    SDA_Output(1);
    // 拉高SCL，发送非应答信号
    SCL_Output(1);
    // 拉低SCL，结束非应答信号的发送
    SCL_Output(0);
}

// 等待从设备的应答信号
uint8_t I2C_WaitAck()
{
    uint8_t byte = 0;
    // 将SDA引脚设置为输入模式（拉高）
    SDA_Output(1);
    // 拉高SCL，准备读取应答信号
    SCL_Output(1);
    // 读取SDA引脚的电平，获取应答信号
    byte = SDA_Input();
    // 拉低SCL，结束应答信号的读取
    SCL_Output(0);

    return byte;
}