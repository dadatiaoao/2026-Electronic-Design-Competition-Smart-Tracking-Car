# 电赛2026 - 黑线循迹小车项目

## 项目概述

本项目是一个基于STM32F103微控制器的黑线循迹智能小车系统。小车采用8路灰度传感器检测地面黑线，通过PID控制算法实现精准循迹。系统集成了MPU6050姿态传感器、编码器测速、OLED显示、蜂鸣器提示、按键控制等功能，具备路程累计、路程点触发、自动路线选择等高级特性。

## 硬件架构

### 主控芯片
- STM32F103C8T6 (ARM Cortex-M3内核)

### 传感器模块
1. **8路灰度传感器** - 地面黑线检测
   - 传感器布局：D1~D8从左到右排列
   - 检测逻辑：1=检测到黑线，0=未检测到黑线
2. **MPU6050六轴传感器** - 姿态测量(加速度计+陀螺仪)
   - 用于角度环PID控制
   - 提供航向角(yaw)反馈
3. **增量式编码器** - 电机转速测量
   - 左右电机各一个编码器
   - 提供速度反馈用于速度环PID

### 执行机构
1. **直流电机** ×2
   - 左右轮独立驱动
   - PWM调速控制
2. **电机驱动电路** - 双H桥驱动

### 人机交互
1. **OLED显示屏** - 状态信息显示
2. **蜂鸣器** - 系统提示音
3. **LED指示灯** - 系统状态指示
4. **按键** ×3 - 模式选择和控制

### 通信接口
1. **UART串口** ×3
   - UART1: 调试输出(115200bps)
   - UART2: 视觉识别模块通信
   - UART3: 语音模块通信
2. **I2C总线** - MPU6050传感器通信

## 软件架构

### 目录结构
```
daima/
├── code/           # 核心算法模块
│   ├── filter.c/.h     # 滤波算法(卡尔曼滤波、Mahony滤波)
│   ├── gray_track.c/.h # 灰度传感器读取与循迹算法
│   ├── pid.c/.h        # PID控制器实现
│   ├── motor.c/.h      # 电机驱动与编码器
│   ├── Key.c/.h        # 按键扫描
│   ├── LB.c/.h         # 蜂鸣器控制
│   └── readme.txt      # 模块说明
├── ml_libs/        # 底层硬件驱动库
│   ├── headfile.h      # 全局头文件包含
│   ├── ml_adc.c/.h     # ADC驱动
│   ├── ml_delay.c/.h   # 延时函数
│   ├── ml_exti.c/.h    # 外部中断
│   ├── ml_gpio.c/.h    # GPIO操作
│   ├── ml_i2c.c/.h     # I2C总线驱动
│   ├── ml_mpu6050.c/.h # MPU6050传感器驱动
│   ├── ml_nvic.c/.h    # 中断控制器
│   ├── ml_oled.c/.h    # OLED显示屏驱动
│   ├── ml_pwm.c/.h     # PWM输出
│   ├── ml_tim.c/.h     # 定时器配置
│   └── ml_uart.c/.h    # 串口通信
├── user/           # 用户应用程序
│   ├── main_循迹.c     # 主循迹程序(带路程累计)
│   ├── main_忽略.c     # 主程序(忽略路程点)
│   ├── isr.c           # 中断服务程序
│   ├── LED.c/.h        # LED指示灯控制
│   └── .vscode/        # VSCode配置
├── sys/            # 系统文件
│   ├── core_cm3.h      # Cortex-M3内核头文件
│   ├── stm32f10x.h     # STM32F10x系列寄存器定义
│   ├── system_stm32f10x.c/.h # 系统时钟配置
│   └── startup_stm32f10x_*.s # 启动文件
└── keilkilll.bat   # 清理中间文件脚本
```

### 核心模块详解

#### 1. PID控制器模块 (pid.c/.h)
**功能**: 实现增量式和位置式PID控制算法，用于电机速度环和车身角度环控制。

**关键数据结构**:
```c
typedef struct {
    float target;     // 目标值
    float now;        // 当前值
    float error[3];   // 误差队列(当前、上次、上上次)
    float p, i, d;    // PID参数
    float pout, iout, dout; // 各分量输出
    float out;        // 总输出
    uint32_t pid_mode; // PID模式(POSITION_PID/DELTA_PID)
} pid_t;
```

**关键函数**:
- `void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)` - PID初始化
- `void pid_cal(pid_t *pid)` - PID计算(位置式/增量式)
- `void pid_control(int basicspeed)` - PID控制主函数(在TIM3中断中调用)
- `void motor_target_set(int spe1, int spe2)` - 设置电机目标速度
- `uint8_t rotate_to_angle(...)` - 旋转到指定角度(用于定点转向)

**控制流程**:
1. TIM3定时器中断每10ms触发一次
2. 调用`pid_control(basicspeed)`函数
3. 角度环PID计算：目标角度`jiaodu` vs 当前角度`yaw_gyro`
4. 如果循迹模式使能(`xunji=1`)，调用`track()`函数调整左右速度差
5. 速度环PID计算：编码器反馈 vs 目标速度
6. 输出PWM控制电机

#### 2. 灰度循迹模块 (gray_track.c/.h)
**功能**: 8路灰度传感器数据采集与黑线循迹算法。

**传感器布局权重**:
```
传感器: D1  D2  D3  D4  D5  D6  D7  D8
权重:   -7  -5  -3  -1   1   3   5   7
位置:   左 ←                    → 右
```

**关键函数**:
- `void gray_init(void)` - 传感器初始化
- `void gray_read(void)` - 读取8路传感器状态(存入D1~D8全局变量)
- `void track(void)` - 循迹控制算法核心

**循迹算法原理**:
1. 计算加权误差: `error = Σ(weight[i] * sensor[i]) / hit_count`
2. 一阶低通滤波: `filt_error = 0.7*filt_error + 0.3*error`
3. 计算转向量: `turn = 16*filt_error + 8*(filt_error - last_error)`
4. 动态调整基础速度: `dynamic_base = basicspeed - 4*|filt_error|`
5. 计算左右轮速度: `left = dynamic_base + turn`, `right = dynamic_base - turn`
6. 限幅处理: 速度限制在40~255之间

#### 3. 电机控制模块 (motor.c/.h)
**功能**: 电机PWM驱动和编码器脉冲计数。

**关键函数**:
- `void motor_init(void)` - 电机PWM和方向控制初始化
- `void motorA_duty(int duty)`, `void motorB_duty(int duty)` - 设置电机占空比
- `void encoder_init(void)` - 编码器外部中断初始化

**编码器计数**:
- `Encoder_count1`, `Encoder_count2` - 左右编码器脉冲计数
- 在EXTI2和EXTI4中断中根据方向信号增减计数值
- 每10ms在`pid_control()`中读取并清零，用于速度计算

#### 4. 滤波算法模块 (filter.c/.h)
**功能**: 传感器数据滤波处理，提高姿态测量精度。

**算法类型**:
1. **卡尔曼滤波(Kalman Filter)**
   - 用于MPU6050数据融合
   - 结构体`KF_t`包含状态估计参数
   - 函数`float Kalman_Filter(KF_t *kf, float obsValue, float ut)`
2. **Mahony互补滤波**
   - 简单高效的姿态融合算法
   - 函数`float Mahony_Filter(float gyro, float acc)`

#### 5. 姿态传感器模块 (ml_mpu6050.c/.h)
**功能**: MPU6050六轴传感器驱动，提供姿态角数据。

**关键函数**:
- `void MPU6050_Init(void)` - 传感器初始化
- `void MPU6050_GetData(void)` - 读取原始数据(加速度、角速度)
- 在EXTI7中断中自动调用数据读取和姿态解算

**输出变量**:
- `int16_t ax, ay, az, gx, gy, gz` - 原始数据
- `float yaw_gyro, yaw_acc, yaw_Kalman` - 三种航向角估计值

#### 6. 系统主程序 (main_循迹.c / main_忽略.c)
**两种主程序变体**:
1. `main_循迹.c` - 完整功能版本，包含路程累计和路程点触发
2. `main_忽略.c` - 简化版本，忽略路程点功能

**系统初始化流程**:
1. 外设初始化: OLED、电机、编码器、按键、LED、串口、灰度传感器、I2C、MPU6050
2. PID参数初始化: 电机A/B速度环(增量式PID)，角度环(位置式PID)
3. 中断配置: EXTI_PB7(外部中断)，TIM3(10ms定时中断)
4. 系统状态设置: 使能循迹模式，设置基础速度，配置路程阈值
5. 启动提示: 蜂鸣器和LED闪烁
6. 进入主循环

**主循环功能**:
- `main_循迹.c`: 检测路程点触发，检查最终停止阈值
- `main_忽略.c`: 简单延时，防止CPU占用过高

#### 7. main.c程序流程详解
**文件位置**: `user/main.c`

**程序总体流程**:
1. **系统外设初始化** (第39-49行)
   - 蜂鸣器、OLED、电机、编码器、按键、LED、串口、灰度传感器、I2C、MPU6050初始化
2. **PID控制器参数初始化** (第51-57行)
   - 电机A/B速度环PID (增量式PID): KP=100.0, KI=0.0, KD=5.0
   - 角度环PID (位置式PID): KP=2.0, KI=0.0, KD=0.1
3. **中断配置** (第59-61行)
   - EXTI_PB7中断 (MPU6050数据就绪)，上升沿触发
   - TIM3定时器中断，10ms周期，优先级0 (最高)
4. **系统初始状态设置** (第63-79行)
   - `xunji = 1`: 使能循迹模式
   - `jiaodu = 0`: 初始目标角度0度
   - `basicspeed = 130`: 基础前进速度
   - `distance_threshold_cm = 457.2f`: 停止阈值
   - `tracking_distance_cm = 0.0f`: 清零累计路程
   - `pulse_to_cm_factor = 0.0776f`: 脉冲到厘米转换因子
   - 重置路程点状态
5. **系统启动提示** (第81-84行)
   - 蜂鸣器响一声，LED闪烁一次，500ms延时
6. **主循环** (第91-138行)
   - **角度数据输出**: 每500ms通过串口输出角速度和卡尔曼滤波角度
   - **路程点检测**: 检查`distance_points`数组中每个路程点的触发状态，触发后蜂鸣器提示
   - **最终停止检查**: 当`distance_threshold_reached`置位时，停止循迹模式，电机速度归零
   - **延时控制**: `delay_ms(10)`防止CPU占用率过高

**关键全局变量**:
- `xunji`: 循迹模式使能标志 (1=开启，0=关闭)
- `basicspeed`: 基础前进速度 (0-255)
- `jiaodu`: 目标角度 (度)
- `tracking_distance_cm`: 累计循迹路程 (厘米)
- `distance_threshold_reached`: 阈值到达标志

**中断驱动控制**:
- **TIM3中断 (10ms周期)**: 调用`pid_control(basicspeed)`执行核心控制算法
  - 读取灰度传感器状态 (如果`xunji=1`)
  - 调用`track()`函数计算转向调整
  - 计算角度环PID (目标角度 vs 当前角度)
  - 计算速度环PID (编码器反馈 vs 目标速度)
  - 输出PWM控制电机
- **EXTI7中断 (MPU6050数据就绪)**: 读取传感器数据并更新姿态角

**注意事项**:
- 主循环主要负责状态监控和路程点处理，核心控制逻辑在TIM3中断中执行
- 路程累计在`pid_control()`函数中根据编码器脉冲数更新
- 角度环PID可根据赛道需求选择使用或关闭

#### 8. 中断服务程序 (isr.c)
**关键中断**:
1. **TIM3中断** - 系统控制周期(10ms)
   - 调用`pid_control(basicspeed)`执行核心控制算法
2. **EXTI2中断** - 编码器A脉冲计数
   - 根据PA3方向信号增减`Encoder_count1`
3. **EXTI4中断** - 编码器B脉冲计数
   - 根据PA5方向信号增减`Encoder_count2`
4. **EXTI7中断** - MPU6050数据就绪中断
   - 调用`MPU6050_GetData()`读取传感器数据
   - 计算三种姿态角: 陀螺仪积分、加速度计解算、卡尔曼滤波
5. **USART1/2/3中断** - 串口接收中断
   - USART2用于视觉识别模块通信，调用`auto_vision_rx_byte()`

### 控制参数配置

#### PID参数 (在main函数中设置)
```c
// 电机A速度环PID(增量式)
pid_init(&motorA, DELTA_PID, 100.0f, 0.0f, 5.0f);
// 电机B速度环PID(增量式)
pid_init(&motorB, DELTA_PID, 100.0f, 0.0f, 5.0f);
// 角度环PID(位置式)
pid_init(&angle, POSITION_PID, 2.0f, 0.0f, 0.1f);
```

#### 循迹参数 (在gray_track.c中)
```c
const int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7}; // 传感器权重
float filt_error = 0.70f * filt_error + 0.30f * error; // 低通滤波系数
float turn = 16.0f * filt_error + 8.0f * (filt_error - last_error); // 转向增益
```

#### 路程累计参数 (在pid.c中)
```c
volatile float distance_threshold_cm = 200.0f;     // 停止阈值: 200cm
float pulse_to_cm_factor = 0.0567f;                // 脉冲到厘米转换因子
distance_point_t distance_points[] = {             // 路程点配置
    {50.0f, 0},   // 50cm触发点
    {100.0f, 0},  // 100cm触发点
    {150.0f, 0},  // 150cm触发点
    {180.0f, 0},  // 180cm触发点
};
```

### 全局变量说明

#### 控制状态变量
- `volatile int xunji` - 循迹模式使能(1=开启，0=关闭)
- `volatile float jiaodu` - 目标角度(度)
- `volatile int basicspeed` - 基础前进速度(0-255)
- `volatile uint8_t auto_run_enable` - 自动运行使能

#### 传感器数据变量
- `unsigned char D1~D8` - 8路灰度传感器状态
- `int16_t ax, ay, az, gx, gy, gz` - MPU6050原始数据
- `float yaw_gyro, yaw_acc, yaw_Kalman` - 航向角估计值
- `int Encoder_count1, Encoder_count2` - 编码器脉冲计数

#### 路程累计变量
- `volatile float tracking_distance_cm` - 累计循迹路程
- `volatile uint8_t distance_threshold_reached` - 阈值到达标志
- `volatile uint8_t tracking_distance_enabled` - 路程累计使能

### 函数调用关系图

```
主函数(main)
├── 系统初始化
│   ├── OLED_Init()
│   ├── motor_init()
│   ├── encoder_init()
│   ├── Key_Init()
│   ├── LED_Init()
│   ├── uart_init()
│   ├── gray_init()
│   ├── Buzzer_Init()
│   ├── I2C_Init()
│   └── MPU6050_Init()
├── PID初始化
│   ├── pid_init(&motorA, ...)
│   ├── pid_init(&motorB, ...)
│   └── pid_init(&angle, ...)
├── 中断配置
│   ├── exti_init(EXTI_PB7, ...)
│   └── tim_interrupt_ms_init(TIM_3, 10, 0)
└── 主循环
    ├── 路程点检测(check_distance_points)
    └── 最终停止检查

TIM3中断(10ms周期)
└── pid_control(basicspeed)
    ├── gray_read() [如果xunji=1]
    ├── track()     [如果xunji=1]
    ├── pid_cal(&angle)        # 角度环
    ├── pid_cal(&motorA)       # 电机A速度环
    ├── pid_cal(&motorB)       # 电机B速度环
    ├── pidout_limit(&motorA)  # 输出限幅
    ├── pidout_limit(&motorB)  # 输出限幅
    ├── motorA_duty(motorA.out) # PWM输出
    └── motorB_duty(motorB.out) # PWM输出

EXTI7中断(MPU6050数据就绪)
└── MPU6050_GetData()
    ├── I2C读取原始数据
    └── 姿态解算
        ├── 陀螺仪积分: yaw_gyro += gz/16.4*0.005
        ├── 加速度计解算: yaw_acc = atan2(ay,ax)*57.296
        └── 卡尔曼滤波: yaw_Kalman = Kalman_Filter(...)
```

### 编译与调试

#### 开发环境
- IDE: Keil uVision / VSCode with Keil Assistant
- 编译器: ARMCC / GCC ARM Embedded
- 调试器: ST-Link / J-Link

#### 编译步骤
1. 打开项目文件: `user/Project.uvprojx`
2. 配置目标设备: STM32F103C8
3. 设置编译选项
4. 编译生成HEX文件

#### 调试接口
- **串口1(115200bps)**: 调试信息输出，PID数据可视化
- **OLED显示屏**: 实时状态显示
- **蜂鸣器/LED**: 系统状态提示

### 使用说明

#### 基本操作流程
1. 上电启动: 系统初始化，蜂鸣器响一声，LED闪烁
2. 自动循迹: 小车自动检测黑线并开始循迹
3. 路程点触发: 到达预设距离时蜂鸣器提示
4. 最终停止: 达到总路程阈值后自动停止

#### 按键功能
- **KEY1(PB12)**: 任务1选择
- **KEY2(PB13)**: 任务2选择  
- **KEY3(PB14)**: 任务3选择

#### 参数调整建议
1. **基础速度**: 根据赛道调整`basicspeed`(默认100)
2. **PID参数**: 根据电机特性调整KP、KI、KD
3. **循迹灵敏度**: 调整`gray_track.c`中的权重和增益系数
4. **路程阈值**: 修改`distance_threshold_cm`设置停止距离
5. **脉冲转换因子**: 校准`pulse_to_cm_factor`确保距离准确

### 扩展功能

#### 视觉识别集成
- 通过UART2接收视觉模块数据
- 支持路线点识别(A/B/C/D)和事件识别(剪刀/锤子/打火机)
- 自动路线选择功能

#### 语音提示
- 通过UART3连接语音模块
- 到达路线点时播报提示信息

#### 数据可视化
- 通过串口发送PID控制数据帧(0x03 0xfc开头)
- 可使用上位机软件实时显示控制曲线

### 注意事项

1. **电源要求**: 确保电机电源与MCU电源隔离，防止电机干扰
2. **传感器校准**: 首次使用需校准灰度传感器阈值和MPU6050零点
3. **机械安装**: 确保传感器离地高度一致，车轮对称
4. **中断优先级**: TIM3中断优先级设为0(最高)，确保控制周期稳定
5. **路程累计**: 仅在循迹模式使能时累计路程，离开循迹模式自动清零

### 故障排除

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 小车不启动 | 电源问题 | 检查电池电压和电源连接 |
| 不检测黑线 | 传感器问题 | 检查灰度传感器接线和阈值 |
| 循迹抖动 | PID参数不当 | 调整PID参数，降低P增益 |
| 角度漂移 | MPU6050未校准 | 重新校准MPU6050零点 |
| 编码器计数异常 | 接线错误 | 检查编码器A/B相接线 |
| 通信失败 | 波特率不匹配 | 检查串口波特率设置 |

### 版本历史

- **v1.0** (2026-04-05): 基本循迹功能，PID控制，路程累计
- **v1.1** (规划中): 增加视觉识别集成，优化控制算法

### 许可证

本项目仅供学习交流使用，遵循MIT开源协议。

### 贡献者

- 电赛2026参赛团队

---
*最后更新: 2026年4月10日*