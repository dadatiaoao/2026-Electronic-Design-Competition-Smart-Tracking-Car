# 电赛2026 - 8字路线循迹小车项目 (状态机版本)

## 项目概述

本项目是一个基于STM32F103微控制器的智能小车系统，专门针对电赛2026第二题8字路线循迹任务设计。小车采用8路灰度传感器检测地面黑线，通过状态机(FSM)精确控制8字轨迹的各个阶段。系统在保留第一题PID控制架构的基础上，引入了高级状态机管理，实现对角直行、弧线循迹、角度对齐等复杂动作的自动切换。

**核心特性**:
- **状态机控制**: 8阶段状态机管理8字轨迹的全流程
- **几何轨迹规划**: 基于预设的对角线距离(102cm)、弧线长度(83cm)、旋转角度(50°)进行精确控制
- **自适应触发**: 支持距离触发和传感器触发双保险机制
- **角度对齐**: 使用MPU6050进行航向角对齐，确保轨迹闭合精度
- **模块化设计**: 状态机、几何参数、PID控制分层独立，便于调试调整

## 硬件架构

### 主控芯片
- STM32F103C8T6 (ARM Cortex-M3内核)

### 传感器模块
1. **8路灰度传感器** - 地面黑线检测
   - 传感器布局：D1~D8从左到右排列
   - 检测逻辑：1=检测到黑线，0=未检测到黑线
2. **MPU6050六轴传感器** - 姿态测量(加速度计+陀螺仪)
   - 用于角度对齐和旋转控制
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
2. **蜂鸣器** - 阶段切换提示
3. **LED指示灯** - 系统状态指示
4. **按键** ×3 - 模式选择和控制

### 通信接口
1. **UART串口** ×3
   - UART1: 调试输出(115200bps)
   - UART2: 视觉识别模块通信(预留)
   - UART3: 语音模块通信(预留)
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
│   ├── main.c          # 8字路线状态机主程序
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

#### 1. 状态机控制模块 (main.c)
**功能**: 实现8字路线轨迹的状态机管理，控制8个阶段的自动切换。

**状态枚举**:
```c
typedef enum
{
    STATE_INIT = 0,      // 初始化状态
    STATE_TURN_AC,       // A点向内旋转，准备A->C对角直行
    STATE_GO_C,          // A->C 直行
    STATE_ALIGN_CB,      // C点对齐，准备进入右侧半弧循迹
    STATE_TRACK_CB,      // C->B 半弧循迹
    STATE_TURN_BD,       // B点向内旋转，准备B->D对角直行
    STATE_GO_D,          // B->D 直行
    STATE_ALIGN_DA,      // D点对齐，准备进入左侧半弧循迹
    STATE_TRACK_DA,      // D->A 半弧循迹
    STATE_FINISH,        // 完成状态
    STATE_IDLE           // 空闲状态
} Task2_State_t;
```

**关键函数**:
- `void Task2_Process(void)` - 状态机处理函数，在主循环中调用
- `void node_action(void)` - 节点动作函数，清零里程并声光提示
- `float distance_cm_snapshot(void)` - 获取里程快照（原子操作）
- `uint8_t detect_track(void)` - 检测是否有传感器检测到黑线

**状态转移条件**:
- 距离触发: `dist_cm >= (预设距离 - 容差)`
- 传感器触发: `detect_track()`检测到黑线
- 角度对齐: `angle_aligned()`满足角度容差要求

#### 2. 几何参数配置模块 (main.c)
**预定义几何常量**:
```c
#define DIST_DIAGONAL_CM        102.0f  // 对角线长度
#define DIST_ARC_CM             83.0f   // 弧线长度（修正值）
#define ANGLE_DIAGONAL          50.0f   // 旋转角度

#define SPEED_STRAIGHT          150     // 直行速度
#define SPEED_TRACK             130     // 循迹速度
#define SPEED_TURN              90      // 转弯速度
```

**角度微调接口** (赛道实测后调整):
```c
#define TRIM_TURN_AC            +15.0f  // AC转向微调
#define TRIM_ALIGN_CB           -5.0f   // CB对齐微调
#define TRIM_TURN_BD            +5.0f   // BD转向微调
#define TRIM_ALIGN_DA           -10.0f  // DA对齐微调
```

**旋转方向符号**:
```c
#define SIGN_TURN_AC            (-1.0f) // AC转向方向
#define SIGN_ALIGN_CB           (+1.0f) // CB对齐方向
#define SIGN_TURN_BD            (+1.0f) // BD转向方向
#define SIGN_ALIGN_DA           (-1.0f) // DA对齐方向
```

#### 3. PID控制器模块 (pid.c/.h)
**功能**: 沿用第一题的PID控制架构，用于电机速度环和车身角度环控制。

**关键参数**:
```c
// 电机A速度环PID(增量式)
pid_init(&motorA, DELTA_PID, 100.0f, 0.0f, 5.0f);
// 电机B速度环PID(增量式)
pid_init(&motorB, DELTA_PID, 100.0f, 0.0f, 5.0f);
// 角度环PID(位置式)
pid_init(&angle, POSITION_PID, 2.0f, 0.0f, 0.1f);
```

**控制流程**:
1. TIM3定时器中断每10ms触发一次
2. 调用`pid_control(basicspeed)`函数
3. 如果循迹模式使能(`xunji=1`)，调用`track()`函数调整左右速度差
4. 速度环PID计算：编码器反馈 vs 目标速度
5. 输出PWM控制电机

#### 4. 灰度循迹模块 (gray_track.c/.h)
**功能**: 8路灰度传感器数据采集与黑线循迹算法，与第一题相同。

**传感器布局权重**:
```
传感器: D1  D2  D3  D4  D5  D6  D7  D8
权重:   -7  -5  -3  -1   1   3   5   7
位置:   左 ←                    → 右
```

#### 5. 电机控制模块 (motor.c/.h)
**功能**: 电机PWM驱动和编码器脉冲计数，与第一题相同。

#### 6. 滤波算法模块 (filter.c/.h)
**功能**: 传感器数据滤波处理，提高姿态测量精度，与第一题相同。

#### 7. 姿态传感器模块 (ml_mpu6050.c/.h)
**功能**: MPU6050六轴传感器驱动，提供姿态角数据，与第一题相同。

### main.c程序流程详解
**文件位置**: `user/main.c`

**程序总体流程**:
1. **硬件初始化** (第245-255行)
   - 蜂鸣器、OLED、电机、编码器、按键、LED、串口、灰度传感器、I2C、MPU6050初始化
2. **PID参数初始化** (第257-260行)
   - 电机A/B速度环PID (增量式PID): KP=100.0, KI=0.0, KD=5.0
   - 角度环PID (位置式PID): KP=2.0, KI=0.0, KD=0.1
3. **中断初始化** (第262-264行)
   - EXTI_PB7中断 (MPU6050数据就绪)，上升沿触发
   - TIM3定时器中断，10ms周期，优先级0 (最高)
4. **初始状态配置** (第266-277行)
   - `xunji = 0`: 初始关闭循迹模式
   - `basicspeed = 0`: 初始速度归零
   - `jiaodu = yaw_gyro`: 初始角度设为当前角度
   - `tracking_distance_cm = 0.0f`: 清零累计路程
   - `tracking_distance_enabled = 1`: 使能路程累计
   - `distance_threshold_cm = 10000.0f`: 设置超大阈值（由状态机自行判断）
   - `pulse_to_cm_factor = 0.0776f`: 脉冲到厘米转换因子
5. **启动提示** (第277-278行)
   - 调用`node_action()`: 蜂鸣器响、LED闪烁、清零里程
   - 串口输出启动信息
6. **主循环** (第280-284行)
   - 调用`Task2_Process()`状态机处理函数
   - `delay_ms(10)`防止CPU占用率过高

**状态机详细流程**:
```
初始化(STATE_INIT)
    ↓
A点旋转(STATE_TURN_AC): 旋转50°准备A->C对角直行
    ↓
A->C直行(STATE_GO_C): 直行102cm或检测到黑线
    ↓
C点对齐(STATE_ALIGN_CB): 旋转50°准备C->B弧线循迹
    ↓
C->B弧线循迹(STATE_TRACK_CB): 循迹83cm且角度对齐
    ↓
B点旋转(STATE_TURN_BD): 旋转50°准备B->D对角直行
    ↓
B->D直行(STATE_GO_D): 直行102cm或检测到黑线
    ↓
D点对齐(STATE_ALIGN_DA): 旋转50°准备D->A弧线循迹
    ↓
D->A弧线循迹(STATE_TRACK_DA): 循迹83cm且角度对齐
    ↓
完成(STATE_FINISH): 停止电机，进入空闲状态
    ↓
空闲(STATE_IDLE)
```

**关键全局变量**:
- `g_task2_state`: 当前状态机状态
- `g_initial_yaw`: 初始角度记录
- `xunji`: 循迹模式使能标志 (1=开启，0=关闭)
- `basicspeed`: 基础前进速度 (0-255)
- `jiaodu`: 目标角度 (度)
- `tracking_distance_cm`: 累计循迹路程 (厘米)

**中断驱动控制**:
- **TIM3中断 (10ms周期)**: 调用`pid_control(basicspeed)`执行核心控制算法
- **EXTI7中断 (MPU6050数据就绪)**: 读取传感器数据并更新姿态角

### 控制参数配置

#### 状态机参数 (在main.c中设置)
```c
#define DIST_DIAGONAL_CM        102.0f  // 对角线长度，根据实际赛道调整
#define DIST_ARC_CM             83.0f   // 弧线长度，根据实际赛道调整
#define ANGLE_DIAGONAL          50.0f   // 旋转角度，根据实际赛道调整

#define SPEED_STRAIGHT          150     // 直行速度，根据电机性能调整
#define SPEED_TRACK             130     // 循迹速度，根据循迹稳定性调整
#define SPEED_TURN              90      // 转弯速度，根据转向精度调整

#define TURN_TIMEOUT_MS         2600U   // 旋转超时时间
#define FINISH_STOP_MS          500U    // 完成停止延时
#define DIST_REACH_EPS_CM       1.0f    // 距离到达容差
#define ANGLE_ALIGN_TOLERANCE   6.0f    // 角度对齐容差
```

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

### 全局变量说明

#### 状态机变量
- `Task2_State_t g_task2_state` - 当前状态机状态
- `float g_initial_yaw` - 初始角度记录

#### 控制状态变量
- `volatile int xunji` - 循迹模式使能(1=开启，0=关闭)
- `volatile float jiaodu` - 目标角度(度)
- `volatile int basicspeed` - 基础前进速度(0-255)

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
├── 硬件初始化
│   ├── Buzzer_Init()
│   ├── OLED_Init()
│   ├── motor_init()
│   ├── encoder_init()
│   ├── Key_Init()
│   ├── LED_Init()
│   ├── uart_init()
│   ├── gray_init()
│   ├── I2C_Init()
│   └── MPU6050_Init()
├── PID初始化
│   ├── pid_init(&motorA, ...)
│   ├── pid_init(&motorB, ...)
│   └── pid_init(&angle, ...)
├── 中断配置
│   ├── exti_init(EXTI_PB7, ...)
│   └── tim_interrupt_ms_init(TIM_3, 10, 0)
├── 初始状态配置
└── 主循环
    └── Task2_Process()
        ├── distance_cm_snapshot()
        ├── detect_track()
        ├── node_action()
        ├── rotate_to_angle()
        └── angle_aligned()

TIM3中断(10ms周期)
└── pid_control(basicspeed)
    ├── gray_read() [如果xunji=1]
    ├── track()     [如果xunji=1]
    ├── pid_cal(&motorA)       # 电机A速度环
    ├── pid_cal(&motorB)       # 电机B速度环
    ├── pidout_limit(&motorA)  # 输出限幅
    ├── pidout_limit(&motorB)  # 输出限幅
    ├── motorA_duty(motorA.out) # PWM输出
    └── motorB_duty(motorB.out) # PWM输出
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
- **串口1(115200bps)**: 状态机切换信息、角度数据输出
- **OLED显示屏**: 实时状态显示
- **蜂鸣器/LED**: 阶段切换提示

### 使用说明

#### 基本操作流程
1. 上电启动: 系统初始化，蜂鸣器响一声，LED闪烁
2. 自动运行: 小车自动执行8字路线状态机
3. 阶段提示: 每个阶段切换时蜂鸣器提示
4. 完成停止: 完成8字路线后自动停止

#### 按键功能 (预留)
- **KEY1(PB12)**: 启动/停止控制
- **KEY2(PB13)**: 模式选择
- **KEY3(PB14)**: 参数重置

#### 参数调整建议
1. **几何参数**: 根据实际赛道调整`DIST_DIAGONAL_CM`、`DIST_ARC_CM`、`ANGLE_DIAGONAL`
2. **速度参数**: 根据电机性能调整`SPEED_STRAIGHT`、`SPEED_TRACK`、`SPEED_TURN`
3. **微调参数**: 赛道实测后调整`TRIM_TURN_AC`等四个微调值
4. **方向符号**: 如果旋转方向相反，修改`SIGN_TURN_AC`等四个符号
5. **PID参数**: 根据电机特性调整KP、KI、KD
6. **循迹灵敏度**: 调整`gray_track.c`中的权重和增益系数

### 扩展功能

#### 视觉识别集成 (预留)
- 通过UART2接收视觉模块数据
- 支持路线点识别和事件识别

#### 语音提示 (预留)
- 通过UART3连接语音模块
- 到达关键点时播报提示信息

#### 数据可视化
- 通过串口发送状态机切换数据帧
- 可使用上位机软件实时显示状态转移

### 注意事项

1. **电源要求**: 确保电机电源与MCU电源隔离，防止电机干扰
2. **传感器校准**: 首次使用需校准灰度传感器阈值和MPU6050零点
3. **机械安装**: 确保传感器离地高度一致，车轮对称
4. **中断优先级**: TIM3中断优先级设为0(最高)，确保控制周期稳定
5. **状态机调试**: 通过串口输出观察状态转移，确保每个阶段触发条件正确
6. **角度对齐**: 确保MPU6050安装方向与车体坐标系一致

### 故障排除

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 状态机卡在某个状态 | 触发条件未满足 | 检查距离计算或传感器检测逻辑 |
| 旋转角度不准确 | MPU6050未校准 | 重新校准MPU6050零点 |
| 直行距离偏差大 | 脉冲转换因子不准确 | 校准`pulse_to_cm_factor` |
| 弧线循迹偏离 | 循迹参数不合适 | 调整循迹权重和增益 |
| 角度对齐失败 | 角度容差设置过小 | 调整`ANGLE_ALIGN_TOLERANCE` |
| 通信失败 | 波特率不匹配 | 检查串口波特率设置 |

### 版本历史

- **v2.0** (2026-04-09): 8字路线状态机版本，支持几何轨迹规划和角度对齐
- **v1.0** (2026-04-05): 基本循迹功能，PID控制，路程累计

### 许可证

本项目仅供学习交流使用，遵循MIT开源协议。

### 贡献者

- 电赛2026参赛团队

---
*最后更新: 2026年4月10日*