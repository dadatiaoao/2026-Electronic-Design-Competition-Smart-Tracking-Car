| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

# ESP32 BLE串口透传服务器（STM32数据无线转发）

## 项目简介
本项目基于ESP-IDF的BLE SPP示例修改，用于将STM32的串口输出数据通过蓝牙BLE无线传输到手机或电脑。ESP32作为BLE SPP服务器，从UART1接收STM32的`printf()`输出（波特率115200），并通过蓝牙BLE将数据转发到连接的客户端设备。

## 功能特性
- 接收STM32串口数据（115200波特率，8N1）
- 通过蓝牙BLE广播SPP串口服务
- 支持手机端蓝牙串口APP连接
- 单向数据传输（STM32 → ESP32 → 手机/电脑）
- 自动数据分包处理，支持长数据包

## 硬件连接

### ESP32引脚配置
```
STM32 (115200波特率)      ESP32 (UART1)
-------------------      --------------------
TX  ---------------->    GPIO9   (UART1_RX)
RX  <----------------    GPIO10  (UART1_TX) - 可选，本项目无需反向传输
GND ----------------     GND
```

### 修改说明
本项目在原ESP-IDF BLE SPP示例基础上进行了以下修改：

1. **UART端口**：从UART0改为UART1，避免占用USB串口
2. **引脚定义**：TX=GPIO10，RX=GPIO9
3. **流控设置**：禁用硬件流控（`UART_HW_FLOWCTRL_DISABLE`）
4. **波特率**：115200（与STM32 `printf`输出匹配）
5. **代码修复**：修复了硬编码UART端口问题，使用任务参数传递端口号

## 编译和烧录

### 环境要求
- ESP-IDF v5.5.3 或更高版本
- 已设置ESP-IDF开发环境

### 编译步骤
```bash
# 进入项目目录
cd "C:\Users\X\Downloads\电赛2026\spp_server"

# 配置项目（可选）
idf.py menuconfig
# 可修改BLE设备名称（默认：nimble-ble-spp-svr）

# 编译项目
idf.py build

# 烧录到ESP32
idf.py -p COMx flash monitor  # COMx为ESP32的串口号
```

### 监控输出
烧录后，串口监控将显示：
- BLE初始化状态
- 设备MAC地址
- 蓝牙广播状态
- 连接建立/断开信息
- 数据发送日志

按 `Ctrl+]` 退出监控。

## 手机端测试

### Android手机
1. 安装 **Serial Bluetooth Terminal**（或其他蓝牙串口APP）
2. 打开APP，搜索蓝牙设备
3. 找到并连接 **nimble-ble-spp-svr**
4. STM32发送 `printf("测试数据\n")`，手机端应能接收

### iOS手机
1. 安装 **LightBlue** 或其他BLE调试工具
2. 搜索并连接设备
3. 查找SPP服务特征值，订阅通知

### 电脑端
1. 使用支持蓝牙SPP的串口工具（如Putty + 蓝牙串口驱动）
2. 配对ESP32蓝牙设备
3. 创建蓝牙串口连接

## 技术细节

### 数据包结构
- 最大单次接收长度：120字节
- MTU协商：ESP32之间为200字节，手机连接通常小于123字节
- 长数据分包：超过MTU的数据会自动分片，每片添加4字节包头（`##` + 总包数 + 当前包序号）
- 手机APP需处理分包逻辑（多数串口APP已支持）

### GATT服务特征值
| 特征值 | UUID | 权限 | 功能 |
|--------|------|------|------|
| SPP_DATA_RECV_CHAR | 0xABF1 | READ&WRITE_NR | 接收数据 |
| SPP_DATA_NOTIFY_CHAR | 0xABF2 | READ&NOTIFY | 发送数据（通知） |
| SPP_COMMAND_CHAR | 0xABF3 | READ&WRITE_NR | 命令通道 |
| SPP_STATUS_CHAR | 0xABF4 | READ&NOTIFY | 状态通知 |

### 软件架构
```
STM32 printf()
    ↓ (UART 115200)
ESP32 UART1接收
    ↓ (FreeRTOS队列)
ble_server_uart_task
    ↓ (BLE GATT通知)
手机/电脑蓝牙客户端
```

## 故障排除

### 常见问题
1. **手机搜不到设备**
   - 确认ESP32已上电且程序运行
   - 查看串口监控是否有"BLE Host Task Started"和"advertise"日志
   - 确保手机蓝牙已开启

2. **连接后收不到数据**
   - 检查STM32到ESP32的物理连接
   - 确认STM32波特率为115200
   - 验证手机APP已订阅通知（开启"接收"功能）
   - 查看监控是否有"Notification sent successfully"日志

3. **数据乱码或丢失**
   - 检查地线连接
   - 确认双方波特率一致（115200, 8N1）
   - 尝试降低STM32数据发送频率

### 修改引脚
如需更改UART引脚，修改`main.c`中`ble_spp_uart_init()`函数的`uart_set_pin()`调用：
```c
uart_set_pin(UART_NUM_1, new_tx_pin, new_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
```

## 示例输出
```
I (464) NimBLE_SPP_BLE_PRPH: BLE Host Task Started
GAP procedure initiated: stop advertising.
Device Address: 7c:df:a1:40:3e:fa
GAP procedure initiated: advertise; disc_mode=2 adv_channel_map=0 own_addr_type=0 adv_filter_policy=0 adv_itvl_min=0 adv_itvl_max=0
connection established; status=0 handle=1 our_ota_addr_type=0 our_ota_addr=7c:df:a1:40:3e:fa our_id_addr_type=0 our_id_addr=7c:df:a1:40:3e:fa peer_ota_addr_type=0 peer_ota_addr=7c:df:a1:c2:19:92 peer_id_addr_type=0 peer_id_addr=7c:df:a1:c2:19:92 conn_itvl=40 conn_latency=0 supervision_timeout=256 encrypted=0 authenticated=0 bonded=0

I (6924) NimBLE_SPP_BLE_PRPH: Data received in write event,conn_handle = 1,attr_handle = 11
1b5b41I
(10824) NimBLE_SPP_BLE_PRPH: Notification sent successfully
```

## 许可证
本项目基于ESP-IDF示例代码修改，遵循原项目的许可证（Unlicense OR CC0-1.0）。
