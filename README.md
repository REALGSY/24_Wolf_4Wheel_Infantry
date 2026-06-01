# 24_Wolf_4Wheel_Infantry

RoboMaster 四舵轮步兵机器人控制代码，包含底盘控制与云台控制两部分工程。

## 项目简介

本仓库为 2024 赛季四舵轮步兵机器人电控代码，主要实现四舵轮底盘运动控制、云台控制、射击控制、视觉通信、CAN 通信、IMU 数据处理、功率控制、设备状态检测以及裁判系统 UI 等功能。

代码基于 STM32 嵌入式平台开发，使用 C 语言编写，并结合 FreeRTOS 进行多任务调度，适用于 RoboMaster 步兵机器人底盘与云台分布式控制系统。

## 仓库结构

```text
24_Wolf_4Wheel_Infantry
├── Chassis
│   └── GOODFOLLOW          # 底盘控制工程
│       ├── Apps            # 底盘应用层控制逻辑
│       ├── BSP             # 板级支持包
│       ├── Devices         # 外设驱动封装
│       ├── Drivers         # STM32 HAL 驱动
│       ├── Middlewares     # FreeRTOS 等中间件
│       ├── Task            # FreeRTOS 任务
│       └── MDK-ARM         # Keil 工程文件
│
├── Head
│   └── DJI_CIMU            # 云台控制工程
│       ├── Application     # 应用层逻辑
│       ├── Apps            # 云台控制、射击等功能模块
│       ├── Bsp             # 板级支持包
│       ├── Devices         # 设备驱动封装
│       ├── Drivers         # STM32 HAL 驱动
│       ├── Middlewares     # 中间件
│       ├── Task            # FreeRTOS 任务
│       └── MDK-ARM         # Keil 工程文件
│
└── README.md
```

## 主要功能

### 底盘控制

* 四舵轮底盘运动控制
* 底盘跟随云台控制
* CAN 总线电机通信
* 遥控器与上位机控制输入处理
* 底盘功率限制与功率管理
* 机器人状态更新与设备离线检测
* 与云台、IMU、视觉模块的数据交互

### 云台控制

* 云台 yaw / pitch 控制
* IMU 姿态解算与补偿
* 射击机构控制
* 视觉自瞄通信接口
* 裁判系统数据处理
* 云台与底盘之间的数据通信
* FreeRTOS 多任务调度

## 技术栈

* 编程语言：C
* 开发平台：STM32
* 实时操作系统：FreeRTOS
* 开发工具：Keil MDK-ARM、STM32CubeMX
* 通信方式：CAN、UART
* 应用场景：RoboMaster 四舵轮步兵机器人

## 编译与烧录

1. 克隆仓库：

```bash
git clone https://github.com/REALGSY/24_Wolf_4Wheel_Infantry.git
```

2. 打开对应工程：

底盘工程：

```text
Chassis/GOODFOLLOW/MDK-ARM
```

云台工程：

```text
Head/DJI_CIMU/MDK-ARM
```

3. 使用 Keil MDK 打开工程文件。

4. 根据实际硬件修改串口、CAN、电机 ID、遥控器参数等配置。

5. 编译工程并通过 ST-Link 下载到对应 STM32 主控板。

## 模块说明

### Chassis/GOODFOLLOW/Apps

该目录主要包含底盘应用层代码，例如：

* `Chassis_control.c/.h`：底盘运动控制
* `Cloud_control.c/.h`：云台/底盘联动控制
* `Control_Vision.c/.h`：视觉控制接口
* `Power_control.c/.h`：功率控制
* `Robot_control.c/.h`：机器人整体控制逻辑
* `Shot.c/.h`：射击相关控制
* `Status_Update.c/.h`：机器人状态更新
* `DevicesCheck.c/.h`：设备状态检测
* `RMClient_UI.c/.h`：客户端 UI 绘制与交互

### Chassis/GOODFOLLOW/Task

该目录主要包含底盘侧 FreeRTOS 任务，例如：

* CAN 消息处理任务
* 机器人控制任务
* IMU 数据处理任务
* 设备检测任务
* UI 更新任务
* 主机通信任务

### Head/DJI_CIMU

该目录为云台侧工程，主要负责云台姿态控制、IMU 数据处理、射击机构控制、视觉通信以及与底盘之间的数据交互。

## 使用说明

本代码为 RoboMaster 机器人比赛工程代码，使用前需要根据实际机器人硬件进行参数适配，包括：

* 电机型号与电机 ID
* CAN 总线分配
* 遥控器通道映射
* 串口配置
* IMU 安装方向
* 云台零位
* 底盘轮距、轴距、舵轮安装方向
* 功率限制参数
* 裁判系统与视觉模块通信协议

## 注意事项

* 烧录前请确认底盘与云台工程对应的主控板型号。
* 上电前请检查 CAN 线、串口线、电源线连接是否正确。
* 初次运行时建议架空机器人，防止电机方向或 PID 参数错误导致失控。
* 修改电机 ID、串口或 CAN 配置后，需要同步检查相关初始化代码和任务代码。
* 本项目代码主要面向比赛实车调试，部分参数需要结合实际机械结构进行整定。

## 项目状态

该仓库为 2024 赛季四舵轮步兵机器人控制代码，当前主要用于代码归档、复盘和后续参考。

## Author

REALGSY

## License

本仓库暂未声明开源许可证，仅供学习、交流与 RoboMaster 相关技术参考使用。
