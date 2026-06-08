# ysu-znc-21-smarteyes

> 🏎️ **燕山大学 · 第21届全国大学生智能汽车竞赛 — 智能视觉组**

[![Platform](https://img.shields.io/badge/Platform-NXP%20RT1064-blue)](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/i-mx-rt-crossover-mcus/i-mx-rt1064-crossover-mcu-with-arm-cortex-m7-core:i.MX-RT1064)
[![IDE](https://img.shields.io/badge/IDE-IAR%208.32%20%7C%20Keil%205.33-green)](#开发环境)
[![Language](https://img.shields.io/badge/Language-C-orange)](https://en.wikipedia.org/wiki/C_(programming_language))
[![License](https://img.shields.io/badge/License-GPLv3-lightgrey)](./libraries/doc/GPL3_permission_statement.txt)

---

## 📖 项目简介

本项目是燕山大学参加**第21届全国大学生智能汽车竞赛（智能视觉组）**的参赛代码，基于 **NXP RT1064 (MIMXRT1064DVL6A)** 微控制器开发，运行于逐飞科技 RT1064 核心板。

智能视觉组要求小车具备**自主视觉识别 + 运动控制**能力，能够完成目标检测、标识识别、路径规划与自主行驶等任务。

---

## 🛠️ 硬件平台

| 类别 | 型号 |
|------|------|
| **MCU** | NXP MIMXRT1064DVL6A (Cortex-M7, 600MHz) |
| **核心板** | 逐飞科技 RT1064 核心板 |
| **相机** | MT9V03X / OV7725 / SCC8660 / TSL1401 线阵CCD |
| **显示器** | IPS114 (1.14") / IPS200 (2.0") / TFT180 (1.8") / OLED (0.96") |
| **IMU** | MPU6050 / ICM20602 / IMU660RA / IMU963RA |
| **测距** | DL1A / DL1B TOF |
| **无线通信** | BLE6A20 蓝牙 / CH9141 蓝牙 / WiFi-SPI / WiFi-UART |
| **其他** | 绝对编码器 / GNSS/GPS / 虚拟示波器 |

---

## 📁 项目结构

```
ysu-znc-21-smarteyes/
├── README.md                          # 项目说明
├── libraries/                         # 库文件（逐飞开源库）
│   ├── doc/                           # 文档与版本记录
│   ├── sdk/                           # NXP MCUXpresso SDK
│   │   ├── board/                     # 板级初始化、引脚配置
│   │   ├── CMSIS/                     # CMSIS 核心
│   │   ├── cmsis_drivers/             # CMSIS 驱动
│   │   ├── components/                # SDK 中间件
│   │   ├── deceive/                   # 设备头文件与系统文件
│   │   ├── drives/                    # fsl_* 外设驱动
│   │   ├── startup/                   # IAR/MDK 启动文件
│   │   ├── utilities/                 # 调试控制台、Shell等工具
│   │   └── xip/                       # XIP Flash/SDRAM 配置
│   ├── components/                    # 第三方组件
│   │   ├── fatfs/                     # FAT 文件系统
│   │   ├── sdmmc/                     # SD/MMC 驱动
│   │   └── usb/                       # USB 协议栈
│   ├── zf_common/                     # 逐飞通用工具库
│   ├── zf_components/                 # 逐飞助手/调试接口
│   ├── zf_device/                     # 外设驱动模块（相机、IMU、屏幕等）
│   └── zf_driver/                     # 底层驱动抽象
└── project/                           # 工程文件
    ├── code/                          # 用户自添加代码
    ├── iar/                           # IAR EWARM 工程
    │   ├── rt1064.ewp                 # IAR 项目文件
    │   ├── icf/                       # 链接脚本
    │   └── program/                   # 调试配置
    ├── mdk/                           # Keil MDK 工程
    │   ├── rt1064.uvprojx             # Keil 项目文件
    │   └── scf/                       # 分散加载文件
    └── user/                          # 用户应用代码
        ├── inc/                       # 头文件
        └── src/                       # 源文件（main.c、PID、运动控制等）
```

---

## 💻 开发环境

| 工具 | 版本 |
|------|------|
| **IAR Embedded Workbench** | 8.32.4 |
| **Keil MDK (µVision)** | 5.33 |
| **编译器** | IAR ICCARM / ARMCLANG |

> ⚠️ 项目配置了 `nor_sdram_zf_dtcm` 构建目标，使能了 `XIP_EXTERNAL_FLASH`、`USB_STACK_BM` 等宏。

---

## 🧠 软件架构

### 主控流程 (`project/user/src/main.c`)

1. **初始化阶段**
   - 系统时钟 → 600MHz
   - 调试串口、TFT 显示屏、按键
   - Flash 存储、编码器、IMU
   - UART 通信、电机 PWM、PID
   - 摄像头串口、PIT 定时器

2. **主循环**
   - TFT 显示编码器 & IMU 数据
   - **竞赛状态机**：
     - `NoGame` → `GameMap` → `Waiting` → `Look` → `Run` → `GameOver` → `End`

3. **PIT 定时中断**（5ms / 10ms 周期）
   - 编码器 & IMU 数据采集
   - 里程计 & 航向角更新
   - PID 控制计算
   - 电机 PWM 输出
   - 摄像头数据获取
   - 串口数据更新

### 支持的外设驱动 (`libraries/zf_device/`)

| 类型 | 驱动文件 | 支持硬件 |
|------|----------|----------|
| 相机 | `zf_device_mt9v03x` | MT9V03X |
| 相机 | `zf_device_ov7725` | OV7725 |
| 相机 | `zf_device_scc8660` | SCC8660 |
| 相机 | `zf_device_tsl1401` | TSL1401 线阵CCD |
| 显示屏 | `zf_device_ips114` | IPS 1.14" |
| 显示屏 | `zf_device_ips200` | IPS 2.0" |
| 显示屏 | `zf_device_tft180` | TFT 1.8" |
| 显示屏 | `zf_device_oled` | OLED 0.96" |
| IMU | `zf_device_mpu6050` | MPU6050 |
| IMU | `zf_device_icm20602` | ICM20602 |
| IMU | `zf_device_imu660ra` | IMU660RA |
| IMU | `zf_device_imu963ra` | IMU963RA |
| 测距 | `zf_device_dl1a` | DL1A TOF |
| 测距 | `zf_device_dl1b` | DL1B TOF |
| 蓝牙 | `zf_device_ble6a20` | BLE6A20 |
| 蓝牙 | `zf_device_bluetooth_ch9141` | CH9141 |
| WiFi | `zf_device_wifi_spi` | WiFi-SPI |
| WiFi | `zf_device_wifi_uart` | WiFi-UART |
| 编码器 | `zf_device_absolute_encoder` | 360° 绝对编码器 |
| GNSS | `zf_device_gnss` | GPS/GNSS |
| 按键 | `zf_device_key` | 按键输入 |
| 调试 | `zf_device_virtual_oscilloscope` | 虚拟示波器 |

---

## 🚀 构建与烧录

### IAR
1. 打开 `project/iar/rt1064.ewp`
2. 选择构建目标：`nor_sdram_zf_dtcm`
3. 编译 → 下载到 RT1064 核心板

### Keil MDK
1. 打开 `project/mdk/rt1064.uvprojx`
2. 选择构建目标：`nor_sdram_zf_dtcm`
3. 编译 → 使用 DAP-Link / J-Link 烧录

---

## 📄 开源协议

本项目基于逐飞科技 RT1064 开源库开发，底层库遵循 **[GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html)** 协议。

详见：[`libraries/doc/GPL3_permission_statement.txt`](./libraries/doc/GPL3_permission_statement.txt)

---

## 🙏 致谢

- [逐飞科技 (SeekFree)](https://seekfree.com.cn/) — RT1064 开源库与核心板
- [NXP Semiconductors](https://www.nxp.com/) — i.MX RT1064 平台
- 燕山大学智能车团队全体成员

---

> 🏁 *Keep Racing, Keep Dreaming!*