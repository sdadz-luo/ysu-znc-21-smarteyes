# AGENTS.md — ysu-znc-21-smarteyes

NXP RT1064 (Cortex-M7, 600 MHz) 智能车竞赛固件。  
第21届全国大学生智能汽车竞赛 — 智能视觉组。

全文中英文注释混用。无测试框架。仅 IDE 构建。

---

## 构建与烧录

仅支持 IDE 构建：

| IDE | 项目文件 | 构建目标 |
|-----|---------|----------|
| Keil MDK 5.33 | `project/mdk/rt1064.uvprojx` | `nor_sdram_zf_dtcm` |
| IAR EWARM 8.32 | `project/iar/rt1064.eww` | `nor_sdram_zf_dtcm` |

**无命令行构建方式**。编译后通过 DAP-Link / J-Link 烧录。  
添加 `.c`/`.h` 文件时需同时放入 `project/code/` **并在 IDE 工程树中添加**。

关键预定义宏：`CPU_MIMXRT1064DVL6A`, `XIP_EXTERNAL_FLASH=1`, `USB_STACK_BM`, `MCUXPRESSO_SDK`, `SKIP_SYSCLK_INIT`, `PRINTF_FLOAT_ENABLE=1`。

C 标准：**C99**。编译器：ARMCLANG (Keil) / ARMCC (IAR)。

---

## 代码布局

```
project/
├── user/src/main.c      # 入口、状态机、PIT 中断控制循环
├── user/src/isr.c       # 外设中断处理（UART、CSI、GPIO、摄像头）
├── code/                # 用户应用代码 — 新文件放这里（平铺，无子目录）
├── mdk/                 # Keil 工程 + 分散加载文件 (scf/)
└── iar/                 # IAR 工程 + 链接脚本 (icf/)
libraries/               # 逐飞开源库 V3.9.2（**请勿修改**）
├── zf_common/           # headfile、typedef、clock、debug、fifo、math
├── zf_driver/           # 底层驱动抽象
├── zf_device/           # 外设驱动（摄像头、IMU、显示屏、BLE、TOF 等）
├── zf_components/       # 助手/调试接口
├── sdk/                 # NXP MCUXpresso SDK 2.12（fsl_* 驱动）
└── components/          # FATFS、SDMMC、USB 协议栈
```

唯一库包含头文件：`#include "zf_common_headfile.h"` — 提供所有逐飞 API（stdio、stdint、string、stdbool、math 及所有 fsl_* 外设驱动）。

---

## 用户代码（project/code/）

| 文件 | 用途 |
|------|------|
| `control.c/h` | 麦克纳姆轮运动学：`motor_solution(vx, vy, yaw, wz)` → 各轮速度 |
| `motor.c/h` | PWM 10kHz 初始化 + 四路电机占空比（FL/FR/BL/BR），H桥双极性控制 |
| `pid.c/h` | PID 控制（增量式 + 位置式），`pid_increm` + `pid_location` |
| `path_planning.c/h` | A\*、BFS、推箱子求解器、炸弹规划、ID 学习。**12×16=192格** |
| `cam_uart.c/h` | 摄像头 UART 协议 — 从 OPENMV 读取栅格 + 车位坐标 |
| `encoder.c/h` | 正交编码器读取（FL/FR/BL/BR），里程计 `distance(yaw)` |
| `imu963.c/h` | IMU963RA — 四元数 → 欧拉角（yaw）
| `menu.c/h` | TFT 菜单系统（模式选择、PID 调参、原点标定） |
| `tft180.c/h` | TFT180 显示屏（1.8"） |
| `my_uart.c/h` | 自定义调试串口 |
| `wireless_uart.c/h` | BLE 无线串口（遥控 / 遥测） |
| `key.c/h` | 按键输入 |

---

## 游戏状态机

```
NoGame → GameMap → Run → Waiting → Look → Run → ... → GameOver → End
```

`game_mode` 控制 5 种模式：

| 模式 | 行为 |
|------|------|
| 0 (Start) | 导航到栅格元素，通过摄像头读取其 ID |
| 1 (Normal Run) | 路径跟踪 + 推箱子 |
| 2 (Look) | 访问路径点，偏航对准后扫描 ID |
| 3 (ID Run) | 利用已学习的 ID 关联来行驶 |
| 4 (Bomb) | 推炸弹式推箱子来破墙 |

主控制循环在 **PIT 定时器 ISR**（`PIT_IRQHandler`，位于 `project/user/src/main.c`）：
- CH0：5ms — 编码器读取、IMU、里程计、PID、电机 PWM 输出
- CH1：10ms — 摄像头 UART 读取、状态更新

---

## 中断路由（易错点）

中断处理全部位于 `project/user/src/isr.c`。UART 映射（与 `cam_uart.c` 定义配合）：

| 外设 | 引脚 | 用途 | 处理函数 |
|------|------|------|---------|
| LPUART1 | B12/B13 | 调试串口 / 副摄像头 | `cam_uart_isc_2()` |
| LPUART2 | — | 未使用（空 handler） | — |
| LPUART3 | B22/B23 | **OPENMV 主摄像头** | `cam_uart_isc_1()` |
| LPUART4 | — | FlexIO 摄像头 + GNSS | `flexio_camera_uart_handler()` + `gnss_uart_callback()` |
| LPUART5 | — | 摄像头（备用） | `camera_uart_handler()` |
| LPUART6 | — | 未使用 | — |
| LPUART8 | D16/D17 | BLE 无线 + 调试中断 | `wireless_module_uart_handler()` + `debug_interrupr_handler()` |

**CAM UART 内部定义**（`cam_uart.c`）：
- `UART1` = `UART_3` (LPUART3, B22/B23) → OPENMV，接收栅格 + 车位数据
- `UART2` = `UART_1` (LPUART1, B12/B13) → ID 通信/调试

---

## PID 体系

`pid.c/h` 定义通用 PID 结构体，`pid_init()` 初始化全局实例。各 PID 用途：

| 实例 | 类型 | 用途 |
|------|------|------|
| `pid_FL/FR/BL/BR` | 增量式 `pid_increm` | 四轮速度闭环 |
| `pid_x` | 位置式 `pid_location` | X 方向位置环 |
| `pid_y` | 位置式 `pid_location` | Y 方向位置环 |
| `pid_yaw` | 位置式 `pid_location` | 偏航角闭环 |
| `pid_gyro` | — | 角速度环 |

**PID 参数运行时调节**（`menu.h` 全局变量）：`pid_x_p/i/d`、`pid_y_p/i/d`、`x_acc/dec`、`y_acc/dec`，通过 TFT 菜单设置，非硬编码。

---

## 运动学

`motor_solution(vx, vy, current_yaw, wz)` 执行：
1. 场坐标系 → 车体坐标系旋转
2. 非对称一阶低通滤波（加减速独立系数）
3. 麦克纳姆轮分解（轮距因子硬编码 `0.64f`）

---

## 硬件引脚分配

**摄像头**（OPENMV 通过 UART 通信，非 CSI 直连）：
- UART TX=C29 / RX=C28（副），TX=B22 / RX=B23（主）

电机（PWM 10 kHz，`MAX_DUTY=9000`）：
- FL=D13/D12, FR=D15/D14, BL=D1/D0, BR=D3/D2

正交编码器：C0/C1、C2/C24、C3/C25、B18/B19  
调试串口：TX=B12 RX=B13  
TFT：SCK=B0 MOSI=B1 CS=B3  
舵机：C30 C31  
BLE：TX=D16 RX=D17  
**C4-C15 避免用作输入**（flexio 摄像头限制）。

---

## OPENMV 通信协议

摄像头通过 UART 发送文本协议，`cam_uart.c` 中 `push_cam_data()` 解析：

1. 小车位置：`"car:%.1f,%.1f;"` → `cam_uart_data.car_cx`, `car_cy`
2. 栅格数据：12 行 × 16 列，`uint8_t grid[12][16]`

**栅格元素值**：
- `0` = 空地, `1` = 墙, `2` = 小车, `3` = 箱子, `6` = 目标点, `7` = 炸弹

ID 通信：`cam_uart2_write(type)` 发送类型，`cam_uart2_read()` 读取识别的 ID。

---

## 路径规划内存消耗

路径规划在 MCU 上运行，资源紧张：

| 结构 | 最大尺寸 |
|------|---------|
| A\* open set | 50000 |
| 优先队列 | 10000 |
| A\* 搜索次数 | 50000 |
| 单步 A\* 路径 | 500 点 |
| 完整路径 | 500 点 |
| BFS 模拟队列 | 16384 |

堆大小定义在链接脚本中（`project/mdk/scf/`、`project/iar/icf/`）。

---

## 内存模型

- 代码：XIP 从外部 Flash 执行
- 大缓冲区：SDRAM
- 时序关键数据：DTCM（中断、惯导、PID 变量）
- 构建目标 `nor_sdram_zf_dtcm` 对应此模型

---

## OpenCode 使用

- `opencode.json` 未配置 — 如需代理/指令请在仓库根目录创建
- `.gitignore` 排除了 `opencode/` 和 `.vscode/` — 本地配置不提交
- 最相关的用户技能：`copilot-embedded`、`copilot-cpp`、`copilot-general`、`embedded-engineer`
- 所有代码为 C99，注意 ARMCC 编译器扩展
- 始终尊重 `libraries/` 边界 — 逐飞库 + NXP SDK 代码无需修改
- 调试手段有限：TFT 显示 + 串口输出，无硬件调试器时依赖这两者
- 待解决问题记录在 `/待解决问题.md`
