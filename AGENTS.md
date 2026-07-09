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

clangd 附加宏（`compile_flags.txt`）：`__GNUC__`, `-U_WIN32`, `-nostdinc`。

C 标准：**C99**。编译器：ARMCLANG (Keil) / ARMCC (IAR)。

---

## 代码布局

```
project/
├── user/src/main.c      # 入口、状态机、PIT 中断控制循环
├── user/src/isr.c       # 外设中断处理（UART、CSI、GPIO、FlexIO、ToF）
├── user/inc/isr.h       # ISR 头文件（当前为空占位符）
├── code/                # 用户应用代码 — 新文件放这里（平铺，无子目录）
├── mdk/                 # Keil 工程 + 分散加载文件 (scf/)
├── iar/                 # IAR 工程 + 链接脚本 (icf/)
├── RT1064核心板丝印与芯片引脚对应表格.xlsx
└── RT1064智能车推荐引脚分配.txt   # 官方引脚分配建议
libraries/               # 逐飞开源库 V3.9.2（**请勿修改**）
├── zf_common/           # headfile、typedef、clock、debug、fifo、math
├── zf_driver/           # 底层驱动抽象
├── zf_device/           # 外设驱动（摄像头、IMU、显示屏、BLE、TOF 等）
├── zf_components/       # 助手/调试接口
├── sdk/                 # NXP MCUXpresso SDK 2.12（fsl_* 驱动）
└── components/          # FATFS、SDMMC、USB 协议栈
.opencode/               # OpenCode 配置
├── opencode.json        # 指令引用 AGENTS.md，启用 copilot-embedded/cpp/general
├── lsp.json             # clangd LSP 配置
├── .omo/                # OpenCode 运行时缓存（gitignored）
└── .opencode/           # OpenCode 运行时记忆（gitignored）
.clangd                  # clangd 编译标志（额外 ARMCLANG include 路径）
compile_flags.txt        # clangd 编译标志（完整 include 路径 + 预定义宏）
```

唯一库包含头文件：`#include "zf_common_headfile.h"` — 提供所有逐飞 API（stdio、stdint、string、stdbool、math 及所有 fsl_* 外设驱动）。

---

## 用户代码（project/code/）

| 文件 | 公共函数 | 用途 |
|------|----------|------|
| `control.c/h` | `motor_solution(vx, vy, yaw, wz)` | 麦克纳姆轮运动学：场→车体旋转 + 非对称滤波 + 逆运动学分解 |
| `motor.c/h` | `motor_init()`, `motor_duty(FL,FR,BL,BR)` | PWM 10kHz 初始化 + 四路电机占空比，H桥双极性控制（正负值控制正反转） |
| `pid.c/h` | `PID_init()`, `pid_init()`, `pid_increm()`, `pid_location()`, `pid_target()`, `pid_wheel_target()`, `pid_position_target()`, `pid_position_speed()`, `pid_change()`, `pid_yaw_target()`, `pid_reset()` | 增量式 + 位置式 PID 控制器，7 个全局实例，运行时可调 |
| `path_planning.c/h` | `path_start_calculation()`, `path_calculation()`, `path_look_calculation()`, `id_input()`, `path_id_calculation()`, `path_boom_calculation()`, `map_boom_out()`, `reset_planning_system()` | A\*、BFS、推箱子求解器、炸弹规划、ID 学习。**12×16=192格**，~4447 行 |
| `cam_uart.c/h` | `cam_uart_init()`, `cam_uart_isc_1/2()`, `cam_uart1_read()`, `cam1_uart_send()`, `cam_uart2_read/write()`, `enc_cam()` | 摄像头 UART 协议 — 从 OPENMV 读取栅格 + 车位坐标，`enc_cam` 声明但未实现 |
| `encoder.c/h` | `encoder_init()`, `encoder_get()`, `distance(yaw)` | 正交编码器读取（FL/FR/BL/BR），里程计计算 |
| `imu963.c/h` | `imu_init()`, `imu_get()` | IMU963RA — Mahony 自适应增益 AHRS，四元数 → 欧拉角 |
| `menu.c/h` | `menu()`, `data_init()`（内部） | TFT 菜单系统（5 子菜单：PID/ORIGIN/CORR/control/datasave），Flash 持久化 |
| `tft180.c/h` | `tft_init()` | TFT180 1.8" 显示屏（SPI，竖屏，8×16 字体） |
| `my_uart.c/h` | `my_uart_init()`, `my_uart_write()`, `my_uaer_send()` | 自定义调试串口 — SeekFree 无线示波器协议 |
| `wireless_uart.c/h` | 同 `my_uart.*`（重复文件） | BLE 无线串口（代码与 `my_uart` 完全相同） |
| `key.c/h` | `my_key_init()`, `key_read()` | 按键输入 — 4 按键（C30/C29/C31/C28），3 态去抖状态机 |
| `mpu_config.c` | `SystemInitHook()` | **MPU 配置 + D-Cache 使能** — SDK 启动时自动调用，6 个 MPU Region |

### mpu_config.c — MPU 配置详解

`SystemInitHook()` 在 `main()` 之前被 SDK 弱函数覆盖调用，配置 6 个 MPU Region 并启用 D-Cache：

| Region | 地址 | 大小 | 类型 | 目的 |
|--------|------|------|------|------|
| 0 | 0x80000000 | 32MB | Normal WB/WA | SDRAM 可缓存（`hash_table`、`pq` 等大数据结构） |
| 1 | 0x81E00000 | 2MB | Device nGnRnE | SDRAM 不可缓存（DMA 缓冲区、摄像头帧缓冲） |
| 2 | 0x00000000 | 64KB | Normal | ITCM（`ITCM_NonCacheable` 段 — 时序关键函数） |
| 3 | 0x20000000 | 512KB | Normal | DTCM（`g_dist_map`、`g_bfs_visited`、`g_obs_buf` 等） |
| 4 | 0x20200000 | 512KB | Normal | OCRAM（片内 RAM，备选缓冲区） |
| 5 | 0x70000000 | 4MB | Device RO | FlexSPI 外部 Flash（I-Cache 已覆盖指令读取） |

最后调用 `SCB_EnableDCache()` 启用 D-Cache。

---

## 全局调参变量（menu.h）

所有参数运行时在 TFT 菜单中调节并保存到 Flash（sector 127, page 3）：

| 类别 | 变量 |
|------|------|
| PID 增益 | `pid_x_p/i/d`, `pid_y_p/i/d`, `pid_x/y_speed` |
| 原点标定 | `origin_x_enc`, `origin_y_enc` |
| 修正系数 | `corr_x_enc`, `corr_y_enc`, `corr_x_cam`, `corr_y_cam`, `corr_yaw` |
| 加减速 | `x_acc`, `x_dec`, `y_acc`, `y_dec` |

默认值（`menu.c`）：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `pid_x_p/i/d` | 3.50 / 0.003 / 7.00 | X 方向位置环 PID |
| `pid_x_speed` | 110 | X 方向最大输出（速度限幅） |
| `pid_y_p/i/d` | -3.50 / -0.003 / -7.0 | Y 方向位置环 PID（负值） |
| `pid_y_speed` | 110 | Y 方向最大输出 |
| `origin_x/y_enc` | 1 / 7 | 起点栅格坐标 |
| `corr_x/y_enc` | 0.760 / 0.795 | 编码器里程计修正系数 |
| `corr_x/y_cam` | 0 / 1 | 摄像头位置修正（像素） |
| `corr_yaw` | 0.0 | 偏航角修正 |
| `x/y_acc` | 0.01 | 加速滤波系数 |
| `x/y_dec` | 0.06 | 减速滤波系数 |

---

## 游戏状态机

```
NoGame → GameMap → Run → Waiting → Look → Run → ... → GameOver → End
```

`Car_State` 枚举在 `main.c` 中定义，控制 7 种状态：

```c
typedef enum { NoGame, GameMap, Waiting, Look, Run, GameOver, End } Car_State;
```

**状态转换逻辑（`state_judgment()`）：**

| 当前状态 | 转换条件 | 下一状态 | 操作 |
|----------|----------|----------|------|
| NoGame | 摄像头检测到起始区 | GameMap | 初始化位置 `(x_cam→x_enc)` |
| GameMap | 始终 | Run | 设置 `x/y_target` 到原点 |
| Run | 到达目标点 | Waiting | 停止运动 |
| Waiting | `path_process()` 完成 | Run/Look | 根据 game_mode 调度路径 |
| Look | ID 扫描完成 | Run/Waiting | 调用 OPENMV 读取 ID |
| GameOver | 3 次循环 | End | 停止 |
| GameOver | <3 次 | NoGame | 重置系统重新开始 |

`game_mode` 控制 5 种路径规划模式：

| 模式 | 触发条件 | 行为 |
|------|----------|------|
| 0 (Start) | 首次启动 | `path_start_calculation()` — 导航到最近元素，偏航对准后读取 ID |
| 1 (Normal Run) | ID==10 或 game_count==0 | `path_calculation()` — A\* 推箱子求解，无 ID 信息 |
| 2 (Look) | ID≠10 或 game_mode==4 完成 | `path_look_calculation()` — 访问路径点扫描 ID |
| 3 (ID Run) | Mode 2 学习完成 | `path_id_calculation()` — 利用已学习 ID 精确配对推箱 |
| 4 (Bomb) | 栅格有元素 7 | `path_boom_calculation()` — 炸弹破墙，完成后回 Mode 2 |

主控制循环在 **PIT 定时器 ISR**（`PIT_IRQHandler`，位于 `project/user/src/main.c`）：
- CH0：**5ms** — 编码器读取 → IMU → 里程计 → 位置环 PID → 偏航角 PID → 运动学合成 → 速度环 PID → PWM 输出
- CH1：**10ms** — 摄像头 UART 读取 → 数据更新（低通滤波）

### path_process() 调度逻辑

`path_process()` 在 `Waiting` 状态被调用，执行以下步骤：

1. **地图扫描**：检查栅格中是否有小车(2)、箱子(3)、目标(6)、炸弹(7)，根据 `game_count` 决定初始模式
2. **路径规划**：根据 `game_mode` 分派到对应的 `path_*_calculation()` 函数
3. **路径执行**：根据 `game_mode` 进入不同的执行逻辑：
   - Mode 0：逐点导航到元素，到达后转 Look 状态扫描 ID
   - Mode 1/3：`process_normal_path()` 逐点跟踪，到达终点后触发 GameOver
   - Mode 2：按照 `path_look_car` 路径访问各点，遇到 `is_look` 点转 Look 扫描，完成后自动切换到 Mode 3
   - Mode 4：逐点导航，遇到 `is_push` 点延时 1.5s（等待炸弹爆炸），完成后调用 `map_boom_out()` 更新栅格

---

## 中断路由（易错点）

中断处理全部位于 `project/user/src/isr.c`。完整中断映射：

| 外设 | 引脚 | 用途 | 处理函数 |
|------|------|------|---------|
| CSI | — | 摄像头并行接口 | `CSI_DriverIRQHandler()` |
| LPUART1 | B12/B13 | 调试串口 / ID 通信 | `cam_uart_isc_2()` |
| LPUART2 | — | 未使用（空 handler） | — |
| LPUART3 | B22/B23 | **OPENMV 主摄像头** | `cam_uart_isc_1()` |
| LPUART4 | — | FlexIO 摄像头 + GNSS | `flexio_camera_uart_handler()` + `gnss_uart_callback()` |
| LPUART5 | — | 摄像头（备用） | `camera_uart_handler()` |
| LPUART6 | — | 未使用 | — |
| LPUART8 | D16/D17 | BLE 无线 + 调试中断 | `wireless_module_uart_handler()` + `debug_interrupr_handler()` |
| GPIO1_Combined_0_15 | B0 | GPIO 外部中断 | EXTI 标志清除 |
| GPIO1_Combined_16_31 | B16 | 无线 SPI | `wireless_module_spi_handler()` + EXTI 清除 |
| GPIO2_Combined_0_15 | C0 | FlexIO 摄像头帧同步 | `flexio_camera_vsync_handler()` + EXTI 清除 |
| GPIO2_Combined_16_31 | C16 | ToF 传感器中断 | `tof_module_exti_handler()` + EXTI 清除 |
| GPIO3_Combined_0_15 | D4 | GPIO 外部中断 | EXTI 标志清除 |

**CAM UART 内部定义**（`cam_uart.c`）：
- `UART1` = `UART_3` (LPUART3, B22/B23, 115200) → OPENMV，接收栅格 + 车位数据
- `UART2` = `UART_1` (LPUART1, B12/B13, 115200) → ID 通信/调试

**协议格式**：帧以 `start` 开头、`end` 结尾，FIFO 缓冲，帧同步精确切除。
- 主通道发送角度指令：`0°→01`, `90°→02`, `−90°→03`, `180°→04`，格式 `start%02dend\r\n`
- ID 通道：发送 `start%02dend\r\n`，读取两位十进制数

---

## PID 体系

`pid.c/h` 定义通用 PID 结构体，`pid_init()` 初始化全局实例：

```c
typedef struct {
    float kp, ki, kd;
    float error, error_last, error_last2;
    float integral, maxintegral;
    float output, output_1, maxOutput;
    float target;
    float dead;
} pid;
```

各 PID 用途：

| 实例 | 类型 | 初始化参数（kp/ki/kd/maxOut） | 用途 |
|------|------|-------------------------------|------|
| `pid_FL/FR/BL/BR` | 增量式 `pid_increm` | 60/10/50/8000 | 四轮速度闭环 |
| `pid_x` | 位置式 `pid_location` | `pid_x_p/i/d` / `pid_x_speed` | X 方向位置环 |
| `pid_y` | 位置式 `pid_location` | `pid_y_p/i/d` / `pid_y_speed` | Y 方向位置环 |
| `pid_yaw` | 位置式 `pid_location` | 5/0.001/30/70 | 偏航角闭环（运行时切换 10/0.001/30） |
| `pid_gyro` | 位置式 | — | 角速度环（备用） |

**算法特性**：
- 增量式：`Δu = kp·(e−e_last) + ki·e + kd·(e−2e_last+e_last2)`，输出累积，死区内衰减 0.9
- 位置式：`u = kp·e + ki·∫e + kd·(e−e_last)`，积分限幅，死区内积分衰减 0.95
- 运行时参数切换：`pid_change(&pid_yaw, kp, ki, kd)` 在 Run 状态设为 10/0.001/30，其他状态 5/0.001/30
- 急停模式：`pid_position_speed()` 将 `maxOutput` 设为 130
- 完全重置：`pid_reset()` 清零所有误差/积分/输出状态

**PID 参数运行时调节**：通过 TFT 菜单设置全局变量，非硬编码。每次启动从 Flash 加载。

---

## 运动学

`motor_solution(vx, vy, current_yaw, wz)` 执行：
1. **场坐标系 → 车体坐标系旋转**：`vx_body = vx·cos(yaw) + vy·sin(yaw)` / `vy_body = −vx·sin(yaw) + vy·cos(yaw)`
2. **非对称一阶低通滤波**：加速用 `x_acc/y_acc` 系数，减速用 `x_dec/y_dec` 系数（急刹快、起步柔）
3. **麦克纳姆轮逆运动学分解**（轮距因子硬编码 `0.64f`）：
   - `FL = vx − vy − 0.64·wz`
   - `FR = vx + vy + 0.64·wz`
   - `BL = vx + vy − 0.64·wz`
   - `BR = vx − vy + 0.64·wz`
4. 最终输出四轮目标速度 → `pid_wheel_target(FL, FR, BL, BR)` → 增量式 PID 速度环

---

## 硬件引脚分配

**摄像头**（OPENMV 通过 UART 通信，非 CSI 直连，CSI 接口保留给并行摄像头）：
- 主摄像头 UART：TX=B22 / RX=B23（LPUART3）
- 副摄像头/调试 UART：TX=B12 / RX=B13（LPUART1）
- FlexIO 摄像头：TX=C17 / RX=C16，帧同步 VSY=C7, HREF=C6, PCLK=C5, D0-D7=C8-C15

电机（PWM 10 kHz，`MAX_DUTY=9000`）：

| 电机 | 正转 PWM | 反转 PWM | 编码器 A/B |
|------|----------|----------|------------|
| FL | D13 | D12 | C0/C1 |
| FR | D15 | D14 | C2/C24 |
| BL | D1 | D0 | C3/C25 |
| BR | D3 | D2 | B18/B19 |

调试串口：TX=B12 RX=B13  
TFT：SCK=B0 MOSI=B1 CS=B3（SPI 模式）  
舵机：C30 C31  
BLE：TX=D16 RX=D17  
按键：C30 / C29 / C31 / C28（上拉输入）  
**C4-C15 避免用作输入**（flexio 摄像头限制）。

---

## OPENMV 通信协议

摄像头通过 UART 发送文本协议，`cam_uart.c` 中 `push_cam_data()` 解析：

1. 小车位置：`"car:%.1f,%.1f;"` → `cam_uart_data.car_cx`, `car_cy`
2. 栅格数据：12 行 × 16 列，文本格式 `"map:0123456789abcdef..."` → `uint8_t grid[12][16]`

**栅格元素值**：
- `0` = 空地, `1` = 墙, `2` = 小车, `3` = 箱子, `6` = 目标点, `7` = 炸弹

ID 通信：`cam_uart2_write(type)` 发送类型（格式 `start%02dend\r\n`），`cam_uart2_read()` 读取识别的 ID（两位十进制数）。

角度指令：`cam1_uart_send(angle)` 将角度编码为 1-4 发送给 OPENMV 调整摄像头方向。

---

## 路径规划内存消耗

路径规划在 MCU 上运行，资源紧张。内存通过链接脚本精细分配到 SDRAM/DTCM：

### SDRAM（`SDRAM_CACHE` 段，MPU Region 0，D-Cache 加速）

| 结构 | 最大尺寸 | 说明 |
|------|---------|------|
| A\* 哈希表 | 65536 槽 | djb2 哈希 + 线性探测，epoch 递增避免 memset |
| 优先队列 | 10000 | A\* 推箱子状态搜索 |
| A\* 搜索次数 | 50000 | 超时视为不可解 |

### DTCM（`$DTCM` 段，MPU Region 3，零等待）

| 结构 | 用途 |
|------|------|
| BFS 距离图 | `g_dist_map` — 全图曼哈顿最短距离 |
| BFS 访问标记 | `g_bfs_visited` — epoch 标记避免 memset |
| 障碍物缓冲区 | `g_obs_buf` — 动态障碍物位图 |
| 模仿队列 | `MAX_SIM_QUEUE=16384` — 炸弹规划 BFS |

### ITCM（`ITCM_NonCacheable` 段，MPU Region 2）

时序关键函数放置在 ITCM 以 1-cycle 访问：
`manhattan_distance`, `is_corner_deadlock`, `is_soft_corner_deadlock`, `hash_lookup_insert`, `pq_push`, `pq_pop`, `heuristic`, `get_successors`, `bfs_compute_distances`, `bfs_compute_reachability`, `simple_astar`, `solve_single_box_a_star`, `compute_player_region_with_walls`

### 路径缓冲区

| 结构 | 最大尺寸 |
|------|---------|
| 单步 A\* 路径 | 500 点 |
| 完整路径 | 500 点 |
| BFS 模拟队列 | 16384 |

堆大小定义在链接脚本中（`project/mdk/scf/MIMXRT1064xxxxx_flexspi_nor.scf`、`project/iar/icf/MIMXRT1064xxxxx_flexspi_nor.icf`）。

---

## 内存模型

| 区域 | MPU Region | 用途 |
|------|-----------|------|
| XIP Flash (0x70000000) | Region 5 (Device RO) | 代码执行（I-Cache 覆盖） |
| ITCM (0x00000000) | Region 2 (Normal) | 时序关键函数（1-cycle） |
| DTCM (0x20000000) | Region 3 (Normal) | BFS 距离图、访问标记、障碍物缓冲区 |
| OCRAM (0x20200000) | Region 4 (Normal) | 备选缓冲区 |
| SDRAM 可缓存 (0x80000000) | Region 0 (Normal WB/WA) | 哈希表、优先队列（32MB，D-Cache 加速） |
| SDRAM 不可缓存 (0x81E00000) | Region 1 (Device) | DMA 缓冲区、帧缓冲（2MB） |
| 构建目标 `nor_sdram_zf_dtcm` | — | 代码 XIP、大缓冲区 SDRAM、时序数据 DTCM |

---

## OpenCode 使用

- `project/.opencode/opencode.json` 引用本 `AGENTS.md` 作为指令，启用 `copilot-embedded`、`copilot-cpp`、`copilot-general` 技能
- `project/.opencode/lsp.json` 配置 clangd LSP
- `.gitignore` 排除了 `.opencode/.omo/`、`.opencode/.opencode/memory/`、`.vscode/`
- 最相关的用户技能：`copilot-embedded`、`copilot-cpp`、`copilot-general`、`embedded-engineer`
- 所有代码为 C99，注意 ARMCC 编译器扩展
- 始终尊重 `libraries/` 边界 — 逐飞库 + NXP SDK 代码无需修改
- 调试手段有限：TFT 显示 + 串口输出（SeekFree 无线示波器模式），无硬件调试器时依赖这两者
- `wireless_uart.c/h` 与 `my_uart.c/h` 内容重复

## 已知问题

- `cam_uart.h` 中声明了 `enc_cam()` 函数但尚无实现
- `isr.h` 为空占位头文件
- `待解决问题.md` 未找到（已从引用中移除）
