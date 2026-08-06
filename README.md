# ysu-znc-21-smarteyes

> 🏎️ **燕山大学 · 第21届全国大学生智能汽车竞赛 — 智能视觉组**

[![Platform](https://img.shields.io/badge/Platform-NXP%20RT1064-blue)](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/i-mx-rt-crossover-mcus/i-mx-rt1064-crossover-mcu-with-arm-cortex-m7-core:i.MX-RT1064)
[![IDE](https://img.shields.io/badge/IDE-Keil%205.33-green)](#开发环境)
[![Language](https://img.shields.io/badge/Language-C99-orange)](https://en.wikipedia.org/wiki/C_(programming_language))
[![RAM](https://img.shields.io/badge/DTCM-448KB%20%7C%20SDRAM-32MB-lightgrey)]()

---

## 📖 项目简介

本项目是燕山大学参加 **第21届全国大学生智能汽车竞赛（智能视觉组）** 的参赛代码，基于 **NXP RT1064 (MIMXRT1064DVL6A)** 微控制器开发，运行于逐飞科技 RT1064 核心板。

智能视觉组要求小车具备 **自主视觉识别 + 运动控制 + 逻辑推理** 能力。OPENMV 摄像头通过 UART 将栅格地图和车位坐标发送给 MCU，MCU 负责路径规划、推箱子求解、炸弹破墙规划、PID 运动控制和状态机调度。**所有推理任务均在 Cortex-M7（600MHz）裸机上完成**，无上位机、无操作系统。

### 核心能力

| 能力 | 实现模块 | 说明 |
|------|---------|------|
| 栅格建图 | `cam_uart.c` + `main.c` | OPENMV 摄像头文本协议，解析 12×16 栅格 |
| 姿态估计 | `imu963.c` | Mahony 自适应增益 AHRS，偏航角闭环 |
| 里程计 | `encoder.c` | 麦克纳姆轮正运动学 + 体→世界坐标系旋转 |
| 位置/速度控制 | `pid.c` + `control.c` | 串级 PID（位置环→偏航环→速度环） |
| 路径规划 | `path_planning.c` | A\*、BFS、推箱子求解、炸弹规划、ID 学习推理 |
| 人机交互 | `menu.c` + `key.c` | TFT 菜单调参、4 按键导航、Flash 持久化 |
| 调试 | `wireless_uart.c` | 逐飞无线示波器（8 通道，SeekFree Assistant） |

---

## 🛠️ 硬件平台

| 类别 | 型号 |
|------|------|
| **MCU** | NXP MIMXRT1064DVL6A (Cortex-M7 @ 600MHz, 4MB 内部 FlexSPI Flash) |
| **核心板** | 逐飞科技 RT1064 核心板 |
| **摄像头** | OPENMV5-RT（主摄像头，UART 通信，LPUART3） |
| **显示器** | TFT180 (1.8", 128×160, SPI) |
| **IMU** | IMU963RA（三轴加速度计 + 三轴陀螺仪） |
| **无线通信** | BLE6A20 蓝牙（UART 透传，LPUART8） |
| **电机** | 直流减速电机 × 4（麦克纳姆轮，正交编码器 × 4） |
| **舵机** | 2 路（C30/C31，预留） |

> 注：板载 CSI 摄像头接口保留但未使用，主摄像头通过 UART（OPENMV）通信。

---

## 🧩 软件架构概览

### 层次结构

```
┌─────────────────────────────────────────────────────────────┐
│                    main.c 状态机调度                          │
│  NoGame → GameMap → Run → Waiting → Look → Run → GameOver   │
├─────────────────────────────────────────────────────────────┤
│               PIT_IRQHandler (5ms / 10ms)                   │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐  │
│  │ encoder  │ │  imu963  │ │   pid    │ │ motor_solution │  │
│  │ 读取滤波 │ │ Mahony   │ │ 位置/偏航 │ │ 运动学合成     │  │
│  └──────────┘ └──────────┘ └──────────┘ └───────────────┘  │
├─────────────────────────────────────────────────────────────┤
│                    path_planning.c (Waiting 状态)            │
│   A* 寻路 ─ BFS 距离场 ─ 推箱子求解 ─ 炸弹规划 ─ ID 学习    │
├─────────────────────────────────────────────────────────────┤
│                    驱动层 (逐飞库 + SDK)                     │
│  zf_driver ─ zf_device ─ zf_common ─ fsl_* (NXP SDK)      │
└─────────────────────────────────────────────────────────────┘
```

### 初始化序列

```
main()
  ├─ clock_init(600MHz)
  ├─ debug_init()
  ├─ system_delay_ms(300)
  └─ Init()
      ├─ tft_init() → tft180_clear() → my_key_init()
      ├─ flash_init() → menu()                ← TFT 菜单等待用户按"start"
      ├─ system_delay_ms(1000)
      ├─ encoder_init()                       ← 4 路正交编码器
      ├─ imu_init()                           ← IMU963RA + 零偏校准 (400 样本)
      ├─ motor_init()                         ← 8 路 PWM (10kHz)
      ├─ pid_init()                           ← 8 个 PID 实例
      ├─ cam_uart_init()                      ← 双 UART (LPUART1/LPUART3)
      ├─ cam1_uart_send(0)                    ← OPENMV 角度回零
      ├─ reset_planning_system()
      ├─ pit_ms_init(PIT_CH0, 5ms)
      └─ pit_ms_init(PIT_CH1, 10ms)
  └─ interrupt_global_enable()
  └─ while(1) { state_judgment(); }
```

---

## 📁 代码文件详解

### 主控与中断（`project/user/src/`）

| 文件 | 行数 | 职责 |
|------|------|------|
| `main.c` | 489 | 入口 → `Init()` → `state_judgment()` 状态机 + `PIT_IRQHandler` 控制循环 |
| `isr.c` | 320 | 13 个外设中断处理函数（UART、CSI、GPIO、FlexIO、ToF） |
| `isr.h` | 0 | **空占位**（当前无内容） |

**main.c 核心结构：**

```c
typedef enum { NoGame, GameMap, Waiting, Look, Run, GameOver, End } Car_State;
```

- `state_judgment()` — 状态转换主逻辑（switch-case，每轮 while 循环执行）
- `path_process()` — 路径规划调度核心（在 Waiting 状态被调用）
- `process_normal_path()` — 模式 1/3 的路径跟踪逻辑
- `uart_updata()` — 摄像头数据更新（低通滤波）
- `cam2_process()` — Look 状态下的 ID 扫描逻辑
- `PIT_IRQHandler()` — **所有实时控制在此完成**：

```
PIT_CH0 (5ms):
  1. encoder_get()          ← 编码器读取 + 低通滤波
  2. imu_get()              ← IMU 读取 + Mahony 更新 → yaw
  3. distance(yaw)          ← 里程计积分 → x_enc/y_enc
  4. [每 10ms] 位置环 PID   ← pid_x 和 pid_y 计算 vx/vy
  5. [每 10ms] 偏航环 PID   ← pid_yaw 计算 vz
  6. motor_solution(vx,vy,yaw,vz)  ← 运动学合成
  7. pid_increm × 4         ← 四轮速度环 PID
  8. motor_duty(FL,FR,BL,BR) ← PWM 输出

PIT_CH1 (10ms):
  1. cam_uart1_read()       ← 读取 OPENMV 数据帧
  2. uart_updata()          ← 数据低通滤波 (α=0.8)
```

### 用户应用代码（`project/code/`）

#### 1. `control.c/h` — 运动学合成（44 行）

| 函数 | 功能 |
|------|------|
| `motor_solution(vx, vy, yaw, wz)` | 场→体坐标旋转 → 非对称滤波 → 麦克纳姆逆运动学 |

**核心算法：**
```
1. 场→体：vx_body =  vx·cos(yaw) + vy·sin(yaw)
           vy_body = -vx·sin(yaw) + vy·cos(yaw)
2. 非对称滤波：加速=[x_acc=0.01,y_acc=0.01], 减速=[x_dec=0.06,y_dec=0.06]
3. 逆运动学：FL = vx - vy - 0.64·wz  (FR/BL/BR 同理)
4. ⇒ pid_wheel_target(FL, FR, BL, BR)
```

#### 2. `motor.c/h` — 电机 PWM 驱动（97 行）

| 函数 | 功能 |
|------|------|
| `motor_init()` | 8 路 PWM 初始化（10kHz，初始占空比 0） |
| `motor_duty(FL, FR, BL, BR)` | H 桥双极性控制，占空比限幅 [-8000, 8000] |

**引脚映射：**

| 电机 | PWM1 | PWM2 | 编码器 A/B |
|------|------|------|-----------|
| FL | PWM1_CH0_B (D13) | PWM1_CH0_A (D12) | QTIMER1_CH1 (C0) / CH2 (C1) |
| FR | PWM1_CH1_B (D15) | PWM1_CH1_A (D14) | QTIMER1_CH1 (C2) / CH2 (C24) |
| BL | PWM1_CH3_B (D1) | PWM1_CH3_A (D0) | QTIMER2_CH1 (C3) / CH2 (C25) |
| BR | PWM2_CH3_B (D3) | PWM2_CH3_A (D2) | QTIMER3_CH1 (B18) / CH2 (B19) |

#### 3. `pid.c/h` — PID 控制器（154 行）

| 实例 | 类型 | 初始化参数 | 作用 |
|------|------|-----------|------|
| `pid_FL/FR/BL/BR` | 增量式 | kp=60, ki=10, kd=50, maxOut=8000 | 四轮速度内环 |
| `pid_x` | 位置式 | 菜单可调 (默认 3.50/0.003/7.00/110) | X 方向位置外环 |
| `pid_y` | 位置式 | 菜单可调 (默认 -3.50/-0.003/-7.0/110) | Y 方向位置外环 |
| `pid_yaw` | 位置式 | 5/0.001/30/70 (Run 态 10/0.001/30) | 偏航角闭环 |
| `pid_gyro` | 位置式 | **未初始化** | 角速度环（备⽤） |

**算法：**
- **增量式**：`Δu = kp·(e−e_last) + ki·e + kd·(e−2e_last+e_last2)`，累积输出，死区衰减 0.9
- **位置式**：`u = kp·e + ki·∫e + kd·(e−e_last)`，积分限幅，死区衰减 0.95
- **运行时切换**：`pid_yaw` 在 Run 态 kp=10（快速响应），其他态 kp=5（平稳）
- **终点冲刺**：`pid_position_speed()`（GameOver 冲线时调用）将 maxOutput 从 110 提升至 130
- **完全重置**：`pid_reset()` 清零所有累计状态

#### 4. `encoder.c/h` — 编码器与里程计（151 行）

| 函数 | 功能 |
|------|------|
| `encoder_init()` | 初始化 4 路正交编码器（QTIMER1/2/3） |
| `encoder_get()` | 读取原始值 → 一阶低通滤波 (α=0.9) → 清计数值 |
| `distance(yaw)` | 麦克纳姆正运动学 → 体→世界坐标旋转 → 累积 x_enc/y_enc |

**机械参数：**
- 编码器每圈脉冲 (CPR×4)：16384（4 倍频）
- 减速比：30/70（编码器齿轮/轮齿轮）
- 车轮半径：3.0 cm → 周长 C = 2π×3.0 ≈ 18.85 cm
- 转换系数 `dc = C / (B × N)`，其中 B = 30/70

**里程计算法：**
```c
// 体坐标系位移（麦克纳姆正运动学）
dx = (FL + FR + BL + BR) / 4;
dy = (FL - FR - BL + BR) / 4;

// 体→世界坐标旋转
world_x = dx·cos(yaw) + dy·sin(yaw);
world_y = -dx·sin(yaw) + dy·cos(yaw);

// 累积世界坐标（带修正系数 corr_x/y_enc）
x_enc += world_x × corr_x_enc;
y_enc += world_y × corr_y_enc;
```

#### 5. `imu963.c/h` — IMU963RA 姿态估计（212 行）

| 函数 | 功能 |
|------|------|
| `imu_init()` | 400 样本零偏校准 → 初始化 IMU |
| `imu_get()` | 读取 → 低通滤波 → Mahony 更新 → 输出 yaw（每 5ms 调用） |

**Mahony 自适应增益算法：**
- **自适应 Kp**：加速度模值偏离 1g 程度决定对加速度计的信任
  - `|acc_norm - 9.8| < 1.0` → Kp = 0.5（静止信任）
  - `|acc_norm - 9.8| > 3.0` → Kp = 0.2（机动不信任）
  - 中间值线性插值
- **积分限幅**：Ki = 0.002，积分值钳位 ±1.0
- **低通滤波**：加速度计 α=0.06（~1.9Hz），陀螺仪 α=0.25（~8.8Hz）
- **死区**：`0.005 rad/s` 以下置零
- **偏航角修正**：运行时 `corr_yaw` 补偿（在 gyro_z 上累加）

**关键输出：** `yaw`（全局变量，范围 -180°~180°）

#### 6. `cam_uart.c/h` — OPENMV 双 UART 通信（242 行）

**硬件通道：**

| 通道 | UART | 引脚 | 方向 | 用途 |
|------|------|------|------|------|
| UART1 | LPUART3 | TX=B22, RX=B23 | 双向 | 主摄像头：栅格 + 车位，角度指令 |
| UART2 | LPUART1 | TX=B12, RX=B13 | 双向 | ID 通信 / 调试 |

**协议格式：** `start<data>end\r\n`，FIFO 缓冲 + 帧同步精确切除

| 函数 | 功能 |
|------|------|
| `cam_uart_init()` | 双 UART 初始化（115200 baud，FIFO 缓冲 512 字节） |
| `cam_uart_isc_1()` | UART1 ISR：逐字节写入 FIFO |
| `cam_uart_isc_2()` | UART2 ISR：逐字节写入 FIFO |
| `cam_uart1_read()` | 从 FIFO 提取完整帧 → `push_cam_data()` 解析 |
| `cam1_uart_send(angle)` | 发送角度指令：0°→01, 90°→02, -90°→03, 180°→04 |
| `cam_uart2_read()` | 从 FIFO 提取 ID 帧，返回两位十进制数 |
| `cam_uart2_write(id)` | 发送待识别元素类型 |

**帧解析 `push_cam_data()`：**
- `"car:%.1f,%.1f;"` → `cam_uart_data.car_cx/car_cy`
- `"map:0123456789abcdef..."` → `uint8_t grid[12][16]`

#### 7. `path_planning.c/h` — 路径规划核心（~4447 行）

这是项目最复杂的模块，详见下文第五部分。

#### 8. `menu.c/h` — TFT 菜单系统（505 行）

5 个子菜单，4 按键导航（C30=左, C29=右, C31=下, C28=上），3 态去抖。

| 菜单 | 可调参数 |
|------|---------|
| **PID** | `pid_x_p/i/d`, `pid_x_speed`, `pid_y_p/i/d`, `pid_y_speed` |
| **ORIGIN** | `origin_x_enc`, `origin_y_enc` |
| **CORR** | `corr_x_enc`, `corr_y_enc`, `corr_x_cam`, `corr_y_cam`, `corr_yaw` |
| **control** | `x_acc`, `x_dec`, `y_acc`, `y_dec` |
| **datasave** | 全部参数保存到 Flash (sector 127, page 3) |

Flash 存储布局（`flash_union_buffer` 索引）：

| 索引 | 参数 | 类型 |
|------|------|------|
| 0-2 | `pid_x_p/i/d` | float |
| 3 | `pid_x_speed` | uint8_t |
| 4-6 | `pid_y_p/i/d` | float |
| 7 | `pid_y_speed` | uint8_t |
| 8-9 | `corr_x/y_enc` | float |
| 10-11 | `corr_x/y_cam` | int8_t |
| 12 | `corr_yaw` | float |
| 13-14 | `origin_x/y_enc` | uint8_t |
| 15-18 | `x_acc/x_dec/y_acc/y_dec` | float |

#### 9. `tft180.c/h` — TFT 显示屏（13 行）

竖屏 (PORTRAIT)，8×16 字体，SPI 模式，RGB565 白底黑字。

#### 10. `key.c/h` — 按键驱动（45 行）

3 态去抖状态机：`IDLE → PRESSED → DONE`，4 键上拉输入。
`key[0]=C30(左)`, `key[1]=C29(右)`, `key[2]=C31(下)`, `key[3]=C28(上)`。

**注意：** 菜单中按键映射与物理引脚命名有差异（`KEY_UP=key[2]`=C31=下按键，`KEY_DOWN=key[3]`=C28=上按键）。

#### 11. `wireless_uart.c/h` — 无线示波器调试（31 行）

基于逐飞 SeekFree 无线示波器协议，8 通道数据发送。命名与 `my_uart` 名义重复（头文件保护宏为 `_CODE_MY_UART_h_`）。

```c
my_uart_write(channel, data);  // 写入通道 0-7
my_uart_send(count);           // 发送 count 通道数据
```

#### 12. `mpu_config.c` — MPU 与 Cache 配置（122 行）

详见下文第六部分。

---

## 🧠 路径规划与推箱子求解器（核心算法）

`path_planning.c` 是项目最大模块（~4447 行），在 MCU 裸机（600MHz Cortex-M7）上完成所有推理。

### 5.1 栅格地图

12 行 × 16 列 = 192 格，由 OPENMV 摄像头通过 UART 实时发送。

| 元素 | 值 | 说明 |
|------|----|------|
| 空地 | 0 | 可通行 |
| 墙 | 1 | 不可通行 |
| 小车 | 2 | 玩家位置 |
| 箱子 | 3 | 可推动 |
| 目标点 | 6 | 箱子最终归属 |
| 炸弹 | 7 | 可破坏墙体 |

### 5.2 基础算法

- **`simple_astar()`**：加权 A\*（`f = g + 1.2·h`），曼哈顿距离，转向惩罚 `TURN_WEIGHT=4`
  - 最小堆优化，5000 节点空间
- **`bfs_compute_distances()`**：全图曼哈顿最短距离场（epoch 标记，避免 memset）
- **`bfs_compute_reachability()`**：可达性判断（epoch 标记）
- **障碍物位图**：每行 `uint16_t` 压缩表示 16 列，O(1) 查询

### 5.3 推箱子（Sokoban）求解器

针对最多 5 个箱子的推箱子问题：

1. **`generate_greedy_pairing()`**：贪心为每个箱子分配最近未使用目标
2. **`backtrack_validate()`**：递归回溯验证，关键优化：
   - 距离排序优先尝试最近配对
   - **软死锁检测** `is_soft_corner_deadlock()`：区分硬墙阻塞 vs 另一箱子阻塞
   - **破锁机制**：阻塞箱子推开最多 10 步
   - **ID 感知阻塞**：同 ID 的箱子和目标互不为障碍
3. **`solve_single_box_a_star()`**：完整 A\* 推箱子状态搜索
   - 状态空间：`(px, py, bx, by, wall_bitmap, last_dir)`，16-bit 编码
   - 哈希去重：djb2 + 线性探测，65536 槽，epoch 递增
   - 优先队列：10000 容量
   - 搜索上限：50000 步

### 5.4 炸弹墙规划（Bomb Mode）

当栅格中出现炸弹（值 7）时激活，这是最复杂的子系统：

1. **可炸墙检测**：`is_breakable_wall_on()` — 墙至少有一邻接空地且不在地图边界
2. **问题区域分析**：检测隔离区（ENC）、目标不可达、拥堵三类问题
3. **引爆炸点评分**：推距 + 玩家到推站位的 BFS 距离
4. **多炸弹组合搜索**：`search_multi_bomb_combination()` — 递归枚举，增量收益
5. **执行序列生成**：`plan_bomb_execution_sequence()` — 按可行性排序
6. **验证缓存**：`hash_walls()` — 墙面配置哈希避免重复验证

### 5.5 ID 学习与推理

```
Mode 2 (Look) → 逐个访问箱子和目标 → OPENMV 扫描 ID → id_record()
    ↓
id_inference() → 频率均衡 + 排列枚举 → 建立箱↔目标映射
    ↓
Mode 3 (ID Run) → 利用 ID 精准配对推箱
```

- **`id_learning()`**：BFS 规划访问路径，预留最后一个锚点
- **`id_record()`**：到达后调用 OPENMV 扫描 ID
- **`id_inference()`**：频率均衡 + `next_permutation()` 排列枚举
- **`build_solution_from_id_pairing()`**：ID 映射 → 求解方案

### 5.6 状态机与游戏模式

```
NoGame → GameMap → Run → Waiting → Look → Run → ... → GameOver → End
```

| 模式 | 触发 | 规划函数 | 行为 |
|------|------|---------|------|
| 0 (Start) | 首次启动 | `path_start_calculation()` | 导航到最近元素 → 偏航对准 → 扫描 ID |
| 1 (Normal) | ID==10 或首次 | `path_calculation()` | A\* 盲推，无 ID 信息 |
| 2 (Look) | ID≠10 或炸弹完成 | `path_look_calculation()` | BFS 访问所有箱/目标，扫描记录 ID |
| 3 (ID Run) | Look 学习完成 | `path_id_calculation()` | 利用 ID 精准配对推箱 |
| 4 (Bomb) | 发现炸弹 | `path_boom_calculation()` | 炸弹破墙 → 更新栅格 → 回 Mode 2 |

---

## 💾 内存模型与 MPU 配置

### 物理内存映射

| 区域 | 地址范围 | 容量 | 速度 | 用途 |
|------|---------|------|------|------|
| ITCM | 0x00000000–0x0000FFFF | 64KB | 1-cycle | 时序关键函数（路径规划核心） |
| DTCM | 0x20000000–0x2007FFFF | 512KB | 0-wait | BFS 距离图、访问标记、障碍物缓冲区 |
| OCRAM | 0x20200000–0x2027FFFF | 512KB | 2-3 cycle | 备选缓冲区 |
| FlexSPI Flash | 0x70000000–0x703FFFFF | 4MB | XIP | 代码存储+指令执行 |
| SDRAM Cacheable | 0x80000000–0x81DFFFFF | ~30MB | D-Cache | 哈希表、优先队列 |
| SDRAM NonCacheable | 0x81E00000–0x81FFFFFF | 2MB | Device | DMA 缓冲区、帧缓冲 |

### 链接脚本分配（Keil .scf）

```c
Stack_Size = 0x8000  (32KB, DTCM 顶部)
Heap_Size  = 0x0400  (1KB, DTCM 中)

// 自定义段
SDRAM_ZI (UNINIT):  .bss.SDRAM_CACHE   ← hash_table[65536]
ITCM_ncache:        ITCM_NonCacheable   ← 时序函数
OCRAM_cache:        OCRAM_CACHE         ← 备选缓冲区
SDRAM_ncache:       SDRAM_NonCacheable  ← DMA 缓冲区
```

### MPU 6 区域配置

| Region | 地址 | 大小 | 类型 | 目的 |
|--------|------|------|------|------|
| 0 | 0x80000000 | 32MB | Normal WB/WA | SDRAM 可缓存 |
| 1 | 0x81E00000 | 2MB | Device nGnRnE | SDRAM 不可缓存 |
| 2 | 0x00000000 | 64KB | Normal | ITCM |
| 3 | 0x20000000 | 512KB | Normal | DTCM |
| 4 | 0x20200000 | 512KB | Normal | OCRAM |
| 5 | 0x70000000 | 4MB | Device RO | FlexSPI Flash |

### ITCM 中放置的时序关键函数

```c
manhattan_distance, is_corner_deadlock, is_soft_corner_deadlock,
hash_lookup_insert, pq_push, pq_pop, heuristic, get_successors,
bfs_compute_distances, bfs_compute_reachability, simple_astar,
solve_single_box_a_star, compute_player_region_with_walls
```

### D-Cache 注意事项

- Region 0 SDRAM 启用了 D-Cache（Write-back/Write-allocate），加速 A\* 哈希表和优先队列访问
- Region 1 的 DMA 缓冲区设为 Device nGnRnE，避免 cache coherence 问题
- 时序关键函数放在 ITCM（1-cycle，绕过 Cache）

---

## 🔌 中断路由

全部中断处理位于 `project/user/src/isr.c`。

| 外设 | 引脚 | 用途 | ISR |
|------|------|------|-----|
| PIT_CH0 | — | **5ms 控制循环** | `PIT_IRQHandler` (main.c) |
| PIT_CH1 | — | **10ms 数据读取** | `PIT_IRQHandler` (main.c) |
| CSI | — | 摄像头并行接口 | `CSI_DriverIRQHandler()` |
| LPUART1 | B12/B13 | 调试串口 / ID 通信 | `cam_uart_isc_2()` |
| LPUART2 | — | 未使用（空 handler） | — |
| LPUART3 | B22/B23 | **OPENMV 主摄像头** | `cam_uart_isc_1()` |
| LPUART4 | — | FlexIO 摄像头 + GNSS | `flexio_camera_uart_handler()`, `gnss_uart_callback()` |
| LPUART5 | — | 备用摄像头 | `camera_uart_handler()` |
| LPUART6 | — | 未使用 | — |
| LPUART8 | D16/D17 | BLE 无线 + 调试中断 | `wireless_module_uart_handler()`, `debug_interrupr_handler()` |
| GPIO1_0_15 | B0 | GPIO 外部中断 | EXTI flag clear |
| GPIO1_16_31 | B16 | 无线 SPI | `wireless_module_spi_handler()` + EXTI clear |
| GPIO2_0_15 | C0 | FlexIO 帧同步 | `flexio_camera_vsync_handler()` + EXTI clear |
| GPIO2_16_31 | C16 | **ToF 传感器** | `tof_module_exti_handler()` + EXTI clear |
| GPIO3_0_15 | D4 | GPIO 外部中断 | EXTI flag clear |

---

## 🔧 开发环境

| 工具 | 版本 |
|------|------|
| Keil MDK (µVision) | 5.33 |
| 编译器 | ARMCLANG (Keil) |
| C 标准 | **C99** |
| 构建目标 | `nor_sdram_zf_dtcm` |
| 关键预定义宏 | `CPU_MIMXRT1064DVL6A`, `XIP_EXTERNAL_FLASH=1`, `USB_STACK_BM`, `MCUXPRESSO_SDK`, `SKIP_SYSCLK_INIT`, `PRINTF_FLOAT_ENABLE=1` |
| clangd 附加宏 | `__GNUC__`, `-U_WIN32`, `-nostdinc` |

> ⚠️ **仅支持 IDE 构建**（无命令行构建方式）。添加 `.c`/`.h` 文件时需同时放入 `project/code/` **并在 IDE 工程树中添加**。

**编译 / 烧录：**
- Keil: 打开 `project/mdk/rt1064.uvprojx` → 选择 `nor_sdram_zf_dtcm` → 编译 → DAP-Link/J-Link 烧录

---

## 📂 完整项目结构

```
ysu-znc-21-smarteyes/
├── Claude.md                        # Claude Code 指令文档（原 AGENTS.md 重命名）
├── README.md
├── .clangd                          # clangd ARMCLANG include 路径
├── compile_flags.txt                # clangd 编译标志（57 个 include 路径）
├── .gitignore
├── libraries/                       # 逐飞开源库 V3.9.2（请勿修改）
│   ├── zf_common/                   # 通用头文件、typedef、时钟、FIFO、数学
│   ├── zf_driver/                   # 底层驱动抽象层
│   ├── zf_device/                   # 外设驱动（摄像头、IMU、显示屏、BLE、TOF 等）
│   ├── zf_components/               # 助手/调试接口
│   ├── sdk/                         # NXP MCUXpresso SDK 2.12（fsl_* 驱动）
│   ├── components/                  # FATFS、SDMMC、USB 协议栈
│   └── doc/                         # 文档、版本记录、许可证
└── project/
    ├── RT1064核心板丝印与芯片引脚对应表格.xlsx
    ├── RT1064智能车推荐引脚分配.txt
    ├── code/                        # ★ 用户代码目录（平铺，无子目录）
    │   ├── path_planning.c/h        # A*/BFS/推箱子/炸弹/ID 学习（~4447 行）
    │   ├── control.c/h              # 麦克纳姆轮运动学合成
    │   ├── pid.c/h                  # PID 控制器（增量式+位置式，8 实例）
    │   ├── motor.c/h                # PWM 电机驱动（10kHz，H 桥双极性）
    │   ├── encoder.c/h              # 正交编码器 + 里程计
    │   ├── imu963.c/h               # IMU963RA + Mahony AHRS
    │   ├── cam_uart.c/h             # OPENMV 双 UART 通信协议
    │   ├── menu.c/h                 # TFT 菜单 + Flash 持久化
    │   ├── tft180.c/h               # TFT180 显示屏驱动
    │   ├── wireless_uart.c/h        # 无线示波器调试（与 my_uart 名义重复）
    │   ├── key.c/h                  # 4 按键 3 态去抖
    │   ├── mpu_config.c             # MPU 6 Region + D-Cache 使能
    │   └── 本文件夹作用.txt          # 文件放置说明
    ├── user/src/
    │   ├── main.c                   # 入口、状态机、PIT 中断（489 行）
    │   └── isr.c                    # 外设中断处理（320 行）
    │   inc/isr.h                    # 空占位文件
    └── mdk/                         # Keil MDK 工程（uvprojx/uvoptx/scf）
```

---

## 📄 开源协议

本项目基于逐飞科技 RT1064 开源库开发，底层库遵循 **GNU General Public License v3.0**。

---

## 🙏 致谢

- [逐飞科技 (SeekFree)](https://seekfree.com.cn/) — RT1064 开源库与核心板
- [NXP Semiconductors](https://www.nxp.com/) — i.MX RT1064 平台
- 燕山大学智能车团队全体成员

---

> 🏁 *Keep Racing, Keep Dreaming!*
