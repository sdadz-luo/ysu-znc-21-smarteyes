# ysu-znc-21-smarteyes

> 🏎️ **燕山大学 · 第21届全国大学生智能汽车竞赛 — 智能视觉组**

[![Platform](https://img.shields.io/badge/Platform-NXP%20RT1064-blue)](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/i-mx-rt-crossover-mcus/i-mx-rt1064-crossover-mcu-with-arm-cortex-m7-core:i.MX-RT1064)
[![IDE](https://img.shields.io/badge/IDE-IAR%208.32%20%7C%20Keil%205.33-green)](#开发环境)
[![Language](https://img.shields.io/badge/Language-C-orange)](https://en.wikipedia.org/wiki/C_(programming_language))
[![License](https://img.shields.io/badge/License-GPLv3-lightgrey)](./libraries/doc/GPL3_permission_statement.txt)

---

## 📖 项目简介

本项目是燕山大学参加**第21届全国大学生智能汽车竞赛（智能视觉组）**的参赛代码，基于 **NXP RT1064 (MIMXRT1064DVL6A)** 微控制器开发，运行于逐飞科技 RT1064 核心板。

智能视觉组要求小车具备**自主视觉识别 + 运动控制 + 逻辑推理**能力。OPENMV 摄像头通过 UART 将栅格地图和车位坐标发送给 MCU，MCU 负责路径规划、推箱子求解、弹道规划、PID 运动控制和状态机调度，**所有推理任务均在 Cortex-M7（600MHz）裸机上完成**。

---

## 🛠️ 硬件平台

| 类别 | 型号 |
|------|------|
| **MCU** | NXP MIMXRT1064DVL6A (Cortex-M7, 600MHz) |
| **核心板** | 逐飞科技 RT1064 核心板 |
| **摄像头** | OPENMV5-RT |
| **显示器** | TFT180 (1.8") |
| **IMU** | IMU963RA (三轴加速度计 + 三轴陀螺仪) |
| **无线通信** | BLE6A20 蓝牙 |
| **其他** | 正交编码器 ×4、直流减速电机 ×4（麦克纳姆轮） |

---

## 🧠 算法体系

### 1. 姿态估计 — 自适应增益 Mahony 互补滤波

`imu963.c` 实现基于四元数的 Mahony AHRS 算法：

- 陀螺仪零偏校准：上电采集 400 样本取均值
- 自适应增益 Kp：根据加速度模值偏离 1g 的程度，在 `0.2~0.5` 之间线性调整，机动时降低对加速度计的信任，抑制非重力加速度干扰
- 一阶低通滤波：加速度计截止 ~1.9Hz，陀螺仪 ~8.8Hz，各自独立系数
- 陀螺仪死区：`0.005 rad/s` 以下置零，抑制零漂抖动
- 积分限幅抗饱和：积分项绝对值钳位在 1.0
- 输出欧拉角（roll/pitch/yaw），yaw 范围 `-180°~180°`

### 2. 里程计 — 麦克纳姆轮正运动学

`encoder.c` 实现四轮正交编码器读数 → 场坐标系位移：

- 四路编码器独立正交解码（QTIMER 模块）
- 一阶低通滤波 (`alpha=0.8`) 抑制测量噪声，float 全精度累积避免整数截断
- 体坐标系位移 → 通过偏航角旋转 → 场坐标系位移
- 可调修正系数 `corr_x_enc` / `corr_y_enc` 补偿打滑和标定误差
- 每 5ms PIT 中断执行（`dc/dt` 预计算好）

### 3. PID 控制 — 串级双环

`pid.c` 实现增量式和位置式两种 PID，构成**位置环 + 速度环**串级结构：

| PID 实例 | 类型 | 作用 |
|----------|------|------|
| `pid_FL/FR/BL/BR` | 增量式 | 四轮速度内环 |
| `pid_x` / `pid_y` | 位置式 | X/Y 方向位置外环 |
| `pid_yaw` | 位置式 | 偏航角闭环 |
| `pid_gyro` | — | 角速度环（备用） |

- 积分分离 + 死区：误差在死区内积分衰减（`×0.9`），防止频繁震荡
- 输出限幅 + 积分抗饱和：`maxIntegral` / `maxOutput` 硬限制
- 所有 PID 参数（kp/ki/kd/限速）可在 TFT 菜单**运行时实时调节**并保存到 Flash

### 4. 运动学合成 — 车场分离 + 非对称滤波

`control.c` 中的 `motor_solution(vx_field, vy_field, yaw, wz)` 完成：

1. **场→车坐标系旋转**：将全局规划速度旋转到车体坐标系（当前 yaw 角）
2. **非对称一阶低通滤波**：加速和减速使用不同系数（`x_acc`/`x_dec`），急刹快、起步柔
3. **麦克纳姆轮逆运动学分解**：`FL = vx - vy - 0.64·wz` 等，轮距因子硬编码 0.64
4. 最终输出四轮目标速度 → 送入增量式 PID 速度环

### 5. 路径规划与推箱子求解器（核心算法）

`path_planning.c` 是项目最大、最复杂的模块（~3000 行，全在 MCU 裸机上运行），处理五种游戏模式：

#### 5.1 栅格地图（12×16 = 192 格）

| 元素 | 值 | 说明 |
|------|----|------|
| 空地 | 0 | 可通行 |
| 墙 | 1 | 不可通行 |
| 小车 | 2 | 玩家位置 |
| 箱子 | 3 | 可推动 |
| 目标点 | 6 | 箱子最终归属 |
| 炸弹 | 7 | 可破坏墙体 |

#### 5.2 基础算法工具

- **`simple_astar`**：通用网格 A*，采用加权估价 `f = g + 1.2·h`（曼哈顿距离），带转向惩罚（`TURN_WEIGHT=4`）产生平滑路径。使用最小堆优化，支持 5000 节点空间
- **BFS 距离场**：`bfs_compute_distances()` 计算全图曼哈顿最短距离，`bfs_compute_reachability()` 用于可达性判断（epoch 标记避免 memset 开销）
- **障碍物位图**：每行用 `uint16_t` 位压缩表示 16 列，O(1) 查询

#### 5.3 推箱子（Sokoban）求解器

针对最大 5 个箱子的推箱子问题，采用**带回溯验证的贪心分配 + A* 单箱搜索**：

1. **`generate_greedy_pairing`**：贪心为每个箱子分配最近的未使用目标（BFS 距离）
2. **`backtrack_validate`**：递归回溯验证分配方案可行性。每一步：
   - 按接**近距离排序**剩余箱子，优先尝试最近的
   - 用 **`solve_single_box_a_star`**（完整 A*）规划玩家推箱路径
   - **软死锁检测**（`is_soft_corner_deadlock`）：区分硬墙阻塞 vs 另一未解箱子阻塞，后者触发**破锁机制**——将阻塞箱子推开最多 10 步，再尝试原箱子
   - **ID 感知阻塞**：在 Mode 2 中，同 ID 的箱子和目标点互为障碍，防止交叉占位
3. **`validate_solution`**：外层调用，输出优化后的推箱顺序

#### 5.4 A* 推箱子状态搜索

`solve_single_box_a_star` 采用哈希去重（djb2 + 线性探测，50000 槽，epoch 递增避免 memset）加优先队列（10000 容量）：

- 状态空间：`(player_x, player_y, box_x, box_y, wall_bitmap)` + last_dir
- 后继生成支持推箱子和纯移动两种动作
- 转向惩罚 `TURN_WEIGHT=1` 使得路径更平滑
- 搜索上限 50000 步，超时视为不可解

#### 5.5 ID 学习与推理（Mode 2 → Mode 3）

1. **`id_learning`**：在真实 ID 未知时，BFS 规划访问所有箱子和目标点的接近路径。预留最后一个元素（保证至少有一个参考锚点），若箱子/目标被障碍堵住则激活**救援推箱子**逻辑
2. **`id_record`**：到达接近点后，调用 OPENMV 扫描 ID，关联到当前元素
3. **`id_inference`**：对未知 ID 执行统计推理——频率均衡 + **排列枚举优化**（`next_permutation`）处理多配对场景，最小化总推箱距离
4. **`build_solution_from_id_pairing`**：将学习到的 ID → 箱/目标映射构建为求解方案

#### 5.6 炸弹墙规划（Mode 4）

当栅格中出现炸弹（值 7）时，激活炸弹模式：

1. **可炸墙检测**（`is_breakable_wall_on`）：墙至少有一邻接空地，且不在地图边界
2. **问题区域分析**：检测隔离区（ENC）、目标不可达、拥堵三类问题
3. **引爆炸点评分**：估计每个炸点到炸弹的推距 + 玩家到推站位的 BFS 距离，评分推弹可行性
4. **多炸弹组合搜索**（`search_multi_bomb_combination`）：递归枚举炸弹组合，计算增量收益
5. **执行序列生成**（`plan_bomb_execution_sequence`）：按可行性和代价排序，生成逐步执行计划
6. **验证缓存**（`hash_walls`）：对墙面配置哈希，避免重复验证

### 6. OPENMV 通信协议

`cam_uart.c` 实现双 UART 通信：

**主通道**（LPUART3, B22/B23, 115200 baud）：
- 接收文本协议帧 `start...end`，解析小车位置 `car:cx,cy;` 和 12×16 栅格数组 `map:01234567...`
- FIFO 中断接收，帧同步精确切除

**ID 通道**（LPUART1, B12/B13, 115200 baud）：
- `cam_uart2_write(type)` 发送待识别元素类型
- `cam_uart2_read()` 阻塞读取 OPENMV 返回的 ID 号（十进制两位数）

---

## 💻 软件架构

### 总体结构

```
main.c                       ← 入口、状态机、PIT 中断控制循环
├── Init()                   ← 硬件初始化 + TFT 菜单
├── state_judgment()         ← 游戏状态机调度
├── PIT_IRQHandler          ← 5ms 控制环 + 10ms 数据环
│   ├── encoder_get()        ← 编码器读取
│   ├── imu_get()            ← IMU 读取 + Mahony 滤波
│   ├── distance(yaw)        ← 里程计更新
│   ├── pid_location(x/y)    ← 位置环 PID
│   ├── pid_location(yaw)    ← 偏航角 PID
│   ├── motor_solution()     ← 运动学合成
│   ├── pid_increm(FL/FR/..) ← 四轮速度环 PID
│   ├── motor_duty()         ← PWM 输出
│   └── cam_uart1_read()     ← 摄像头数据接收
└── path_process()           ← 路径规划和模式切换
    ├── path_start_calculation()  ← 模式0: 元素接近
    ├── path_calculation()        ← 模式1: 推箱子路径
    ├── path_look_calculation()   ← 模式2: ID学习路径
    ├── path_id_calculation()     ← 模式3: ID行驶路径
    └── path_boom_calculation()   ← 模式4: 炸弹墙规划
```

### 游戏状态机

```
NoGame → GameMap → Run → Waiting → Look → Run → ... → GameOver → End
```

- **Mode 0 (Start)**：导航到最近元素（箱子/目标点），偏航对准后呼叫 OPENMV 扫描 ID，ID=10 跳 Mode 1，否则进入 Mode 2
- **Mode 1 (Normal Run)**：A* 推箱子求解 + 路径跟踪，无 ID 信息的盲推
- **Mode 2 (Look)**：按规划路径访问箱子和目标点，逐个扫描记录 ID，完成后推理 ID 映射
- **Mode 3 (ID Run)**：利用已学习的 ID 关联进行精准配对推箱子（避免交叉占位）
- **Mode 4 (Bomb)**：炸弹推墙，破坏后重规划回到 Mode 0

### 内存模型

| 区域 | 用途 |
|------|------|
| XIP Flash | 代码执行 |
| SDRAM（`.bss.SDRAM_CACHE`） | 大规模缓冲区：A* 哈希表(50000)、BFS 队列(16384)、路径点(500) |
| DTCM（`ITCM_NonCacheable`） | 时序关键函数：PID、A* 核心循环、BFS、位图查询 |
| OCRAM | 运行时数据 |

### 中断路由

```
PIT_CH0 (5ms)      → 控制循环（编码器/IMU/PID/电机）
PIT_CH1 (10ms)     → 摄像头数据读取 + 串口更新
LPUART3 (B22/B23)  → OPENMV 主摄像头（栅格/车位数据）
LPUART1 (B12/B13)  → ID 通信 / 调试
LPUART8 (D16/D17)  → BLE 无线串口
```

---

## 📁 项目结构

```
ysu-znc-21-smarteyes/
├── README.md
├── libraries/                      # 逐飞开源库 V3.9.2（请勿修改）
│   ├── zf_common/                  # 通用头文件、typedef、时钟、FIFO、数学
│   ├── zf_driver/                  # 底层驱动抽象
│   ├── zf_device/                  # 外设驱动（摄像头、IMU、显示屏、BLE、TOF 等）
│   ├── zf_components/              # 助手/调试接口
│   ├── sdk/                        # NXP MCUXpresso SDK 2.12
│   ├── components/                 # FATFS、SDMMC、USB 协议栈
│   └── doc/                        # 文档与版本记录
└── project/
    ├── code/                       # 用户应用代码（★ 新文件放这里）
    │   ├── path_planning.c/h       # 路径规划核心（A*/BFS/推箱子/炸弹/ID学习）
    │   ├── control.c/h             # 麦克纳姆轮运动学
    │   ├── pid.c/h                 # PID 控制器（增量式+位置式）
    │   ├── motor.c/h               # 电机 PWM 10kHz 驱动
    │   ├── encoder.c/h             # 正交编码器 + 里程计
    │   ├── imu963.c/h              # IMU963RA + Mahony 姿态估计
    │   ├── cam_uart.c/h            # OPENMV 双 UART 通信协议
    │   ├── menu.c/h                # TFT 菜单系统（PID调参/校准）
    │   ├── tft180.c/h              # TFT180 显示屏驱动
    │   ├── my_uart.c/h             # 自定义调试串口
    │   ├── wireless_uart.c/h       # BLE 无线串口
    │   └── key.c/h                 # 按键输入
    ├── user/src/
    │   ├── main.c                  # 入口、状态机、PIT 中断
    │   └── isr.c                   # 外设中断处理（UART/CSI/GPIO/摄像头）
    ├── mdk/                        # Keil MDK 工程 + 分散加载文件
    └── iar/                        # IAR EWARM 工程 + 链接脚本
```

---

## 💻 开发环境

| 工具 | 版本 |
|------|------|
| Keil MDK (µVision) | 5.33 |
| IAR EWARM | 8.32 |
| 编译器 | ARMCLANG (Keil) / ARMCC (IAR) |
| C 标准 | C99 |
| 构建目标 | `nor_sdram_zf_dtcm` |
| 关键预定义宏 | `CPU_MIMXRT1064DVL6A`, `XIP_EXTERNAL_FLASH=1`, `USB_STACK_BM`, `MCUXPRESSO_SDK`, `SKIP_SYSCLK_INIT`, `PRINTF_FLOAT_ENABLE=1` |

> ⚠️ 仅支持 IDE 构建（无命令行构建方式）。添加 `.c`/`.h` 文件时需同时放入 `project/code/` 并在 IDE 工程树中添加。

---

## 🚀 构建与烧录

### Keil MDK
1. 打开 `project/mdk/rt1064.uvprojx`
2. 选择构建目标：`nor_sdram_zf_dtcm`
3. 编译 → 使用 DAP-Link / J-Link 烧录

---

## 🧩 关键约束与设计决策

- **全部 MCU 端计算**：路径规划、推箱子求解、炸弹规划均在 600MHz Cortex-M7 裸机上运行，未使用上位机或外部计算资源
- **内存极度紧张**：堆大小在链接脚本中精心平衡（A* 50000 槽、BFS 16384 队列、路径 500 点），epoch 标记法替代 memset 是关键优化
- **无操作系统**：裸机中断驱动，5ms PIT 控制周期硬实时
- **通信不确定性**：OPENMV 通过 UART 发送文本协议，帧同步 `start...end` 确保乱序恢复
- **逐飞库边界**：`libraries/` 目录为第三方开源库，所有自定义代码在 `project/code/` 中

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
