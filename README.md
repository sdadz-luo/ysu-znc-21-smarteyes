# ysu-znc-21-smarteyes

> 🏎️ **燕山大学 · 第21届全国大学生智能汽车竞赛 — 智能视觉组**

[![Platform](https://img.shields.io/badge/Platform-NXP%20RT1064-blue)](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/i-mx-rt-crossover-mcus/i-mx-rt1064-crossover-mcu-with-arm-cortex-m7-core:i.MX-RT1064)
[![IDE](https://img.shields.io/badge/IDE-IAR%208.32%20%7C%20Keil%205.33-green)](#开发环境)
[![Language](https://img.shields.io/badge/Language-C-orange)](https://en.wikipedia.org/wiki/C_(programming_language))
[![License](https://img.shields.io/badge/License-GPLv3-lightgrey)](./libraries/doc/GPL3_permission_statement.txt)

---

## 📖 项目简介

本项目是燕山大学参加**第21届全国大学生智能汽车竞赛（智能视觉组）**的参赛代码，基于 **NXP RT1064 (MIMXRT1064DVL6A)** 微控制器开发，运行于逐飞科技 RT1064 核心板。

智能视觉组要求小车具备**自主视觉识别 + 运动控制 + 逻辑推理**能力。OPENMV 摄像头通过 UART 将栅格地图和车位坐标发送给 MCU，MCU 负责路径规划、推箱子求解、炸弹破墙规划、PID 运动控制和状态机调度，**所有推理任务均在 Cortex-M7（600MHz）裸机上完成**。

---

## 🛠️ 硬件平台

| 类别 | 型号 |
|------|------|
| **MCU** | NXP MIMXRT1064DVL6A (Cortex-M7, 600MHz, 4MB 内部 Flash) |
| **核心板** | 逐飞科技 RT1064 核心板 |
| **摄像头** | OPENMV5-RT（主摄像头，UART 通信） |
| **显示器** | TFT180 (1.8", 128x160, SPI) |
| **IMU** | IMU963RA (三轴加速度计 + 三轴陀螺仪) |
| **无线通信** | BLE6A20 蓝牙（UART 透传） |
| **其他** | 正交编码器 ×4、直流减速电机 ×4（麦克纳姆轮） |

---

## 🧠 算法体系

### 1. 姿态估计 — 自适应增益 Mahony 互补滤波

`imu963.c` 实现基于四元数的 Mahony AHRS 算法：

- **陀螺仪零偏校准**：上电采集 400 样本（约 2 秒）取均值
- **自适应增益 Kp**：根据加速度模值偏离 1g 的程度，在 `0.2~0.5` 之间线性调整，机动时降低对加速度计的信任，抑制非重力加速度干扰
- **一阶低通滤波**：加速度计截止 ~1.9Hz（α=0.06），陀螺仪 ~8.8Hz（α=0.25），各自独立系数
- **陀螺仪死区**：`0.005 rad/s` 以下置零，抑制零漂抖动
- **积分限幅抗饱和**：积分项绝对值钳位在 1.0
- **偏航角修正**：运行时可通过 `corr_yaw` 参数补偿安装偏角
- 输出欧拉角（roll/pitch/yaw），yaw 范围 `-180°~180°`

**核心函数**：
```c
void imu_init(void);    // 校准偏置，初始化 IMU
void imu_get(void);     // 读取 IMU → 低通滤波 → Mahony 更新 → 输出 yaw
```

---

### 2. 里程计 — 麦克纳姆轮正运动学

`encoder.c` 实现四轮正交编码器读数 → 场坐标系位移：

- **四路编码器独立正交解码**（QTIMER1/2/3 模块）
- **一阶低通滤波** (`α=0.9`) 抑制测量噪声，float 全精度累积避免整数截断
- **体坐标系位移** → 通过偏航角旋转 → **场坐标系位移**
- **可调修正系数** `corr_x_enc` / `corr_y_enc` 补偿打滑和标定误差
- 每 **5ms PIT 中断**执行（每周期调用 5ms 后 clear count，`dc_dt` 预计算好）

**机械参数**（`encoder.c` 常量）：

| 参数 | 值 | 说明 |
|------|-----|------|
| 编码器每圈脉冲 (CPR×4) | 16384 | 正交解码 4 倍频 |
| 齿轮齿数 (编码器/轮) | 30/70 | 减速比 3:7 |
| 车轮半径 | 3.0 cm | — |

**核心函数**：
```c
void encoder_init(void);    // 初始化 4 路编码器
void encoder_get(void);     // 读取 + 滤波 + 清计数值
void distance(float yaw);   // 计算并累积世界坐标 (x_enc, y_enc)
```

**里程计算法细节**：
```c
// 1. 体坐标系位移（Mecanum 正运动学）
dx = (FL + FR + BL + BR) / 4;
dy = (FL - FR - BL + BR) / 4;

// 2. 体→世界坐标系旋转（通过当前 yaw 角）
world_x = shift_x * cos(yaw) - shift_y * sin(yaw);
world_y = shift_x * sin(yaw) + shift_y * cos(yaw);

// 3. 累积位置（带修正系数）
x_enc += world_x * corr_x_enc;
y_enc += world_y * corr_y_enc;
```

---

### 3. PID 控制 — 串级双环

`pid.c` 实现增量式和位置式两种 PID，构成**位置环 + 速度环**串级结构。7 个全局 PID 实例：

| PID 实例 | 类型 | 初始化参数 | 作用 |
|----------|------|-----------|------|
| `pid_FL/FR/BL/BR` | 增量式 `pid_increm` | kp=60, ki=10, kd=50, maxOut=8000 | 四轮速度内环 |
| `pid_x` | 位置式 `pid_location` | 菜单可调 (默认 3.50/0.003/7.00/110) | X 方向位置外环 |
| `pid_y` | 位置式 `pid_location` | 菜单可调 (默认 -3.50/-0.003/-7.0/110) | Y 方向位置外环 |
| `pid_yaw` | 位置式 `pid_location` | 5/0.001/30/70（Run 态切换 10/0.001/30） | 偏航角闭环 |
| `pid_gyro` | 位置式 | — | 角速度环（备用） |

#### PID 结构体
```c
typedef struct {
    float kp, ki, kd;           // 比例、积分、微分增益
    float error, error_last, error_last2;  // 误差及其历史
    float integral, maxintegral;           // 积分项及限幅
    float output, output_1, maxOutput;     // 输出及限幅
    float target;                // 目标值
    float dead;                  // 死区
} pid;
```

#### 增量式（速度环）
```c
// Δu = kp·(e−e_last) + ki·e + kd·(e−2e_last+e_last2)
// output += Δu，死区内衰减系数 0.9
float pid_increm(pid *pid_struct, float now_value);
```

#### 位置式（位置环/偏航环）
```c
// u = kp·e + ki·∫e + kd·(e−e_last)
// 积分限幅，死区内积分衰减系数 0.95
float pid_location(pid *pid_struct, float now_value);
```

#### 运行时参数切换
```c
void pid_change(&pid_yaw, 10, 0.001f, 30);  // Run 状态：快速响应
void pid_change(&pid_yaw, 5, 0.001f, 30);   // 其他状态：平稳
void pid_position_speed(void);                // 急停模式：maxOutput→130
void pid_reset(void);                         // 完全重置所有 PID
```

- **积分分离 + 死区**：误差在死区内积分衰减（位置式 `×0.95`，增量式 `×0.9`），防止频繁震荡
- **输出限幅 + 积分抗饱和**：`maxIntegral` / `maxOutput` 硬限制
- 所有 PID 参数（kp/ki/kd/限速）可在 **TFT 菜单运行时实时调节**并保存到 Flash

---

### 4. 运动学合成 — 车场分离 + 非对称滤波

`control.c` 中的 `motor_solution(vx_field, vy_field, yaw, wz)` 完成：

1. **场→车坐标系旋转**：将全局规划速度旋转到车体坐标系（当前 yaw 角）
   ```c
   vx_body =  vx_field * cos(yaw) + vy_field * sin(yaw);
   vy_body = -vx_field * sin(yaw) + vy_field * cos(yaw);
   ```
2. **非对称一阶低通滤波**：加速和减速使用不同系数（`x_acc=0.01`/`x_dec=0.06`），**急刹快、起步柔**
   ```c
   if (|target| > |current|) a = acc_coeff;  // 加速柔
   else a = dec_coeff;                        // 减速快
   ```
3. **麦克纳姆轮逆运动学分解**（轮距因子硬编码 `0.64f`）：
   ```c
   FL = vx - vy - 0.64 * wz;
   FR = vx + vy + 0.64 * wz;
   BL = vx + vy - 0.64 * wz;
   BR = vx - vy + 0.64 * wz;
   ```
4. 最终输出四轮目标速度 → 送入增量式 PID 速度环

---

### 5. 路径规划与推箱子求解器（核心算法）

`path_planning.c` 是项目最大、最复杂的模块（~4447 行，全在 MCU 裸机上运行），处理五种游戏模式。内存通过链接脚本精细分配到 SDRAM（D-Cache 加速）和 DTCM（零等待）。

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

- **`simple_astar`**：通用网格 A\*，采用加权估价 `f = g + 1.2·h`（曼哈顿距离），带转向惩罚（`TURN_WEIGHT=4`）产生平滑路径。使用最小堆优化，支持 5000 节点空间
- **BFS 距离场**：`bfs_compute_distances()` 计算全图曼哈顿最短距离，`bfs_compute_reachability()` 用于可达性判断（**epoch 标记避免 memset** 开销）
- **障碍物位图**：每行用 `uint16_t` 位压缩表示 16 列，O(1) 查询

#### 5.3 推箱子（Sokoban）求解器

针对最大 5 个箱子的推箱子问题，采用**带回溯验证的贪心分配 + A\* 单箱搜索**：

1. **`generate_greedy_pairing`**：贪心为每个箱子分配最近的未使用目标（BFS 距离）
2. **`backtrack_validate`**：递归回溯验证分配方案可行性。每一步：
   - 按接近**距离排序**剩余箱子，优先尝试最近的
   - 用 **`solve_single_box_a_star`**（完整 A\*）规划玩家推箱路径
   - **软死锁检测**（`is_soft_corner_deadlock`）：区分硬墙阻塞 vs 另一未解箱子阻塞，后者触发**破锁机制**——将阻塞箱子推开最多 10 步，再尝试原箱子
   - **ID 感知阻塞**：在 Mode 2 中，同 ID 的箱子和目标点互为障碍，防止交叉占位
3. **`validate_solution`**：外层调用，输出优化后的推箱顺序

#### 5.4 A\* 推箱子状态搜索

`solve_single_box_a_star` 采用**哈希去重**（djb2 + 线性探测，**65536 槽**，epoch 递增避免 memset）加**优先队列**（10000 容量）：

- **状态空间**：`(player_x, player_y, box_x, box_y, wall_bitmap)` + last_dir
- 状态编码为 **16-bit**：`px:4 | py:4 | bx:4 | by:4`
- 后继生成支持推箱子和纯移动两种动作
- 转向惩罚 `TURN_WEIGHT=1` 使得路径更平滑
- 搜索上限 **50000 步**，超时视为不可解

**关键数据结构**（见 `path_planning.h`）：
```c
// A* 状态
typedef struct { Point player, box, target; uint16_t wall_bitmap[MAP_ROWS]; uint8_t last_dir; } State;
// 哈希表条目
typedef struct { State state; uint16_t g, came_from; uint8_t used; } HashEntry;
// 路径结果
typedef struct { int32_t cost; Point final_player_pos; bool success; Point path_points[500]; uint16_t path_len; } AStarResult;
```

#### 5.5 ID 学习与推理（Mode 2 → Mode 3）

1. **`id_learning`**：在真实 ID 未知时，BFS 规划访问所有箱子和目标点的接近路径。预留最后一个元素（保证至少有一个参考锚点），若箱子/目标被障碍堵住则激活**救援推箱子**逻辑
2. **`id_record`**：到达接近点后，调用 OPENMV 扫描 ID，关联到当前元素
3. **`id_inference`**：对未知 ID 执行统计推理——频率均衡 + **排列枚举优化**（`next_permutation`）处理多配对场景，最小化总推箱距离
4. **`build_solution_from_id_pairing`**：将学习到的 ID → 箱/目标映射构建为求解方案

#### 5.6 炸弹墙规划（Mode 4）

当栅格中出现炸弹（值 7）时，激活炸弹模式——这是项目中**最复杂的规划子系统**：

1. **可炸墙检测**（`is_breakable_wall_on`）：墙至少有一邻接空地，且不在地图边界
2. **问题区域分析**：检测隔离区（ENC）、目标不可达、拥堵三类问题
3. **引爆炸点评分**：估计每个炸点到炸弹的推距 + 玩家到推站位的 BFS 距离，评分推弹可行性
4. **多炸弹组合搜索**（`search_multi_bomb_combination`）：递归枚举炸弹组合，计算增量收益
5. **执行序列生成**（`plan_bomb_execution_sequence`）：按可行性和代价排序，生成逐步执行计划
6. **验证缓存**（`hash_walls`）：对墙面配置哈希，避免重复验证

**炸弹规划数据结构**（18+ 个结构体）：
```c
// 引爆炸弹计划
typedef struct {
    Point detonate_pos; Point walls_covered[9]; uint8_t wall_count;
    uint8_t bomb_index; Point bomb_initial_pos;
    uint16_t bomb_push_distance; int total_benefit;
    uint16_t resolved_mask;
} DetonatePlan;

// 完整执行方案
typedef struct {
    BombExecutionStep steps[MAX_BOOMS]; uint8_t step_count;
    uint16_t total_cost; bool is_valid;
} BombExecutionPlan;
```

---

### 6. OPENMV 通信协议

`cam_uart.c` 实现双 UART 通信：

**主通道**（LPUART3, TX=B22, RX=B23, 115200 baud）：
- FIFO 中断接收，帧同步 `start...end` 精确切除
- 解析文本协议帧，**`push_cam_data()`** 处理两种数据：
  - 小车位置：`"car:x,y;"` → `cam_uart_data.car_cx/cy`
  - 栅格数据：`"map:0123456789abcdef..."` → `uint8_t grid[12][16]`
- 角度指令：`cam1_uart_send(angle)` 将角度映射为编码发送：
  - `0° → "start01end\r\n"`, `90° → "start02end\r\n"`
  - `-90° → "start03end\r\n"`, `180° → "start04end\r\n"`

**ID 通道**（LPUART1, TX=B12, RX=B13, 115200 baud）：
- `cam_uart2_write(type)` 发送待识别元素类型 → `"start02end\r\n"`
- `cam_uart2_read()` 阻塞读取 OPENMV 返回的 ID 号（十进制两位数）

---

## 💻 软件架构

### 初始化序列（`Init()`）

```c
main():
  clock_init(600MHz) → debug_init() → delay(300ms)
  → Init():
      tft_init() → tft180_clear() → my_key_init()
      → flash_init() → menu() [TFT 菜单循环直到用户选择"start"]
      → delay(1000ms) → encoder_init() → imu_init() → my_uart_init()
      → motor_init() → pid_init() → cam_uart_init()
      → cam1_uart_send(0) → reset_planning_system()
      → pit_ms_init(PIT_CH0, 5ms) → pit_ms_init(PIT_CH1, 10ms)
  → interrupt_global_enable()
  → while(1) { state_judgment(); }
```

### PIT 中断控制循环（`PIT_IRQHandler`）

```
PIT_CH0 (5ms):
  1. encoder_get()        ← 读取并滤波编码器
  2. imu_get()            ← IMU + Mahony 姿态更新
  3. distance(yaw)        ← 里程计积分
  4. if (Run or GameOver) [每 10ms]:
       pid_position_target(x, y)  ← 设置位置目标
       vx = pid_location(&pid_x, x_enc)  ← X 位置环
       vy = pid_location(&pid_y, y_enc)  ← Y 位置环
  5. [每 10ms]:
       pid_change(&pid_yaw, 10/0.001/30)  ← Run 态切换增益
       yaw_error = yaw_target - yaw (±360° wrap)
       pid_target(&pid_yaw, yaw + yaw_error)
       vz = pid_location(&pid_yaw, yaw)   ← 偏航环 PID
  6. motor_solution(vx, vy, yaw, vz)      ← 运动学合成
  7. FL = pid_increm(&pid_FL, encoder/4)  ← 四轮速度环
     FR/BL/BR 同理
  8. motor_duty(FL, FR, BL, BR)           ← PWM 输出

PIT_CH1 (10ms):
  1. car_data = cam_uart1_read()  ← 读取 OPENMV 数据
  2. uart_updata()                ← 低通滤波
```

### 游戏状态机

```
NoGame → GameMap → Run → Waiting → Look → Run → ... → GameOver → End
```

```c
typedef enum { NoGame, GameMap, Waiting, Look, Run, GameOver, End } Car_State;
```

**状态转换详解**：

| 当前状态 | 触发条件 | 下一状态 | 动作 |
|----------|---------|----------|------|
| **NoGame** | 摄像头检测到起始区<br>`0≤x_cam≤2.5×16.2` `5×15.8≤y_cam≤7×15.8` | GameMap | 初始化位置：x_enc = x_cam/16.2×20, y_enc = y_cam/15.8×20 |
| **GameMap** | 始终 | Run | 设置目标到原点 `(origin_x_enc+0.5)×20, (origin_y_enc+0.5)×20` |
| **Run** | 到达目标点 `|x_enc - x_target| ≤ 1` | Waiting | 停止运动 (vx=0, vy=0) |
| **Waiting** | `path_process()` 完成路径计算 | Run | 设置下一步目标点，恢复运动 |
| **Waiting** | `path_process()` 切换到 Look 模式 | Look | 进入 ID 扫描 |
| **Look** | ID 扫描完成（模式 0 首次定位） | Run/Waiting | 根据 game_mode 跳转 |
| **GameOver** | game_count ≤ 3 | NoGame | 重置系统重新开始 |
| **GameOver** | game_count ≥ 3 | End | 永久停止 |

#### game_mode 路径规划模式

| 模式 | 触发条件 | 规划函数 | 行为 |
|------|---------|----------|------|
| **0 (Start)** | 首次启动 | `path_start_calculation()` | 导航到最近元素（箱子/目标）→ 偏航对准 → OPENMV 扫描 ID → ID=10 跳 Mode 1，否则 Mode 2 |
| **1 (Normal Run)** | ID==10 或 game_count==0 | `path_calculation()` | A\* 推箱子求解 + `process_normal_path()` 跟踪，无 ID 信息的盲推 |
| **2 (Look)** | ID≠10 或炸弹完成 | `path_look_calculation()` | BFS 规划访问所有箱子和目标，逐个 `is_look` 点扫描记录 ID，完成后推理 ID 映射 → 自动切 Mode 3 |
| **3 (ID Run)** | Mode 2 学习完成 | `path_id_calculation()` | 利用已学习的 ID 精准配对推箱（避免交叉占位） |
| **4 (Bomb)** | 栅格有元素 7 | `path_boom_calculation()` | 炸弹破墙规划 → 逐点导航 → 遇到 `is_push` 点延时 1.5s（等待爆炸）→ `map_boom_out()` 更新栅格 → 回 Mode 2 |

#### path_process() 详细调度

```c
static void path_process(void) {
  // 1. 扫描地图：检测 car(2), box(3), target(6), bomb(7)
  //    根据 game_count 决定初始 game_mode (0→1, >0→2)
  //    发现炸弹(7) → 强制 game_mode=4
  
  // 2. 根据 game_mode 调用规划函数：
  if (mode==0) path_start_car = path_start_calculation(grid);
  if (mode==1) path_car = path_calculation(grid);
  if (mode==2) path_look_car = path_look_calculation(grid);
  if (mode==3) path_car = path_id_calculation();
  if (mode==4) path_boom_car = path_boom_calculation(grid);
  
  // 3. 路径执行（针对各模式的逐点跟踪）
  if (mode==1||3) process_normal_path(&path_car);  // 逐点前进，终点→GameOver
  if (mode==0)    逐点前进至最近元素→到达后转 Look
  if (mode==2)    逐点前进，遇到 is_look 点→转 Look→扫描→继续→完成→切 Mode 3
  if (mode==4)    逐点前进，遇到 is_push→delay(1.5s)→继续→完成→map_boom_out()→回 Mode 2
}
```

**`process_normal_path()`** 为实现路径跟踪的核心函数：
```c
static void process_normal_path(const Path *path) {
  if (到达当前目标点) {
    step++;
    if (step > path->len) {  // 路径走完
      x_target = end_x * x_enc_uint;  // 急停到终点
      y_target = end_y * y_enc_uint;
      pid_position_speed();  // maxOutput=130 急停
      game_over_flag = 1;
      car_state = GameOver;
    } else {
      x_target = (path->x[step] + 0.5) * x_enc_uint;  // 下一步
      y_target = (path->y[step] + 0.5) * y_enc_uint;
      car_state = Run;
    }
  }
}
```

---

### 中断路由

| 外设 | 引脚 | 用途 | ISR |
|------|------|------|-----|
| PIT_CH0 | — | 5ms 控制循环 | `PIT_IRQHandler` |
| PIT_CH1 | — | 10ms 数据读取 | `PIT_IRQHandler` |
| CSI | — | 摄像头并行接口 | `CSI_IRQHandler` → `CSI_DriverIRQHandler()` |
| LPUART1 | B12/B13 | 调试串口 / ID 通信 | `LPUART1_IRQHandler` → `cam_uart_isc_2()` |
| LPUART2 | — | 未使用 | `LPUART2_IRQHandler`（空） |
| LPUART3 | B22/B23 | **OPENMV 主摄像头** | `LPUART3_IRQHandler` → `cam_uart_isc_1()` |
| LPUART4 | — | FlexIO 摄像头 + GNSS | `LPUART4_IRQHandler` → `flexio_camera_uart_handler()`, `gnss_uart_callback()` |
| LPUART5 | — | 备用摄像头 | `LPUART5_IRQHandler` → `camera_uart_handler()` |
| LPUART6 | — | 未使用 | `LPUART6_IRQHandler`（空） |
| LPUART8 | D16/D17 | BLE 无线 + 调试中断 | `LPUART8_IRQHandler` → `wireless_module_uart_handler()`, `debug_interrupr_handler()` |
| GPIO1_0_15 | B0 | GPIO 外部中断 | EXTI flag clear |
| GPIO1_16_31 | B16 | 无线 SPI | `wireless_module_spi_handler()` + EXTI clear |
| GPIO2_0_15 | C0 | FlexIO 摄像头帧同步 | `flexio_camera_vsync_handler()` + EXTI clear |
| GPIO2_16_31 | C16 | **ToF 传感器中断** | `tof_module_exti_handler()` + EXTI clear |
| GPIO3_0_15 | D4 | GPIO 外部中断 | EXTI flag clear |

---

### MPU 配置与内存模型

`mpu_config.c` 中的 `SystemInitHook()` 在 `main()` 之前由 SDK 弱函数自动调用，配置 6 个 MPU Region 并启用 D-Cache。

#### MPU 6 区域配置

| Region | 地址范围 | 大小 | 类型 | 用途 |
|--------|---------|------|------|------|
| 0 | 0x80000000–0x81FFFFFF | 32MB | Normal WB/WA | **SDRAM 可缓存**：A\* `hash_table[65536]`、优先队列 |
| 1 | 0x81E00000–0x81FFFFFF | 2MB | Device nGnRnE | **SDRAM 不可缓存**：DMA 缓冲区、帧缓冲 |
| 2 | 0x00000000–0x0000FFFF | 64KB | Normal | **ITCM**：`ITCM_NonCacheable` 时序关键函数 |
| 3 | 0x20000000–0x2007FFFF | 512KB | Normal | **DTCM**：BFS 距离图、访问标记、障碍物缓冲区 |
| 4 | 0x20200000–0x2027FFFF | 512KB | Normal | **OCRAM**：备选缓冲区 |
| 5 | 0x70000000–0x703FFFFF | 4MB | Device RO | **FlexSPI Flash**：代码指令（I-Cache 覆盖） |

#### 内存使用分布

| 段 | 内容 | 位置 |
|----|------|------|
| `ITCM_NonCacheable` | `manhattan_distance`, `is_corner_deadlock`, `hash_lookup_insert`, `pq_push/pop`, `heuristic`, `get_successors`, `bfs_compute_distances/reachability`, `simple_astar`, `solve_single_box_a_star`, `compute_player_region_with_walls` | ITCM (1-cycle) |
| `$DTCM` | `g_dist_map`, `g_bfs_visited`, `g_obs_buf`, `g_bfs_queue`, `pq` | DTCM (0-wait) |
| `SDRAM_CACHE` | `hash_table[MAX_OPENSET=65536]` | SDRAM (D-Cache) |

---

### TFT 菜单系统

`menu.c` 实现完整的 TFT 菜单系统，用于运行时调参：

**5 个子菜单**：

| 菜单 | 可调参数 | 步进 |
|------|---------|------|
| **PID** | `pid_x_p/i/d`, `pid_x_speed`, `pid_y_p/i/d`, `pid_y_speed` | 0.1 / 0.001 / 0.1 / 1 |
| **ORIGIN** | `origin_x_enc`, `origin_y_enc` | 1 |
| **CORR** | `corr_x_enc`, `corr_y_enc`, `corr_x_cam`, `corr_y_cam`, `corr_yaw` | 0.005 / 0.005 / 1 / 1 / 0.000001 |
| **control** | `x_acc`, `x_dec`, `y_acc`, `y_dec` | 0.005 |
| **datasave** | 保存全部参数到 Flash (sector 127, page 3) | — |

导航：4 键（上/下/左/右），左=C30, 右=C29, 下=C31, 上=C28，3 态去抖状态机。

---

### 按键驱动

`key.c` 使用 3 态去抖状态机：

```c
enum { KEY_STATE_IDLE=0, KEY_STATE_PRESSED=1, KEY_STATE_DONE=2 };
```

- IDLE → 检测到低电平 → PRESSED
- PRESSED → 持续低电平 → 设置 single_flag → DONE
- DONE → 检测到高电平 → IDLE

---

## 📁 项目结构

```
ysu-znc-21-smarteyes/
├── AGENTS.md                        # OpenCode 代理指令文件
├── README.md
├── .clangd                          # clangd 额外 ARMCLANG 路径
├── compile_flags.txt                # clangd 编译标志（含所有 include 路径）
├── .opencode/                       # OpenCode 配置
│   ├── opencode.json                # 指令引用 + copilot 技能启用
│   └── lsp.json                     # clangd LSP 启用
├── libraries/                       # 逐飞开源库 V3.9.2（请勿修改）
│   ├── zf_common/                   # 通用头文件、typedef、时钟、FIFO、数学
│   ├── zf_driver/                   # 底层驱动抽象
│   ├── zf_device/                   # 外设驱动（摄像头、IMU、显示屏、BLE、TOF 等）
│   ├── zf_components/               # 助手/调试接口（TFT 显示、无线示波器等）
│   ├── sdk/                         # NXP MCUXpresso SDK 2.12（fsl_* 驱动）
│   ├── components/                  # FATFS、SDMMC、USB 协议栈
│   └── doc/                         # 文档与版本记录
└── project/
    ├── RT1064核心板丝印与芯片引脚对应表格.xlsx
    ├── RT1064智能车推荐引脚分配.txt  # 官方引脚分配建议
    ├── code/                        # 用户应用代码（★ 新文件放这里）
    │   ├── path_planning.c/h        # 路径规划核心（A*/BFS/推箱子/炸弹/ID学习）
    │   ├── control.c/h              # 麦克纳姆轮运动学（场→体变换+非对称滤波）
    │   ├── pid.c/h                  # PID 控制器（增量式+位置式，7实例）
    │   ├── motor.c/h                # 电机 PWM 10kHz 驱动（H桥双极性）
    │   ├── encoder.c/h              # 正交编码器 + 里程计（麦克纳姆正运动学）
    │   ├── imu963.c/h               # IMU963RA + Mahony 自适应增益姿态估计
    │   ├── cam_uart.c/h             # OPENMV 双 UART 通信协议（帧同步）
    │   ├── menu.c/h                 # TFT 菜单系统（PID调参/校准/Flash存储）
    │   ├── tft180.c/h               # TFT180 1.8" 显示屏驱动（SPI/竖屏）
    │   ├── my_uart.c/h              # 自定义调试串口（SeekFree 无线示波器）
    │   ├── wireless_uart.c/h        # BLE 无线串口（⚠与 my_uart 重复）
    │   ├── key.c/h                  # 按键输入（4键/3态去抖）
    │   └── mpu_config.c             # MPU 配置 + D-Cache 使能（SystemInitHook）
    ├── user/src/
    │   ├── main.c                   # 入口、状态机、PIT 中断（5ms/10ms）
    │   └── isr.c                    # 外设中断处理（13个 ISR）
    │   inc/isr.h                    # 空占位头文件
    ├── mdk/                         # Keil MDK 工程 + 分散加载文件 (scf/)
    └── iar/                         # IAR EWARM 工程 + 链接脚本 (icf/)
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
| clangd 附加宏 | `__GNUC__`, `-U_WIN32`, `-nostdinc` |

> ⚠️ 仅支持 IDE 构建（无命令行构建方式）。添加 `.c`/`.h` 文件时需同时放入 `project/code/` 并在 IDE 工程树中添加。

---

## 🚀 构建与烧录

### Keil MDK
1. 打开 `project/mdk/rt1064.uvprojx`
2. 选择构建目标：`nor_sdram_zf_dtcm`
3. 编译 → 使用 DAP-Link / J-Link 烧录

### IAR EWARM
1. 打开 `project/iar/rt1064.eww`
2. 选择构建目标：`nor_sdram_zf_dtcm`
3. 编译 → 烧录

---

## 🧩 关键约束与设计决策

- **全部 MCU 端计算**：路径规划、推箱子求解、炸弹规划均在 600MHz Cortex-M7 裸机上运行，未使用上位机或外部计算资源。所有算法代码在 `path_planning.c`（~4447 行）中
- **内存极度紧张**：堆大小在链接脚本中精心平衡。A\* 哈希表 65536 槽、BFS 队列 16384、路径 500 点。**epoch 标记法替代 memset** 是关键优化（避免每次搜索清零）
- **无操作系统**：裸机中断驱动，5ms PIT 控制周期硬实时。所有任务在 `PIT_IRQHandler` 中调度
- **D-Cache 处理**：SDRAM 通过 MPU Region 0 设为 Normal WB/WA 启用了 D-Cache，但 Region 1 的 DMA 缓冲区设为 Device 类型，避免 cache coherence 问题。时序关键函数放置在 ITCM（1-cycle）中
- **通信不确定性**：OPENMV 通过 UART 发送文本协议，帧同步 `start...end` 确保乱序恢复。`cam_uart.c` 使用 FIFO 缓冲 + 帧切除机制
- **逐飞库边界**：`libraries/` 目录为第三方开源库，所有自定义代码在 `project/code/` 中
- **Flash 持久化**：调参参数使用 Flash sector 127/page 3 存储，每次启动从 Flash 加载
- **逐飞无线示波器**：通过 `my_uart.c` 实现 8 通道无线示波器数据发送（SeekFree Assistant 协议），用于调试和参数整定

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
