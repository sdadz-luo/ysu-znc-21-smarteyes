# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# ysu-znc-21-smarteyes — 智能车竞赛固件

> NXP RT1064 (Cortex-M7 @ 600MHz) 智能车竞赛固件。
> 第21届全国大学生智能汽车竞赛 — 智能视觉组。
> **裸机（无 OS）** | **C99** | **仅 IDE 构建** | **无测试框架** | **中英文注释混用**

---

## 一、工程概述（AI 快速入口）

```
MCU:  NXP MIMXRT1064DVL6A (Cortex-M7, 600MHz)
RAM:  ITCM 64KB + DTCM 512KB + OCRAM 512KB + SDRAM 32MB
Flash: 4MB FlexSPI XIP (代码原地执行)
摄像头: OpenART_Plus over UART (文本协议, 115200 baud)
控制周期: 5ms (PIT_CH0) + 10ms (PIT_CH1)
路径规划: 全部在 MCU 端裸机运行 (A*/BFS/推箱子/炸弹/ID 学习)
```

### 核心文件（按修改优先级）

| 优先级 | 文件 | 行数 | 作用 |
|--------|------|------|------|
| ⭐⭐⭐ | `project/code/path_planning.c` | ~4446 | **路径规划核心**（最复杂，谨慎修改） |
| ⭐⭐⭐ | `project/code/path_planning.h` | 427 | 数据结构 + 函数声明 |
| ⭐⭐⭐ | `project/user/src/main.c` | 489 | **主入口 + 状态机 + PIT 中断控制循环** |
| ⭐⭐ | `project/code/imu963.c` | 211 | IMU 姿态估计 |
| ⭐⭐ | `project/code/encoder.c` | 151 | 编码器 + 里程计 |
| ⭐⭐ | `project/code/pid.c` | 154 | PID 控制器 |
| ⭐⭐ | `project/code/control.c` | 44 | 运动学合成 |
| ⭐⭐ | `project/code/cam_uart.c` | 240 | OpenART_Plus 双 UART 通信 |
| ⭐ | `project/code/menu.c` | 505 | TFT 菜单 + Flash 存储 |
| ⭐ | `project/code/key.c` | 45 | 4 按键 3 态去抖 |
| ⭐ | `project/code/tft180.c` | 13 | TFT180 显示屏驱动 |
| ⭐ | `project/code/motor.c` | 96 | PWM 电机驱动 |
| ⭐ | `project/user/src/isr.c` | 320 | 中断路由 |
| ⭐ | `project/code/mpu_config.c` | 122 | MPU + D-Cache（启动时自动运行） |

---

## 二、构建与烧录

| IDE | 项目文件 | 构建目标 |
|-----|---------|----------|
| Keil MDK 5.33 | `project/mdk/rt1064.uvprojx` | `nor_sdram_zf_dtcm` |

**⚠️ 无命令行构建方式。** 编译后通过 DAP-Link / J-Link 烧录。

### 关键预定义宏

```c
// 在 IDE 工程中设置（非 compile_flags.txt）
CPU_MIMXRT1064DVL6A
XIP_EXTERNAL_FLASH=1
USB_STACK_BM
MCUXPRESSO_SDK
SKIP_SYSCLK_INIT
PRINTF_FLOAT_ENABLE=1
```

### clangd 配置（`.clangd` + `compile_flags.txt`，仅用于 LSP）

由 `keil-clangd-setup` skill 从 `rt1064.uvprojx` 自动生成，合并项目自定义项后完整内容：

```c
-target arm-none-eabi -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard // 架构目标（脚本生成）
-fms-extensions -fdeclspec        // ARMCC 兼容模式（脚本生成）
-nostdlibinc                      // 禁用宿主 libc（脚本生成，自动检测 ARM GCC newlib）
-I C:/Path/arm-gcc/arm-none-eabi/include  // newlib 头文件（脚本生成）
-D__GNUC__                        // 欺骗 clangd 使用 GCC 兼容模式（自定义，在 .clangd.extra.txt）
-U_WIN32                          // 取消 Windows 宏（自定义，在 .clangd.extra.txt）
-std=c99                          // Keil AC5 默认标准（自定义，在 .clangd.extra.txt）
// 另含 14 个预定义宏 + 32 个 -I 包含路径（脚本生成）
```

**✅ 更新规则**：`keil-clangd-setup` 脚本会覆盖 `.clangd` / `compile_flags.txt`，但**项目自定义项全部放在 `.clangd.extra.txt`（每行一个 flag，`#` 为注释），脚本重跑自动合并、不再丢失**。需要新增自定义 flag 时改 `.clangd.extra.txt` 后重跑脚本即可。

### 文件添加规则

添加 `.c`/`.h` 文件时需：
1. 放入 `project/code/`（平铺，不创建子目录）
2. **在 IDE 工程树中手动添加文件**

### 唯一库包含头文件

```c
#include "zf_common_headfile.h"
// 提供: stdio, stdint, string, stdbool, math, 所有 fsl_* 外设驱动, 逐飞 API
```

---

## 三、代码布局（文件级）

### 3.1 入口与中断（`project/user/src/`）

#### `main.c` — 主入口 + 状态机 + PIT 中断（489 行）

**全局数据流（跨模块 extern 变量）：**

```
┌─ encoder.c ──→ encoder_data_FL/FR/BL/BR (extern volatile float)
├─ encoder.c ──→ x_enc, y_enc           (extern volatile float)
├─ imu963.c  ──→ yaw                     (extern volatile float)
├─ pid.c     ──→ pid_FL/FR/BL/BR/gyro/yaw/x/y (extern pid)
├─ menu.h   ──→ pid_x_p/i/d, corr_*, origin_*, x/y_acc/dec (全局变量)
└─ cam_uart.c─→ CAMDATA                   (局部结构体)
```

**状态机枚举**（`main.c:12-20`）：
```c
typedef enum { NoGame, GameMap, Waiting, Look, Run, GameOver, End } Car_State;
```

**状态转换矩阵**（`state_judgment()`, `main.c:184-271`）：

| 当前态 | 条件 | 下一态 | 关键动作 |
|--------|------|--------|---------|
| NoGame | OpenART_Plus 检测到起始区 | GameMap | `x_enc = (x_cam/x_cam_uint)×x_enc_uint` |
| GameMap | 无条件 | Run | 目标设到 `(origin_x_enc+0.5)×20, (origin_y_enc+0.5)×20` |
| Run | `|x_enc-target|≤1` | Waiting | 停止运动 (vx=vy=0) |
| Waiting | `path_process()` 完成 | Run/Look | 见下文 |
| Look | ID 扫描完成 | Run/Waiting | 调用 `cam2_process()` |
| GameOver | game_count≥3 | End | 停止 |
| GameOver | game_count<3 | NoGame | 重置系统重新开始 |

**PIT 中断**（`main.c:89-133`）：
- **CH0 (5ms)**：编码器 → IMU → 里程计 → 位置环 → 偏航环 → 运动学 → 速度环 → PWM
- **CH1 (10ms)**：读取 OpenART_Plus → 数据低通滤波

**路径规划调度**（`path_process()`, `main.c:314-431`）：

```
path_process():
  ├─ 扫描栅格: 检测 2/3/6/7 → 设置 game_mode
  ├─ game_mode 分派:
  │   0: path_start_calculation()  → Path_Start
  │   1: path_calculation()        → Path
  │   2: path_look_calculation()   → Path_Look
  │   3: path_id_calculation()     → Path
  │   4: path_boom_calculation()   → Path (含 is_push 标志)
  └─ 路径执行:
       mode 0: 逐点→最近元素→Look
       mode 1/3: process_normal_path() → 终点→GameOver
       mode 2: 逐点→is_look→Look→扫描→继续→完成→mode 3
       mode 4: 逐点→is_push→delay(1.5s)→继续→完成→map_boom_out()→mode 2
```

**ID 扫描函数**（`cam2_process()`, `main.c:432-489`）：
- 等待偏航对准（`|yaw_diff| ≤ 1°`）
- 发送角度指令 → 发送类型 → 阻塞读取 ID
- Mode 0：ID=10→Mode 1, 否则 Mode 2
- Mode 2：`id_input(id)` 记录 → 继续 Look 路径

#### `isr.c` — 外设中断处理（320 行）

见下文第四节中断路由表。

### 3.2 用户代码（`project/code/`）

#### `path_planning.h` — 路径规划头文件（427 行）

**关键常量**：
```c
#define MAP_ROWS        12          // 栅格行数
#define MAP_COLS        16          // 栅格列数
#define MAX_BOXES       8           // 最大箱子数
#define MAX_PATH_LEN    500         // 最大路径点
#define MAX_OPENSET     65536       // A* 哈希表大小
#define MAX_PQ_SIZE     10000       // 优先队列大小
#define MAX_SEARCH_CNT  65536       // A* 搜索上限
#define TURN_WEIGHT     4           // 转向惩罚
```

**核心数据结构**（`path_planning.h:79-267`）：

| 结构 | 行号 | 用途 | 内存放置 |
|------|------|------|---------|
| `Point` | 79-82 | 二维坐标 (x,y) | — |
| `PathNode` | 85-92 | A\* 节点 (pos/g/f/parent/closed) | — |
| `State` | 120-126 | A\* 状态 (player/box/target/wall_bitmap) | SDRAM |
| `HashEntry` | 129-134 | 哈希表条目 (state/g/came_from/used) | SDRAM |
| `AStarResult` | 143-149 | A\* 结果 (cost/path/success) | — |
| `Path` | 175-180 | 模式 1/3 路径 (x/y/len/is_push) | — |
| `Path_Look` | 196-203 | 模式 2 Look 路径 (x/y/len/angle/type/is_look) | — |
| `Path_Start` | 165-171 | 模式 0 路径 (x/y/len/angle/type) | — |
| `DetonatePlan` | 232-237 | 引爆炸弹计划 | — |
| `BombExecutionPlan` | 261-266 | 完整炸弹执行方案 | — |

**外部接口函数**（`path_planning.h:419-426`）：
- `path_start_calculation(map)` → `Path_Start`（模式 0）
- `path_calculation(map)` → `Path`（模式 1，盲推）
- `path_look_calculation(map)` → `Path_Look`（模式 2，ID 扫描）
- `id_input(id)` — 输入 ID 数据
- `path_id_calculation()` → `Path`（模式 3，ID 推箱）
- `path_boom_calculation(map)` → `Path`（模式 4，炸弹）
- `map_boom_out(map)` — 更新炸弹后的栅格
- `reset_planning_system()` — 重置所有规划状态

#### `control.h` — 运动学（6 行）

```c
void motor_solution(float vx_field, float vy_field, float current_yaw, float wz);
```

#### `pid.h` — PID 定义（33 行）

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

**函数列表**：

| 函数 | 功能 | 调用位置 |
|------|------|---------|
| `PID_init(p, kp, ki, kd, maxI, maxO, target, dead)` | 初始化单个 PID | `pid_init()` |
| `pid_init()` | 初始化 7 个 PID 实例 | `Init()` |
| `pid_increm(p, now)` | 增量式 PID | `main.c:125-128`（每 5ms） |
| `pid_location(p, now)` | 位置式 PID | `main.c:103-104,118`（每 10ms） |
| `pid_target(p, target)` | 设置目标值 | — |
| `pid_wheel_target(FL,FR,BL,BR)` | 设置四轮目标 | `control.c:42` |
| `pid_position_target(x,y)` | 设置位置目标 | `main.c:101` |
| `pid_position_speed()` | 终点冲刺（提速） | `main.c:300` |
| `pid_change(p, kp, ki, kd)` | 运行时改参数 | `main.c:110-111` |
| `pid_reset()` | 完全重置 | `main.c:250` |

#### `motor.h` — 电机（16 行）

```c
void motor_init(void);
void motor_duty(int FL, int FR, int BL, int BR);  // 值域 [-8000, 8000]
```

#### `encoder.h` — 编码器（13 行）

```c
extern volatile float x_enc, y_enc;  // 世界坐标系位置 (cm)
void encoder_init(void);
void encoder_get(void);
void distance(float yaw);
```

#### `imu963.h` — IMU（22 行）

```c
typedef struct { float w, x, y, z; } Quaternion;
typedef struct { float roll, pitch, yaw; } EulerAngle;
void imu_init(void);
void imu_get(void);
extern volatile float yaw;  // 偏航角 (-180°~180°)
```

#### `cam_uart.h` — 摄像头通信（18 行）

```c
typedef struct {
    float car_cx, car_cy;          // 小车位置
    uint8_t grid[12][16];          // 栅格地图
} CAMDATA;

void cam_uart_init(void);
void cam_uart_isc_1(void);         // UART1 ISR (LPUART3, OpenART_Plus)
void cam_uart_isc_2(void);         // UART2 ISR (LPUART1, ID)
CAMDATA cam_uart1_read(void);
void cam1_uart_send(float angle);  // 0→01, 90→02, -90→03, 180→04
int cam_uart2_read(void);          // 返回两位数 ID, -1=无数据
void cam_uart2_write(uint8_t id);
```

#### `menu.h` — 菜单全局变量（35 行）

```c
// PID 参数
extern float   pid_x_p, pid_x_i, pid_x_d;
extern uint8_t pid_x_speed;       // 默认 110
extern float   pid_y_p, pid_y_i, pid_y_d;
extern uint8_t pid_y_speed;       // 默认 110

// 原点
extern uint8_t origin_x_enc, origin_y_enc;  // 默认 (1, 7)

// 修正系数
extern float  corr_x_enc, corr_y_enc;       // 默认 0.760, 0.795
extern int8_t corr_x_cam, corr_y_cam;       // 默认 0, 1
extern float  corr_yaw;                     // 默认 0.0

// 加减速
extern float x_acc, x_dec, y_acc, y_dec;    // 默认 0.01, 0.06, 0.01, 0.06
```

#### `key.h` — 按键（29 行）

```c
struct keys {
    char  value;        // 状态: IDLE=0, PRESSED=1, DONE=2
    bool  key_sta;      // 电平: 0=按下, 1=释放
    int   time, double_time;
    bool  single_flag;  // 单次事件标志
};
extern struct keys key[4];  // [0]=C30(左), [1]=C29(右), [2]=C31(下), [3]=C28(上)
```

#### `wireless_uart.h` — 无线示波器（9 行）

```c
void my_uart_init(void);
void my_uart_write(int num, float data);  // channel 0-7
void my_uart_send(int sum);               // 发送 sum 个通道
```

**⚠️ 注意**：`wireless_uart.h` 头文件保护宏是 `_CODE_MY_UART_h_`（与文件名不一致）。

#### `tft180.c/h` — TFT180 显示屏（13 行）

竖屏 (PORTRAIT)，8×16 字体，SPI 模式，RGB565 白底黑字。仅 `tft180_clear()` 一个接口，被 `menu.c` 和启动流程使用。

#### `key.c` — 按键驱动（45 行）

3 态去抖状态机：`IDLE → PRESSED → DONE`，4 键上拉输入。
`key[0]=C30(左)`, `key[1]=C29(右)`, `key[2]=C31(下)`, `key[3]=C28(上)`。
**注意：** 菜单中按键映射与物理引脚命名有差异（`KEY_UP=key[2]`=C31=下按键，`KEY_DOWN=key[3]`=C28=上按键，见 `menu.c:46-52`）。

#### `mpu_config.c` — MPU 配置（122 行）

```c
void SystemInitHook(void);  // SDK 弱函数覆盖，main() 前自动调用
```

---

## 四、中断路由表（易错点，高优先级）

**所有 ISR 位于 `project/user/src/isr.c`**（SDK 标准向量名），内部再调用用户函数。PIT 中断在 `main.c` 中。

| 外设 | 引脚 | 用途 | ISR 向量（isr.c 行号） | 内部调用 |
|------|------|------|----------------------|---------|
| PIT_CH0/1 | — | **5ms+10ms 控制循环** | `PIT_IRQHandler()` (`main.c:89`) | — |
| CSI | — | 摄像头并行接口（未用） | `CSI_IRQHandler` (43) | `CSI_DriverIRQHandler()`（SDK 内部） |
| LPUART1 | B12/B13 | ID 通信/调试 | `LPUART1_IRQHandler` (49) | `cam_uart_isc_2()` |
| LPUART2 | — | **未使用（空 handler）** | `LPUART2_IRQHandler` (62) | — |
| LPUART3 | B22/B23 | **OpenART_Plus 主摄像头** | `LPUART3_IRQHandler` (73) | `cam_uart_isc_1()` |
| LPUART4 | — | FlexIO 摄像头 + GNSS | `LPUART4_IRQHandler` (86) | `flexio_camera_uart_handler()` + `gnss_uart_callback()` |
| LPUART5 | — | 备用摄像头 | `LPUART5_IRQHandler` (99) | `camera_uart_handler()` |
| LPUART6 | — | **未使用** | `LPUART6_IRQHandler` (110) | — |
| LPUART8 | D16/D17 | BLE 无线 + 调试中断 | `LPUART8_IRQHandler` (122) | `wireless_module_uart_handler()` + `debug_interrupr_handler()` |
| GPIO1_0_15 | B0 | GPIO 外部中断 | `GPIO1_Combined_0_15_IRQHandler` (138) | EXTI flag clear |
| GPIO1_16_31 | B16 | 无线 SPI | `GPIO1_Combined_16_31_IRQHandler` (148) | `wireless_module_spi_handler()` + EXTI clear |
| GPIO2_0_15 | C0 | FlexIO 帧同步 | `GPIO2_Combined_0_15_IRQHandler` (159) | `flexio_camera_vsync_handler()` + EXTI clear |
| GPIO2_16_31 | C16 | **ToF 传感器** | `GPIO2_Combined_16_31_IRQHandler` (170) | `tof_module_exti_handler()` + EXTI clear |
| GPIO3_0_15 | D4 | GPIO 外部中断 | `GPIO3_Combined_0_15_IRQHandler` (186) | EXTI flag clear |

**⚠️ 注意**：
- `isr.c:129` 函数名 `debug_interrupr_handler` 拼写错误（应为 `debug_interrupt_handler`），且仅在 `DEBUG_UART_USE_INTERRUPT` 宏下编译
- LPUART2/6 的 handler 为空，仅清除溢出标志
- 所有 GPIO ISR 仅清除 EXTI 标志，无实际业务逻辑

---

## 五、内存模型（精确到链接脚本）

### 5.1 物理内存分配

| 区域 | 起始 | 大小 | 速度 | MPU Region | 用途 |
|------|------|------|------|-----------|------|
| ITCM | 0x00000000 | 64KB | 1-cycle | 2 (Normal) | 时序关键函数 |
| DTCM | 0x20000000 | 448KB+32KB(stack)+1KB(heap) | 0-wait | 3 (Normal) | BFS 距离图/访问标记/障碍物 |
| OCRAM | 0x20200000 | 512KB | 2-3 cycle | 4 (Normal) | 备选缓冲区 |
| FlexSPI Flash | 0x70000000 | 4MB | XIP | 5 (Device RO) | 代码 + 只读数据 |
| SDRAM (cache) | 0x80000000 | ~30MB | D-Cache WB/WA | 0 (Normal WB/WA) | 哈希表/优先队列 |
| SDRAM (ncache) | 0x81E00000 | 2MB | Device | 1 (Device nGnRnE) | DMA 缓冲区/帧缓冲 |

### 5.2 链接脚本自定义段

| 段名 | 内存区域 | 变量/函数 | 行号 (scf) |
|------|---------|-----------|-----------|
| `SDRAM_ZI` (UNINIT) | SDRAM cacheable | `hash_table[65536]`, `pq[MAX_PQ_SIZE]` | `scf:153-155` |
| `ITCM_NonCacheable` | ITCM | 13 个时序函数 | `scf:173-176` |
| `OCRAM_CACHE` | OCRAM | 备选缓冲区 | `scf:179-182` |
| `SDRAM_NonCacheable` | SDRAM ncache | DMA 缓冲区 | `scf:185-188` |

### 5.3 ITCM 函数列表

以下函数通过 `__attribute__((section("ITCM_NonCacheable")))` 放置在 ITCM：

```
manhattan_distance          (path_planning.c)
is_corner_deadlock          (path_planning.c)
is_soft_corner_deadlock     (path_planning.c)
hash_lookup_insert          (path_planning.c)
pq_push                     (path_planning.c)
pq_pop                      (path_planning.c)
heuristic                   (path_planning.c)
get_successors              (path_planning.c)
bfs_compute_distances       (path_planning.c)
bfs_compute_reachability    (path_planning.c)
simple_astar                (path_planning.c)
solve_single_box_a_star     (path_planning.c)
compute_player_region_with_walls (path_planning.c)
```

---

## 六、PID 系统（8 个全局实例）

### 6.1 实例列表（初始化参数见 `pid.c:99-107`）

| 变量名 | 类型 | kp | ki | kd | maxI | maxOut | 用途 |
|--------|------|-----|-----|-----|------|--------|------|
| `pid_FL` | 增量式 | 60 | 10 | 50 | 1000 | 8000 | 左前轮速度环 |
| `pid_FR` | 增量式 | 60 | 10 | 50 | 1000 | 8000 | 右前轮速度环 |
| `pid_BL` | 增量式 | 60 | 10 | 50 | 1000 | 8000 | 左后轮速度环 |
| `pid_BR` | 增量式 | 60 | 10 | 50 | 1000 | 8000 | 右后轮速度环 |
| `pid_yaw` | 位置式 | 5/10 | 0.001 | 30 | 1000 | 70 | 偏航角闭环 |
| `pid_x` | 位置式 | 菜单可调 | 菜单可调 | 菜单可调 | 500 | 110 (pid_x_speed) | X 位置环 |
| `pid_y` | 位置式 | 菜单可调 | 菜单可调 | 菜单可调 | 500 | 110 (pid_y_speed) | Y 位置环 |
| `pid_gyro` | 位置式 | — | — | — | — | — | **未初始化**（`pid_init()` 未调用，不可用） |

### 6.2 控制循环

```
5ms:
  位置环 (每 10ms):  vx = pid_location(&pid_x, x_enc)
                      vy = pid_location(&pid_y, y_enc)
  偏航环 (每 10ms):  vz = pid_location(&pid_yaw, yaw)
  运动学:            motor_solution(vx, vy, yaw, vz)
  速度环 (每 5ms):   FL = pid_increm(&pid_FL, encoder_data_FL/4.0)
```

### 6.3 运行时参数切换

```c
// Run 状态: 快速响应 (main.c:110)
pid_change(&pid_yaw, 10, 0.001f, 30);
// 其他状态: 平稳 (main.c:111)
pid_change(&pid_yaw, 5, 0.001f, 30);
// 偏航角误差 ±180° 归一化 (main.c:113-115)
if (yaw_error > 180.0f) yaw_error -= 360.0f;
else if (yaw_error < -180.0f) yaw_error += 360.0f;
// 终点冲刺: maxOutput→130 (main.c:300, GameOver 冲线提速)
pid_position_speed();
```

---

## 七、路径规划系统（path_planning.c）

### 7.1 函数调用层级

```
path_start_calculation()         ── 模式 0
  └─ find_nearest_approach()     ── 找最近元素
      └─ bfs_compute_distances()
      └─ simple_astar()
  └─ extract_start_turn_points()

path_calculation()               ── 模式 1（盲推）
  └─ generate_greedy_pairing()   ── 贪心分配
  └─ validate_solution()
      └─ backtrack_validate()    ── 回溯验证
          └─ solve_single_box_a_star()  ── A* 推箱

path_look_calculation()          ── 模式 2（ID 扫描）
  └─ id_learning()               ── BFS 规划访问路径

path_id_calculation()            ── 模式 3（ID 推箱）
  └─ id_inference()              ── 频率均衡 + 排列枚举
  └─ build_solution_from_id_pairing()
  └─ path_id_calculate()

path_boom_calculation()          ── 模式 4（炸弹）
  └─ detect_and_generate_problems()
  └─ iterative_bomb_breakthrough()
  └─ search_multi_bomb_combination()
  └─ plan_bomb_execution_sequence()

map_boom_out()                   ── 炸弹后更新栅格
```

### 7.2 内存消耗

| 结构 | 位置 | 大小 | 说明 |
|------|------|------|------|
| `hash_table` | SDRAM | 65536 × sizeof(HashEntry) | A\* 状态去重 |
| `pq` (优先队列) | SDRAM | 10000 × sizeof(PQNode) | A\* 开放集 |
| `g_dist_map` | DTCM | 12×16 = 192 int16_t | BFS 距离场 |
| `g_bfs_visited` | DTCM | epoch 标记 | 避免 memset |
| `g_obs_buf` | DTCM | 12×uint16_t | 障碍物位图 |
| `g_bfs_queue` | DTCM | MAX_QUEUE | BFS 队列 |
| 路径缓冲区 | — | 500 × Point × 2 | A\* 路径 + 完整路径 |
| BFS 模拟队列 | DTCM | 16384 | 炸弹规划 |

---

## 八、OpenART_Plus 通信协议

### 8.1 硬件通道

| 通道 | UART 外设 | TX/RX | 方向 | 数据 |
|------|----------|-------|------|------|
| UART1 (cam_uart.c) | LPUART3 | B22/B23 | 双向 | 栅格 + 小车坐标 ←; 角度指令 → |
| UART2 (cam_uart.c) | LPUART1 | B12/B13 | 双向 | ID 请求（发送类型）→; ID 返回 ← |

### 8.2 帧格式

```
统一帧: start<data>end\r\n
FIFO 缓冲: 512 字节, ISR 逐字节写入, 主循环提取帧
```

### 8.3 下行（MCU→OpenART_Plus）格式

```c
// 角度指令 (cam1_uart_send, cam_uart.c:166-180)
0°     → "start01end\r\n"
90°    → "start02end\r\n"
-90°   → "start03end\r\n"
180°   → "start04end\r\n"

// ID 请求 (cam_uart2_write, cam_uart.c:235-242)
"start%02dend\r\n"  // 如 type=2 → "start02end\r\n"
```

### 8.4 上行（OpenART_Plus→MCU）格式

```c
// 小车位置 (push_cam_data, cam_uart.c:47-86)
"car:%.1f,%.1f;"   → cam_uart_data.car_cx, car_cy

// 栅格数据 (push_cam_data)
"map:0123456789..." → cam_uart_data.grid[12][16]

// ID 返回 (cam_uart2_read, cam_uart.c:183-233)
"startNNend\r\n"    → 两位十进制数, 如 "start15end\r\n" → 15
```

### 8.5 角度指令编码

```c
if (angle == 0.0f)    tx = 1;
if (angle == 90.0f)   tx = 2;
if (angle == -90.0f)  tx = 3;
if (angle == 180.0f)  tx = 4;
else                   tx = 1;  // 默认 0°
```

---

## 九、全局调参变量

所有参数在 `menu.c` 中定义默认值，通过 TFT 菜单调节后写入 Flash。

| 变量 | 默认值 | 菜单步进 | 类型 | 说明 |
|------|--------|---------|------|------|
| `pid_x_p` | 3.60 | 0.1 | float | X 位置环 P |
| `pid_x_i` | 0.003 | 0.001 | float | X 位置环 I |
| `pid_x_d` | 7.50 | 0.1 | float | X 位置环 D |
| `pid_x_speed` | 110 | 5 | uint8_t | X 最大速度 |
| `pid_y_p` | -3.60 | 0.1 | float | Y 位置环 P（负值） |
| `pid_y_i` | -0.003 | 0.001 | float | Y 位置环 I |
| `pid_y_d` | -7.50 | 0.1 | float | Y 位置环 D |
| `pid_y_speed` | 110 | 5 | uint8_t | Y 最大速度 |
| `origin_x_enc` | 1 | 1 | uint8_t | 起点栅格 X |
| `origin_y_enc` | 7 | 1 | uint8_t | 起点栅格 Y |
| `corr_x_enc` | 0.760 | 0.005 | float | X 里程计修正 |
| `corr_y_enc` | 0.795 | 0.005 | float | Y 里程计修正 |
| `corr_x_cam` | 0 | 1 | int8_t | X 摄像头偏置 |
| `corr_y_cam` | 1 | 1 | int8_t | Y 摄像头偏置 |
| `corr_yaw` | 0.0 | 0.000001 | float | 偏航角修正 |
| `x_acc` | 0.01 | 0.005 | float | X 加速系数 |
| `x_dec` | 0.06 | 0.005 | float | X 减速系数 |
| `y_acc` | 0.01 | 0.005 | float | Y 加速系数 |
| `y_dec` | 0.06 | 0.005 | float | Y 减速系数 |

Flash 存储：`Sector 127, Page 3`（共 19 个参数映射到 `flash_union_buffer[0..18]`）。

---

## 十、已知问题（AI 修改时特别注意）

### 10.1 代码质量问题

| # | 问题 | 文件 | 行号 | 说明 |
|---|------|------|------|------|
| 1 | ITCM MPU 属性待验证 | `mpu_config.c` | Region 2 | XN 位配置与 ITCM 可执行 section 的设计意图矛盾，需用 Keil 镜像和硬件确认 |
| 2 | `isr.h` 为空 | `project/user/inc/isr.h` | — | 无任何内容，仅占位 |
| 3 | 路径边界访问风险 | `path_planning.c` | `get_successors()` | 边界邻居可能参与目标位图索引，需先验证行列范围 |
| 4 | 头文件保护宏与文件名不一致 | `wireless_uart.h` | 2 | 保护宏 `_CODE_MY_UART_h_` 但文件为 `wireless_uart.h` |
| 5 | 函数名拼写错误 | `isr.c` | 128 | `debug_interrupr_handler` 多了一个 'r' |
| 6 | 按键映射命名混乱 | `menu.c` | 46-52 | `KEY_UP=key[2]=C31(下)`, `KEY_DOWN=key[3]=C28(上)` |
| 7 | 阻塞等待无超时 | `main.c` | `cam2_process()` | `while (id == -1)` 可能因摄像头或串口故障永久阻塞 |
| 8 | FIFO 满载与不安全格式化 | `cam_uart.c` | `cam_uart_isc_*()`、发送函数 | FIFO 写入返回值未处理；发送使用 `sprintf()`，应增加边界和丢帧策略 |

### 10.2 架构约束

| # | 约束 | 说明 |
|---|------|------|
| 1 | **请勿修改 `libraries/`** | 逐飞开源库 V3.9.2 + NXP SDK 2.12，所有修改在 `project/code/` 中 |
| 2 | **裸机无 OS** | 所有实时控制在 PIT 中断中（5ms 硬实时），`main()` 仅做状态调度 |
| 3 | **D-Cache 兼容性** | SDRAM Region 0 有 Cache，DMA 缓冲区必须在 Region 1（Device 类型），否则 cache coherence 问题 |
| 4 | **路径规划耗时** | 推箱子 A\* 搜索上限 50000 步，可能在 Waiting 状态持续多个控制周期 |
| 5 | **无测试框架** | 目前没有主机单元测试；应补充帧解析、路径边界、ID 超时和 FIFO 满载测试，至少保留可复现的静态或硬件验收步骤 |

---

## 十一、硬件引脚分配（完整版）

### 摄像头（OpenART_Plus 通过 UART 通信）

```
主摄像头:  TX=B22 (LPUART3_TX), RX=B23 (LPUART3_RX), 115200 baud
副摄像头:  TX=B12 (LPUART1_TX), RX=B13 (LPUART1_RX), 115200 baud
FlexIO 摄像头: TX=C17, RX=C16, VSY=C7, HREF=C6, PCLK=C5, D0-D7=C8-C15
```

**⚠️ C4-C15 避免用作其他输入**（FlexIO 摄像头占用）。

### 电机 PWM（PWM1/PWM2, 10kHz, MAX_DUTY=8000）

| 电机 | 正转 PWM | 反转 PWM | 编码器 A 相 | 编码器 B 相 |
|------|----------|----------|------------|------------|
| FL | D13 (PWM1_CH0_B) | D12 (PWM1_CH0_A) | C0 (QTIMER1_CH1) | C1 (QTIMER1_CH2) |
| FR | D15 (PWM1_CH1_B) | D14 (PWM1_CH1_A) | C2 (QTIMER1_CH1) | C24 (QTIMER1_CH2) |
| BL | D1 (PWM1_CH3_B) | D0 (PWM1_CH3_A) | C3 (QTIMER2_CH1) | C25 (QTIMER2_CH2) |
| BR | D3 (PWM2_CH3_B) | D2 (PWM2_CH3_A) | B18 (QTIMER3_CH1) | B19 (QTIMER3_CH2) |

### 其他引脚

```
TFT 显示屏:  SCK=B0 (SPI), MOSI=B1, CS=B3
舵机:        C30, C31
BLE 蓝牙:    TX=D16 (LPUART8_RX), RX=D17 (LPUART8_TX)
按键:        C30(左), C29(右), C31(下), C28(上) — 上拉输入
```

---

## 十二、编写规则

- 所有代码为 **C99**，注意 ARMCC 编译器扩展（`__attribute__`、内联汇编等）
- **始终尊重 `libraries/` 边界** — 逐飞库 + NXP SDK 代码无需修改
- 调试手段有限：TFT 显示 + 串口输出（SeekFree 无线示波器模式）
- 注释风格：中英文混用，函数头部无统一格式
- 变量命名：snake_case（C 风格）
- 静态分析：使用 clangd LSP（`compile_flags.txt` + `.clangd`）做代码检查

---

## 十三、Git 提交规范

```bash
<类型>: <描述>   # 使用中文
# 类型: feat / fix / refactor / docs / style / chore
# 示例: fix: 修复炸弹规划中墙面更新后可达性计算错误
```

不接受的提交信息：`update`、`fix bug`、`修改`。
