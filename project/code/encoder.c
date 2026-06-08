#include "encoder.h"
#include "zf_common_headfile.h"
#include <math.h>

// 引用 imu963.c 中的全局变量 (加速度 cm/s?, 航向角 度)
extern float acc_x, acc_y, acc_z;

// ================= 硬件引脚定义 =================
// 左前轮 (FL) - QTIMER1 Encoder1
#define ENCODER_FL       (QTIMER1_ENCODER1)
#define ENCODER_FL_A     (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_FL_B     (QTIMER1_ENCODER1_CH2_C1)

// 右前轮 (FR) - QTIMER1 Encoder2
#define ENCODER_FR       (QTIMER1_ENCODER2)
#define ENCODER_FR_A     (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_FR_B     (QTIMER1_ENCODER2_CH2_C24)

// 左后轮 (BL) - QTIMER2 Encoder1
#define ENCODER_BL       (QTIMER2_ENCODER1)
#define ENCODER_BL_A     (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_BL_B     (QTIMER2_ENCODER1_CH2_C25)

// 右后轮 (BR) - QTIMER3 Encoder2
#define ENCODER_BR       (QTIMER3_ENCODER2)
#define ENCODER_BR_A     (QTIMER3_ENCODER2_CH1_B18)
#define ENCODER_BR_B     (QTIMER3_ENCODER2_CH2_B19)


#define N    4096                        // 编码器每圈脉冲数 (CPR * 4倍频)
// ================= 机械参数 (根据实际齿轮确认) =================
#define GEAR_ENCODER    30              // 编码器齿轮齿数
#define GEAR_WHEEL      70              // 轮子大齿轮齿数
#define B               (GEAR_ENCODER / (float)GEAR_WHEEL)  // = 30/70 = 3/7
#define C               (2 * PI * 3)                // 轮子周长 (2 * PI * 半径3.0cm)

// ================= 全局变量 =================
float x_enc = 0, y_enc = 0;             // 编码器全局累积位移 (单位: cm)
float x_imu = 0, y_imu = 0;             // IMU 积分全局位置 (单位: cm)
// dc = 编码器每1个脉冲对应的轮子位移
//     编码器1脉冲 → 编码器转 1/N 圈 → 轮子转 B/N 圈 → 位移 C*B/N
static float dc = C / (B * N);                  // (cm/脉冲)

// 原始编码器数据 (中间值，每次读取后清零)
int encoder_data_FL = 0;
int encoder_data_FR = 0;
int encoder_data_BL = 0;
int encoder_data_BR = 0;

// 一阶低通滤波历史数据（float 保留小数，防止截断累积误差）
static float encoder_data_FL_last = 0;
static float encoder_data_FR_last = 0;
static float encoder_data_BL_last = 0;
static float encoder_data_BR_last = 0;

// 航迹推算用快照（float，保留滤波后的全部精度）
static float encoder_data_FL_enc = 0;
static float encoder_data_FR_enc = 0;
static float encoder_data_BL_enc = 0;
static float encoder_data_BR_enc = 0;

// ================= 函数实现 =================

// 初始化四个轮子的编码器接口
void encoder_init(void){
    encoder_quad_init(ENCODER_FL, ENCODER_FL_A, ENCODER_FL_B);
    encoder_quad_init(ENCODER_FR, ENCODER_FR_A, ENCODER_FR_B);
    encoder_quad_init(ENCODER_BL, ENCODER_BL_A, ENCODER_BL_B);
    encoder_quad_init(ENCODER_BR, ENCODER_BR_A, ENCODER_BR_B);
}

// 获取编码器原始数据，并进行滤波处理
void encoder_get(void){
    // 1. 获取硬件原始增量值
    int raw_FL =  encoder_get_count(ENCODER_FL);
    int raw_FR = -encoder_get_count(ENCODER_FR);
    int raw_BL =  encoder_get_count(ENCODER_BL);
    int raw_BR = -encoder_get_count(ENCODER_BR);

    // 2. 一阶低通滤波 (Alpha = 0.8)，浮点运算保留精度
    float filt_FL = 0.8f * raw_FL + 0.2f * encoder_data_FL_last;
    float filt_FR = 0.8f * raw_FR + 0.2f * encoder_data_FR_last;
    float filt_BL = 0.8f * raw_BL + 0.2f * encoder_data_BL_last;
    float filt_BR = 0.8f * raw_BR + 0.2f * encoder_data_BR_last;

    // 3. 保存滤波历史（float 全精度，避免截断累积）
    encoder_data_FL_last = filt_FL;
    encoder_data_FR_last = filt_FR;
    encoder_data_BL_last = filt_BL;
    encoder_data_BR_last = filt_BR;

    // 4. 航迹推算快照：保留 float 全精度，不做 int 截断
    encoder_data_FL_enc = filt_FL;
    encoder_data_FR_enc = filt_FR;
    encoder_data_BL_enc = filt_BL;
    encoder_data_BR_enc = filt_BR;

    // 5. 对外输出截断 int（供 PID 速度环使用，截断误差在此隔离）
    encoder_data_FL = (int)filt_FL;
    encoder_data_FR = (int)filt_FR;
    encoder_data_BL = (int)filt_BL;
    encoder_data_BR = (int)filt_BR;
    
    // 6. 清除硬件计数器，以便下次读取的是增量值
    encoder_clear_count(ENCODER_FL);
    encoder_clear_count(ENCODER_FR);
    encoder_clear_count(ENCODER_BL);
    encoder_clear_count(ENCODER_BR);
}

// ================= 纯编码器里程计解算 =================
void distance(){
    
    // ========== 1. 车体系位移增量（脉冲/10ms）==========
    // 标准麦克纳姆轮正向运动学
    float dx = ( encoder_data_FL_enc + encoder_data_FR_enc + encoder_data_BL_enc + encoder_data_BR_enc) / 4.0f;
    float dy = ( encoder_data_FL_enc - encoder_data_FR_enc - encoder_data_BL_enc + encoder_data_BR_enc) / 4.0f;
    
    // dx, dy 是车体 x/y 方向的脉冲增量（相对于车体坐标系）
    
    // ========== 2. 脉冲 → 厘米（每10ms周期的位移）==========
    float shift_x = dx * dc;    // 车体x方向位移 (cm)
    float shift_y = dy * dc;    // 车体y方向位移 (cm)
    
    // ========== 3. 累加到全局位置 ==========
    x_enc += shift_x * corr_x_enc;
    y_enc += shift_y * corr_y_enc;
}

float x_a = 0,y_a = 0;

// ================= IMU 加速度二次积分里程计 =================
// 基于 IMU 加速度计数据二次积分计算世界坐标系位移
// 调用频率: 与 imu_get() 一致 (5ms 周期)，在 distance() 之后调用
// 依赖: imu963.c 中的 acc_x, acc_y (cm/s?)
// 注意: 纯二次积分不做速度衰减！静止时由编码器辅助置零速度来抑制漂移
//       麦克纳姆轮振动 (30-80Hz) 通过二级低通 + 编码器互补滤波联合抑制
void imu_distance(void) {
    static const float dt = 0.005f;           // 5ms 积分周期 (与 imu_get 一致)
    static float vel_x_imu = 0, vel_y_imu = 0; // IMU 积分速度 (cm/s)

    // ========== 1. 静止检测（编码器辅助零速更新 ZUPT）==========
    float enc_max = fabsf(encoder_data_FL_enc);
    if (fabsf(encoder_data_FR_enc) > enc_max) enc_max = fabsf(encoder_data_FR_enc);
    if (fabsf(encoder_data_BL_enc) > enc_max) enc_max = fabsf(encoder_data_BL_enc);
    if (fabsf(encoder_data_BR_enc) > enc_max) enc_max = fabsf(encoder_data_BR_enc);
    if (enc_max < 5.0f) {
        vel_x_imu = 0;
        vel_y_imu = 0;
        return;
    }

    // ========== 2. 二级低通滤波（专门抑制麦克纳姆轮 30-80Hz 振动）==========
    static float acc_x_lp2 = 0, acc_y_lp2 = 0;
    acc_x_lp2 = acc_x * 0.1f + acc_x_lp2 * 0.9f;  // fc ≈ 3.4 Hz
    acc_y_lp2 = acc_y * 0.1f + acc_y_lp2 * 0.9f;

    // ========== 3. 加速度死区 ==========
    float acc_w_x = acc_x_lp2;
    float acc_w_y = acc_y_lp2;
    x_a = acc_w_x;  // 调试用
    y_a = acc_w_y;
    if (fabsf(acc_w_x) < 1.0f) acc_w_x = 0;
    if (fabsf(acc_w_y) < 1.0f) acc_w_y = 0;

    // ========== 4. 速度积分（欧拉积分，无衰减）==========
    vel_x_imu += acc_w_x * dt;
    vel_y_imu += acc_w_y * dt;

    // ========== 5. 编码器-IMU 互补滤波（用编码器速度抑制振动漂移）==========
    // 编码器不受振动干扰, 以 0.5%/周期的速率缓慢拉回 IMU 速度
    // 时间常数 ≈ 1s: IMU 保留快速瞬态响应, 编码器提供长期基准
    float vel_enc_x = encoder_data_FL_enc + encoder_data_FR_enc 
                    + encoder_data_BL_enc + encoder_data_BR_enc;
    float vel_enc_y = encoder_data_FL_enc - encoder_data_FR_enc 
                    - encoder_data_BL_enc + encoder_data_BR_enc;
    vel_enc_x = vel_enc_x / 4.0f * dc / dt * corr_x_enc;  // 编码器 X 速度 (cm/s)
    vel_enc_y = vel_enc_y / 4.0f * dc / dt * corr_y_enc;  // 编码器 Y 速度 (cm/s)

    vel_x_imu = vel_x_imu * 0.95f + vel_enc_x * 0.05f;
    vel_y_imu = vel_y_imu * 0.95f + vel_enc_y * 0.05f;

    // ========== 6. 位置积分 ==========
    x_imu += vel_x_imu * dt;
    y_imu += vel_y_imu * dt;
}
