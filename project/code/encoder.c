#include "encoder.h"
#include "zf_common_headfile.h"
#include <math.h>

// 引用 imu963.c 中的全局变量 (加速度 cm/s, 航向角 度)
extern float acc_x, acc_y, acc_z;

// ================= 硬件引脚定义 =================
// 左前轮 (FL)
#define ENCODER_FL       (QTIMER1_ENCODER1)
#define ENCODER_FL_A     (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_FL_B     (QTIMER1_ENCODER1_CH2_C1)

// 右前轮 (FR)
#define ENCODER_FR       (QTIMER1_ENCODER2)
#define ENCODER_FR_A     (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_FR_B     (QTIMER1_ENCODER2_CH2_C24)

// 左后轮 (BL)
#define ENCODER_BL       (QTIMER2_ENCODER1)
#define ENCODER_BL_A     (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_BL_B     (QTIMER2_ENCODER1_CH2_C25)

// 右后轮 (BR)
#define ENCODER_BR       (QTIMER3_ENCODER2)
#define ENCODER_BR_A     (QTIMER3_ENCODER2_CH1_B18)
#define ENCODER_BR_B     (QTIMER3_ENCODER2_CH2_B19)


#define N    4096                       // 编码器每圈脉冲数 (CPR * 4倍频)
// ================= 机械参数 (根据实际齿轮确认) =================
#define GEAR_ENCODER    30              // 编码器齿轮齿数
#define GEAR_WHEEL      70              // 轮子大齿轮齿数
#define B               (GEAR_ENCODER / (float)GEAR_WHEEL)  // = 30/70 = 3/7
#define C               (2 * PI * 3)                        // 轮子周长 (2 * PI * 半径3.0cm)

// ================= 滤波与算法参数 =================
#define ENC_FILTER_ALPHA        0.8f    // 编码器一阶低通滤波系数
#define ENC_FILTER_BETA         0.2f    // 编码器一阶低通滤波历史权重 (1 - alpha)
#define IMU_ACC_ALPHA           0.1f    // IMU 加速度二级低通滤波系数
#define IMU_ACC_BETA            0.9f    // IMU 加速度二级低通滤波历史权重 (1 - alpha)
#define IMU_COMPLEMENTARY_ALPHA 0.5f    // IMU-编码器互补滤波融合系数
#define ZUPT_THRESHOLD          5.0f    // 零速更新检测阈值 (脉冲/周期)
#define ACC_DEAD_ZONE           1.0f    // 加速度死区阈值 (cm/s?)
#define YAW_AXIS_THRESHOLD      3.0f    // 航向角接近坐标轴判定阈值 (度)

// ================= 全局变量 =================
float x_enc = 0, y_enc = 0;             // 编码器全局累积位移 (单位: cm)
float x_imu = 0, y_imu = 0;             // IMU 积分全局位置 (单位: cm)
static float dc = C / (B * N);          // 脉冲→厘米转换系数 (cm/脉冲)
static float dc_dt = 0;                 // 预计算: dc / dt, 用于编码器速度换算 (在 encoder_init 中初始化)

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

/**
 * @brief 初始化四个轮子的编码器接口
 * 
 * 配置并启动左前、右前、左后、右后四个轮子的正交编码器硬件接口。
 */
void encoder_init(void){
    encoder_quad_init(ENCODER_FL, ENCODER_FL_A, ENCODER_FL_B);
    encoder_quad_init(ENCODER_FR, ENCODER_FR_A, ENCODER_FR_B);
    encoder_quad_init(ENCODER_BL, ENCODER_BL_A, ENCODER_BL_B);
    encoder_quad_init(ENCODER_BR, ENCODER_BR_A, ENCODER_BR_B);
    dc_dt = dc / 0.005f;  // 预计算 dc/dt，避免每次 imu_distance 中重复计算
}

/**
 * @brief 获取编码器原始数据，并进行滤波处理
 * 
 * 读取四个轮编码器的硬件计数增量，应用一阶低通滤波以平滑噪声，
 * 并将结果分别保存为高精度浮点快照（用于里程计解算）和整数形式（用于速度环PID）。
 * 最后清除硬件计数器以准备下一次增量读取。
 */
void encoder_get(void){
    // 1. 获取硬件原始增量值
    int raw_FL =  encoder_get_count(ENCODER_FL);
    int raw_FR = -encoder_get_count(ENCODER_FR);
    int raw_BL =  encoder_get_count(ENCODER_BL);
    int raw_BR = -encoder_get_count(ENCODER_BR);

    // 2. 一阶低通滤波，浮点运算保留精度
    float filt_FL = ENC_FILTER_ALPHA * raw_FL + ENC_FILTER_BETA * encoder_data_FL_last;
    float filt_FR = ENC_FILTER_ALPHA * raw_FR + ENC_FILTER_BETA * encoder_data_FR_last;
    float filt_BL = ENC_FILTER_ALPHA * raw_BL + ENC_FILTER_BETA * encoder_data_BL_last;
    float filt_BR = ENC_FILTER_ALPHA * raw_BR + ENC_FILTER_BETA * encoder_data_BR_last;

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

/**
 * @brief 纯编码器里程计解算
 * 
 * 基于麦克纳姆轮运动学模型，将编码器脉冲增量转换为车体在场地坐标系下的位移。
 * 
 * @param yaw 当前机器人的航向角（单位：度），用于将车体系位移旋转至世界坐标系。
 */
void distance(float yaw){
    
    // ========== 1. 车体系位移增量（脉冲/10ms）==========
    // 标准麦克纳姆轮正向运动学
    float dx = ( encoder_data_FL_enc + encoder_data_FR_enc + encoder_data_BL_enc + encoder_data_BR_enc) / 4.0f;
    float dy = ( encoder_data_FL_enc - encoder_data_FR_enc - encoder_data_BL_enc + encoder_data_BR_enc) / 4.0f;
    
    // dx, dy 是车体 x/y 方向的脉冲增量（相对于车体坐标系）
    
    // ========== 2. 脉冲 → 厘米 ==========
    float shift_x = dx * dc;   // 车体x方向位移 (cm)
    float shift_y = dy * dc;   // 车体y方向位移 (cm)
    
    // ========== 3. 车体系 → 场地坐标系（偏航旋转）==========
    float rad = yaw * PI / 180.0f;
    float cos_yaw = cosf(rad);
    float sin_yaw = sinf(rad);
    float world_x = shift_x * cos_yaw + shift_y * sin_yaw;
    float world_y = -shift_x * sin_yaw + shift_y * cos_yaw;

    // 航向接近 X/Y 轴时使用纵向校正系数，否则交换校正系数
    float cx, cy;
    if (fabsf(yaw) <= YAW_AXIS_THRESHOLD || fabsf(yaw - 180.0f) <= YAW_AXIS_THRESHOLD) {
        cx = corr_x_enc;
        cy = corr_y_enc;
    } else {
        cx = corr_y_enc;
        cy = corr_x_enc;
    }
    x_enc += world_x * cx;
    y_enc += world_y * cy;

}

/**
 * @brief IMU 加速度二次积分里程计
 * 基于 IMU 加速度计数据进行二次积分以计算世界坐标系下的位移。
 * 采用编码器辅助的零速更新（ZUPT）和互补滤波策略，以抑制麦克纳姆轮振动引起的漂移。
 * @note 依赖外部变量 acc_x, acc_y (来自 imu963.c)。
 */
void imu_distance(void) {
    static const float dt = 0.005f;           // 5ms 积分周期 (与 imu_get 一致)
    static float vel_x_imu = 0, vel_y_imu = 0; // IMU 积分速度 (cm/s)

    // ========== 1. 静止检测（编码器辅助零速更新 ZUPT）==========
    float abs_FL = fabsf(encoder_data_FL_enc);
    float abs_FR = fabsf(encoder_data_FR_enc);
    float abs_BL = fabsf(encoder_data_BL_enc);
    float abs_BR = fabsf(encoder_data_BR_enc);
    float enc_max = abs_FL;
    if (abs_FR > enc_max) enc_max = abs_FR;
    if (abs_BL > enc_max) enc_max = abs_BL;
    if (abs_BR > enc_max) enc_max = abs_BR;
    if (enc_max < ZUPT_THRESHOLD) {
        vel_x_imu = 0;
        vel_y_imu = 0;
        return;
    }

    // ========== 2. 二级低通滤波（专门抑制麦克纳姆轮 30-80Hz 振动）==========
    static float acc_x_lp2 = 0, acc_y_lp2 = 0;
    acc_x_lp2 = acc_x * IMU_ACC_ALPHA + acc_x_lp2 * IMU_ACC_BETA;  // fc ≈ 3.4 Hz
    acc_y_lp2 = acc_y * IMU_ACC_ALPHA + acc_y_lp2 * IMU_ACC_BETA;

    // ========== 3. 加速度死区 ==========
    float acc_w_x = acc_x_lp2;
    float acc_w_y = acc_y_lp2;
    if (fabsf(acc_w_x) < ACC_DEAD_ZONE) acc_w_x = 0;
    if (fabsf(acc_w_y) < ACC_DEAD_ZONE) acc_w_y = 0;

    // ========== 4. 速度积分（欧拉积分，无衰减）==========
    vel_x_imu += acc_w_x * dt;
    vel_y_imu += acc_w_y * dt;

    // ========== 5. 编码器-IMU 互补滤波（用编码器速度抑制振动漂移）==========
    // 编码器不受振动干扰, 以 50%/周期的速率融合 IMU 速度
    // 时间常数 ≈ 1s: IMU 保留快速瞬态响应, 编码器提供长期基准
    float vel_enc_x = encoder_data_FL_enc + encoder_data_FR_enc
                    + encoder_data_BL_enc + encoder_data_BR_enc;
    float vel_enc_y = encoder_data_FL_enc - encoder_data_FR_enc
                    - encoder_data_BL_enc + encoder_data_BR_enc;
    vel_enc_x = vel_enc_x * 0.25f * dc_dt;  // 编码器 X 速度 (cm/s)
    vel_enc_y = vel_enc_y * 0.25f * dc_dt;  // 编码器 Y 速度 (cm/s)

    vel_x_imu = vel_x_imu * IMU_COMPLEMENTARY_ALPHA + vel_enc_x * IMU_COMPLEMENTARY_ALPHA;
    vel_y_imu = vel_y_imu * IMU_COMPLEMENTARY_ALPHA + vel_enc_y * IMU_COMPLEMENTARY_ALPHA;

    // ========== 6. 位置积分 ==========
    x_imu += vel_x_imu * dt * corr_x_enc;
    y_imu += vel_y_imu * dt * corr_y_enc;
}