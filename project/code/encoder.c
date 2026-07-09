#include "encoder.h"
#include "zf_common_headfile.h"

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


#define N    16384                       // 编码器每圈脉冲数 (CPR * 4倍频)
// ================= 机械参数 (根据实际齿轮确认) =================
#define GEAR_ENCODER    30              // 编码器齿轮齿数
#define GEAR_WHEEL      70              // 轮子大齿轮齿数
#define B               (GEAR_ENCODER / (float)GEAR_WHEEL)  // = 30/70 = 3/7
#define C               (2.0f * PI * 3.0f)                        // 轮子周长 (2 * PI * 半径3.0cm)

// ================= 滤波与算法参数 =================
#define ENC_FILTER_ALPHA        0.9f    // 编码器一阶低通滤波系数
#define ENC_FILTER_BETA         0.1f    // 编码器一阶低通滤波历史权重 (1 - alpha)

// ================= 全局变量 =================
float x_enc = 0, y_enc = 0;             // 编码器全局累积位移 (单位: cm)
static float dc = C / (B * N);          // 脉冲→厘米转换系数 (cm/脉冲)
static float dc_dt = 0;                 // 预计算: dc / dt, 用于编码器速度换算 (在 encoder_init 中初始化)


// 原始编码器数据 (中间值，每次读取后清零)
float encoder_data_FL = 0;
float encoder_data_FR = 0;
float encoder_data_BL = 0;
float encoder_data_BR = 0;

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
    encoder_data_FL = filt_FL;
    encoder_data_FR = filt_FR;
    encoder_data_BL = filt_BL;
    encoder_data_BR = filt_BR;
    
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

    x_enc += world_x * corr_x_enc;
    y_enc += world_y * corr_y_enc;

}
