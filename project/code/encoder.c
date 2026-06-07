#include "encoder.h"
#include "zf_common_headfile.h"
 
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
#define C   (2 * PI * 3)                // 轮子周长 (2 * PI * 半径3.0cm)

#define x_xiu  3/4
#define y_xiu  2.85/4

// ================= 全局变量 =================
float x_enc = 0, y_enc = 0;             // 全局累积位移 (单位: cm)
// dc = 编码器每1个脉冲对应的轮子位移
//     编码器1脉冲 → 编码器转 1/N 圈 → 轮子转 B/N 圈 → 位移 C*B/N
float dc = C / (B * N);                  // (cm/脉冲)

// 原始编码器数据 (中间值，每次读取后清零)
int encoder_data_FL = 0;
int encoder_data_FR = 0;
int encoder_data_BL = 0;
int encoder_data_BR = 0;

// 一阶低通滤波历史数据
static int encoder_data_FL_last = 0;
static int encoder_data_FR_last = 0;
static int encoder_data_BL_last = 0;
static int encoder_data_BR_last = 0;

static int encoder_data_FL_enc = 0;
static int encoder_data_FR_enc = 0;
static int encoder_data_BL_enc = 0;
static int encoder_data_BR_enc = 0;

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
    // 1. 获取硬件计数值
    encoder_data_FL =  encoder_get_count(ENCODER_FL);
    encoder_data_FR = -encoder_get_count(ENCODER_FR);
    encoder_data_BL =  encoder_get_count(ENCODER_BL);
    encoder_data_BR = -encoder_get_count(ENCODER_BR);

    encoder_data_FL_enc = encoder_data_FL;
    encoder_data_FR_enc = encoder_data_FR;
    encoder_data_BL_enc = encoder_data_BL;
    encoder_data_BR_enc = encoder_data_BR;
    
    // 2. 一阶低通滤波 (Alpha = 0.8)
    encoder_data_FL = 0.8 * encoder_data_FL + 0.2 * encoder_data_FL_last;
    encoder_data_FL_last = encoder_data_FL;
    
    encoder_data_FR = 0.8 * encoder_data_FR + 0.2 * encoder_data_FR_last;
    encoder_data_FR_last = encoder_data_FR;
    
    encoder_data_BL = 0.8 * encoder_data_BL + 0.2 * encoder_data_BL_last;
    encoder_data_BL_last = encoder_data_BL;
    
    encoder_data_BR = 0.8 * encoder_data_BR + 0.2 * encoder_data_BR_last;
    encoder_data_BR_last = encoder_data_BR;
    
    // 3. 清除硬件计数器，以便下次读取的是增量值
    encoder_clear_count(ENCODER_FL);
    encoder_clear_count(ENCODER_FR);
    encoder_clear_count(ENCODER_BL);
    encoder_clear_count(ENCODER_BR);
}

// ================= 纯编码器里程计解算 =================
void distance(float yaw){
    
    // ========== 1. 车体系位移增量（脉冲/10ms）==========
    // 标准麦克纳姆轮正向运动学
    float dx = ( encoder_data_FL_enc + encoder_data_FR_enc + encoder_data_BL_enc + encoder_data_BR_enc) / 4.0f;
    float dy = ( encoder_data_FL_enc - encoder_data_FR_enc - encoder_data_BL_enc + encoder_data_BR_enc) / 4.0f;
    
    // dx, dy 是车体 x/y 方向的脉冲增量（相对于车体坐标系）
    
    // ========== 2. 脉冲 → 厘米（每10ms周期的位移）==========
    float shift_x = dx * dc;    // 车体x方向位移 (cm)
    float shift_y = dy * dc;    // 车体y方向位移 (cm)
    
    // ========== 3. 坐标旋转：车体系 → 全局坐标系 ==========
    float yaw_rad = yaw * PI / 180.0f;
    float cos_y = cosf(yaw_rad);
    float sin_y = sinf(yaw_rad);
    
    float global_dx = shift_x * cos_y - shift_y * sin_y;
    float global_dy = shift_x * sin_y + shift_y * cos_y;
    
    // ========== 4. 累加到全局位置 ==========
    x_enc += global_dx * x_xiu;
    y_enc += global_dy * y_xiu;
}
