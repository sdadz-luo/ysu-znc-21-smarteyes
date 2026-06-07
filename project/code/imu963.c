#include "zf_common_headfile.h"
#include "imu963.h"

// 陀螺仪滑动滤波器实例（初始化全零）
// 在原始整数域（int16）做滤波，避免浮点运算，保持原始分辨率
static FILTER_TYPE gyro_filter_x = {{0}, 0, 0, 0};
static FILTER_TYPE gyro_filter_y = {{0}, 0, 0, 0};
static FILTER_TYPE gyro_filter_z = {{0}, 0, 0, 0};

/**
 * @brief 滑动平均滤波（整数域，O(1) 时间复杂度优化版）
 * @param filter 滤波器结构体指针
 * @param newVal 新采集的原始整数数据（int16 转 float 传入）
 * @return 滤波后的均值（浮点，用于后续单位转换）
 * @note 使用累积和 + 循环缓冲区，每次插入 O(1)
 *       在单位转换前滤波，保持原始分辨率
 */
static float gyroFilter(FILTER_TYPE *filter, float newVal) {
    unsigned char i = filter->index;
    
    // 1. 更新累积和：减掉被覆盖的旧值，加入新值
    filter->sum -= filter->buf[i];
    filter->buf[i] = newVal;
    filter->sum += newVal;
    
    // 2. 循环索引递增
    filter->index = (i + 1) % FILTER_SIZE;
    
    // 3. 填满标记
    if (!filter->filled) {
        filter->filled = (filter->index == 0) ? 1 : 0;
        return filter->sum / (filter->index ? filter->index : FILTER_SIZE);
    }
    
    return filter->sum / FILTER_SIZE;
}

float yaw = 0.0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;
float acc_x = 0, acc_y = 0, acc_z = 0;
static float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
static float acc_x_offset = 0, acc_y_offset = 0, acc_z_offset = 0;
static float gyro_z_dead = 0.003f;  // 陀螺仪Z轴死区(rad/s)，初始化时由3σ噪声自动计算

void imu_init(void){
    imu963ra_init();
    
    // 传感器稳定预热
    for (int i = 0; i < 100; i++){
        imu963ra_get_acc();
        imu963ra_get_gyro();
        system_delay_ms(5);
    }

    // 陀螺仪零偏校准（同时统计Z轴噪声用于死区计算）
    float gyro_z_sq_sum = 0;
    for (int i = 0; i < 600; i++){
        imu963ra_get_acc();
        imu963ra_get_gyro();
        gyro_x_offset += imu963ra_gyro_x;
        gyro_y_offset += imu963ra_gyro_y;
        gyro_z_offset += imu963ra_gyro_z;
        acc_x_offset += imu963ra_acc_x;
        acc_y_offset += imu963ra_acc_y;
        acc_z_offset += imu963ra_acc_z;
        system_delay_ms(5);
    }
    gyro_x_offset /= 600;
    gyro_y_offset /= 600;
    gyro_z_offset /= 600;
    acc_x_offset /= 600;
    acc_y_offset /= 600;
    acc_z_offset /= 600;

}

/**
 * @brief 获取IMU数据并直接积分计算偏航角
 * @note  仅使用陀螺仪Z轴角速度积分，无加速度计/磁力计修正
 *        初始化时已做零偏校准和自适应死区计算
 */
void imu_get(void){
    static const float dt = 0.005f;  // 5ms 采样周期

    // 1. 获取IMU原始数据
    imu963ra_get_acc();
    imu963ra_get_gyro();

    // 2. 加速度计零偏修正 + 一阶低通滤波 + 单位转换 (cm/s?)
    //    ±8g量程 → 4096 LSB/g, 1g = 980 cm/s?
    acc_x = ((float)(imu963ra_acc_x - acc_x_offset) * 0.15f * 980.0f) / 4098.0f + acc_x * 0.85f;
    acc_y = ((float)(imu963ra_acc_y - acc_y_offset) * 0.15f * 980.0f) / 4098.0f + acc_y * 0.85f;
    acc_z = ((float)(imu963ra_acc_z - acc_z_offset) * 0.15f * 980.0f) / 4098.0f + acc_z * 0.85f;

    // 3. 陀螺仪去偏 → 滑动滤波 → 单位转换 (rad/s)
    float gyro_z_raw = ((float)imu963ra_gyro_z - gyro_z_offset);
    float gyro_z_f   = gyroFilter(&gyro_filter_z, gyro_z_raw);
    gyro_z = gyro_z_f * PI / 180.0f / 14.3f;

    // 4. 自适应死区（阈值由初始化时静止噪声3σ确定）
    if (fabsf(gyro_z) <= gyro_z_dead) gyro_z = 0;

    // 5. 第二级 IIR 低通滤波，抑制麦克纳姆轮高频振动（α=0.3, fc≈11Hz）
    static float gyro_z_lp = 0;
    gyro_z_lp = gyro_z * 0.3f + gyro_z_lp * 0.7f;
    gyro_z = gyro_z_lp;

    // 6. 直接积分得到偏航角（度），归一化至 [-180, 180]
    yaw -= gyro_z * 180.0f / PI * dt;
    while (yaw > 180.0f)  yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;

}
