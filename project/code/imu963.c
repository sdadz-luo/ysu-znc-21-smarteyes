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

// Mahony算法参数，可根据实际调试情况调整
#define Kp 0.0f    // 比例增益（加速度计/陀螺仪权重）
#define Ki 0.000f  // 积分增益（可选，若不需要积分可设为0）

float yaw = 0.0;
// 全局变量：扩展欧拉角、四元数、滤波后数据、积分项等
float acc_x = 0,acc_y = 0,acc_z = 0;
float gyro_x = 0,gyro_y = 0,gyro_z = 0;
static float gyro_x_offset  = 0,gyro_y_offset  = 0,gyro_z_offset  = 0;
static float acc_x_offset = 0,acc_y_offset = 0,acc_z_offset = 0;
static Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f}; // 全局四元数，初始化为单位四元数
static EulerAngle euler = {0.0f, 0.0f, 0.0f};   // 全局欧拉角
static float integral_fx = 0, integral_fy = 0, integral_fz = 0; // 积分误差项

/**
 * @brief 四元数转欧拉角，定义前向坐标系：X/Y顺时针为正，Z逆时针为正
 * @param q 输入四元数，需为单位四元数
 * @return 欧拉角：roll/pitch/yaw（单位：度），范围-180~180度
 * @note pitch 使用 -asin() 此时参数需限幅在 [-1,1] 防止浮点误差导致 NaN
 */
static EulerAngle quaternion_to_euler(Quaternion q) {
    EulerAngle angle;
    const float RAD_TO_DEG = 180 / PI;

    // 1. 计算横滚角Roll（X轴，顺时针为正）
    float roll_rad = -atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y));
    angle.roll = roll_rad * RAD_TO_DEG;

    // 2. 计算俯仰角Pitch（Y轴，顺时针为正）— 限幅防止 asin 越界
    float pitch_arg = 2 * (q.w*q.y - q.x*q.z);
    if (pitch_arg > 1.0f)  pitch_arg = 1.0f;
    if (pitch_arg < -1.0f) pitch_arg = -1.0f;
    float pitch_rad = -asin(pitch_arg);
    angle.pitch = pitch_rad * RAD_TO_DEG;

    // 3. 计算偏航角Yaw（Z轴，逆时针为正）
    float yaw_rad = atan2(2 * (q.x*q.y - q.w*q.z), 1 - 2 * (q.y*q.y + q.z*q.z));
    angle.yaw = yaw_rad * RAD_TO_DEG;

    // 4. 限制角度范围：-180~180度
    if (angle.roll > 180) angle.roll -= 360;
    if (angle.pitch > 180) angle.pitch -= 360;
    if (angle.yaw > 180) angle.yaw -= 360;

    return angle;
}

/**
 * @brief Mahony互补滤波算法，利用IMU数据更新四元数
 * @param gx/gy/gz 陀螺仪角速度（rad/s），已去偏
 * @param ax/ay/az 加速度计数据（m/s2），已滤波
 * @param dt 采样时间（s）
 */
static void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1. 对加速度计数据进行归一化处理
    norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return; // 避免除零（加速度计数据异常）
    ax /= norm;
    ay /= norm;
    az /= norm;

    // 2. 将四元数旋转到当前姿态，计算重力向量在机体坐标系下的分量
    vx = 2*(q.x*q.z - q.w*q.y);
    vy = 2*(q.w*q.x + q.y*q.z);
    vz = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;

    // 3. 计算误差（加速度计测量值与四元数推算值的叉积）
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    // 4. 积分误差（可选，Ki=0可关闭积分）
    integral_fx += ex * Ki * dt;
    integral_fy += ey * Ki * dt;
    integral_fz += ez * Ki * dt;
    
    // 限制积分项防止发散
    integral_fx = fabs(integral_fx) > 1.0f ? (integral_fx>0?1.0f:-1.0f) : integral_fx;
    integral_fy = fabs(integral_fy) > 1.0f ? (integral_fy>0?1.0f:-1.0f) : integral_fy;
    integral_fz = fabs(integral_fz) > 1.0f ? (integral_fz>0?1.0f:-1.0f) : integral_fz;

    // 5. 修正陀螺仪数据（比例+积分）
    gx += Kp * ex + integral_fx;
    gy += Kp * ey + integral_fy;
    gz += Kp * ez + integral_fz;

    // 6. 四元数微分更新，利用四元数运动学方程
    float qw_dot = -0.5f * (q.x*gx + q.y*gy + q.z*gz);
    float qx_dot = 0.5f * (q.w*gx + q.y*gz - q.z*gy);
    float qy_dot = 0.5f * (q.w*gy + q.z*gx - q.x*gz);
    float qz_dot = 0.5f * (q.w*gz + q.x*gy - q.y*gx);

    // 7. 迭代更新四元数
    q.w += qw_dot * dt;
    q.x += qx_dot * dt;
    q.y += qy_dot * dt;
    q.z += qz_dot * dt;

    // 8. 对四元数进行归一化，保证单位四元数
    norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
}


void imu_init(void){
    // 初始化四元数为单位四元数
    q.w = 1.0f; q.x = 0.0f; q.y = 0.0f; q.z = 0.0f;
    EulerAngle angle1 = quaternion_to_euler(q);
    
    imu963ra_init();
    
    for (int i = 0;i < 100;i++){
        imu963ra_get_acc(); 
        imu963ra_get_gyro();
        system_delay_ms(5);
    }

    // 陀螺仪零偏校准
    for (int i = 0;i < 600;i++){
        imu963ra_get_acc(); 
        imu963ra_get_gyro();
        gyro_x_offset  += imu963ra_gyro_x;
        gyro_y_offset  += imu963ra_gyro_y;
        gyro_z_offset  += imu963ra_gyro_z;
        acc_x_offset += imu963ra_acc_x;
        acc_y_offset += imu963ra_acc_y;
        acc_z_offset += imu963ra_acc_z;
        system_delay_ms(5);
    }
    gyro_x_offset  /= 600;
    gyro_y_offset  /= 600;
    gyro_z_offset  /= 600;
    acc_x_offset /= 600;
    acc_y_offset /= 600;
    acc_z_offset /= 600;
}

void imu_get(void){

    float dt = 0.005f; // 5ms 采样周期，与PIT定时器一致·

    // 2. 获取IMU原始数据
    imu963ra_get_acc();
    imu963ra_get_gyro();  
    
    // 3. 加速度计一阶低通滤波+单位转换（m/s2）
    acc_x = (((float) imu963ra_acc_x - acc_x_offset) * 0.15 * 9.8f) / 4098 + acc_x * (1 - 0.15);
    acc_y = (((float) imu963ra_acc_y - acc_y_offset) * 0.15 * 9.8f) / 4098 + acc_y * (1 - 0.15);
    acc_z = -(((float) imu963ra_acc_z) * 0.15 * 9.8f) / 4098 + acc_z * (1 - 0.15);;

    // 4. 陀螺仪去偏（原始整数域），经过滑动滤波，最后单位转换
    //    先滤波（整数域）再转 rad/s，保持原始分辨率
    float gyro_x_raw = -((float) imu963ra_gyro_x - gyro_x_offset);
    float gyro_y_raw = -((float) imu963ra_gyro_y - gyro_y_offset);
    float gyro_z_raw = ((float) imu963ra_gyro_z - gyro_z_offset);
    
    float gyro_x_f = gyroFilter(&gyro_filter_x, gyro_x_raw);
    float gyro_y_f = gyroFilter(&gyro_filter_y, gyro_y_raw);
    float gyro_z_f = gyroFilter(&gyro_filter_z, gyro_z_raw);
    
    // 单位转换（原始均值 × PI/180/14.3 → rad/s）
    gyro_x = gyro_x_f * PI / 180 / 14.3f;
    gyro_y = gyro_y_f * PI / 180 / 14.3f;
    gyro_z = gyro_z_f * PI / 180 / 14.3f;
    
    // 陀螺仪死区
    if (fabs(gyro_x) <= 0.005f) gyro_x = 0;
    if (fabs(gyro_y) <= 0.005f) gyro_y = 0;
    if (fabs(gyro_z) <= 0.005f) gyro_z = 0;
    else gyro_z += 0.00002f; // 经验补偿，减少偏航漂移

    // 5. 核心：调用Mahony算法，利用IMU数据更新四元数
    mahony_update(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, dt);

    // 6. 四元数转欧拉角，得到最终的roll/pitch/yaw
    euler = quaternion_to_euler(q);

    yaw = euler.yaw;
    
}
