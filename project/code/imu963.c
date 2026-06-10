#include "imu963.h"
#include "zf_common_headfile.h"
#include "stdint.h"
#include "string.h"
#include "math.h"


// Mahony算法参数
#define Kp 0.3f    // 比例增益（加速度计/磁力计修正）
#define Ki 0.002f  // 积分增益（可选，若不需要积分可设为0）

float yaw = 0.0;
// 全局变量扩展：添加四元数、欧拉角、采样时间、积分误差
float acc_x = 0,acc_y = 0,acc_z = 0;
float gyro_x = 0,gyro_y = 0,gyro_z = 0;
static float gyro_x_offset = 0,gyro_y_offset = 0,gyro_z_offset = 0;
static float acc_x_offset = 0,acc_y_offset = 0;
static Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f}; // 全局四元数（初始无旋转）
static EulerAngle euler = {0.0f, 0.0f, 0.0f};   // 全局欧拉角
static float integral_fx = 0, integral_fy = 0, integral_fz = 0; // 积分误差

/**
 * @brief 四元数转欧拉角（适配前右上坐标系，X/Y顺时针正，Z逆时针正）
 * @param q 输入四元数（单位四元数）
 * @return 欧拉角（roll/pitch/yaw，单位：度，范围0~360°）
 */
static EulerAngle quaternion_to_euler(Quaternion q) {
    EulerAngle angle;
    const float RAD_TO_DEG = 180 / PI;

    // 1. 计算滚转角Roll（绕X轴，顺时针为正）
    float roll_rad = -atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y));
    angle.roll = roll_rad * RAD_TO_DEG;

    // 2. 计算俯仰角Pitch（绕Y轴，顺时针为正）
    float pitch_rad = -asin(2 * (q.w*q.y - q.x*q.z));
    angle.pitch = pitch_rad * RAD_TO_DEG;

    // 3. 计算偏航角Yaw（绕Z轴，逆时针为正）
    float yaw_rad = atan2(2 * (q.x*q.y - q.w*q.z), 1 - 2 * (q.y*q.y + q.z*q.z));
    angle.yaw = yaw_rad * RAD_TO_DEG;

    // 4. 修正角度范围到 0~360°（也可保留-180~180°，根据需求调整）
    if (angle.roll > 180) angle.roll -= 360;
    if (angle.pitch > 180) angle.pitch -= 360;
    if (angle.yaw > 180) angle.yaw -= 360;

    return angle;
}

/**
 * @brief Mahony互补滤波算法：从IMU数据更新四元数
 * @param gx/gy/gz 陀螺仪角速度（rad/s，已去偏置）
 * @param ax/ay/az 加速度计数据（m/s2，已滤波）
 * @param dt 采样时间（s）
 */
static void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1. 归一化加速度计数据（单位向量）
    norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return; // 避免除零（加速度计无数据）
    ax /= norm;
    ay /= norm;
    az /= norm;

    // 2. 从四元数计算重力向量（机体坐标系→导航坐标系）
    vx = 2*(q.x*q.z - q.w*q.y);
    vy = 2*(q.w*q.x + q.y*q.z);
    vz = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;

    // 3. 计算误差（加速度计测量值与理论重力向量的叉乘）
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    // 4. 积分误差（可选，Ki=0则关闭积分）
    integral_fx += ex * Ki * dt;
    integral_fy += ey * Ki * dt;
    integral_fz += ez * Ki * dt;
		
		// 限幅到±1.0，避免积分发散
    integral_fx = fabs(integral_fx) > 1.0f ? (integral_fx>0?1.0f:-1.0f) : integral_fx;
    integral_fy = fabs(integral_fy) > 1.0f ? (integral_fy>0?1.0f:-1.0f) : integral_fy;
    integral_fz = fabs(integral_fz) > 1.0f ? (integral_fz>0?1.0f:-1.0f) : integral_fz;

    // 5. 陀螺仪数据修正（比例+积分）
    gx += Kp * ex + integral_fx;
    gy += Kp * ey + integral_fy;
    gz += Kp * ez + integral_fz;

    // 6. 四元数微分更新（四元数动力学方程）
    float qw_dot = -0.5f * (q.x*gx + q.y*gy + q.z*gz);
    float qx_dot = 0.5f * (q.w*gx + q.y*gz - q.z*gy);
    float qy_dot = 0.5f * (q.w*gy + q.z*gx - q.x*gz);
    float qz_dot = 0.5f * (q.w*gz + q.x*gy - q.y*gx);

    // 7. 积分更新四元数
    q.w += qw_dot * dt;
    q.x += qx_dot * dt;
    q.y += qy_dot * dt;
    q.z += qz_dot * dt;

    // 8. 归一化四元数（保证单位四元数）
    norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
}


void imu_init(void){
    // 初始化四元数（无旋转）
    q.w = 1.0f; q.x = 0.0f; q.y = 0.0f; q.z = 0.0f;
    EulerAngle angle1 = quaternion_to_euler(q);
	
    imu963ra_init();
	
    // 陀螺仪偏置校准
    for (int i = 0;i < 400;i++){
        imu963ra_get_acc(); 
		imu963ra_get_gyro();
        gyro_x_offset  += imu963ra_gyro_x;
        gyro_y_offset  += imu963ra_gyro_y;
        gyro_z_offset  += imu963ra_gyro_z;
		acc_x_offset += imu963ra_acc_x;
		acc_y_offset += imu963ra_acc_y;
        system_delay_ms(5);
    }
    gyro_x_offset /= 400;
    gyro_y_offset /= 400;
    gyro_z_offset /= 400;
	acc_x_offset  /= 400;
	acc_y_offset  /= 400;
}

void imu_get(void){

    float dt = 0.005;

    // 2. 读取IMU原始数据
    imu963ra_get_acc();
    imu963ra_get_gyro();  
	
    // 3. 加速度计一阶低通滤波+单位转换（m/s2）
    acc_x = (((float) imu963ra_acc_x - acc_x_offset) * 0.06 * 9.8f) / 4098 + acc_x * (1 - 0.06);
    acc_y = (((float) imu963ra_acc_y - acc_y_offset) * 0.06 * 9.8f) / 4098 + acc_y * (1 - 0.06);
    acc_z = -(((float) imu963ra_acc_z) * 0.06 * 9.8f) / 4098 + acc_z * (1 - 0.06);

    // 4. 陀螺仪去偏置+单位转换（deg/s → rad/s）
    gyro_x = -((float) imu963ra_gyro_x - gyro_x_offset) * 0.25 * PI / 180 / 14.3f + gyro_x * (1 - 0.25);
    gyro_y = -((float) imu963ra_gyro_y - gyro_y_offset) * 0.25 * PI / 180 / 14.3f + gyro_y * (1 - 0.25);
    gyro_z = ((float) imu963ra_gyro_z - gyro_z_offset) * 0.25 * PI / 180 / 14.3f + gyro_z * (1 - 0.25);
		
	if (fabs(gyro_x) <= 0.005) gyro_x = 0;
	if (fabs(gyro_y) <= 0.005) gyro_y = 0;
	if (fabs(gyro_z) <= 0.005) gyro_z = 0;

    // 5. 核心：调用Mahony算法更新四元数（接入IMU数据）
    mahony_update(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, dt);

    // 6. 四元数转欧拉角（得到最终的roll/pitch/yaw）
    euler = quaternion_to_euler(q);
		
	yaw = euler.yaw;
		
}