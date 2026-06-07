#include "control.h"
#include "zf_common_headfile.h"

// 缓加速快减速：加速系数小（缓慢），减速系数大（快速）
#define ACCEL_SLOW_X  0.02f   // X轴慢加速系数（值越小加速越慢）
#define DECEL_FAST_X  0.05f   // X轴快减速系数（值越大减速越快）
#define ACCEL_SLOW_Y  0.02f   // Y轴慢加速系数
#define DECEL_FAST_Y  0.05f   // Y轴快减速系数

static float vx = 0, vy = 0;

/**
 * @brief  非对称一阶低通滤波（缓加速快减速）
 * @param  current  当前平滑后的速度值
 * @param  target   目标速度值
 * @param  a_slow   加速时平滑系数
 * @param  a_fast   减速时平滑系数
 * @return 滤波后的速度值
 */
static inline float asymmetric_filter(float current, float target, float a_slow, float a_fast)
{
    float a;
    // 判断加速还是减速：比较绝对值大小
    if (fabsf(target) > fabsf(current)) a = a_slow;   // 加速：系数小，变化缓慢
    else a = a_fast;   // 减速：系数大，变化快速
    return target * a + current * (1.0f - a);
}

void motor_solution(float vx_body, float vy_body, float current_yaw, float wz)
{
    vx = asymmetric_filter(vx, vx_body, ACCEL_SLOW_X, DECEL_FAST_X);
    vy = asymmetric_filter(vy, vy_body, ACCEL_SLOW_Y, DECEL_FAST_Y);

     float FL = vx - vy - 0.75f * wz;
     float FR = vx + vy + 0.75f * wz;
     float BL = vx + vy - 0.75f * wz;
     float BR = vx - vy + 0.75f * wz;

     pid_wheel_target(FL, FR, BL, BR);
}

