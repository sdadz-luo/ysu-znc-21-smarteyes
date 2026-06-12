#include "control.h"
#include "zf_common_headfile.h"

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

void motor_solution(float vx_field, float vy_field, float current_yaw, float wz)
{
    // ========== 1. 场地系 → 车体系速度（偏航逆旋转）==========
    float rad = current_yaw * PI / 180.0f;
    float cos_yaw = cosf(rad);
    float sin_yaw = sinf(rad);
    float vx_body =  vx_field * cos_yaw + vy_field * sin_yaw;
    float vy_body = -vx_field * sin_yaw + vy_field * cos_yaw;

    // ========== 2. 非对称低通滤波 ==========
    vx = asymmetric_filter(vx, vx_body, x_acc, x_dec);
    vy = asymmetric_filter(vy, vy_body, y_acc, y_dec);

    // ========== 3. 麦克纳姆轮运动学分解（车体系）==========
    float FL = vx - vy - 0.64f * wz;
    float FR = vx + vy + 0.64f * wz;
    float BL = vx + vy - 0.64f * wz;
    float BR = vx - vy + 0.64f * wz;

    pid_wheel_target(FL, FR, BL, BR);
}

