#include "pid.h"
#include "zf_common_headfile.h"

pid pid_FL, pid_FR, pid_BL, pid_BR, pid_gyro, pid_yaw, pid_x, pid_y;

void PID_init(pid *pid_struct,
              float kp,
              float ki,
              float kd,
              float maxIntegral,
              float maxOutput,
              float target,
              float dead){
    pid_struct->kp          = kp;
    pid_struct->ki          = ki;
    pid_struct->kd          = kd;
    pid_struct->error       = 0.0f;
    pid_struct->error_last  = 0.0f;
    pid_struct->error_last2 = 0.0f;
    pid_struct->integral    = 0.0f;
    pid_struct->maxintegral = maxIntegral;
    pid_struct->output      = 0.0f;
    pid_struct->output_1    = 0.0f;
    pid_struct->maxOutput   = maxOutput;
    pid_struct->target      = target;
    pid_struct->dead        = dead;
}

float pid_increm(pid *pid_struct, float now_value){
    // 微分先行
    pid_struct->error_last2  = pid_struct->error_last;           // 保存上上次误差
    pid_struct->error_last   = pid_struct->error;                // 保存上一次误差
    pid_struct->error        = pid_struct->target - now_value;   // 计算当前误差

    // 增量计算
    pid_struct->output_1 = pid_struct->kp * (pid_struct->error - pid_struct->error_last)                                          // 比例
                         + pid_struct->ki * pid_struct->error                                                                     // 积分
                         + pid_struct->kd * (pid_struct->error - 2 * pid_struct->error_last + pid_struct->error_last2);           // 微分

    pid_struct->output += pid_struct->output_1;

    // 死区处理：小误差时衰减增量，防止频繁震荡
    if (fabsf(pid_struct->error) <= pid_struct->dead) {
        pid_struct->output_1 *= 0.9f;
    }

    // 输出限幅
    if (pid_struct->output > pid_struct->maxOutput) {
        pid_struct->output = pid_struct->maxOutput;
    }
    if (pid_struct->output < -pid_struct->maxOutput) {
        pid_struct->output = -pid_struct->maxOutput;
    }

    return pid_struct->output;
}

float pid_location(pid *pid_struct, float now_value){
    // 微分先行
    pid_struct->error_last2  = pid_struct->error_last;           // 保存上上次误差
    pid_struct->error_last   = pid_struct->error;                // 保存上一次误差
    pid_struct->error        = pid_struct->target - now_value;   // 计算当前误差

    if (fabsf(pid_struct->error) >= pid_struct->dead) {
        // 积分累加
        pid_struct->integral += pid_struct->error;

        // 积分限幅
        if (pid_struct->integral > pid_struct->maxintegral) {
            pid_struct->integral = pid_struct->maxintegral;
        }
        if (pid_struct->integral < -pid_struct->maxintegral) {
            pid_struct->integral = -pid_struct->maxintegral;
        }

        // 位置式PID计算
        pid_struct->output = pid_struct->kp * pid_struct->error                              // 比例
                           + pid_struct->ki * pid_struct->integral                           // 积分
                           + pid_struct->kd * (pid_struct->error - pid_struct->error_last);  // 微分
    } else {
        // 死区内清空积分，输出置零
        pid_struct->integral *= 0.9f;
        pid_struct->output   = 0.0f;
    }

    // 输出限幅
    if (pid_struct->output > pid_struct->maxOutput) {
        pid_struct->output = pid_struct->maxOutput;
    }
    if (pid_struct->output < -pid_struct->maxOutput) {
        pid_struct->output = -pid_struct->maxOutput;
    }

    return pid_struct->output;
}

void pid_init(void){
    PID_init(&pid_FL, 60, 10, 100,   1000, 9000, 0, 1);
    PID_init(&pid_FR, 60, 10, 100,   1000, 9000, 0, 1);
    PID_init(&pid_BL, 60, 10, 100,   1000, 9000, 0, 1);
    PID_init(&pid_BR, 50, 10, 90,    1000, 9000, 0, 1);
    PID_init(&pid_yaw, 4, 0.001f, 20, 1000, 60, 0, 0.1f);
    PID_init(&pid_x, pid_x_p, pid_x_i, pid_x_d, 500, pid_x_speed, 0, 1);
    PID_init(&pid_y, pid_y_p, pid_y_i, pid_y_d, 500, pid_y_speed, 0, 1);
}

void pid_target(pid *pid_struct, float target){
    pid_struct->target = target;
}

void pid_wheel_target(float FL, float FR, float BL, float BR){
    pid_target(&pid_FL, FL);
    pid_target(&pid_FR, FR);
    pid_target(&pid_BL, BL);
    pid_target(&pid_BR, BR);
}

void pid_position_target(float x, float y){
    pid_target(&pid_x, x);
    pid_target(&pid_y, y);
}

void pid_position_speed(void){
    pid_x.maxOutput = 120;
    pid_y.maxOutput = 120;
}

void pid_change(pid *pid_struct, float kp, float ki, float kd){
    pid_struct->kp = kp;
    pid_struct->ki = ki;
    pid_struct->kd = kd;
}

static void PID_reset(pid *pid_struct){
    pid_struct->error       = 0.0f;
    pid_struct->error_last  = 0.0f;
    pid_struct->error_last2 = 0.0f;
    pid_struct->integral    = 0.0f;
    pid_struct->output      = 0.0f;
    pid_struct->output_1    = 0.0f;
}

void pid_reset(void){
    PID_reset(&pid_FL);
    PID_reset(&pid_FR);
    PID_reset(&pid_BL);
    PID_reset(&pid_BR);
    PID_reset(&pid_x);
    PID_reset(&pid_y);
    pid_x.maxOutput = pid_x_speed;
    pid_y.maxOutput = pid_y_speed;
}
