#ifndef _CODE_ENCODER_h_
#define _CODE_ENCODER_h_

// 全局位置变量
extern float x_enc, y_enc;      // 编码器里程计位置 (cm)
extern float x_imu, y_imu;      // IMU 积分位置 (cm)

void encoder_init(void);        // 初始化编码器硬件接口
void encoder_get(void);         // 读取编码器数据
void distance(float yaw);       // 编码器里程计坐标计算

#endif
