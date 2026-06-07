#ifndef _CODE_imu963_h_
#define _CODE_imu963_h_

// 定义欧拉角结构体（单位：度）
typedef struct {
    float roll;   // 滚转角（绕X轴，顺时针为正，范围-180~180°）
    float pitch;  // 俯仰角（绕Y轴，顺时针为正，范围-180~180°）
    float yaw;    // 偏航角（绕Z轴，逆时针为正，范围-180~180°）
} EulerAngle;

// 滑动滤波参数（15点窗口，截止频率~4.2Hz，抑制麦克纳姆轮30Hz+振动）
#define FILTER_SIZE 15

// 滑动滤波器结构体
typedef struct {
    float buf[FILTER_SIZE];   // 循环缓冲区
    float sum;                // 运行累加和，避免每次重新求和
    unsigned char index;      // 当前写入位置
    unsigned char filled;     // 是否已填满（填满前均值需用实际数据个数）
} FILTER_TYPE;

void imu_init(void);
void imu_get(void);

#endif
