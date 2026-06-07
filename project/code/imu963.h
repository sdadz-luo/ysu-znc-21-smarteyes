#ifndef _CODE_imu963_h_
#define _CODE_imu963_h_

// 定义四元数结构体，方便使用
typedef struct {
    float w;  // 实部
    float x;  // 绕X轴虚部
    float y;  // 绕Y轴虚部
    float z;  // 绕Z轴虚部
} Quaternion;

// 定义欧拉角结构体（单位：度）
typedef struct {
    float roll;   // 滚转角（绕X轴，顺时针为正，范围-180~180°）
    float pitch;  // 俯仰角（绕Y轴，顺时针为正，范围-180~180°）
    float yaw;    // 偏航角（绕Z轴，逆时针为正，范围-180~180°）
} EulerAngle;

// 滑动滤波参数
#define FILTER_SIZE 5

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
