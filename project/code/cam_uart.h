#ifndef _CAM_UART_H_
#define _CAM_UART_H_

typedef struct {
    float car_cx;          // 小车中心 X
    float car_cy;          // 小车中心 Y
    uint8_t grid[12][16];  // 栅格地图
} CAMDATA;

void cam_uart_init(void);
void cam_uart_isc_1(void);
void cam_uart_isc_2(void);
CAMDATA cam_uart1_read(void);
void cam1_uart_send(float angle);
int cam_uart2_read(void);
void cam_uart2_write(uint8_t id);
float enc_cam(float x, float y, int x_target, int y_target, int mode);

#endif