#ifndef _CODE_MENU_h_
#define _CODE_MENU_h_

#include <stdint.h>

/* PID 参数 */
extern float   pid_x_p;
extern float   pid_x_i;
extern float   pid_x_d;
extern uint8_t pid_x_speed;
extern float   pid_y_p;
extern float   pid_y_i;
extern float   pid_y_d;
extern uint8_t pid_y_speed;

/* 原点参数 */
extern uint8_t origin_x_enc;
extern uint8_t origin_y_enc;

/* 修正值 */
extern float  corr_x_enc;
extern float  corr_y_enc;
extern int8_t corr_x_cam;
extern int8_t corr_y_cam;
extern float  corr_yaw;

/* 加减速控制 */
extern float x_acc;
extern float x_dec;
extern float y_acc;
extern float y_dec;

void menu(void);

#endif