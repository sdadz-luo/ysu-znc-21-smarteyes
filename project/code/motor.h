#ifndef _CODE_MOTOR_h_
#define _CODE_MOTOR_h_

/**
 * @brief 初始化电机相关硬件资源
 */
void motor_init(void);

/**
 * @brief 设置四个电机的PWM占空比以控制运动
 * 通过分别设定前左、前右、后左、后右四个电机的 duty cycle（占空比），
 * @note 正值通常表示正转，负值表示反转，具体行为取决于硬件接线和驱动逻辑。
 */
void motor_duty(int FL,int FR,int BL,int BR);

#endif