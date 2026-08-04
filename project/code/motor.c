#include "motor.h"
#include "zf_common_headfile.h"


#define MAX_DUTY                    (9000)                      // 最大 MAX_DUTY% 占空比
#define MOTOR_FL_PWM1               (PWM1_MODULE0_CHB_D13)
#define MOTOR_FL_PWM2               (PWM1_MODULE0_CHA_D12)

#define MOTOR_FR_PWM1               (PWM1_MODULE1_CHB_D15)
#define MOTOR_FR_PWM2               (PWM1_MODULE1_CHA_D14)

#define MOTOR_BL_PWM1               (PWM1_MODULE3_CHB_D1)
#define MOTOR_BL_PWM2               (PWM1_MODULE3_CHA_D0)

#define MOTOR_BR_PWM1               (PWM2_MODULE3_CHB_D3)
#define MOTOR_BR_PWM2               (PWM2_MODULE3_CHA_D2)



/**
 * @brief 初始化四个电机的PWM引脚
 * 
 * 配置前左(FL)、前右(FR)、后左(BL)、后右(BR)四个电机对应的两个PWM通道，
 * 设置频率为10000Hz，初始占空比为0。
 */
void motor_init(void){

	pwm_init(MOTOR_FL_PWM1, 10000, 0); 
	pwm_init(MOTOR_FL_PWM2, 10000, 0); 
	pwm_init(MOTOR_FR_PWM1, 10000, 0); 
	pwm_init(MOTOR_FR_PWM2, 10000, 0); 
	pwm_init(MOTOR_BL_PWM1, 10000, 0); 
	pwm_init(MOTOR_BL_PWM2, 10000, 0); 
	pwm_init(MOTOR_BR_PWM1, 10000, 0); 
	pwm_init(MOTOR_BR_PWM2, 10000, 0); 
	
}

/**
 * @brief 设置四个电机的转速和方向
 * 
 * 通过正负值控制电机方向，绝对值控制占空比（速度）。
 * 正值表示正向旋转，负值表示反向旋转。
 * 每个电机使用两个PWM引脚进行H桥驱动控制：
 * - 正转时：PWM1输出占空比，PWM2输出0
 * - 反转时：PWM1输出0，PWM2输出占空比
 * @return 无
 */
void motor_duty(int FL,int FR,int BL,int BR){
	
	/* 限幅：将每个电机的占空比限制在 [-MAX_DUTY, MAX_DUTY] 范围内 */
	if (FL > MAX_DUTY) FL = MAX_DUTY;
	if (FL < -MAX_DUTY) FL = -MAX_DUTY;
	if (FR > MAX_DUTY) FR = MAX_DUTY;
	if (FR < -MAX_DUTY) FR = -MAX_DUTY;
	if (BL > MAX_DUTY) BL = MAX_DUTY;
	if (BL < -MAX_DUTY) BL = -MAX_DUTY;
	if (BR > MAX_DUTY) BR = MAX_DUTY;
	if (BR < -MAX_DUTY) BR = -MAX_DUTY;
	
	/* 控制前左电机(FL)的方向和占空比 */
	if (FL >= 0){
		pwm_set_duty(MOTOR_FL_PWM1,FL);
		pwm_set_duty(MOTOR_FL_PWM2,0);
	}else{
		pwm_set_duty(MOTOR_FL_PWM1,0);
		pwm_set_duty(MOTOR_FL_PWM2,-FL);
	}
	
	/* 控制前右电机(FR)的方向和占空比 */
	if (FR >= 0){
		pwm_set_duty(MOTOR_FR_PWM1,FR);
		pwm_set_duty(MOTOR_FR_PWM2,0);
	}else{
		pwm_set_duty(MOTOR_FR_PWM1,0);
		pwm_set_duty(MOTOR_FR_PWM2,-FR);
	}
	
	/* 控制后左电机(BL)的方向和占空比 */
	if (BL >= 0){
		pwm_set_duty(MOTOR_BL_PWM1,BL);
		pwm_set_duty(MOTOR_BL_PWM2,0);
	}else{
		pwm_set_duty(MOTOR_BL_PWM1,0);
		pwm_set_duty(MOTOR_BL_PWM2,-BL);
	}
	
	/* 控制后右电机(BR)的方向和占空比 */
	if (BR >= 0){
		pwm_set_duty(MOTOR_BR_PWM1,BR);
		pwm_set_duty(MOTOR_BR_PWM2,0);
	}else{
		pwm_set_duty(MOTOR_BR_PWM1,0);
		pwm_set_duty(MOTOR_BR_PWM2,-BR);
	}
	
}