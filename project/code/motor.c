#include "motor.h"
#include "zf_common_headfile.h"


#define MAX_DUTY                    (9000)                                                // 最大 MAX_DUTY% 占空比
#define MOTOR_FL_PWM1               (PWM1_MODULE0_CHB_D13)
#define MOTOR_FL_PWM2               (PWM1_MODULE0_CHA_D12)

#define MOTOR_FR_PWM1               (PWM1_MODULE1_CHB_D15)
#define MOTOR_FR_PWM2               (PWM1_MODULE1_CHA_D14)

#define MOTOR_BL_PWM1               (PWM1_MODULE3_CHB_D1)
#define MOTOR_BL_PWM2               (PWM1_MODULE3_CHA_D0)

#define MOTOR_BR_PWM1               (PWM2_MODULE3_CHB_D3)
#define MOTOR_BR_PWM2               (PWM2_MODULE3_CHA_D2)



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

void motor_duty(int FL,int FR,int BL,int BR){
	
	if (FL >= 0){
		pwm_set_duty(MOTOR_FL_PWM1,FL);
		pwm_set_duty(MOTOR_FL_PWM2,0);
	}else{
		pwm_set_duty(MOTOR_FL_PWM1,0);
		pwm_set_duty(MOTOR_FL_PWM2,-FL);
	}
	
	if (FR >= 0){
		pwm_set_duty(MOTOR_FR_PWM1,FR);
		pwm_set_duty(MOTOR_FR_PWM2,0);
	}else{
		pwm_set_duty(MOTOR_FR_PWM1,0);
		pwm_set_duty(MOTOR_FR_PWM2,-FR);
	}
	
	if (BL >= 0){
		pwm_set_duty(MOTOR_BL_PWM1,BL);
		pwm_set_duty(MOTOR_BL_PWM2,0);
	}else{
		pwm_set_duty(MOTOR_BL_PWM1,0);
		pwm_set_duty(MOTOR_BL_PWM2,-BL);
	}
	
	if (BR >= 0){
		pwm_set_duty(MOTOR_BR_PWM1,BR);
		pwm_set_duty(MOTOR_BR_PWM2,0);
	}else{
		pwm_set_duty(MOTOR_BR_PWM1,0);
		pwm_set_duty(MOTOR_BR_PWM2,-BR);
	}
	
}
