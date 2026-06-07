#ifndef _CODE_PID_h_
#define _CODE_PID_h_

typedef struct {
    float kp,ki,kd;                 		
    float error,error_last,error_last2; 
    float integral,maxintegral;     		
    float output,output_1,maxOutput;      
    float target;                     
		float dead;
}pid;

void PID_init(pid* pid_struct,
              float kp,
              float ki,
              float kd,
              float maxIntegral,
              float maxOutput,
              float target,
							float dead);
							
void pid_init(void);
void pid_target(pid* pid_struct,float target);
void pid_yaw_target(float yaw);							
void pid_wheel_target(float FL,float FR,float BL,float BR);
void pid_position_target(int x,int y);
int pid_increm(pid* pid_struct,float now_value);
float pid_location(pid* pid_struct,float now_value);
float pid_location_weizi(pid* pid_struct,float now_value);
void pid_location_target(int x,int y);

#endif