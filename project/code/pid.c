#include "pid.h"
#include "zf_common_headfile.h"

pid pid_FL,pid_FR,pid_BL,pid_BR,pid_gyro,pid_yaw,pid_x,pid_y;

extern float gyro_z;  // 陀螺仪Z轴角速度

void PID_init(pid* pid_struct,
              float kp,
              float ki,
              float kd,
              float maxIntegral,
              float maxOutput,
              float target,
							float dead){
    pid_struct ->  kp           =kp;
    pid_struct ->  ki           =ki;
    pid_struct ->  kd           =kd;
    pid_struct ->  error        =0;
    pid_struct ->  error_last   =0;
    pid_struct ->  error_last2  =0;
    pid_struct ->  integral     =0;
    pid_struct ->  maxintegral  =maxIntegral;
    pid_struct ->  output       =0;
    pid_struct ->  output_1     =0;
    pid_struct ->  maxOutput    =maxOutput;
    pid_struct ->  target       =target;
	pid_struct ->  dead       	=dead;			
}

float pid_increm(pid* pid_struct,float now_value){
    //微分先行
    pid_struct->error_last2  =   pid_struct->error_last;              //保存上上次误差
    pid_struct->error_last   =   pid_struct->error;                   //保存上一次误差
    pid_struct->error 		 =   pid_struct->target - now_value;      //计算当前误差
	
		//增量计算
    pid_struct->output_1 = pid_struct->kp  *  (pid_struct->error - pid_struct->error_last)                   			  //比例
                         +  pid_struct->ki *   pid_struct->error                                 													//积分
                         +  pid_struct->kd *  (pid_struct->error - 2*(pid_struct->error_last) + pid_struct->error_last2); //微分
	
    pid_struct->output   += pid_struct->output_1;
	
	if (fabsf(pid_struct->error) > pid_struct->dead){
		pid_struct->output_1 *= 1;
	}else{
		pid_struct->output_1 *= 0.9;
	}
	
	 //输出限幅
   if(pid_struct->output >   pid_struct->maxOutput )  pid_struct->output = pid_struct->maxOutput;
   if(pid_struct->output < -(pid_struct->maxOutput))  pid_struct->output = -(pid_struct->maxOutput);

	return pid_struct->output;
}

float pid_location(pid* pid_struct,float now_value){

    //微分先行
    pid_struct->error_last2  =   pid_struct->error_last;              //保存上上次误差
    pid_struct->error_last   =   pid_struct->error;                   //保存上一次误差
    pid_struct->error 		 =   pid_struct->target - now_value;      //计算当前误差
	
	if (fabsf(pid_struct->error) >= pid_struct->dead){
		
        //积分处理
        pid_struct->integral   +=  pid_struct->error;
        if(pid_struct->integral >   pid_struct->maxintegral )    pid_struct->integral= pid_struct->maxintegral;
        if(pid_struct->integral < -(pid_struct->maxintegral))    pid_struct->integral=-pid_struct->maxintegral;

        //位置式PID计算
        pid_struct->output = pid_struct->kp *   pid_struct->error                                    //比例
                           + pid_struct->ki *   pid_struct->integral                                 //积分
                           + pid_struct->kd *  (pid_struct->error-pid_struct->error_last);           //微分
	}else{
        pid_struct->integral = 0;
		pid_struct->output = 0;
	}

    //输出限幅
    if(pid_struct->output >   pid_struct->maxOutput ) pid_struct->output=pid_struct->maxOutput;
    if(pid_struct->output < -(pid_struct->maxOutput)) pid_struct->output=-(pid_struct->maxOutput);

    return pid_struct->output;
}

void pid_init(void){
	
	PID_init(&pid_FL,60,10,100,1000,7000,0,1);
	PID_init(&pid_FR,60,10,100,1000,7000,0,1);
	PID_init(&pid_BL,60,10,100,1000,7000,0,1);
	PID_init(&pid_BR,50,10,90,1000,7000,0,1);
    PID_init(&pid_yaw,3,0.001,15,1000,60,0,0.1);
	PID_init(&pid_x,pid_x_p,pid_x_i,pid_x_d,1000,pid_x_speed,0,1);
	PID_init(&pid_y,pid_y_p,pid_y_i,pid_y_d,1000,pid_y_speed,0,1);

}

void pid_target(pid* pid_struct,float target){
    pid_struct -> target = target;
}

void pid_wheel_target(float FL,float FR,float BL,float BR){
	
	pid_target(&pid_FL,FL);
	pid_target(&pid_FR,FR);
	pid_target(&pid_BL,BL);
	pid_target(&pid_BR,BR);

}

void pid_position_target(float x,float y){

	pid_target(&pid_x,x);
	pid_target(&pid_y,y);
	
}

void pid_change(pid* pid_struct,float kp,float ki,float kd){
    pid_struct ->  kp  =kp;
    pid_struct ->  ki  =ki;
    pid_struct ->  kd  =kd;
}
