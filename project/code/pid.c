#include "pid.h"
#include "zf_common_headfile.h"

pid pid_FL,pid_FR,pid_BL,pid_BR,pid_x,pid_y;

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

int pid_increm(pid* pid_struct,float now_value){
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
	PID_init(&pid_x,pid_x_p,pid_x_i,pid_x_d,1000,pid_x_speed,0,1);
	PID_init(&pid_y,pid_y_p,pid_y_i,pid_y_d,1000,pid_y_speed,0,1);

}

void pid_target(pid* pid_struct,float target){
    pid_struct -> target = target;
}

void pid_yaw_target(float yaw){

	if (yaw > 180) yaw -= 360;
	else if (yaw < -180) yaw += 360;

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

// ================= 角度环双KD控制器 =================
float pid_yaw(float target, float current) {
    // ===== 可调参数 =====
    static const float kp         = 2.0f;    // 线性P增益
    static const float kp2        = 0.1f;    // 平方P增益
    static const float kd         = 25.0f;   // 陀螺仪D
    static const float ki         = 0.005f;  // 积分增益 (仅大误差激活)
    static const float dead       = 0.3f;    // 死区
    static const float i_thr      = 2.0f;    // 积分激活阈值
    static const float max_i      = 1000.0f;  // 积分限幅
    static const float max_out    = 60.0f;   // 输出限幅
    static float integral = 0;

    // 1. 误差计算 (角度归一化, 处理 ±180° 穿越)
    float error = target - current;
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 2. 死区: 微小偏差不修正, 避免直行抖动
    if (fabsf(error) < dead) {
        integral = 0;
        return 0;
    }

    // 3. 线性P + 平方P
    float p_out = kp * error + kp2 * error * fabsf(error);

    // 4. 陀螺仪D: 直接用角速度做阻尼, 无微分噪声
    float d_out = kd * gyro_z;

    // 5. 条件积分: 仅大误差时激活, 消除弯道静差
    float i_out = 0;
    if (fabsf(error) > i_thr) {
        integral += error;
        if (integral >  max_i) integral =  max_i;
        if (integral < -max_i) integral = -max_i;
        i_out = ki * integral;
    } else {
        integral = 0;
    }

    // 6. 合成输出
    float output = p_out + d_out + i_out;

    // 7. 输出限幅
    if (output >  max_out) output =  max_out;
    if (output < -max_out) output = -max_out;

    return output;
}
