#include "control.h"
#include "zf_common_headfile.h"

void motor_solution(float vx_body, float vy_body, float current_yaw, float wz){
	
	static float vx = 0,vy = 0;
	if (vx < vx_body){
		vx += 1;
		if (vx >= vx_body) vx = vx_body;
	}else if (vx > vx_body){
		vx -= 1;
		if (vx <= vx_body) vx = vx_body;
	}
	if (vy < vy_body){
		vy += 1;
		if (vy >= vy_body) vy = vy_body;
	}else if (vy > vy_body){
		vy -= 1;
		if (vy <= vy_body) vy = vy_body;
	}


     float FL = vx - vy - 0.75f * wz;
     float FR = vx + vy + 0.75f * wz;
     float BL = vx + vy - 0.75f * wz;
     float BR = vx - vy + 0.75f * wz;

     pid_wheel_target(FL, FR, BL, BR);
}

