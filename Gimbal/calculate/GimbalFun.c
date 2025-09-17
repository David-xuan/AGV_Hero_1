#include "GimbalFun.h"
#include "DJI3508.h"
#include "pid.h"

void Gimbal_Init(Gimbal_t* gimbal)
{
	
	float M3508_spdpid[3] = {32 , 0.78 , 0};
	float M3508_angpid[3] = {120, 0, 5};
	PID_init(&M3508_speedloop[0] , PID_POSITION , M3508_spdpid , 800 , 16383);
	PID_init(&M3508_angleloop , PID_POSITION , M3508_angpid , 100 , 1500);

	Motor_3508[0].total_angle = 0;
	Motor_3508[0].target_angle = 0;
}
