#include "bsp_can.h"
#include "Shoot_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "DJI3508.h"
#include "my_fun.h"

void Shoot_task(void const * argument)
{
	float M3508_spdpid[3] = {0, 0 ,0};
	float M3508_angpid[3] = {0, 0, 0};
	PID_init(&M3508_speedloop , PID_POSITION , &M3508_spdpid[3] , 2000 , 16383);
	PID_init(&M3508_angleloop , PID_POSITION , &M3508_angpid[3] , 10 , 80);
	for(;;)
	{
		Motor_3508[4].target_angle += PI/8;
		AngleLoop_f(&Motor_3508[4].target_angle , 2*PI);
		PID_Angle_calc(&M3508_angleloop , Motor_3508[4].angle , Motor_3508[4].target_angle , -PI , PI);
		Motor_3508[4].target_speed_rpm = M3508_angleloop.out;
		PID_calc(&M3508_speedloop , Motor_3508[4].speed_rpm , Motor_3508[4].target_speed_rpm);
		Motor_3508_send(&hcan1 , 0x1FF , 0 , 0 , 0 , 0);//M3508_speedloop.out
		osDelay(2000);
	}
}
