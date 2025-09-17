#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "GimbalFun.h"
#include "DJI3508.h"
#include "remote_control.h"
#include "pid.h"
#include "bsp_can.h"

Gimbal_t Gimbal;
void Gimbal_task(void const * argument)
{
	Gimbal_Init(&Gimbal);
	for(;;)
	{
		Get_Total_Angle(&Motor_3508[0]);

		Motor_3508[0].target_angle -= 0.00008*rc_ctrl.rc.ch[1];
		
		PID_calc(&M3508_angleloop , Motor_3508[0].total_angle , Motor_3508[0].target_angle);
		Motor_3508[0].target_speed_rpm = M3508_angleloop.out;
		PID_calc(&M3508_speedloop[0] , Motor_3508[0].speed_rpm , Motor_3508[0].target_speed_rpm);
		Motor_3508[0].target_current = M3508_speedloop[0].out;
		
		Motor_3508_send(&hcan1 , 0x200 , Motor_3508[0].target_current , 0 , 0 ,0);//Motor_3508[0].target_current
		
		osDelay(1);
	}
}
