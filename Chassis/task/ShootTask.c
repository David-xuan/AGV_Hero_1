#include "bsp_can.h"
#include "ShootTask.h"
#include "ShootFun.h"
#include "cmsis_os.h"
#include "pid.h"
#include "DJI3508.h"
#include "DM4310.h"
#include "my_fun.h"
#include "remote_control.h"

Shoot_t Shoot;
void ShootTask(void const * argument)
{
	Shoot_Init(&Shoot);	
	Shoot_Wheel_Init();
	for(;;)
	{
/*
		Motor_3508[4].target_angle += PI/8;
		AngleLoop_f(&Motor_3508[4].target_angle , 2*PI);
		Get_Total_Angle(&Motor_3508[4]);
		PID_Angle_calc(&M3508_angleloop , Motor_3508[4].total_angle , Motor_3508[4].target_angle , -PI , PI);
		Motor_3508[4].target_speed_rpm = M3508_angleloop.out;
		PID_calc(&M3508_speedloop[4] , Motor_3508[4].speed_rpm , Motor_3508[4].target_speed_rpm);
*/
		Motor_DM4310_Enable(&hcan1,RAMMER_CANID);
		if(rc_ctrl.rc.s[1] == 3)
		{
			PID_calc(&M3508_speedloop[0] , Motor_3508[0].speed_rpm , -4950);
			PID_calc(&M3508_speedloop[1] , Motor_3508[1].speed_rpm , 4900);
			PID_calc(&M3508_speedloop[2] , Motor_3508[2].speed_rpm , -4900);
		}
		else
		{
			PID_calc(&M3508_speedloop[0] , Motor_3508[0].speed_rpm , 0);
			PID_calc(&M3508_speedloop[1] , Motor_3508[1].speed_rpm , 0);
			PID_calc(&M3508_speedloop[2] , Motor_3508[2].speed_rpm , 0);
		}
		Motor_3508_send(&hcan1 , 0x200 , M3508_speedloop[0].out , M3508_speedloop[1].out , M3508_speedloop[2].out , 0);//M3508_speedloop.out
		ShootSelate(&Shoot);//状态控制(停止/运行)
		switch(Shoot.State)
		{
			case SHOOT_Init:
			{
				Shoot_Statemachine_2_Stop(&Shoot);
				break;			
			}
			case SHOOT_Run:
			{
				Shoot_Mode_Choose(&Shoot);//发射操控模式选择
				Shoot_clc();//根据不同发射模式计算target_torque
				break;
			}
		  case SHOOT_STOP:
			{
				Clean_Shoot_CtrlMsg(&Shoot);
				break;
			}
		}
		Motor_DM4310_send(&hcan1,RAMMER_CANID, 0, 0, 0, 0, Motor_4310_Rammer.target_torque);//Motor_4310_Rammer.target_torque

		osDelay(1);
	}
}
