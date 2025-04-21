#include "ShootFun.h"
#include "pid.h"
#include "DM4310.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "my_fun.h"

/*声明变量or函数*/
void Shoot_Statemachine_2_Init(Shoot_t* shoot);
void Shoot_Statemachine_2_Run(Shoot_t* shoot);
void Shoot_Statemachine_2_Stop(Shoot_t* shoot);


void Shoot_Init(Shoot_t* shoot)
{
	Shoot_flag_Init(shoot);
	float DM4310_Rammer_angloop[3]	  = {15,0,5};		
	float DM4310_Rammer_spdloop[3] 		= {0.35,0.03,1.5};//{0.4,0.1,1.5};


	
	PID_init(&DM4310_Rammer_angleloop,PID_POSITION,DM4310_Rammer_angloop,10,5);
	PID_init(&DM4310_Rammer_speedloop,PID_POSITION,DM4310_Rammer_spdloop,2,3);

	Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;
	Motor_4310_Rammer.target_speed_rpm = Motor_4310_Rammer.speed_rpm;
}

void Shoot_Wheel_Init(void)
{
	float M3508_spdpid[3] = {8, 0.1 ,0.2};
//	float M3508_angpid[3] = {0, 0, 0};
//	PID_init(&M3508_speedloop[4] , PID_POSITION , M3508_spdpid , 5000 , 16383);
//	PID_init(&M3508_angleloop , PID_POSITION , M3508_angpid , 10 , 80);
	PID_init(&M3508_speedloop[0] , PID_DELTA , M3508_spdpid , 2000 , 16383);
	PID_init(&M3508_speedloop[1] , PID_DELTA , M3508_spdpid , 2000 , 16383);		
	PID_init(&M3508_speedloop[2] , PID_DELTA , M3508_spdpid , 2000 , 16383);	
}

void Shoot_flag_Init(Shoot_t* shoot)
{
	shoot->flag.shoot_finish_flag 		= 1;	
	shoot->flag.left_shoot_flag 			= 0;
	shoot->flag.wheel_shoot_flag 			= 0;	
	shoot->flag.shoot_lock_flag  			= 0;
	
	shoot->State 	= SHOOT_Init;
	shoot->Action = NORMAL;
}

//清除控制量及pid
void Clean_Shoot_CtrlMsg(Shoot_t* shoot)
{
		Motor_4310_Rammer.target_torque = 0;
		PID_clear(&DM4310_Rammer_angleloop);
		PID_clear(&DM4310_Rammer_speedloop);
}

/****************************************************************拨弹盘模式选择****************************************************************/
void Shoot_Mode_Choose(Shoot_t* shoot)
{
	if(rc_ctrl.rc.s[1] == 3)//仅在遥控模式下开启摩擦轮后可用
		Shoot_Mode_choose_remote(shoot);
}


/*遥控器模式选择*/
void Shoot_Mode_choose_remote(Shoot_t* shoot)
{
				if(rc_ctrl.rc.ch[4]>500&&shoot->flag.wheel_shoot_flag==0)
				{
					shoot->Action = SINGLE;
				}
				else if(rc_ctrl.rc.ch[4]<100&&rc_ctrl.rc.ch[4]>-100)
				{
					shoot->flag.wheel_shoot_flag = 0;
					shoot->Action = NORMAL;
				}
}


/****************************************************************拨弹盘计算****************************************************************/
void Shoot_clc(void)
{
	switch(Shoot.Action)
	{
		case NORMAL:
		{
			Shoor_Ctl_NORMAL(&Shoot);		//计算并保持当前位置				
			break;
		}
		case SINGLE:
		{
			Shoor_Ctl_SINGLE(&Shoot);		//单发
			break;
		}
		case STOP:
		{			
			Shoor_Ctl_STOP(&Shoot);			//本用于防卡弹，但疑似会因此出现双发（并不确定，感觉可能性不大但是保险起见暂时不做卡弹处理）
			break;
		}    
	}

}
void Shoor_Ctl_NORMAL(Shoot_t* shoot)
{
	AngleLoop_f(&Motor_4310_Rammer.target_angle,2*PI);//角度回环
	PID_Angle_calc(&DM4310_Rammer_angleloop,Motor_4310_Rammer.angle,Motor_4310_Rammer.target_angle,-PI,PI);//角度环
	Motor_4310_Rammer.target_speed_rpm = DM4310_Rammer_angleloop.out;
	PID_calc(&DM4310_Rammer_speedloop,Motor_4310_Rammer.speed_rpm,Motor_4310_Rammer.target_speed_rpm);//速度环
	Motor_4310_Rammer.target_torque = DM4310_Rammer_speedloop.out;//算出扭矩
/********************************卡弹判断*************************/
	if(fabsf(Motor_4310_Rammer.target_angle - Motor_4310_Rammer.angle)<0.035f) //1°以内
	{
		shoot->flag.shoot_finish_flag = 1;
		shoot->flag.shoot_finish_time = 0;
	}
	if(!shoot->flag.shoot_finish_flag)
	{
		shoot->flag.shoot_finish_time++;
		if(shoot->flag.shoot_finish_time>2500)   
		{	
			shoot->flag.shoot_lock_flag = 1;
			shoot->flag.shoot_finish_flag = 1;
			Motor_4310_Rammer.target_angle -= 2*1.04719f;
			AngleLoop_f(&Motor_4310_Rammer.target_angle,2*PI);		
		}
	}
/***************裁判系统发射机构断电后保持现位，防止上电发射************/
//	if(!Game_Robot_State.power_management_shooter_output)
//		shoot->flag.judge_on_flag = 0;
//	else if(Game_Robot_State.power_management_shooter_output == 1&&shoot->flag.judge_on_flag == 0)
//	{
//		shoot->flag.judge_on_flag = 1;
//		Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;
//	}
}




void Shoor_Ctl_SINGLE(Shoot_t* shoot)
{
	if((!shoot->flag.left_shoot_flag)||(!shoot->flag.wheel_shoot_flag))  //未执行过
	{
		Motor_4310_Rammer.target_angle += 1.04719f;		
		shoot->flag.shoot_finish_flag = 0;		
		shoot->flag.left_shoot_flag  	= 1;
		shoot->flag.wheel_shoot_flag 	= 1;		
		shoot->Action = NORMAL;	
	}
}

//void Shoor_Ctl_BERSERK(Shoot_t* shoot)
//{
//	motor.Rammer.Vpid.goal = 4;
//	DM_speed_control(&motor.Rammer);
//}

void Shoor_Ctl_STOP(Shoot_t* shoot)
{
	Motor_4310_Rammer.target_torque = 0;
	shoot->flag.shoot_stop_time++;
	if(shoot->flag.shoot_stop_time>500) 		//放松1s
	{
		shoot->flag.shoot_stop_time = 0;	
		Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;
		shoot->flag.shoot_finish_time = 0;
		shoot->flag.shoot_lock_flag = 0;
		shoot->Action = NORMAL;
	}
}

/****************************************************************状态机控制****************************************************************/

/* 状态机控制器 */
void ShootSelate(Shoot_t* shoot)
{
		if(rc_ctrl.rc.s[1] == 3)
			Shoot_Statemachine_2_Run(shoot);
		else 
			Shoot_Statemachine_2_Stop(shoot);

}


/* 把拨弹状态机切入初始化模式 */
void Shoot_Statemachine_2_Init(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_Init)
	{
		Clean_Shoot_CtrlMsg(shoot);
		//切过去
		shoot->State = SHOOT_Init;
	}
}


/* 把拨弹盘状态机切入Run模式 */
void Shoot_Statemachine_2_Run(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_Run)
	{
		Clean_Shoot_CtrlMsg(shoot);
		Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;
		//切过去
		shoot->State = SHOOT_Run;
	}
}

/* 把拨弹盘状态机切入Stop模式 */
void Shoot_Statemachine_2_Stop(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_STOP)
	{	
		Clean_Shoot_CtrlMsg(shoot);		
		//切过去
		shoot->State = SHOOT_STOP;
	}
}

