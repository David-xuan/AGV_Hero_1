#include "ChassisFun.h"
#include "ChassisTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "math.h"
#include "RefereeTask.h"

void Chassis_Choose_keyboard(ChassisInstance_s *Chassis)
{
	if(ref_instance->origin_data.ext_remote_control.keyboard_value != 512)
		rc_ctrl.flag.F = 0;
	if(ref_instance->origin_data.ext_remote_control.keyboard_value == 512 
	   && rc_ctrl.flag.F == 0) 
	{
		rc_ctrl.flag.F = 1;
		if(Chassis->Chassis_Mode == CHASSIS_NORMAL)
		{
			Chassis->Chassis_Mode = CHASSIS_FOLLOW_GIMBAL;
		}
		else
		{
			Chassis->Chassis_Mode = CHASSIS_NORMAL;
		}
	
	}

	if(ref_instance->origin_data.ext_remote_control.keyboard_value == 16)
	{
		Chassis->Chassis_Mode = CHASSIS_GYROSCOPE;
	}
	else if(ref_instance->origin_data.ext_remote_control.keyboard_value != 16 
			&& Chassis->Chassis_Mode == CHASSIS_GYROSCOPE)		
	{
		Chassis->Chassis_Mode = CHASSIS_FOLLOW_GIMBAL;
	}
}

void Chassis_Choose_remote(ChassisInstance_s *Chassis)
{
	if(rc_ctrl.rc.s[1] == 1 && rc_ctrl.rc.s[0] == 1)
		Chassis_Change_Mode(Chassis,CHASSIS_FOLLOW_GIMBAL);
	else if(rc_ctrl.rc.s[1] == 3)
		Chassis_Change_Mode(Chassis,CHASSIS_NORMAL);
	else if(rc_ctrl.rc.s[1] == 1 && rc_ctrl.rc.s[0] == 3)
		Chassis_Change_Mode(Chassis,CHASSIS_GYROSCOPE);
}

/**
  * @brief  键盘模式下底盘运动计算
  * @param  速度最大输出量    
  * @retval void
  * @attention  键盘控制前后左右平移,平移无机械和陀螺仪模式之分
  *             需要获取时间来进行斜坡函数计算
  */
/************底盘各类模式的一些辅助定义*************/
int16_t  timeXFron, timeXBack, timeYLeft, timeYRigh;//键盘  s  w  d  a

//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back;
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;
void Chassis_Keyboard_Move_Calculate( float sMoveMax, int16_t sMoveRamp_inc, int16_t sMoveRamp_dec )
{
    static portTickType  ulCurrentTime = 0;
    static uint32_t  ulDelay = 0;

	static uint16_t w_cnt = 0;
	static bool W = 0;
	static uint16_t s_cnt = 0;
	static bool S = 0;
    static uint16_t a_cnt = 0;
    static bool A = 0;
    static uint16_t d_cnt = 0;
    static bool D = 0;
	

    ulCurrentTime = xTaskGetTickCount();//当前系统时间

    if (ulCurrentTime >= ulDelay)//每10ms变化一次斜坡量
    {
        ulDelay = ulCurrentTime + 10;

        if (ref_instance->origin_data.ext_remote_control.keyboard_value == 1)
        {
            w_cnt = 0;
            W = 1;
            timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
        }
        else
        {
            w_cnt++;
        }
        if(w_cnt > 10)
        {
            w_cnt = 0;
            W = 0;
        }

        if (ref_instance->origin_data.ext_remote_control.keyboard_value == 2)
        {
            s_cnt = 0;
            S = 1;
            timeXFron = 0;//同理
        }
        else
        {
            s_cnt++;
        }
        if(s_cnt > 10)
        {
            s_cnt = 0;
            S = 0;
        }

        if (ref_instance->origin_data.ext_remote_control.keyboard_value == 8)
        {
            d_cnt = 0;
            D=1;
            timeYRigh = 0;
        }
        else
        {
            d_cnt++;
        }
        if(d_cnt > 10)
        {
            d_cnt = 0;
            D = 0;
        }

        if (ref_instance->origin_data.ext_remote_control.keyboard_value == 4)
        {
            a_cnt = 0;
            A=1;
            timeYLeft = 0;
        }
        else
        {
            a_cnt++;
        }
        if(a_cnt > 10)
        {
            a_cnt = 0;
            A = 0;
        }

        Slope_Chassis_Move_Fron =  sMoveMax *
                                             Chassis_Key_MoveRamp( W, &timeXFron, sMoveRamp_inc , sMoveRamp_dec ) ;
        
		Slope_Chassis_Move_Back =  -sMoveMax *
                                             Chassis_Key_MoveRamp( S, &timeXBack, sMoveRamp_inc , sMoveRamp_dec ) ;

        Slope_Chassis_Move_Left =  sMoveMax *
                                             Chassis_Key_MoveRamp( A, &timeYRigh, sMoveRamp_inc , sMoveRamp_dec ) ;

        Slope_Chassis_Move_Righ =  -sMoveMax *
                                             Chassis_Key_MoveRamp( D, &timeYLeft, sMoveRamp_inc , sMoveRamp_dec ) ;
		
		Chassis->Chassis_speed.Vx = Slope_Chassis_Move_Fron + Slope_Chassis_Move_Back;
		Chassis->Chassis_speed.Vy = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;
	}
}

void LimitValue_f(float*VALUE, float MAX, float MIN)
{
    if(*VALUE > MAX)
        *VALUE = MAX;
    else if(*VALUE < MIN)
        *VALUE = MIN;
}

/**
  * @brief  底盘键盘斜坡函数
  * @param  判断按键是否被按下, 时间量, 每次增加的量, 一共要减小的量
  * @retval 斜坡比例系数
  * @attention  0~1
  */
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
    float  factor = 0;
    factor = 0.1 * sqrt( 0.1 * (*time) );  //计算速度斜坡,time累加到296.3斜坡就完成

    if (status == 1){  //按键被按下
		
        if (factor < 1)//防止time太大
			*time += inc;
		
    }else{  //按键松开
        if (factor > 0)
        {
            *time -= dec;
            if (*time < 0)
				*time = 0;
        }
    }
    LimitValue_f(&factor,1,0);//注意一定是float类型限幅
	
    return factor;  //注意方向
}


