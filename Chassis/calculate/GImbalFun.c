#include "GimbalFun.h"
#include "GimbalTask.h"
#include "CanTask.h"
#include "RefereeTask.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
  * @attention
  */
float RAMP_float( float final, float now, float ramp )
{
    float buffer = 0;


    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {
            now += ramp;
        }
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于指针输入值
  * @param  要在当前输出量上累加的值,目标输出量,递增快慢
  * @retval 目标输出量
  * @attention
  *
*/
float RampInc_float( float *buffer, float now, float ramp )
{

    if (*buffer > 0)
    {
        if (*buffer > ramp)
        {
            now     += ramp;
            *buffer -= ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }
    else
    {
        if (*buffer < -ramp)
        {
            now     += -ramp;
            *buffer -= -ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }

    return now;
}

/********************keyboard云台控制********************/

void Gimbal_Mode_Choose(void)
{
		//右键按住自瞄
    if(ref_instance->origin_data.ext_remote_control.right_button_down == 1)
		Gimbal_action = GIMBAL_AUTO;
	else 
		Gimbal_action = GIMBAL_NORMAL;
}


/**/
/**
  * @brief  键鼠控制Yaw函数
  * @param  void
  * @retval void
  * @attention
  */
void KeyboardControlGimbal(void)
{
	static portTickType  Key_Ctrl_CurrentTime = 0;
	static uint32_t Mouse_Yaw_Stop  = 0;
	float Mouse_Gyro_Yaw; //键盘陀螺仪模式下鼠标统计yaw偏移量,此值会自己缓慢减小,防止甩头过快
	
	if(Gimbal_action == GIMBAL_NORMAL)
	{
		Key_Ctrl_CurrentTime = xTaskGetTickCount();//获取实时时间,用来做按键延时判断

		/////普通控制部分
		if(ref_instance->origin_data.ext_remote_control.mouse_x != 0)
		{
			Mouse_Gyro_Yaw += ref_instance->origin_data.ext_remote_control.mouse_x * 0.00005f;//yaw仍旧使用机械模式
		}
		else if(ref_instance->origin_data.ext_remote_control.mouse_x == 0)
		{
			Mouse_Yaw_Stop ++ ;
			if(Mouse_Yaw_Stop > 5) //鼠标长时间停留，停止移动
			{
				Mouse_Gyro_Yaw = RAMP_float(0, Mouse_Gyro_Yaw, 0.01);
			}
		}
		yaw_motor->target_position = RampInc_float( &Mouse_Gyro_Yaw, yaw_motor->target_position, 0.0004);
	}	
	else if(Gimbal_action == GIMBAL_AUTO)
	{
		yaw_motor->target_position = Gimbal_msg.vison_yaw.value;
	}
}



