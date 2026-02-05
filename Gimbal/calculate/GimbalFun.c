#include "GimbalFun.h"
#include "GimbalTask.h"
#include "MinipcTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can.h"
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

/*键鼠选择云台模式*/
void Gimbal_Mode_Choose(void)
{
	//右键按住自瞄
    if(IF_MOUSE_PRESSED_RIGH)
		Gimbal_action  = GIMBAL_AUTO;
	else
		Gimbal_action = GIMBAL_NORMAL;
}

/* Keyboard控制云台函数 */
void KeyboardControlGimbal(void)
{
    static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响?
    static float Mouse_Gyro_Pitch;  //键盘陀螺仪模式下鼠标统计Pitch偏移量,此值会自己缓慢减小,防止甩头过快
	if(Gimbal_action == GIMBAL_NORMAL)
	{
		if(MOUSE_Y_MOVE_SPEED != 0)
        {
           Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * 0.000008f;//pitch仍旧使用机械模式
        }
        else if(MOUSE_Y_MOVE_SPEED == 0)
        {
            Mouse_Pitch_Stop ++ ;
            if(Mouse_Pitch_Stop > 5) //鼠标长时间停留，停止移动
            {
                Mouse_Gyro_Pitch = RAMP_float(0, Mouse_Gyro_Pitch, 0.01);
            }
        }
        pitch->target_position = RampInc_float( &Mouse_Gyro_Pitch, pitch->target_position, 0.0004);
	}
	else if(Gimbal_action == GIMBAL_AUTO)
	{
			pitch->target_position = -minipc->message.norm_aim_pack.pitch;
	}
}

