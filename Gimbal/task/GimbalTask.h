#ifndef GIMBALTASK_H
#define GIMBALTASK_H

#include "main.h"
#include "remote_control.h"
#include "dev_motor_dm.h"

#define PI 3.141593f // 圆周率要换位置

typedef enum  
{
	GIMBAL_NORMAL 	= 0x00,  //正常模式
	GIMBAL_AUTO    = 0x01,  //自瞄
}Gimbal_action_t;

extern DmMotorInstance_s *pitch;
extern Gimbal_action_t Gimbal_action;

#endif
