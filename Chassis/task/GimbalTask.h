#ifndef GIMBALTASK_H
#define GIMBALTASK_H

#include "main.h"
#include "dev_motor_lk.h"

typedef enum  
{
	GIMBAL_NORMAL 	= 0x00,  //正常模式
	GIMBAL_AUTO    = 0x01,  //自瞄
}Gimbal_action_t;

extern LkMotorInstance_s *yaw_motor;
extern Gimbal_action_t Gimbal_action;


#endif
