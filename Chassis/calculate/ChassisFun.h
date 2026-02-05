#ifndef __CHASSISFUN_H__
#define __CHASSISFUN_H__
#include "main.h"
#include "ChassisTask.h"

void Chassis_Keyboard_Move_Calculate( float sMoveMax, int16_t sMoveRamp_inc, int16_t sMoveRamp_dec );
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );

void Chassis_Choose_keyboard(ChassisInstance_s *Chassis);
void Chassis_Choose_remote(ChassisInstance_s *Chassis);

#endif 
