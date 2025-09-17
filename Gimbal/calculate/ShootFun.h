#ifndef __SHOOTFUN_H__
#define __SHOOTFUN_H__

#include "ShootTask.h"


void Shoot_Init(Shoot_t* shoot);
void Shoot_Wheel_Init(void);
void Shoot_flag_Init(Shoot_t* shoot);
void Clean_Shoot_CtrlMsg(Shoot_t* shoot);

void Shoot_Mode_choose_remote(Shoot_t* shoot);
void Shoot_Mode_Choose(Shoot_t* shoot);

extern void Shoot_clc(void);
void Shoor_Ctl_STOP(Shoot_t* shoot);
void Shoor_Ctl_SINGLE(Shoot_t* shoot);
void Shoor_Ctl_NORMAL(Shoot_t* shoot);


extern void ShootSelate(Shoot_t* shoot);
extern void Shoot_Statemachine_2_Init(Shoot_t* shoot);
extern void Shoot_Statemachine_2_Run(Shoot_t* shoot);
extern void Shoot_Statemachine_2_Stop(Shoot_t* shoot);


#endif

