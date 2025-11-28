#include "ShootTask.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "ShootFun.h"

Shoot_t Shoot;

DmMotorInstance_s *rammer = NULL;
DmMotorInitConfig_s rammer_config={
    .can_config = {
        .can_number = 2,
        .tx_id = 0x01,
        .rx_id = 0x11,
    },
    .parameters = { 
        .pos_max = 3.14159f,
        .vel_max = 30.0f,   
        .tor_max = 10.0f,
        .kp_max = 100.0f,
        .kd_max = 10.0f,
        .kp_int = 2.0f,
        .kd_int = 1.0f,
    },
    .control_mode = DM_POSITION,
    .topic_name = "1",
    .type = J4310,
    .velocity_pid_config = {
        .kp = 1.0f,
        .ki = 0.075f,
        .kd = 1.5f,
        .i_max = 2.0f,
        .out_max = 3.0f,
	},
	.angle_pid_config = {
		.kp = 30.0f,
		.ki = 0.0f,
		.kd = 5.0f,
		.i_max = 3.0f,
		.out_max = 8.0f,
		.angle_max = 2*PI,
	},
};

DjiMotorInstance_s *wheel[6];
ShootInitConfig_s shoot_config = {
	.config[0] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 1.0f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
	.rx_id=0x201,	
	.tx_id=0x200,
    },
    .velocity_pid_config={
    .kp = 40.0f,
    .ki = 0.3f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
	.config[1] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 1.0f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
	.rx_id=0x202,	
	.tx_id=0x200,
    },
    .velocity_pid_config={
    .kp = 40.0f,
    .ki = 0.3f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
	.config[2] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 1.0f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
	.rx_id=0x203,	
	.tx_id=0x200,
    },
    .velocity_pid_config={
    .kp = 40.0f,
    .ki = 0.3f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
	.config[3] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 1.0f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
	.rx_id=0x204,	
	.tx_id=0x200,
    },
    .velocity_pid_config={
    .kp = 40.0f,
    .ki = 0.3f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
	.config[4] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 1.0f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
	.rx_id=0x205,	
	.tx_id=0x1FF,
    },
    .velocity_pid_config={
    .kp = 40.0f,
    .ki = 0.3f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
	.config[5] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 1.0f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
	.rx_id=0x206,	
	.tx_id=0x1FF,
    },
    .velocity_pid_config={
    .kp = 40.0f,
    .ki = 0.3f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
};

void ShootTask(void const * argument)
{
	rammer = Motor_DM_Register(&rammer_config);
    Motor_Dm_Cmd(rammer, DM_CMD_MOTOR_ENABLE);
    while(rammer->message.out_position == 0)
    {
		rammer->output = 0;
		Motor_Dm_Transmit(rammer);
		rammer->target_position = rammer->message.out_position;
		osDelay(1);
	}
	
	for(int i=0;i<6;i++)
	{
		wheel[i] = Motor_Dji_Register(&shoot_config.config[i]);
	}
	for(;;)
	{	

		if(rc_ctrl.rc.s[0] == 3)
		{
			Motor_Dji_Control(wheel[0], -4950); 
			Motor_Dji_Control(wheel[1], -4950); 
			Motor_Dji_Control(wheel[2], -4950); 
			Motor_Dji_Control(wheel[3], -4950); 
			Motor_Dji_Control(wheel[4], -4950); 
			Motor_Dji_Control(wheel[5], -4950); 
		}
		else
		{
			for(int i = 0;i<6;i++)
			{
				wheel[i]->output = 0;
			}
		}
		Motor_Dji_Transmit(wheel[0]); 
		Motor_Dji_Transmit(wheel[4]);
		
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
				Shoot_clc();
				break;
			}
		  case SHOOT_STOP:
			{
				Clean_Shoot_CtrlMsg(&Shoot);
				break;
			}
		}
		
        Motor_Dm_Mit_Control(rammer, 0.0f, 0.0f, rammer->output); 
        Motor_Dm_Transmit(rammer); 
        osDelay(1);
	}
}
