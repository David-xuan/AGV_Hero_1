#include "GimbalTask.h"
#include "GimbalFun.h"
#include "cmsis_os.h"
#include "dev_motor_dm.h"
#include "remote_control.h"
#include "MinipcTask.h"

DmMotorInstance_s *pitch = NULL;
Gimbal_action_t Gimbal_action;

DmMotorInitConfig_s pitch_config={
    .can_config = {
        .can_number = 2,
        .tx_id = 0x12,
        .rx_id = 0x22,
    },
    .parameters = { 
        .pos_max = 12.5f,
        .vel_max = 20.0f,   
        .tor_max = 28.0f,
        .kp_max = 100.0f,
        .kd_max = 10.0f,
        .kp_int = 2.0f,
        .kd_int = 1.0f,
    },
    .control_mode = DM_POSITION,
    .topic_name = "1",
    .type = J4340,
    .velocity_pid_config = {
        .kp = 10.0f,
        .ki = 0.052f,
        .kd = 0.0f,
        .i_max = 6.0f,
        .out_max = 10.0f,
    },
	.angle_pid_config = {
		.kp = 70.0f,
		.ki = 0.0f,
		.kd = 0.0f,
		.i_max = 3.0f,
		.out_max = 5.0f,
		.angle_max = 2*PI,
	},
};

void GimbalTask(void const * argument)
{
	pitch = Motor_DM_Register(&pitch_config);
    Motor_Dm_Cmd(pitch, DM_CMD_MOTOR_ENABLE);
	while(pitch->target_position == 0)
    {
		Motor_Dm_Transmit(pitch);
		pitch->target_position = pitch->message.out_position;
		osDelay(1);
	}
	for(;;)
	{	
		if(rc_ctrl.rc.s[1] == 2)
		{
			Gimbal_Mode_Choose();
			KeyboardControlGimbal();
		}
		else
		{
			if(rc_ctrl.rc.s[0] == 2)
				pitch->target_position = -minipc->message.norm_aim_pack.pitch;
			else
				pitch->target_position += 0.0000003f*rc_ctrl.rc.ch[1];
		}
		
		if(pitch->target_position < -0.07f)
			pitch->target_position = -0.07f;
		else if (pitch->target_position > 0.68f)
			pitch->target_position = 0.68f;
		Motor_Dm_Control(pitch, pitch->target_position); 
        Motor_Dm_Mit_Control(pitch, 0.0f, 0.0f, pitch->output); 
        Motor_Dm_Transmit(pitch); //发送控制报文
        osDelay(1);
	}
}
