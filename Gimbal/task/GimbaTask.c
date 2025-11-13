#include "GimbalTask.h"
#include "cmsis_os.h"
#include "dev_motor_dm.h"
#include "remote_control.h"

DmMotorInstance_s *pitch = NULL;

DmMotorInitConfig_s pitch_config={
    .can_config = {
        .can_number = 2,
        .tx_id = 0x02,
        .rx_id = 0x12,
    },
    .parameters = { 
        .pos_max = 3.14159f,
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
        .kp = 3.0f,
        .ki = 0.052f,
        .kd = 0.0f,
        .i_max = 4.0f,
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
		pitch->target_position += 0.0000001f*rc_ctrl.rc.ch[1];
		if(pitch->target_position < -0.04f)
			pitch->target_position = -0.04f;
		else if (pitch->target_position > 0.14f)
			pitch->target_position = 0.14f;
		Motor_Dm_Control(pitch, pitch->target_position); 
        Motor_Dm_Mit_Control(pitch, 0.0f, 0.0f, pitch->output); 
        Motor_Dm_Transmit(pitch); //发送控制报文
        osDelay(1);
	}
}
