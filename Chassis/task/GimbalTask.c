#include "GimbalTask.h"
#include "cmsis_os.h"
#include "dev_motor_lk.h"
#include "remote_control.h"

LkMotorInstance_s *yaw_motor;
float target_position = 0;
static LkMotorInitConfig_s config = {
	.id = 1,
	.topic_name = "1",
	.type = MF9025,
	.control_mode = LK_POSITION,
	.reduction_ratio = 1,
	.can_config = {
	.can_number=1,
	.rx_id=0x141,	
	.tx_id=0x280,
	},
	.velocity_pid_config={
    .kp = 0.88f,
    .ki = 0.02f,
    .kd = 0.0f,
    .i_max = 200.0f,
    .out_max = 1000.0f,
    },
	.angle_pid_config={
	.kp = 1100.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.angle_max =2*PI,	
	.i_max = 200.0f,
	.out_max = 800.0f,
	},
};
void GimbalTask(void const * argument)
{
	yaw_motor = Motor_Lk_Register(&config);
	yaw_motor->output = 0;
	while(target_position == 0)
    {
		Motor_LK_Transmit(yaw_motor);
		target_position = yaw_motor->out_position;
		osDelay(1);
	}
    for(;;){
		target_position += 0.000001*rc_ctrl.rc.ch[0];
		while(target_position < -PI)
			target_position += 2*PI;
		while(target_position > PI)
			target_position -= 2*PI;
		Motor_Lk_Control(yaw_motor,target_position);
		Motor_LK_Transmit(yaw_motor);
		osDelay(1);
	}
}
