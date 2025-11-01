#include "GimbalTask.h"
#include "cmsis_os.h"
#include "dev_motor_lk.h"
#include "remote_control.h"

LkMotorInstance_s *yaw_motor;
static LkMotorInitConfig_s config = {
	.id = 1,
	.topic_name = "1",
	.type = MF9025,
	.control_mode = LK_VELOCITY,
	.reduction_ratio = 1,
	.can_config = {
	.can_number=1,
	.rx_id=0x141,	
	.tx_id=0x280,
	},
	.velocity_pid_config={
    .kp = 1.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .i_max = 100.0f,
    .out_max = 1000.0f,
    },
	.angle_pid_config={
	.kp = 0.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.angle_max =2*PI,	
	.i_max = 0.0f,
	.out_max = 0.0f,
	},
};
void GimbalTask(void const * argument)
{
	yaw_motor = Motor_Lk_Register(&config);
    for(;;){
		Motor_Lk_Control(yaw_motor,5*rc_ctrl.rc.ch[0]);
		Motor_LK_Transmit(yaw_motor);
		osDelay(1);
	}
}
