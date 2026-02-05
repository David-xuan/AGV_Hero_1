#include "ChassisTask.h"
#include "ChassisFun.h"
#include "cmsis_os.h"
#include "dev_motor_dji.h"
#include "alg_chassis_calc.h"
#include "remote_control.h"
#include "GimbalTask.h"

	ChassisInstance_s *Chassis;
	static ChassisInitConfig_s Chassis_config = {
	.type = Steering_Wheel,
	.gimbal_yaw_zero = 2.887143f,
	.Gyroscope_Speed = 0.5,
	.omni_steering_message={
	.wheel_radius= 0.058f,
	.chassis_radius= 0.259f,
	},    
	.gimbal_steering_zero[0] = 0.4172f,
	.gimbal_steering_zero[1] = 1.1904f,
	.gimbal_steering_zero[2] = -0.1143f,
	.gimbal_steering_zero[3] = -0.1058f,
	.gimbal_steering_normal[0] = -0.3421f,
	.gimbal_steering_normal[1] = 1.9742f,
	.gimbal_steering_normal[2] = 0.8399f,
	.gimbal_steering_normal[3] = -0.9027f,
	.gimbal_follow_pid_config={
	.kp = -0.8f,
    .ki = 0.0f,
    .kd = 0.0f,
	.dead_zone = 0.01f,
    .i_max = 0.0f,
    .out_max = 2 * 3.141593f,
	},
	.motor_config[0] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 15.76f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
	.rx_id=0x201,	
	.tx_id=0x200,
    },
    .velocity_pid_config={
    .kp = 30.0f,
    .ki = 1.0f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
    .motor_config[1] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 15.76f,
    .topic_name = "2",
    .can_config = {
    .can_number=1,
	.rx_id=0x202,	
	.tx_id=0x200,
    },
    .velocity_pid_config={
    .kp = 30.0f,
    .ki = 1.0f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
    .motor_config[2] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 15.76f,
    .topic_name = "3",
    .can_config = {
    .can_number=1,
	.rx_id=0x203,	
	.tx_id=0x200,
	},
    .velocity_pid_config={
    .kp = 30.0f,
    .ki = 1.0f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
    .motor_config[3] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 15.76f,
	.topic_name = "4",
    .can_config = {
    .can_number=1,
	.rx_id=0x204, 
	.tx_id=0x200,
    },
    .velocity_pid_config={
    .kp = 30.0f,
    .ki = 1.0f,
    .kd = 0.0f,
    .i_max = 1800.0f,
    .out_max = 10000.0f,
    },
  },
	.motor_config[4] = {
    .type = GM6020,
    .control_mode = DJI_POSITION,
    .reduction_ratio = 1.0f,
    .topic_name = "5",
    .can_config = {
    .can_number=2,
	.rx_id=0x205, 
	.tx_id=0x1FE,
    },
    .velocity_pid_config={
    .kp = 150.0f,
    .ki = 1.8f,
    .kd = 0.0f,
    .i_max = 4500.0f,
    .out_max = 16384.0f,
    },
	.angle_pid_config={
	.kp = 120.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.angle_max =2*PI,	
	.i_max = 20.0f,
	.out_max = 200.0f,
	},
  },
	.motor_config[5] = {
    .type = GM6020,
    .control_mode = DJI_POSITION,
    .reduction_ratio = 1.0f,
    .topic_name = "6",
    .can_config = {
    .can_number=2,
	.rx_id=0x206, 
	.tx_id=0x1FE,
    },
    .velocity_pid_config={
    .kp = 120.0f,
    .ki = 1.8f,
    .kd = 0.0f,
    .i_max = 4000.0f,
    .out_max = 16384.0f,
    },
	.angle_pid_config={
	.kp = 120.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.angle_max =2*PI,	
	.i_max = 20.0f,
	.out_max = 200.0f,
	},
  },
	.motor_config[6] = {
    .type = GM6020,
    .control_mode = DJI_POSITION,
    .reduction_ratio = 1.0f,
    .topic_name = "7",
    .can_config = {
    .can_number=2,
	.rx_id=0x207, 
	.tx_id=0x1FE,
    },
    .velocity_pid_config={
    .kp = 120.0f,
    .ki = 1.4f,
    .kd = 0.0f,
    .i_max = 4000.0f,
    .out_max = 16384.0f,
    },
	.angle_pid_config={
	.kp = 120.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.angle_max =2*PI,	
	.i_max = 20.0f,
	.out_max = 200.0f,
	},
  },
	.motor_config[7] = {
    .type = GM6020,
    .control_mode = DJI_POSITION,
    .reduction_ratio = 1.0f,
    .topic_name = "8",
    .can_config = {
    .can_number=2,
	.rx_id=0x208, 
	.tx_id=0x1FE,
    },
    .velocity_pid_config={
    .kp = 150.0f,
    .ki = 1.8f,
    .kd = 0.0f,
    .i_max = 4000.0f,
    .out_max = 16384.0f,
    },
	.angle_pid_config={
	.kp = 120.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.angle_max =2*PI,	
	.i_max = 20.0f,
	.out_max = 200.0f,
	},
  }
  };

void ChassisTask(void const * argument)
{
	Chassis = Chassis_Register(&Chassis_config);
    for(;;)
	{
		Chassis->gimbal_yaw_angle = yaw_motor->out_position;
		if(rc_ctrl.rc.s[1] == 2)									//¼üÊó
		{
			Chassis_Choose_keyboard(Chassis);
			Chassis_Keyboard_Move_Calculate(1 ,10 ,50);
		}
		else														//Ò£¿ØÆ÷
		{
			Chassis_Choose_remote(Chassis);
			Chassis->Chassis_speed.Vy = -0.003*rc_ctrl.rc.ch[2];
			Chassis->Chassis_speed.Vx = 0.003*rc_ctrl.rc.ch[3];
		}
		Chassis_Control(Chassis);
		osDelay(2);
	}
}
