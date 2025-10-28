#include "ChassisTask.h"
#include "cmsis_os.h"
#include "dev_motor_dji.h"
#include "alg_chassis_calc.h"
#include "remote_control.h"

//DjiMotorInstance_s *Wheel1;
//DjiMotorInitConfig_s config1 = {
//    .reduction_ratio = 19.0f,
//    .control_mode = DJI_VELOCITY,
//    .type = M3508,
//    .topic_name = "Wheel1",
//    .velocity_pid_config = {
//        .kp = 10.0f,
//        .ki = 0.0f,
//        .kd = 0.0f,
//        .out_max = 10000.0f,
//	},
//    .can_config = {
//        .can_number = 1,
//        .tx_id = 0x200,
//        .rx_id = 0x201,
//    },
//	};

	ChassisInstance_s *Chassis;
	static ChassisInitConfig_s Chassis_config = {
	.type = Steering_Wheel,
	.gimbal_yaw_zero = 0,//1.82637596f,//-0.0312073231f,
	.omni_steering_message={
	.wheel_radius= 0.058f,
	.chassis_radius= 0.259f,
	},    
	.gimbal_steering_zero[0] = 0.3889f,
	.gimbal_steering_zero[1] = 1.2088f,
	.gimbal_steering_zero[2] = -0.1212f,
	.gimbal_steering_zero[3] = -0.1166f,
	.gimbal_steering_normal[0] = -0.3421f,
	.gimbal_steering_normal[1] = 1.9742f,
	.gimbal_steering_normal[2] = 0.8399f,
	.gimbal_steering_normal[3] = -0.9027f,
	.gimbal_follow_pid_config={
	.kp = -3.0f,
    .ki = 0.0f,
    .kd = 0.0f,
	.dead_zone = 0.2f,
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
    .kp = 120.0f,
    .ki = 1.1f,
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
    .ki = 1.1f,
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
    .ki = 1.1f,
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
    .kp = 125.0f,
    .ki = 1.0f,
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
//	Wheel1 = Motor_Dji_Register(&config1);
//	while(1)
//	{
//		Motor_Dji_Control(Wheel1, 100); 
//        Motor_Dji_Transmit(Wheel1); 
//		osDelay(1);
//	}
	Chassis = Chassis_Register(&Chassis_config);
    for(;;){
		Chassis->gimbal_yaw_angle = 0;//Gimbal->yaw_motor->message.out_position;//这一步为将yaw轴云台编码值数据传给底盘    
		Chassis->Chassis_speed.Vy = -0.003*rc_ctrl.rc.
		ch[2];
		Chassis->Chassis_speed.Vx = 0.003*rc_ctrl.rc.ch[3];
		Chassis_Change_Mode(Chassis,CHASSIS_NORMAL);
		Chassis_Control(Chassis);
		osDelay(1);
	}
}
