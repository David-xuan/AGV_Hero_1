#include "CanTask.h"
#include "bsp_can.h"
#include "app_ins_task.h"
#include "string.h"
#include "remote_control.h"
#include "MinipcTask.h"

angle_measure_t Yaw_angle_t;
static void bc_1_Decode(CanInstance_s *can_instance);
static void bc_2_Decode(CanInstance_s *can_instance);
static void bc_1_Decode(CanInstance_s *can_instance){
    if(can_instance == NULL){
        return;
    }
	rc_ctrl.rc.ch[1] = (int16_t)(can_instance->rx_buff[0]<<8 | can_instance->rx_buff[1]);
	rc_ctrl.rc.s[0] = can_instance->rx_buff[2];
	rc_ctrl.rc.s[1] = can_instance->rx_buff[3];
	rc_ctrl.rc.ch[4] = (int16_t)(can_instance->rx_buff[4]<<8 | can_instance->rx_buff[5]);
	rc_ctrl.mouse.y = (int16_t)(can_instance->rx_buff[6]<<8 | can_instance->rx_buff[7]);
}
static void bc_2_Decode(CanInstance_s *can_instance){
    if(can_instance == NULL){
        return;
    }
	rc_ctrl.mouse.press_l = can_instance->rx_buff[0];
	rc_ctrl.mouse.press_r = can_instance->rx_buff[1];
	Yaw_angle_t.yaw_angle.buff[0] = can_instance->rx_buff[2];
	Yaw_angle_t.yaw_angle.buff[1] = can_instance->rx_buff[3];
	Yaw_angle_t.yaw_angle.buff[2] = can_instance->rx_buff[4];
	Yaw_angle_t.yaw_angle.buff[3] = can_instance->rx_buff[5];
}

CanInstance_s *bc_1,*bc_2;
CanInitConfig_s bc_config1 = {
    .topic_name = "1",
    .can_number = 2,
    .tx_id = 0x301,
    .rx_id = 0x300,
    .can_module_callback = bc_1_Decode,
	.parent_ptr = NULL,
};
CanInitConfig_s bc_config2 = {
    .topic_name = "1",
    .can_number = 2,
    .tx_id = 0x303,
    .rx_id = 0x302,
    .can_module_callback = bc_2_Decode,
	.parent_ptr = NULL,
};

void CanTask(void const * argument){
    bc_1 = Can_Register(&bc_config1);
	bc_2 = Can_Register(&bc_config2);
    for(;;){
		Yaw_angle_t.imu_yaw.value = -Quater.yaw;
		Yaw_angle_t.vison_yaw.value = -minipc->message.norm_aim_pack.yaw;
		uint8_t tx_buf[8] = {Yaw_angle_t.imu_yaw.buff[0], Yaw_angle_t.imu_yaw.buff[1],
							 Yaw_angle_t.imu_yaw.buff[2], Yaw_angle_t.imu_yaw.buff[3],
							 Yaw_angle_t.vison_yaw.buff[0], Yaw_angle_t.vison_yaw.buff[1], 
							 Yaw_angle_t.vison_yaw.buff[2], Yaw_angle_t.vison_yaw.buff[3]};
		memcpy(bc_1->tx_buff, tx_buf, 8);
        Can_Transmit(bc_1);
        osDelay(2);
    }
}
