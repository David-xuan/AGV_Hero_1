#include "CanTask.h"
#include "RefereeTask.h"
#include "bsp_can.h"
#include "string.h"
#include "remote_control.h"

angle_measure_t Gimbal_msg;
static void bc_1_Decode(CanInstance_s *can_instance);
static void bc_1_Decode(CanInstance_s *can_instance){
    if(can_instance == NULL){
        return;
    }
	Gimbal_msg.Yaw_angle.buff[0] = can_instance->rx_buff[0];
	Gimbal_msg.Yaw_angle.buff[1] = can_instance->rx_buff[1];
	Gimbal_msg.Yaw_angle.buff[2] = can_instance->rx_buff[2];
	Gimbal_msg.Yaw_angle.buff[3] = can_instance->rx_buff[3];
	Gimbal_msg.vison_yaw.buff[0] = can_instance->rx_buff[4];
	Gimbal_msg.vison_yaw.buff[1] = can_instance->rx_buff[5];
	Gimbal_msg.vison_yaw.buff[2] = can_instance->rx_buff[6];
	Gimbal_msg.vison_yaw.buff[3] = can_instance->rx_buff[7];	
}

CanInstance_s *bc_1,*bc_2;
CanInitConfig_s bc_config1 = {
    .topic_name = "1",
    .can_number = 2,
    .tx_id = 0x300,
    .rx_id = 0x301,
    .can_module_callback = bc_1_Decode,
	.parent_ptr = NULL,
};
CanInitConfig_s bc_config2 = {
    .topic_name = "1",
    .can_number = 2,
    .tx_id = 0x302,
    .rx_id = 0x303,
    .can_module_callback = bc_1_Decode,
	.parent_ptr = NULL,
};

void CanTask(void const * argument){
    bc_1 = Can_Register(&bc_config1);
	bc_2 = Can_Register(&bc_config2);
    for(;;)
	{
		uint8_t tx_buf_1[8] = {(rc_ctrl.rc.ch[1]>>8) ,rc_ctrl.rc.ch[1],
							   rc_ctrl.rc.s[0] ,rc_ctrl.rc.s[1],
							   (rc_ctrl.rc.ch[4]>>8) ,rc_ctrl.rc.ch[4],
							   ref_instance->origin_data.ext_remote_control.mouse_y>>8,
							   ref_instance->origin_data.ext_remote_control.mouse_y};
		uint8_t tx_buf_2[8] = {ref_instance->origin_data.ext_remote_control.left_button_down,
							   ref_instance->origin_data.ext_remote_control.right_button_down,
							   0 ,0 ,0 ,0 ,0 ,0 };
		memcpy(bc_1->tx_buff, tx_buf_1, 8);
		memcpy(bc_2->tx_buff, tx_buf_2, 8);
        Can_Transmit(bc_1);
		Can_Transmit(bc_2);
        osDelay(2);
    }
}

