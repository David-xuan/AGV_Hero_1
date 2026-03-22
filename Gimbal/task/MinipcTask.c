#include "MinipcTask.h"
#include "cmsis_os.h"
#include "dev_minipc.h"
#include "GimbalTask.h"
#include "app_ins_task.h"
#include "CanTask.h"

MiniPC_Instance* minipc = NULL;
MiniPC_Config minipc_config = {
    .callback = NULL,
    .message_type = USB_MSG_AIM_RX, // 自瞄数据
    .Send_message_type = USB_MSG_AIM_TX // 发送数据类型
};

void MinipcTask(void const * argument)
{
	minipc = Minipc_Register(&minipc_config);
	for(;;)
	{
		Computer_Tx(-Yaw_angle_t.yaw_angle.value, -pitch->message.out_position, 0, 0, 0);//Quater.yaw
		Minipc_UpdateAllInstances();
		osDelay(1);
	}
}
