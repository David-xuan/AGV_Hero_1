#include "bsp_can.h"
#include "DJI3508.h"
#include "can.h"

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
	can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
{
	CAN_RxHeaderTypeDef _RxHeader;
	uint8_t temp[8];

	HAL_CAN_GetRxMessage(_hcan,CAN_RX_FIFO0,&_RxHeader,temp);


	//ignore can1 or can2.
	switch(_RxHeader.StdId)								//接收电机信息
	{
		case CANID_Chassis0:
		case CANID_Chassis1:
		case CANID_Chassis2:
		case CANID_Chassis3:
		case CANID_Gimbal:
		{
			uint8_t i;
			i = _RxHeader.StdId - CANID_Chassis0;
			Motor_3508_receive(&Motor_3508[i],temp,_RxHeader.StdId);
		}
		break;
	}

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/

	  __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void Frequency_calc(void)
{
	Motor_3508[0].RX_Frequancy = Motor_3508[0].Rx_add;
	Motor_3508[0].Rx_add = 0;
	Motor_3508[1].RX_Frequancy = Motor_3508[1].Rx_add;
	Motor_3508[1].Rx_add = 0;	
	Motor_3508[2].RX_Frequancy = Motor_3508[2].Rx_add;
	Motor_3508[2].Rx_add = 0;
	Motor_3508[3].RX_Frequancy = Motor_3508[3].Rx_add;
	Motor_3508[3].Rx_add = 0;
}


