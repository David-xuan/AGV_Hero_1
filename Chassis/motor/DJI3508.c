#include "DJI3508.h"
#include "bsp_can.h"

Motor_3508Type Motor_3508[5];		

void Motor_3508_receive(Motor_3508Type* motor, uint8_t* temp, uint8_t CAN_ID)
{
	motor->ID 						= CAN_ID;
	motor->Angle         	= ((uint16_t)temp[0])<<8 | ((uint16_t)temp[1])*187/3591 ;
	motor->speed_rpm     	= (int16_t)(temp[2]<<8 |temp[3]);
	motor->current 		    = (((int16_t)temp[4])<<8 | ((int16_t)temp[5]));
	motor->tempure       	= temp[6];
	motor->angle            = (float)motor->Angle*2.0f*PI/8191.f;
	
	if(motor->Rx_add < 10000)
	{
		motor->Rx_add ++;
	}
	else
	{
		
	}
	
}

void Motor_3508_send(CAN_HandleTypeDef* hcan, uint32_t StdID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = StdID;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (iq1 >> 8);
	Txtemp[1] = iq1;
	Txtemp[2] = (iq2 >> 8);
	Txtemp[3] = iq2;
	Txtemp[4] = iq3 >> 8;
	Txtemp[5] = iq3;
	Txtemp[6] = iq4 >> 8;
	Txtemp[7] = iq4;
	
	uint8_t count = 0;
	while( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) == 0 && count < 100){
			count++;
	};
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}

void Get_Total_Angle(Motor_3508Type* motor)
{
	float res1 = 0, res2 = 0, delta = 0;
	if(motor->last_angle == 0)
		motor->last_angle = motor->angle;
	if(motor->angle < motor->last_angle)
	{
		res1 = motor->angle + 2*PI - motor->last_angle;
		res2 = motor->angle - motor->last_angle;
	}
	else if(motor->angle == motor->last_angle)
	{
		res1 = 0;
		res2 = 0;
	}
	else
	{
		res1 = motor->angle - 2*PI - motor->last_angle;
		res2 = motor->angle - motor->last_angle;
	}
	if(__fabs(res1)<__fabs(res2))
		delta = __fabs(res1);
	else
		delta = __fabs(res2);
	
	motor->total_angle += delta;
	motor->last_angle = motor->angle;
}
