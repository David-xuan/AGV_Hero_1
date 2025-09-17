#include "DJI3508.h"
#include "bsp_can.h"

Motor_3508Type Motor_3508[5];		

void Motor_3508_receive(Motor_3508Type* motor, uint8_t* temp, uint8_t CAN_ID)
{
	motor->ID 						= CAN_ID;
	motor->Angle         	= ((uint16_t)temp[0])<<8 | ((uint16_t)temp[1]) ;
	motor->speed_rpm     	= (int16_t)(temp[2]<<8 |temp[3]);
	motor->current 		    = (((int16_t)temp[4])<<8 | ((int16_t)temp[5]));
	motor->tempure       	= temp[6];
	motor->angle            = (float)(motor->Angle * 2.0 * PI / 8191.0 - PI);
	
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
    double delta = 0;
    static double last_angle = 0;
    
    // 初始化上一次角度
    if(last_angle == 0) {
        last_angle = motor->angle;
        return;
    }
    
    // 计算角度变化量（考虑0-2π的循环）
    double raw_delta = motor->angle - last_angle;
    
    // 处理角度跨越2π边界的情况
    if(raw_delta > PI) {
        delta = raw_delta - 2*PI;  // 实际是反转
    } 
    else if(raw_delta < -PI) {
        delta = raw_delta + 2*PI;  // 实际是正转
    }
    else {
        delta = raw_delta;         // 正常变化
    }
    
    // 更新总角度（正转增加，反转减少）
    motor->total_angle += delta;
    
    // 保存当前角度供下次使用
    last_angle = motor->angle;
}
