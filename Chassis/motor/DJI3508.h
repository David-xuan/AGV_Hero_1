#ifndef __DJI3508_H__
#define __DJI3508_H__
#include "main.h"


#define    CANID_Chassis0  0x201
#define    CANID_Chassis1  0x202
#define    CANID_Chassis2  0x203
#define    CANID_Chassis3  0x204
#define    CANID_RAMMER    0x205
#define    PI              3.14159265358979f

//3508电机参数
typedef struct Motor_3508Type
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	int16_t	 	speed_rpm;    //转子转速
	int16_t	 	target_speed_rpm;
	int16_t  	current;  //返回电流
	int16_t		target_current;	//设定电流
	float       tempure;      //温度
	float 	angle;				//机械角度
	float  	target_angle; //目标角度 底盘电机一般速度单环
	uint32_t	Rx_add;			
	uint16_t    RX_Frequancy;  //接收频率
} Motor_3508Type;

extern Motor_3508Type Motor_3508[5];

void Motor_3508_receive(Motor_3508Type* motor, uint8_t* temp, uint8_t CAN_ID);
void Motor_3508_send(CAN_HandleTypeDef* hcan, uint32_t StdID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
