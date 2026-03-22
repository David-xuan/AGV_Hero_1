#ifndef CANTASK_H
#define CANTASK_H

#include "main.h"

typedef struct
{
    union
    {
        uint8_t buff[4];
        float value;
		
    }imu_yaw;
    union
    {
        uint8_t buff[4];
        float value;
		
    }vison_yaw;	
	union
	{
		uint8_t buff[4];
		float value;
	}yaw_angle;
}angle_measure_t;

extern angle_measure_t Yaw_angle_t;


#endif
