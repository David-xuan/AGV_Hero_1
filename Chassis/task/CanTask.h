#ifndef CANTASK_H
#define CANTASK_H

#include "main.h"

typedef struct
{
    union
    {
        uint8_t buff[4];
        float value;
		
    }Yaw_angle;
    union
    {
        uint8_t buff[4];
        float value;
		
    }vison_yaw;	
}angle_measure_t;

extern angle_measure_t Gimbal_msg;





#endif
