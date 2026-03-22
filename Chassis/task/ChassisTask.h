#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "main.h"
#include "alg_chassis_calc.h"

extern ChassisInstance_s *Chassis;


typedef struct {
    DjiMotorInitConfig_s config[4];        
}chassisInitConfig;


#endif
