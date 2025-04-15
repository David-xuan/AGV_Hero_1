#ifndef __MY_FUN_H__
#define __MY_FUN_H__

#define	_PI			3.14159265f
#define _2PI		6.28318531f



extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x, float x_min, float x_max, int bits);
extern void LimitValue_f(float*VALUE, float MAX, float MIN);
extern float RAMP_float( float final, float now, float ramp );
extern void AngleLoop_f (float *angle , float max);
extern float AngleLoop_DM(float *target_angle, float *angle) ;
extern float RampInc_float( float *buffer, float now, float ramp );


#endif 

