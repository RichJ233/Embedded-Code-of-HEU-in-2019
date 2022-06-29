#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx_hal.h"

/*user includes begin*/



/*user includes end*/

#define ABS(x) ((x > 0) ? x : -x)

#define PID_PARAM_INIT \
{\
	0,\
	0,\
	0,\
	0,\
	{0,0,0},\
	0,\
	{0,0,0},\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
}

enum {
	LLAST = 0,
	LAST ,
	NOW  ,
	
	POSITION =0,
	SPEED    =1,
};

typedef	struct{
	
	float kp;
	float ki;
	float kd;
	float index;
	
	float   real[3];
	float        ex;
	float    err[3];
	float intergral;
	float intergral_range;
	
	float   p_out;
	float   i_out;
	float   d_out;
	float out_put;
	float last_output;
	
	float   max_out_put;
	float max_intergral;
	#ifdef POWER_LIMIT
	float     max_power;
	#endif
	uint32_t pid_mode;
	
}pid_st;


float abs_limit(float *a, float ABS_MAX);
void i_saturation(pid_st *pid);


/*function may be used*/
float pid_cal(pid_st *pid, float real, float ex);
void pid_init(pid_st *pid, float kp, float ki, float kd, uint32_t pid_mode,
							float intergral_range,float max_out_put, float max_intergral);


/*user fuction code begin*/



/*user fuction code end*/

#endif











