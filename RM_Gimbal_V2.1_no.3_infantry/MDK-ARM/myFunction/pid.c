#include "pid.h"

/* user pid struct define begin */
/* {PID_PARAM_INIT} is init value */



/* user pid struct define end */

/*aim:limit pointer a into ABS_MAX */
float abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        return  ABS_MAX;
    if (*a < -ABS_MAX)
        return -ABS_MAX;
		return *a;
}

/*aim: against pid intergral saturation */
void i_saturation(pid_st *pid)
{	
	if(ABS(pid->err[NOW]) > pid->intergral_range)
		pid->index = 0;
	else
		pid->index = 1;	
}

/*aim: to calculate pid output*/
/*param: pid :pointer whitch contorl pointer position
				 real:measurement
				 ex  :the expect
*/
float pid_cal(pid_st *pid, float real, float ex){
	
	pid->real[NOW] = real;
	pid->ex = ex;
	pid->err[NOW] = ex - real;
	
	if(pid->pid_mode == POSITION)
	{
		pid->intergral += pid->err[NOW];
		if(pid->intergral > pid->intergral_range) pid->intergral = pid->intergral_range;
		else if(pid->intergral < -(pid->intergral_range)) pid->intergral = pid->intergral_range;
		i_saturation(pid);
		
		pid->p_out = pid->kp *  pid->err[NOW];
		pid->i_out = pid->ki *  pid->index* pid->intergral;
		pid->d_out = pid->kd * (pid->err[NOW]-pid->err[LAST]);
		
		pid->out_put = pid->p_out + pid->i_out + pid->d_out;	
	}
	else if(pid->pid_mode == SPEED)
	{
			pid->p_out = pid->kp * (pid->err[NOW] - pid->err[LAST]);
			pid->i_out = pid->ki *  pid->err[NOW];
			pid->d_out = pid->kd * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);
		
			pid->d_out = pid->kd * (2 * pid->real[NOW] - 3 * pid->real[LAST] + pid->real[LLAST]);

			pid->out_put += pid->p_out + pid->i_out - pid->d_out;
	}
	
	pid->err[LLAST] = pid->err[LAST];
	pid->err[LAST]  = pid->err[NOW];	
	
	pid->real[LLAST] = pid->real[LAST];
	pid->real[LAST]  = pid->real[NOW];	
	
	#ifdef POWER_LIMIT        
		/*********************************************POWER LIMIT**************************************/
		pid->out_put = abs_limit(&(pid->out_put) , pid->max_power);
	#endif
	return abs_limit(&(pid->out_put) , pid->max_out_put);
}

/*aim:to clear struct pid members and put p,i,d ratio into struct*/
void pid_init(pid_st *pid, float kp, float ki, float kd,uint32_t pid_mode,
							float intergral_range,float max_out_put, float max_intergral){

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->pid_mode = pid_mode;
	pid->intergral_range = intergral_range;
	pid->max_out_put = max_out_put;				
	pid->max_intergral = max_intergral;		

}







