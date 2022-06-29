#include "chassis_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "pid.h"
#include "DR16.h"
#include "RM_Define.h"
#include "judgement_info.h"
#include "detect_task.h"
#include "math.h"
#include "bsp_flash.h"
#include "super_caps_task.h"

pid_st pid_motor[4] = PID_PARAM_INIT;//have a little bug
pid_st pid_yaw = PID_PARAM_INIT;
float chassis_k[3]={ChassisKP,ChassisKI,ChassisKD};
float yaw_k[3]={YawKP,YawKI,YawKD};

chassis_ctrl_info_t chassis_ctrl_info;
chassis_status_info_t chassis_status_info;
power_limit_data_t power_limit_data;



void ChassisTask(void)
{
	pid_init(&pid_motor[0], 0, 0, 0, SPEED, 5, 7500, 30);
	pid_init(&pid_motor[1], 0, 0, 0, SPEED, 5, 7500, 30);
	pid_init(&pid_motor[2], 0, 0, 0, SPEED, 5, 7500, 30);
	pid_init(&pid_motor[3], 0, 0, 0, SPEED, 5, 7500, 30);
	
	pid_init(&pid_yaw, yaw_k[0], yaw_k[1], yaw_k[2], POSITION, 5, 7000, 30);
	
	uint32_t chassis_task_time = osKernelSysTick();
	for(;;)
	{
		silence_check();

		chassis_status_info.speed[0] = Motor_Data.Speed_Real[0];
		chassis_status_info.speed[1] = Motor_Data.Speed_Real[1];
		chassis_status_info.speed[2] = Motor_Data.Speed_Real[2];
		chassis_status_info.speed[3] = Motor_Data.Speed_Real[3];	
		chassis_status_info.angle = (float)(GimbalMotor.Theta_Real[1]) / 4096.0f * PI;
		
		motion_solution();
		
		power_limit();
		
		chassis_status_info.can_s = send_chassis_cur( pid_motor[0].out_put,
														pid_motor[1].out_put,
														pid_motor[2].out_put,
														pid_motor[3].out_put);
		
		osDelayUntil(&chassis_task_time, CHASSIS_TASK_PERIOD);
	}
}

void motion_solution(void)
{
	x_ctrl(&chassis_ctrl_info);
	y_ctrl(&chassis_ctrl_info);
	z_ctrl(&chassis_ctrl_info);
	
	sum_check(&chassis_ctrl_info);
	if(chassis_ctrl_info.chas_info.stay_f == RM_SET)
	{
		chassis_ctrl_info.chassis_speed.set_speed[0] = 0;
		chassis_ctrl_info.chassis_speed.set_speed[1] = 0;
		chassis_ctrl_info.chassis_speed.set_speed[2] = 0;
		chassis_ctrl_info.chassis_speed.set_speed[3] = 0;
	}
	if(chassis_ctrl_info.chas_info.reset_f == RM_SET)
	{
		mcu_reset();
	}
}

void x_ctrl(chassis_ctrl_info_t* chassis_ctrl_info_b)
{
	if(RC_Ctl.rc.sl == RC_MODE)
	{
		chassis_ctrl_info_b->x_speed_info = RC_Ctl.rc.channel[1];
		chassis_ctrl_info_b->y_speed_info = RC_Ctl.rc.channel[0];
	}
	else
	{
		chassis_ctrl_info_b->x_speed_buf = (RC_Ctl.key.w - RC_Ctl.key.s) * X_SPEED;
		chassis_ctrl_info_b->y_speed_buf = (RC_Ctl.key.d - RC_Ctl.key.a) * Y_SPEED;
		if(RC_Ctl.key.c == 1)
		{
			chassis_ctrl_info_b->x_speed_buf *= SLOW_K;
			chassis_ctrl_info_b->y_speed_buf *= SLOW_K;
		}
		
		if((chassis_ctrl_info_b->x_speed_buf != 0) | (chassis_ctrl_info_b->y_speed_buf != 0))
		{
			chassis_ctrl_info_b->x_speed_info = (chassis_ctrl_info_b->x_speed_buf * cos(chassis_status_info.angle) + \
													chassis_ctrl_info_b->y_speed_buf * sin(chassis_status_info.angle)) * \
													SOFTBOOST + chassis_ctrl_info_b->x_speed_info * (1 - SOFTBOOST);
			
			chassis_ctrl_info_b->y_speed_info = (chassis_ctrl_info_b->y_speed_buf * cos(chassis_status_info.angle) + \
													chassis_ctrl_info_b->x_speed_buf * sin(chassis_status_info.angle)) * \
													SOFTBOOST + chassis_ctrl_info_b->y_speed_info * (1 - SOFTBOOST);
		}
		else
		{
//			chassis_ctrl_info_b->x_speed_info = chassis_ctrl_info_b->x_speed_info * (1 - SOFTBRAKE);
//			chassis_ctrl_info_b->y_speed_info = chassis_ctrl_info_b->y_speed_info * (1 - SOFTBRAKE);
			
			chassis_ctrl_info_b->x_speed_info = 0;
			chassis_ctrl_info_b->y_speed_info = 0;
		}
	}
	chassis_ctrl_info_b->chassis_speed.set_speed[0] = chassis_ctrl_info_b->x_speed_info * ROLL_FORWARD;
	chassis_ctrl_info_b->chassis_speed.set_speed[1] = chassis_ctrl_info_b->x_speed_info * ROLL_FORWARD;
	chassis_ctrl_info_b->chassis_speed.set_speed[2] = chassis_ctrl_info_b->x_speed_info * ROLL_BACK;
	chassis_ctrl_info_b->chassis_speed.set_speed[3] = chassis_ctrl_info_b->x_speed_info * ROLL_BACK;
	
	chassis_ctrl_info_b->chassis_speed.set_speed[0] += chassis_ctrl_info_b->y_speed_info * ROLL_FORWARD;
	chassis_ctrl_info_b->chassis_speed.set_speed[1] += chassis_ctrl_info_b->y_speed_info * ROLL_BACK;
	chassis_ctrl_info_b->chassis_speed.set_speed[2] += chassis_ctrl_info_b->y_speed_info * ROLL_BACK;
	chassis_ctrl_info_b->chassis_speed.set_speed[3] += chassis_ctrl_info_b->y_speed_info * ROLL_FORWARD;
}

void y_ctrl(chassis_ctrl_info_t* chassis_ctrl_info_b)
{
	if(RC_Ctl.rc.sl == RC_MODE)
	{

	}
	else
	{

	}
}

void z_ctrl(chassis_ctrl_info_t* chassis_ctrl_info_b)
{
	if(RC_Ctl.rc.sl == RC_MODE)
		chassis_ctrl_info_b->z_roll_info =  pid_cal(&pid_yaw, GimbalMotor.Theta_Real[1], 0);
	else
	{
		if(RC_Ctl.key.shift == 1)
			chassis_ctrl_info_b->z_roll_info = Z_SPEED;
		
		else if(RC_Ctl.key.ctrl == 1)
		{
			if(chassis_ctrl_info_b->z_roll_flag == 1)
				chassis_ctrl_info_b->z_roll_info = Z_SPEED;
			else
				chassis_ctrl_info_b->z_roll_info = Z_SPEED * -1;
			
			if(chassis_status_info.angle >= Z_ROLL_LIMIT)
				chassis_ctrl_info_b->z_roll_flag = 0;
			else if(chassis_status_info.angle <= (Z_ROLL_LIMIT * -1))
				chassis_ctrl_info_b->z_roll_flag = 1;
		}
		else
		{
			chassis_ctrl_info_b->z_roll_info =  pid_cal(&pid_yaw, GimbalMotor.Theta_Real[1], 0);
		}
	}
	chassis_ctrl_info_b->chassis_speed.set_speed[0] += chassis_ctrl_info_b->z_roll_info * ROLL_FORWARD;
	chassis_ctrl_info_b->chassis_speed.set_speed[1] += chassis_ctrl_info_b->z_roll_info * ROLL_FORWARD;
	chassis_ctrl_info_b->chassis_speed.set_speed[2] += chassis_ctrl_info_b->z_roll_info * ROLL_FORWARD;
	chassis_ctrl_info_b->chassis_speed.set_speed[3] += chassis_ctrl_info_b->z_roll_info * ROLL_FORWARD;
}

void sum_check(chassis_ctrl_info_t* chassis_ctrl_info_b)
{
	//懒得优化，就这样
	if(chassis_ctrl_info_b->chassis_speed.set_speed[0] > chassis_ctrl_info_b->chassis_speed.set_speed[1])
		chassis_ctrl_info_b->speed_temp = chassis_ctrl_info_b->chassis_speed.set_speed[0];
	else
		chassis_ctrl_info_b->speed_temp = chassis_ctrl_info_b->chassis_speed.set_speed[1];
	if(chassis_ctrl_info_b->speed_temp < chassis_ctrl_info_b->chassis_speed.set_speed[2])
		chassis_ctrl_info_b->speed_temp = chassis_ctrl_info_b->chassis_speed.set_speed[2];
	if(chassis_ctrl_info_b->speed_temp < chassis_ctrl_info_b->chassis_speed.set_speed[3])
		chassis_ctrl_info_b->speed_temp = chassis_ctrl_info_b->chassis_speed.set_speed[3];
	
	if(chassis_ctrl_info_b->speed_temp == 0)
		chassis_ctrl_info_b->speed_temp = 0.0001f;
	
	if(chassis_ctrl_info_b->speed_temp >= MOTOR_MAX_SPEED)
	{
		chassis_ctrl_info_b->speed_k = MOTOR_MAX_SPEED / (float)chassis_ctrl_info_b->speed_temp;
		chassis_ctrl_info_b->chassis_speed.set_speed[0] = chassis_ctrl_info_b->chassis_speed.set_speed[0] * chassis_ctrl_info_b->speed_k;
		chassis_ctrl_info_b->chassis_speed.set_speed[1] = chassis_ctrl_info_b->chassis_speed.set_speed[1] * chassis_ctrl_info_b->speed_k;
		chassis_ctrl_info_b->chassis_speed.set_speed[2] = chassis_ctrl_info_b->chassis_speed.set_speed[2] * chassis_ctrl_info_b->speed_k;
		chassis_ctrl_info_b->chassis_speed.set_speed[3] = chassis_ctrl_info_b->chassis_speed.set_speed[3] * chassis_ctrl_info_b->speed_k;
	}

}

void mcu_reset(void)
{
	flash_write.reset_f = 1;
	reset_check();
}

void power_limit(void)
{
////////////////////////////////////////////////////////////////////////////////////////////////
	pid_calcaulate();
////////////////////////////////////////////////////////////////////////////////////////////////
//	if((relay_ctrl_info.ctrl_f == RELAY_CLOSE) || (detect_status.Judgement_S == 0))
//	{
//		if(detect_status.Judgement_S == 0)
//		{
//			judgement_race_data.power_heat_data.chassis_power_buffer = 25;
//			judgement_race_data.power_heat_data.chassis_power = POWER_MAX;
//		}
//		
//		if((judgement_race_data.power_heat_data.chassis_power >= POWER_MAX) && (judgement_race_data.power_heat_data.chassis_power_buffer <= 55))
//		{
//			power_limit_data.out_limit_sum = judgement_race_data.power_heat_data.chassis_power_buffer * \
//												judgement_race_data.power_heat_data.chassis_power_buffer * POWER_LIMIT_PARAM;
//			power_limit_data.motor_out_sum = ABS(power_limit_data.motor_out[0]) + ABS(power_limit_data.motor_out[1]) + \
//												ABS(power_limit_data.motor_out[2]) + ABS(power_limit_data.motor_out[3]);
//			
//			if(power_limit_data.motor_out_sum == 0)
//				power_limit_data.motor_out_sum = 1;
//			
//			power_limit_data.power_div[0] = power_limit_data.motor_out[0] / power_limit_data.motor_out_sum;
//			power_limit_data.power_div[1] = power_limit_data.motor_out[1] / power_limit_data.motor_out_sum;
//			power_limit_data.power_div[2] = power_limit_data.motor_out[2] / power_limit_data.motor_out_sum;
//			power_limit_data.power_div[3] = power_limit_data.motor_out[3] / power_limit_data.motor_out_sum;

//			pid_motor[0].out_put = power_limit_data.out_limit_sum * power_limit_data.power_div[0];
//			pid_motor[1].out_put = power_limit_data.out_limit_sum * power_limit_data.power_div[1];
//			pid_motor[2].out_put = power_limit_data.out_limit_sum * power_limit_data.power_div[2];
//			pid_motor[3].out_put = power_limit_data.out_limit_sum * power_limit_data.power_div[3];
//		}
//	}
}

void pid_calcaulate(void)
{
	power_limit_data.motor_out[0] = pid_cal(&pid_motor[0],chassis_status_info.speed[0],chassis_ctrl_info.chassis_speed.set_speed[0]);
	power_limit_data.motor_out[1] = pid_cal(&pid_motor[1],chassis_status_info.speed[1],chassis_ctrl_info.chassis_speed.set_speed[1]);
	power_limit_data.motor_out[2] = pid_cal(&pid_motor[2],chassis_status_info.speed[2],chassis_ctrl_info.chassis_speed.set_speed[2]);
	power_limit_data.motor_out[3] = pid_cal(&pid_motor[3],chassis_status_info.speed[3],chassis_ctrl_info.chassis_speed.set_speed[3]);
}

void silence_check(void)
{
	if(chassis_ctrl_info.chas_info.silence_f == RM_SET)
	{
		#if CHASSIS_PID_TEST == 0
			pid_motor[0].kp = 0.0f;pid_motor[1].kp = 0.0f;
			pid_motor[2].kp = 0.0f;pid_motor[3].kp = 0.0f;
			pid_motor[0].ki = 0.0f;pid_motor[1].ki = 0.0f;
			pid_motor[2].ki = 0.0f;pid_motor[3].ki = 0.0f;
			pid_motor[0].out_put = 0;
			pid_motor[1].out_put = 0;
			pid_motor[2].out_put = 0;
			pid_motor[3].out_put = 0;
		#endif
	}
	else
	{
		#if CHASSIS_PID_TEST == 0
			pid_motor[0].kp = chassis_k[0];pid_motor[1].kp = chassis_k[0];
			pid_motor[2].kp = chassis_k[0];pid_motor[3].kp = chassis_k[0];
			pid_motor[0].ki = chassis_k[1];pid_motor[1].ki = chassis_k[1];
			pid_motor[2].ki = chassis_k[1];pid_motor[3].ki = chassis_k[1];
		#endif
	}
}

