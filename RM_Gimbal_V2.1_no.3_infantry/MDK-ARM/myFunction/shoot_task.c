#include "shoot_task.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "pid.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "RM_Define.h"
#include "gimbal_task.h"

float shoot_position_k[3] = { ShootPositionKP , 0 , 0 };
float shoot_speed_k[3] = { ShootSpeedKP, ShootSpeedKI , 0 };
float trig_out;

pid_st pid_pluck_position = PID_PARAM_INIT;
pid_st pid_pluck_speed = PID_PARAM_INIT;

shoot_info_t shoot_info;

void pwm_device_init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	htim3.Instance->CCR1 = 1000;
	htim3.Instance->CCR2 = 1000;
	htim4.Instance->CCR1 = 1000;
}

void bsp_io_init(void)
{
	
}

void ShootTask(void)
{
	pid_init(&pid_pluck_position, shoot_position_k[0], shoot_position_k[1], shoot_position_k[2], POSITION, 5, 12000, 30);
	pid_init(&pid_pluck_speed, shoot_speed_k[0], shoot_speed_k[1], shoot_speed_k[2], SPEED, 5, 20000, 30);
	
	uint32_t shoot_task_time = osKernelSysTick();
	
	for(;;)
	{
		friction_ctrl();
		laser_ctrl();
		shoot_ctrl();
		reload_ctrl();
		
		osDelayUntil(&shoot_task_time, SHOOT_TASK_PERIOD);
	}
}

void reload_ctrl(void)
{
	if(gimbal_ctrl_info.reload_info == 1)
		TIM4->CCR1 = Open_CCR;
	else
		TIM4->CCR1 = Close_CCR;
}


uint16_t ccr = 2000;
void friction_ctrl(void)
{
	if(shoot_info.friction_CCR < 1000)
		shoot_info.friction_CCR = 1000;
	if(shoot_info.friction_CCR > 1800)
		shoot_info.friction_CCR = 1800;
	TIM3->CCR1 = shoot_info.friction_CCR;
	TIM3->CCR2 = shoot_info.friction_CCR;
	
//	TIM3->CCR1 = ccr;
//	TIM3->CCR2 = ccr;
}

void laser_ctrl(void)
{
	if(shoot_info.friction_CCR == 1000)
	{
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
}

void shoot_ctrl(void)
{
	trig_out = pluck_DELTA_pid(&pid_pluck_speed, TRIGGER_Motor_Data.Speed_Real, shoot_info.shoot_speed);
}

int16_t pluck_DELTA_pid(pid_st *pid,int real,int ex)
{
	pid->real[NOW] = real;
	pid->ex = ex;
	pid->err[NOW] = pid->real[NOW] - pid->ex;
	pid->p_out = pid->kp * (pid->err[NOW] - pid->err[LAST]);
	pid->i_out = pid->ki * pid->err[NOW];
	
	pid->out_put += pid->p_out + pid->i_out;
	abs_limit(&pid->out_put,pid->max_out_put);

	pid->err[LAST] = pid->err[NOW];	
	
	return pid->out_put;
}

