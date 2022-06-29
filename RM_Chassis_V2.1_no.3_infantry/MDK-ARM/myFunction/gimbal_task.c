#include "gimbal_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "super_caps_task.h"
#include "bsp_can.h"
#include "DR16.h"
#include "RM_Define.h"
#include "cmsis_os.h"
#include "string.h"
#include "judgement_info.h"
#include "usart.h"
#include "RichJlib.h"

command_info_t command_info;
gimbal_ctrl_info_t gimbal_ctrl_info;
auto_aim_info_t auto_aim_info;
uint16_t shoot_motor_speed[] = {SHOOT_SPEED_2,SHOOT_SPEED_3,SHOOT_SPEED_2,SHOOT_SPEED_1};
uint16_t shoot_ccr[] = {0, FrictionCCR25,FrictionCCR20,FrictionCCR14,1000};

uint16_t ccr_test = 1000;

void GimbalTask(void)
{
	
	command_init();
	
	uint32_t gimbal_task_time = osKernelSysTick();
	
	for(;;)
	{
		command_unpack();
		
		gimbal_command_update();
		
		osDelayUntil(&gimbal_task_time, GIMBAL_TASK_PERIOD);
	}
}

void command_init(void)
{
	command_info.z.mode_max = MAX_4;
	command_info.x.mode_max = MAX_4;
	command_info.v.mode_max = MAX_2;
	command_info.g.mode_max = MAX_4;
	command_info.r.mode_max = MAX_2;
	command_info.f.mode_max = MAX_2;
	
	command_info.sr1.mode_max = MAX_1;
	command_info.sr2.mode_max = MAX_1;
}

void command_unpack(void)
{
	key_detect(RC_Ctl.key.z, &command_info.z);
	key_detect(RC_Ctl.key.x, &command_info.x);
	key_detect(RC_Ctl.key.v, &command_info.v);
	key_detect(RC_Ctl.key.g, &command_info.g);
	key_detect(RC_Ctl.key.r, &command_info.r);
	key_detect(RC_Ctl.key.f, &command_info.f);
	
	rc_detect();
	
	chassis_command_unpack();
	
}

void gimbal_command_update(void)
{
	switch(RC_Ctl.rc.sl){
		case RC_MODE:{
			if(detect_status.DR16_S == 0)
			{
				gimbal_ctrl_info.gim_info.gim_mode_info = SILENCE_MODE;
			}
			else
			{
				gimbal_ctrl_info.gim_info.gim_mode_info = CONTROL_MODE;
			}
			gimbal_ctrl_info.gim_info.pit_info = RC_Ctl.rc.channel[3];
			gimbal_ctrl_info.gim_info.yaw_info = RC_Ctl.rc.channel[2];
			
			if(command_info.sr1.mode == 1)
			{
				gimbal_ctrl_info.shoot_info.LASER_FLAG = 1;
				gimbal_ctrl_info.shoot_info.friction_CCR = FrictionCCR20;
				gimbal_ctrl_info.shoot_info.reload_info = 1;
			}
			else
			{
				gimbal_ctrl_info.shoot_info.LASER_FLAG = 0;
				gimbal_ctrl_info.shoot_info.friction_CCR = 1000;
				gimbal_ctrl_info.shoot_info.reload_info = 0;
			}
			
			if(command_info.sr2.mode == 1)
				gimbal_ctrl_info.shoot_info.shoot_speed = 600;
			else
				gimbal_ctrl_info.shoot_info.shoot_speed = 0;
			
		};break;
		
		case KEY_MODE:{
			if(detect_status.DR16_S == 0)
			{
				gimbal_ctrl_info.gim_info.gim_mode_info = SILENCE_MODE;
			}
			else if(RC_Ctl.mouse.press_r == 1)
			{
				if(auto_aim_info.flag == 0)
				{
					gimbal_ctrl_info.gim_info.gim_mode_info = AUTO_AIM_MODE_NO;
					gimbal_ctrl_info.gim_info.pit_info = auto_aim_info.x;
					gimbal_ctrl_info.gim_info.yaw_info = auto_aim_info.y;
					gimbal_ctrl_info.gim_info.save_info = auto_aim_info.z;
				}
				else
				{
					gimbal_ctrl_info.gim_info.gim_mode_info = AUTO_AIM_MODE;
					gimbal_ctrl_info.gim_info.pit_info = auto_aim_info.x;
					gimbal_ctrl_info.gim_info.yaw_info = auto_aim_info.y;
					gimbal_ctrl_info.gim_info.save_info = auto_aim_info.z;
				}
			}
			else
			{
				gimbal_ctrl_info.gim_info.gim_mode_info = CONTROL_MODE;
				gimbal_ctrl_info.gim_info.pit_info = RC_Ctl.mouse_y;
				gimbal_ctrl_info.gim_info.yaw_info = RC_Ctl.mouse_x;
			}
				
			gimbal_ctrl_info.shoot_info.friction_CCR = shoot_ccr[command_info.z.mode];
//			gimbal_ctrl_info.shoot_info.friction_CCR = ccr_test;
			if(command_info.z.mode != 4)
				gimbal_ctrl_info.shoot_info.LASER_FLAG = RM_SET;
			else
				gimbal_ctrl_info.shoot_info.LASER_FLAG = RM_UNSET;
			
			if(command_info.r.mode == 2)
				gimbal_ctrl_info.shoot_info.reload_info = RM_SET;
			else
				gimbal_ctrl_info.shoot_info.reload_info = RM_UNSET;
			
			if(RC_Ctl.mouse.press_l == 1)
				gimbal_ctrl_info.shoot_info.shoot_speed = shoot_motor_speed[command_info.x.mode];
			else
				gimbal_ctrl_info.shoot_info.shoot_speed = 0;
		
			if(command_info.f.mode == 2)//////////////////////////////////////////////////////////////////
			{
				gimbal_ctrl_info.shoot_info.shoot_speed = 0;
				gimbal_ctrl_info.shoot_info.friction_CCR = 0;
				gimbal_ctrl_info.gim_info.gim_mode_info = AUTO_GAIN_MODE;
			}
			
			if(command_info.g.mode == 4)
			{
				gimbal_ctrl_info.gim_info.gim_mode_info = RESET_MODE;
				chassis_ctrl_info.chas_info.reset_f = RM_SET;
				command_info.g.mode = 0;
			}
			
		};break;
		case AUTOmode:{
			if(detect_status.DR16_S == 0)
			{
				gimbal_ctrl_info.gim_info.gim_mode_info = SILENCE_MODE;
			}
			else
			{
				if(auto_aim_info.flag == 0)
				{
					gimbal_ctrl_info.gim_info.gim_mode_info = AUTO_AIM_MODE_NO;
					gimbal_ctrl_info.gim_info.pit_info = auto_aim_info.x;
					gimbal_ctrl_info.gim_info.yaw_info = auto_aim_info.y;
					gimbal_ctrl_info.gim_info.save_info = auto_aim_info.z;
				}
				else
				{
					gimbal_ctrl_info.gim_info.gim_mode_info = AUTO_AIM_MODE;
					gimbal_ctrl_info.gim_info.pit_info = auto_aim_info.x;
					gimbal_ctrl_info.gim_info.yaw_info = auto_aim_info.y;
					gimbal_ctrl_info.gim_info.save_info = auto_aim_info.z;
				}
			}
			if(command_info.sr1.mode == 1)
			{
				gimbal_ctrl_info.shoot_info.LASER_FLAG = 1;
				gimbal_ctrl_info.shoot_info.friction_CCR = FrictionCCR20;
				gimbal_ctrl_info.shoot_info.reload_info = 1;
			}
			else
			{
				gimbal_ctrl_info.shoot_info.LASER_FLAG = 0;
				gimbal_ctrl_info.shoot_info.friction_CCR = 1000;
				gimbal_ctrl_info.shoot_info.reload_info = 0;
			}
			
			if(command_info.sr2.mode == 1)
				gimbal_ctrl_info.shoot_info.shoot_speed = 600;
			else
				gimbal_ctrl_info.shoot_info.shoot_speed = 0;
		};break;		
	}
	
	if((judgement_race_data.game_robot_data.shooter_heat0_cooling_limit - judgement_race_data.power_heat_data.shooter_heat0) <= 35)
		gimbal_ctrl_info.shoot_info.shoot_speed = 0;
	
	PC_updata(&auto_aim_info);
	
	send_gimbal_info(gimbal_ctrl_info.gim_info.pit_info, gimbal_ctrl_info.gim_info.yaw_info, \
						gimbal_ctrl_info.gim_info.gim_mode_info, gimbal_ctrl_info.shoot_info.reload_info, gimbal_ctrl_info.gim_info.save_info);
	
	send_shoot_info(gimbal_ctrl_info.shoot_info.friction_CCR, gimbal_ctrl_info.shoot_info.shoot_speed, \
						gimbal_ctrl_info.shoot_info.LASER_FLAG);
}


void chassis_command_unpack(void)
{
	if(RC_Ctl.key.v == 1)
		chassis_ctrl_info.chas_info.stay_f = RM_SET;
	else
		chassis_ctrl_info.chas_info.stay_f = RM_UNSET;
	
	if(detect_status.DR16_S == 0)
		chassis_ctrl_info.chas_info.silence_f = RM_SET;
	else
		chassis_ctrl_info.chas_info.silence_f = RM_UNSET;
	
}

void rc_detect(void)
{
	switch(RC_Ctl.rc.sr){
		case 1:{
			command_info.sr1.status_f = 1;
		};break;
		case 3:{
			if(command_info.sr1.status_f == 1)
			{
				if(command_info.sr1.mode == 1)
					command_info.sr1.mode = 0;
				else
					command_info.sr1.mode = 1;
				command_info.sr1.status_f = 0;
			}
			
			if(command_info.sr2.status_f == 1)
			{
				if(command_info.sr2.mode == 1)
					command_info.sr2.mode = 0;
				else
					command_info.sr2.mode = 1;
				command_info.sr2.status_f = 0;
			}
		};break;
		case 2:{
			command_info.sr2.status_f = 1;
		};break;
	}
}

void key_detect(uint16_t key_value,key_param_t* key_param)
{
	if(key_value == 1)
	{
		if(key_param->status_f == KEY_RELEASE)
		{
			key_param->status_f = KEY_PRESS;
			if(key_param->first_press_f == 1)
			{
				key_param->first_press_f = 0;
				key_param->mode = 0;
			}
			if(key_param->mode < key_param->mode_max)
				key_param->mode ++;
			key_param->time = KEY_INTERVAL;
		}
	}
	else
	{
		key_param->status_f = KEY_RELEASE;
		if(key_param->time == 0)
		{
			key_param->first_press_f = 1;
		}
		else
		{
			key_param->time --;
		}
	}
}


void PC_unpack(auto_aim_info_t* auto_aim)
{
	if((auto_aim->receive_buf[0] == PC_SOF) || (auto_aim->receive_buf[9] == PC_END))
	{
		auto_aim->flag = auto_aim->receive_buf[1];
		auto_aim->x = (int16_t)(auto_aim->receive_buf[3]<<8 | auto_aim->receive_buf[2]);
		auto_aim->y = (int16_t)(auto_aim->receive_buf[5]<<8 | auto_aim->receive_buf[4]);
		auto_aim->z = (int16_t)(auto_aim->receive_buf[7]<<8 | auto_aim->receive_buf[6]);		
	}
	else
	{
		auto_aim->flag = 0;
	}
}

void PC_updata(auto_aim_info_t* auto_aim)
{
	auto_aim->send_buf[0] = PC_SOF;
	if(judgement_race_data.game_robot_data.robot_id == 0)
		return;
	if(judgement_race_data.game_robot_data.robot_id <= 7)
		auto_aim->send_buf[1] = 0xEE;
	else
		auto_aim->send_buf[1] = 0xDD;
	
	int16_t pit, yaw;
	pit = (GimbalMotor.Theta_Real[0] - PITCH_MID) * 4.3945;
	yaw = GimbalMotor.Theta_Real[1] * 4.3945;
	
	auto_aim->send_buf[2] = pit;
	auto_aim->send_buf[3] = pit >> 8;
	
	auto_aim->send_buf[4] = yaw;
	auto_aim->send_buf[5] = yaw >> 8;
	
	auto_aim->send_buf[6] = 20;
	
	auto_aim->send_buf[8] = 0x00;
	auto_aim->send_buf[9] = PC_END;
	
	RichJ_UART_Transmit_DMA(&PC_HUART, auto_aim->send_buf, 10);
}




