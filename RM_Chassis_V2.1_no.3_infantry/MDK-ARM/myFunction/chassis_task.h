#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "FREERTOS.h"

#define CHASSIS_TASK_PERIOD 10
#define POWER_LIMIT_PARAM 5.5f
#define POWER_MAX 79

#define ChassisKP 4.0f
#define ChassisKI 0.3f
#define ChassisKD  0.0f

#define YawKP 10.0f
#define YawKI 0.0f
#define YawKD 90.0f

#define ROLL_FORWARD 1
#define ROLL_BACK 	-1

#define SPEED_MAX 9000

typedef struct{
	uint8_t silence_f;
	uint8_t normal_f;
	uint8_t roll_f;
	uint8_t shake_f;
	uint8_t stay_f;
	uint8_t reset_f;
}chassis_motion_info_t;

typedef struct{
	float  set_speed[4];
}chassis_speed_t;

typedef struct{
	chassis_motion_info_t chas_info;
	float x_speed_info;
	float x_speed_buf;
	float y_speed_info;
	float y_speed_buf;
	float z_roll_info;
	uint8_t z_roll_mode;
	uint8_t z_roll_flag;//这一坨并不想优化
	chassis_speed_t chassis_speed;
	float speed_temp;
	float   speed_k;
}chassis_ctrl_info_t;

typedef struct{
	int16_t speed[4];
	float   angle;
	uint8_t can_s; 
}chassis_status_info_t;

typedef struct{
	float	power_now[4];
	float	motor_out[4];
	float	motor_out_sum;
	float	out_limit_sum;
	float	power_change[4];
	float	power_buf[4];
	float	power_div[4];
}power_limit_data_t;

extern chassis_ctrl_info_t chassis_ctrl_info;
extern chassis_status_info_t chassis_status_info;
extern power_limit_data_t power_limit_data;

void ChassisTask(void);
void motion_solution(void);
void silence_check(void);
void x_ctrl(chassis_ctrl_info_t* chassis_ctrl_info_b);
void y_ctrl(chassis_ctrl_info_t* chassis_ctrl_info_b);
void z_ctrl(chassis_ctrl_info_t* chassis_ctrl_info_b);
void sum_check(chassis_ctrl_info_t* chassis_ctrl_info_b);
void mcu_reset(void);
void power_limit(void);
void pid_calcaulate(void);


#endif
/*

   \\\  -----憨憨车-----  ///
   \1\--|			   |--/4/
   \\\  |			   |  ///
        |			   |
		|--237--	   |
		|		|	   |
   ///  |	   185	   |  \\\
   /2/--|		|	   |--\3\
   ///  -----憨憨车-----  \\\

*/

