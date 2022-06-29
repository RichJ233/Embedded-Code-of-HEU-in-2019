#ifndef __CONTORL_TASK_H__
#define __CONTORL_TASK_H__

#include "FREERTOS.h"

#define KEY_PRESS 1
#define KEY_RELEASE 0

#define PC_SOF 0X66
#define PC_END 0x88

#define MAX_1 1
#define MAX_2 2
#define MAX_3 3
#define MAX_4 4

typedef struct{
	uint16_t shoot_speed;
	uint16_t friction_CCR;
	uint8_t LASER_FLAG;
	uint8_t reload_info;
}shoot_info_t;

typedef struct{
	int16_t pit_info;
	int16_t yaw_info;
	uint8_t gim_mode_info;
	int16_t save_info;
}gim_info_t;

typedef struct{
	gim_info_t gim_info;
	shoot_info_t shoot_info;	
}gimbal_ctrl_info_t;

typedef struct{
	uint16_t time;
	uint8_t first_press_f;
	uint8_t mode;
	uint8_t status_f;
	uint8_t mode_max;
}key_param_t;

typedef struct{
	uint8_t receive_buf[15];
	uint8_t send_buf[15];
	uint8_t flag;
	int16_t x;
	int16_t y;
	int16_t z;
}auto_aim_info_t;

typedef struct{
	key_param_t z;//ÉäËÙ
	key_param_t x;//ÉäÆµ
	key_param_t r;//µ¯²Ö¸Ç
	key_param_t f;//´ò·û
	key_param_t g;//ÖØÆô
	key_param_t v;//¾²Ä¬Ä£Ê½
	
	key_param_t sr1;
	key_param_t sr2;
}command_info_t;

void GimbalTask(void);
void command_unpack(void);
void gimbal_command_update(void);
void key_detect(uint16_t key_value,key_param_t* key_param);
void command_init(void);
void rc_detect(void);
void chassis_command_unpack(void);
void PC_unpack(auto_aim_info_t* auto_aim);
void PC_updata(auto_aim_info_t* auto_aim);


extern auto_aim_info_t auto_aim_info;

#define GIMBAL_TASK_PERIOD 7

/*
Send






Receive

sof 0X66
uint8_t flag
int16_t x
int16_t y
int16_t z
uint8_t save
end 0x88
*/
#endif


