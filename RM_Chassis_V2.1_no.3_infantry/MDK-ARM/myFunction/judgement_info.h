#ifndef _JUDGEMENT_INFO_1__
#define _JUDGEMENT_INFO_1__

#include "stm32f4xx_hal.h"

#define SOF 				(uint8_t)0xA5
#define GAME_DATA 			0x0201
#define ROBOT_STATE_DATA	0x0001
#define POWER_HEAT_DATA 	0x0202
#define ROBOT_POS_DATA 		0x0203
#define ROBOT_HURT_DATA		0x0206
#define USER_DATA			0x0301

typedef struct
{
	uint8_t  buf[70];
	uint8_t  header[5];
	uint16_t cmd;
	uint8_t  data[30];
	uint8_t  tail[2];
	
} judge_receive_t;

typedef __packed struct
{
  uint8_t game_type;
  uint8_t game_progress;
  uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_heat0_cooling_rate;
 uint16_t shooter_heat0_cooling_limit;
 uint16_t shooter_heat1_cooling_rate;
 uint16_t shooter_heat1_cooling_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct
{
  uint16_t chassis_volt; 
  uint16_t chassis_current; 
  float chassis_power; 
  uint16_t chassis_power_buffer; 
  uint16_t shooter_heat0; 
  uint16_t shooter_heat1; 
} ext_power_heat_data_t;

typedef __packed struct
{
  float x;
  float y;
  float z;
  float yaw;
} ext_game_robot_pos_t;

typedef __packed struct
{
  uint8_t armor_id;
  uint8_t hurt_type;
} ext_robot_hurt_t;

typedef struct
{
	ext_game_state_t 		game_state_data;
	ext_power_heat_data_t  	power_heat_data;
	ext_game_robot_state_t	game_robot_data;
	ext_game_robot_pos_t 	robot_pos_data;
	ext_robot_hurt_t 		robot_hurt_data;
} judgement_race_t;



extern judge_receive_t judgement_receive;
extern judgement_race_t judgement_race_data;
void judgement_info_handle(void);
void unpack_fifo_handle(void);
void judgement_info_updata(void);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void judgement_data_decode(void);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif

/*
1.建立读取（DMA不定长）
2.在回调函数（UART_IdleRxCallback）中调用处理函数（unpack_fifo_handle）
3.处理完后的数据在judgement_race_data结构体中；
*/
