#ifndef __RM_DEFINE_H__
#define __RM_DEFINE_H__

#define Infantry_Test 0

#define RM_SET 		1
#define RM_UNSET 	0

#define PIT_8192 1					//用于云台PIT电机是否超过量程8192，超过则为1，反之。
#define PITCH_MID 9520

#define YAW_MID 690
#define PI 3.141592653f
//////////////////////////////////////宏就完事了嗷
#if YAW_MID > 4096
	#define YAW_MID_HALF YAW_MID-4096 
#endif

#if YAW_MID < 4096
	#define YAW_MID_HALF YAW_MID+4096 
#endif

#define ENCODER_MIDD 4096
#define ENCODER_MAX  8192

#define CHASSIS_PID_TEST 0

#define Channel_K0 9.2
#define Channel_K1 12
#define Channel_K2 -2.5
#define Channel_K3 0.1f
#define Mouse_X_K -7.0f
#define Mouse_Y_K 1.0f

#define DBUS_HUART 		huart1
#define PC_HUART		huart2
#define JUDGEMENT_HUART	huart6

#define MOTOR_MAX_SPEED 9000.0f
#define X_SPEED 8000.0f
#define Y_SPEED 5500.0f
#define Z_SPEED 4500.0f
#define Z_ROLL_LIMIT PI/4
#define SLOW_K 0.5f
#define SOFTBOOST 0.05f//必须小于1 否则车必疯
#define SOFTBRAKE 0.30f//必须小于1 否则车必疯

//caps contorl
#define LOW_VOLTAGE 16.0f
#define LOW_SPEED 2.5f

//gimbal mode
#define KEY_INTERVAL 301/GIMBAL_TASK_PERIOD
#define SHOOT_SPEED_1 480
#define SHOOT_SPEED_2 960
#define SHOOT_SPEED_3 1920
#define FrictionCCR14 1120
#define FrictionCCR20 1180
#define FrictionCCR25 1380



typedef enum
{
  SILENCE_MODE    = 0x00,
  CONTROL_MODE    = 0x01,
  AUTO_AIM_MODE   = 0x02,
  AUTO_AIM_MODE_NO= 0x03,
  AUTO_GAIN_MODE  = 0x04,
  RESET_MODE      = 0xFF,
} gim_mod_id_e;
//chassis mode
typedef enum
{
  CHAS_SILENCE_MODE    = 0x00,
  CHAS_NORMOL_MODE     = 0x01,
  CHAS_ROLL_MODE       = 0x02,
  CHAS_SHAKE_MODE      = 0x03,
  CHAS_STAY_MODE       = 0x04,
  CHAS_RESET_MODE      = 0xFF,
} chas_mod_id_e;
//rc commond
typedef enum
{
  RC_MODE    		   = 0x01,
  KEY_MODE     		   = 0x03,
  AUTOmode    		   = 0x02,
} rc_mod_id_e;


#endif
