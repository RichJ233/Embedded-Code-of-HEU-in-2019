#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "stm32f4xx_hal.h"

#define CAN_TASK_PERIOD 1

typedef struct{
	uint16_t Theta_Real[5];
	int16_t	 Speed_Real[5];
	int16_t	 Current_Real[5];
}BLDC_Motor_Data;

typedef struct{
	int16_t Theta_Real[2];
	int16_t	 Speed_Real[5];
	int16_t	 Current_Real[2];
}BLDC_Gimbal_Data;

void can_device_init(void);
void can_receive_start(void);
void send_gimbal_info(int16_t pit_info, int16_t yaw_info, uint8_t gim_info, uint8_t reload_info,int16_t save_info);
void send_shoot_info(int16_t friction_info, int16_t shoot_speed_info, uint8_t laser_info);
uint8_t send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

/* CAN send and receive ID */
typedef enum
{
  CAN_3508_M1_ID       = 0x201,
  CAN_3508_M2_ID       = 0x202,
  CAN_3508_M3_ID       = 0x203,
  CAN_3508_M4_ID       = 0x204,
  CAN_YAW_MOTOR_ID     = 0x206,
  CAN_PIT_MOTOR_ID     = 0x205, 
  CAN_TRIGGER_MOTOR_ID = 0x207,
  CAN_SHOOT_INFO_ID    = 0x300,
  CAN_GIMBAL_INFO_ID   = 0x301,
  CAN_CHASSIS_ZGYRO_ID = 0x401,
  CAN_GIMBAL_ZGYRO_ID  = 0x402,

  CAN_ZGYRO_RST_ID     = 0x406,
  CAN_CHASSIS_ALL_ID   = 0x200,
  CAN_GIMBAL_ALL_ID    = 0x1ff,

} can_msg_id_e;

extern BLDC_Gimbal_Data GimbalMotor;
extern BLDC_Motor_Data	TRIGGER_Motor_Data;
extern BLDC_Motor_Data	Motor_Data;

#endif


