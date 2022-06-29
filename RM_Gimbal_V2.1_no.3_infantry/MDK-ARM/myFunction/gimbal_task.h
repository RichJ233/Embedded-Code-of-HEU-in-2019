#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "FREERTOS.h"

#define GIMBAL_TASK_PERIOD 1
#define MovintAverageForPitchLength 10
#define MovintAverageForYawLength 10

#define ANGLE_TO_ENCODER 22.7556f

typedef struct{
	int16_t pit_info;
	int16_t yaw_info;
	uint8_t gim_info;
	uint8_t reload_info;
	uint8_t auto_aim_f;
	int16_t save_info;
}gimbal_ctrl_info_t;

typedef struct{
	float gyro;
	float angle;
}gimbal_status_info_t;

typedef struct{
	float ctrl;
	float ex;
	float output;
	uint8_t aim_f;
	float aim_offset;
	float aim_temp;
}gimbal_ctrl_t;

extern gimbal_ctrl_info_t gimbal_ctrl_info;
extern gimbal_status_info_t yaw_status_info;
extern gimbal_status_info_t pit_status_info;
extern gimbal_ctrl_t pit_ctrl;
extern gimbal_ctrl_t yaw_ctrl;
extern float Encoder_Pitch_Buff[MovintAverageForPitchLength];

void GimbalTask(void);
float ctrl_limit(float Input,float Min,float Max);
void MovingAverageForPitch(void);
void MovingAverageForAuto(void);
void angle_handle(void);
void sys_reset(void);

#endif
