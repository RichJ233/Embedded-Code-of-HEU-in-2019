#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#include "FREERTOS.h"
#include "pid.h"

void pwm_device_init(void);
void ShootTask(void);
void bsp_io_init(void);
void friction_ctrl(void);
void laser_ctrl(void);
void reload_ctrl(void);
void shoot_ctrl(void);
int16_t pluck_DELTA_pid(pid_st *pid,int real,int ex);

#define ShootPositionKP 0.15
#define ShootSpeedKP -1.65000005
#define ShootSpeedKI -0.0379999988
#define ShootSpeedKD  0.0f

#define SHOOT_TASK_PERIOD 10


typedef struct{
	uint16_t shoot_speed;
	uint16_t friction_CCR;
	uint8_t LASER_FLAG;
}shoot_info_t;

extern shoot_info_t shoot_info;
extern float trig_out;



#endif

