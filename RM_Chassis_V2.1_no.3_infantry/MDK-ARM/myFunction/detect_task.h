#ifndef __DETECT_H__
#define __DETECT_H__

#include "cmsis_os.h"
#include "stdbool.h"
#include "FreeRTOS.h"

typedef struct{
	bool	DR16_F;
	bool	Chassis_F[4];
	bool	Judgement_F;
	bool    Gimbal_F;
}detect_flag_t;

typedef struct{
	uint16_t	DR16_T;
	uint16_t	Chassis_T[4];
	uint16_t	Judgement_T;
	uint16_t    Gimbal_T;
}detect_time_t;

typedef struct{
	bool	DR16_S;
	bool	Chassis_S[4];
	bool	Judgement_S;
	bool    Gimbal_S;
}detect_status_t;

typedef struct{
	uint8_t reset_f;
	uint8_t unnormal_f;
	uint8_t manual_f;
	uint8_t write_flag;
	uint8_t write_buf_f;
	float   run_time[3];
	detect_status_t detect_status;
}flash_write_t;

extern detect_flag_t detect_flag;
extern detect_time_t detect_time;
extern detect_status_t detect_status;
extern flash_write_t flash_write;




void iwdg_reset(void);
void reset_check(void);
void detect_Task(void);
void DR16_detect(void);
void Judgement_detect(void);
void bsp_flash_read(void);
void flash_init(void);
void bsp_flash_write(void);


#endif


