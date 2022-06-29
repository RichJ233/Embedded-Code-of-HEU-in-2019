#include "detect_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_flash.h"
#include "string.h"
//#include "iwdg.h"

#define detectPERIOD 10

detect_flag_t detect_flag;
detect_time_t detect_time;
detect_status_t detect_status;
flash_write_t flash_write;

void detect_Task(void)
{
	iwdg_reset();
	uint32_t detect_task_time = osKernelSysTick();
	for(;;)
	{
//		iwdg_reset();
		
//		reset_check();
		
		DR16_detect();
		Judgement_detect();
		
		flash_write.run_time[flash_write.write_buf_f] ++;
		
		osDelayUntil(&detect_task_time, detectPERIOD);
	}
}

void flash_init(void)
{	
	bsp_flash_read();
	
	flash_write.write_buf_f ++;
	if(flash_write.write_buf_f >= 3)
		flash_write.write_buf_f = 0;
	flash_write.run_time[flash_write.write_buf_f] = 0;
}

void iwdg_reset(void)
{
//	HAL_IWDG_Refresh(&hiwdg);
}

void reset_check(void)
{	
	if(flash_write.reset_f == 1)
	{
		flash_write.reset_f = 0;
		
//		flash_write.reset_f = 0;
//		flash_write.unnormal_f = 1;
//		memcpy((void*)&flash_write.detect_status,(void*)&detect_status, sizeof(detect_status_t));
//		bsp_flash_write();
//		__set_FAULTMASK(1);
//		NVIC_SystemReset();
	}
	
	if(flash_write.run_time[flash_write.write_buf_f] / 100 == 0)
		bsp_flash_write();
	
	if(flash_write.manual_f == 1)
	{
		flash_write.manual_f = 0;
		bsp_flash_write();
	}
}

void DR16_detect(void)
{
	if(detect_flag.DR16_F == 1)
	{
		detect_flag.DR16_F = 0;
		detect_time.DR16_T = 0;
	}
	else
	{
		detect_time.DR16_T ++;
		if(detect_time.DR16_T >= 2)
			detect_status.DR16_S = 0;
		else
			detect_status.DR16_S = 1;
	}
}


void Judgement_detect(void)
{
	if(detect_flag.Judgement_F == 1)
	{
		detect_flag.Judgement_F = 0;
		detect_time.Judgement_T = 0;
	}
	else
	{
		detect_time.Judgement_T ++;
		if(detect_time.Judgement_T >= 4)
			detect_status.Judgement_S = 0;
		else
			detect_status.Judgement_S = 1;
	}
}

void bsp_flash_read(void)
{
	memcpy((void*)&flash_write, (void*)PARAM_SAVED_START_ADDRESS, sizeof(flash_write_t));
}

void bsp_flash_write(void)
{
  taskENTER_CRITICAL();
  BSP_FLASH_Write((uint8_t*)&flash_write, sizeof(flash_write_t));
  taskEXIT_CRITICAL();
}
