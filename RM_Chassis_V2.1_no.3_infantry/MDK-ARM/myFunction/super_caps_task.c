#include "super_caps_task.h"
#include "cmsis_os.h"
#include "RM_Define.h"
#include "chassis_task.h"
#include "judgement_info.h"
#include "adc.h"
#include "gpio.h"
//#include "dac.h"
#include "pid.h"

float power_k[3] = {0, 0, 0};
caps_ctrl_info_t caps_ctrl_info;
relay_ctrl_info_t relay_ctrl_info;
dac_ctrl_info_t dac_ctrl_info;
pid_st pid_power = PID_PARAM_INIT;

void caps_task(void)
{
	pid_init(&pid_power, power_k[0], power_k[1], power_k[2], SPEED, 0.05, 4.75, 0.3);
	uint32_t caps_task_time = osKernelSysTick();
	for(;;)
	{
		adc_value_get();
		
//		relay_contrl();
		
//		dac_value_set();
		
		osDelayUntil(&caps_task_time, CAPS_TASK_PERIOD);
	}
}


void adc_value_get(void)
{
	static uint8_t num = 0;
	static uint8_t cnt = 0;
	HAL_ADC_Start(&hadc1);
	
	HAL_ADC_PollForConversion(&hadc1, 50);
	
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	{
		caps_ctrl_info.ad_value = HAL_ADC_GetValue(&hadc1);
		
		if((++num) == VOLTAGE_BUF_LENGTH)
			num = 0;
		
		caps_ctrl_info.voltage_buf[num] = (float)caps_ctrl_info.ad_value * VOLTAGE_CONVERT;
		caps_ctrl_info.voltage_sum = 0;
		for(cnt = 0;cnt < VOLTAGE_BUF_LENGTH;cnt ++)
			caps_ctrl_info.voltage_sum += caps_ctrl_info.voltage_buf[cnt];
		
		caps_ctrl_info.chassis_voltage = caps_ctrl_info.voltage_sum / VOLTAGE_BUF_LENGTH;
	}
}

void relay_contrl(void)
{
	if(caps_ctrl_info.chassis_voltage <= LOW_VOLTAGE)
	{
		relay_ctrl_info.ctrl_f = RELAY_CLOSE;
		if((chassis_ctrl_info.x_speed_info == 0) && (chassis_ctrl_info.x_speed_info == 0))
		{
			relay_ctrl_info.ctrl_f = RELAY_OPEN;
		}
	}
	else
	{
		relay_ctrl_info.ctrl_f = RELAY_OPEN;
	}
	
	if(relay_ctrl_info.ctrl_f == RELAY_OPEN)
		HAL_GPIO_WritePin(RELAY_GPIO_Port,RELAY_Pin,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(RELAY_GPIO_Port,RELAY_Pin,GPIO_PIN_SET);
}

//void dac_value_set(void)
//{
//	if(relay_ctrl_info.ctrl_f == RELAY_OPEN)
//	{
//		dac_ctrl_info.power_now = judgement_race_data.power_heat_data.chassis_power;
//		dac_ctrl_info.current_set = pid_cal(&pid_power, dac_ctrl_info.power_now, POWER_MAX);
//		if(dac_ctrl_info.current_set < 0)//讲道理是不会发生的
//			dac_ctrl_info.current_set = 0;
//	}
//	else
//		dac_ctrl_info.current_set = 3.2;
//	
//	dac_ctrl_info.voltage_set = CHARGE_K * dac_ctrl_info.current_set + CHARGE_B;
//	dac_ctrl_info.dac_value = (uint16_t)(dac_ctrl_info.voltage_set * DAC_CONVERT);
//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_ctrl_info.dac_value);
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
//}





