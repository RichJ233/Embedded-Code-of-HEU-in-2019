#ifndef __SUPER_CAPS_TASK_H__
#define __SUPER_CAPS_TASK_H__

#include "cmsis_os.h"

//y=3.184*x-0.048

//U = 0.314 * A + 0.0152
#define CHARGE_K 0.314f
#define CHARGE_B 0.0152f

#define DAC_CONVERT 1241.2f

#define VOLTAGE_CONVERT 0.00805664f
#define VOLTAGE_BUF_LENGTH 10

#define RELAY_OPEN 1
#define RELAY_CLOSE 0

#define CAPS_TASK_PERIOD 10

typedef struct{
	uint8_t  relay_ctrl;
	uint16_t ad_value;
	uint16_t da_value;
	float    chassis_power;
	float    chassis_voltage;
	float    voltage_sum;
	float    voltage_buf[VOLTAGE_BUF_LENGTH];
	float    chassis_ctrl_current;
}caps_ctrl_info_t;

typedef struct{
	float power_now;
	uint16_t dac_value;
	float current_set;
	float voltage_set;
	
}dac_ctrl_info_t;

typedef struct{
	uint8_t ctrl_f;
	uint8_t ctrl_vol_f;
}relay_ctrl_info_t;

extern caps_ctrl_info_t caps_ctrl_info;
extern relay_ctrl_info_t relay_ctrl_info;
extern dac_ctrl_info_t dac_ctrl_info;

void caps_task(void);
void adc_value_get(void);
//void dac_value_set(void);
void relay_contrl(void);


#endif
