#include "calibrate.h"
#include "bsp_flash.h"
#include "string.h"

cali_sys_t cali_param;

/**
  * @brief save calibrate data cali_param, write this structure in chip flash
  * @usage called when calibrate data be changed in calibrate loop
  */
void save_cali_data(void)
{
  taskENTER_CRITICAL();
  BSP_FLASH_Write((uint8_t*)&cali_param, sizeof(cali_sys_t));
  taskEXIT_CRITICAL();
}
/**
  * @brief read calibrate data cali_param from chip flash
  * @usage called after cali_param_init() in main() initialize part.
  */
void cali_data_read(void)
{
  memcpy((void*)&cali_param, (void*)PARAM_SAVED_START_ADDRESS, sizeof(cali_sys_t));
}

void cali_param_init(void)
{
  cali_param.imu_cali_list.name = "gyro";
  
  cali_data_read();
}




