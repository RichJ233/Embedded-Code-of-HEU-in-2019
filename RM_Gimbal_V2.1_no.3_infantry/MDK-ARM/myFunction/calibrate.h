#ifndef __CALIBRATE_H__
#define __CALIBRATE_H__

#include "cmsis_os.h"

typedef enum 
{
  CALI_GYRO    = 0,
  CALI_ACC     = 1,
  CALI_MAG     = 2,
  CALI_IMU_NUM = 3,
} cali_imu_e;

typedef __packed struct
{
  int16_t offset[3];   //x,y,z
  uint8_t cali_cmd;    //1:calibrae  0:not
  char*   name;
} imu_cali_t;


typedef __packed struct
{
  uint32_t   firmware_version;
  uint8_t	 reset_flag;
  imu_cali_t imu_cali_list;
} cali_sys_t;

extern cali_sys_t cali_param;

void cali_param_init(void);
void cali_data_read(void);
void save_cali_data(void);




#endif

