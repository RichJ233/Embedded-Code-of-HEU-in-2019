#ifndef __BSP_IMU_H__
#define __BSP_IMU_H__

#include "stm32f4xx_hal.h"

#define IMU_TASK_PERIOD 1

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;
    
    int32_t ax_offset;
    int32_t ay_offset;
    int32_t az_offset;
  
    int32_t gx_offset;
    int32_t gy_offset;
    int32_t gz_offset;
    
} mpu_data_t;

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    float temp;
    float temp_ref;
  
    float wx;
    float wy;
    float wz;

    float vx;
    float vy;
    float vz;
  
    float rol;
    float pit;
    float yaw;
} imu_data_t;

typedef struct
{
	double xBuf[5];
	double yBuf[5];
	int begin_flag;
} butter_worth_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
	
	butter_worth_t gyro_filter_x;
	butter_worth_t gyro_filter_y;
	butter_worth_t gyro_filter_z;
} gyro_data_t;

typedef struct
{
  int roll_cnt;
  int pitch_cnt;
  int yaw_cnt;
  
  float last_roll;
  float last_pitch;
  float last_yaw;
  float last_temp;

  float roll;
  float pitch;
  float yaw;
} imu_attitude_t;

extern uint8_t Clear_flag;
extern imu_data_t     imu;
extern mpu_data_t     mpu_data;
extern imu_attitude_t atti;


uint8_t imu_device_init(void);
void GyroTask(void);
uint8_t mpu_write_reg(uint8_t const reg, uint8_t const data);
uint8_t mpu_read_reg(uint8_t const reg);
uint8_t mpu_read_regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);
void mpu_offset_cal(void);
void mpu_get_data(void);
void imu_clear(void);

#endif



