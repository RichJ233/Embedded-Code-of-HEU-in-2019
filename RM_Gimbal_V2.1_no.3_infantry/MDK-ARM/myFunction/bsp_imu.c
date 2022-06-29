#include "bsp_imu.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "string.h"
#include "math.h"
#include "spi.h"
#include "mpu6500_reg.h"
#include "imu_k.h"
#include "calibrate.h"
#include "EKF.h"

#define MPU_INIT_DELAY(x) HAL_Delay(x)
#define MPU_DELAY(x)      osDelay(x)

#define MPU_HSPI hspi3
#define MPU_NSS_LOW HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET)

#define DEAD_BAND 4

static uint8_t tx, rx;
static uint8_t tx_buff[14];
static uint8_t mpu_buff[14];
uint8_t mpu_id;

mpu_data_t     	mpu_data;
imu_data_t     	imu;
imu_attitude_t	atti;
gyro_data_t    	gyro_offset_data;

/**
  * @brief      butterworth filter
  * @param[in]  input:Gyro data
  * @retval     output:Gyro filter data
  * @filterparm lowpass Fc=500Hz,Fs=8000Hz
  */
float b[5] = {
   0.032056338012672166 ,  0.12822535205068866  ,   0.19233802807603298  ,  0.12822535205068866  ,	0.032056338012672166 
};
float a[5] = {
   1,   			  -1.1313559029908347  ,    0.8932951181498886  ,   -0.29427475495935135 ,	  0.045236948003052092
};

butter_worth_t gyro_filter_x;
butter_worth_t gyro_filter_y;
butter_worth_t gyro_filter_z;
butter_worth_t acell_filter_x;
butter_worth_t acell_filter_y;
butter_worth_t acell_filter_z;

double filter(butter_worth_t *gyro_filter, double x)
{
	for(int i=4;i>0;i--)
	{
		gyro_filter->yBuf[i] = gyro_filter->yBuf[i-1];
		gyro_filter->xBuf[i] = gyro_filter->xBuf[i-1];
	}
	gyro_filter->xBuf[0] = x;
	if(gyro_filter->begin_flag>0)
	{
		gyro_filter->begin_flag =0;
		gyro_filter->yBuf[0] = x;
		return x;
	}
	gyro_filter->yBuf[0] = b[0]* (gyro_filter->xBuf[0]+gyro_filter->xBuf[4])+b[1]*(gyro_filter->xBuf[1]+gyro_filter->xBuf[3])+b[2]*gyro_filter->xBuf[2]
							-(a[1]* gyro_filter->yBuf[1]+a[2]*gyro_filter->yBuf[2]+a[3]*gyro_filter->yBuf[3]+a[4]*gyro_filter->yBuf[4]);
	return gyro_filter->yBuf[0];
}


/**
  * @brief      butterworth filter
  * @param[in]  input:Gyro data
  * @retval     output:Gyro filter data
  * @filterparm lowpass Fc=500Hz,Fs=8000Hz
  */
float Ab[5] = {
   0.010209480791203138,  0.040837923164812551,   0.061256884747218826,  0.040837923164812551,	0.010209480791203138
};
float Aa[5] = {
   1,   			  -1.9684277869385181,    1.7358607092088865 ,   -0.72447082950736263,	  0.12038959989624452
};

double acell_filter(butter_worth_t *gyro_filter, double x)
{
	for(int i=4;i>0;i--)
	{
		gyro_filter->yBuf[i] = gyro_filter->yBuf[i-1];
		gyro_filter->xBuf[i] = gyro_filter->xBuf[i-1];
	}
	gyro_filter->xBuf[0] = x;
	if(gyro_filter->begin_flag>0)
	{
		gyro_filter->begin_flag =0;
		gyro_filter->yBuf[0] = x;
		return x;
	}
	gyro_filter->yBuf[0] = Ab[0]* (gyro_filter->xBuf[0]+gyro_filter->xBuf[4])+Ab[1]*(gyro_filter->xBuf[1]+gyro_filter->xBuf[3])+Ab[2]*gyro_filter->xBuf[2]
							-(Aa[1]* gyro_filter->yBuf[1]+Aa[2]*gyro_filter->yBuf[2]+Aa[3]*gyro_filter->yBuf[3]+Aa[4]*gyro_filter->yBuf[4]);
	return gyro_filter->yBuf[0];
}

int16_t dead_band(int16_t data, int16_t band_width)
{
	if(fabs((float)data) <= band_width)
		return 0;
	else
		return data;
}

uint8_t mpu_write_reg(uint8_t const reg, uint8_t const data)
{
  MPU_NSS_LOW;
  tx = reg & 0x7F;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  tx = data;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  MPU_NSS_HIGH;
  return 0;
}

uint8_t mpu_read_reg(uint8_t const reg)
{
  MPU_NSS_LOW;
  tx = reg | 0x80;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  MPU_NSS_HIGH;
  return rx;
}

uint8_t mpu_read_regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  MPU_NSS_LOW;
  tx = regAddr | 0x80;
  tx_buff[0] = tx;
  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
  MPU_NSS_HIGH;
  return 0;
}

uint8_t imu_device_init(void)
{
	mpu_write_reg(MPU6500_PWR_MGMT_1, 0x80);
	MPU_INIT_DELAY(100);
	mpu_write_reg(MPU6500_SIGNAL_PATH_RESET, 0x07);
	MPU_INIT_DELAY(100);
	mpu_id = mpu_read_reg(MPU6500_WHO_AM_I);
	
	uint8_t MPU6500_Init_Data[7][2] = {
		{ MPU6500_PWR_MGMT_1,     0x03 }, // Auto selects Clock Source
		{ MPU6500_PWR_MGMT_2,     0x00 }, // all enable
		{ MPU6500_CONFIG,         0x04 }, // gyro bandwidth 184Hz 01
		{ MPU6500_GYRO_CONFIG,    0x18 }, // +-2000dps
		{ MPU6500_ACCEL_CONFIG,   0x10 }, // +-8G
		{ MPU6500_ACCEL_CONFIG_2, 0x04 }, // acc bandwidth 20Hz
		{ MPU6500_USER_CTRL,      0x20 }, // Enable the I2C Master I/F module
								  // pins ES_DA and ES_SCL are isolated from 
								  // pins SDA/SDI and SCL/SCLK.
	};
	uint8_t i = 0;
	for (i = 0; i < 7; i++)
	{
		mpu_write_reg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_INIT_DELAY(1);
	}
	
	mpu_offset_cal();
	return 0;
}

void mpu_offset_cal(void)
{
	if(cali_param.reset_flag == 1)
	{
		cali_param.reset_flag = 0;
		mpu_data.gx_offset = cali_param.imu_cali_list.offset[0];
		mpu_data.gy_offset = cali_param.imu_cali_list.offset[1];
		mpu_data.gz_offset = cali_param.imu_cali_list.offset[2];
		save_cali_data();
		return;
	}
	
  int i;
  for (i = 0; i < 100; i++)
  {
    mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
	MPU_INIT_DELAY(5);
  }
	
  for (i = 0; i < 300; i++)
  {
    mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
	  
    mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

	gyro_offset_data.x = mpu_buff[8] << 8  | mpu_buff[9];
    gyro_offset_data.y = mpu_buff[10] << 8 | mpu_buff[11];
    gyro_offset_data.z = mpu_buff[12] << 8 | mpu_buff[13];
	  
	gyro_offset_data.x = filter(&gyro_offset_data.gyro_filter_x,gyro_offset_data.x);
	gyro_offset_data.y = filter(&gyro_offset_data.gyro_filter_y,gyro_offset_data.y);
	gyro_offset_data.z = filter(&gyro_offset_data.gyro_filter_z,gyro_offset_data.z);

    mpu_data.gx_offset += gyro_offset_data.x;
    mpu_data.gy_offset += gyro_offset_data.y;
    mpu_data.gz_offset += gyro_offset_data.z;

    MPU_INIT_DELAY(5);
  }
  mpu_data.ax_offset=mpu_data.ax_offset / 300;
  mpu_data.ay_offset=mpu_data.ay_offset / 300;
  mpu_data.az_offset=mpu_data.az_offset / 300;
  mpu_data.gx_offset=mpu_data.gx_offset	/ 300;
  mpu_data.gy_offset=mpu_data.gy_offset	/ 300;
  mpu_data.gz_offset=mpu_data.gz_offset	/ 300;
  
  cali_param.imu_cali_list.offset[0] = mpu_data.gx_offset;
  cali_param.imu_cali_list.offset[1] = mpu_data.gy_offset;
  cali_param.imu_cali_list.offset[2] = mpu_data.gz_offset;
}

int gyro_watch;

void mpu_get_data(void)
{
  mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

  mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
  mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
  mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
  mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

  mpu_data.gx = ((mpu_buff[8] << 8 | mpu_buff[9])   - mpu_data.gx_offset);
  mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
  mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);
	
	mpu_data.gx = dead_band(mpu_data.gx, DEAD_BAND);
	mpu_data.gy = dead_band(mpu_data.gy, DEAD_BAND);
	mpu_data.gz = dead_band(mpu_data.gz, DEAD_BAND);
	
	mpu_data.gx = filter(&gyro_filter_x,mpu_data.gx);
	mpu_data.gy = filter(&gyro_filter_y,mpu_data.gy);
	mpu_data.gz = filter(&gyro_filter_z,mpu_data.gz);
	
	mpu_data.ax = acell_filter(&acell_filter_x,mpu_data.ax);
	mpu_data.ay = acell_filter(&acell_filter_y,mpu_data.ay);
	mpu_data.az = acell_filter(&acell_filter_z,mpu_data.az);
	
	gyro_watch = mpu_data.gz;

  memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
  imu.temp = 21 + mpu_data.temp / 333.87f;
  imu.wx   = mpu_data.gx / 16.384f / 57.3f; //2000dps -> rad/s
  imu.wy   = mpu_data.gy / 16.384f / 57.3f; //2000dps -> rad/s
  imu.wz   = mpu_data.gz / 16.384f / 57.3f; //2000dps -> rad/s
/*
  imu.vx   += (mpu_data.ax - mpu_data.ax_offset) / 4096.*100.*0.005; //2000dps -> rad/s
  imu.vy   += (mpu_data.ay - mpu_data.ay_offset) / 4096.*100.*0.005; //2000dps -> rad/s
  imu.vz   += (mpu_data.az - mpu_data.az_offset) /4096.*100.*0.005; //2000dps -> rad/s
*/
}




