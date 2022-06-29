#include "gimbal_task.h"
#include "RM_Define.h"
#include "pid.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_imu.h"
#include "imu_k.h"
#include "shoot_task.h"
#include "stdbool.h"
#include "calibrate.h"
#include "EKF.h"

pid_st pid_yaw_position = PID_PARAM_INIT;
pid_st pid_yaw_speed = PID_PARAM_INIT;
pid_st pid_pit_position = PID_PARAM_INIT;
pid_st pid_pit_speed = PID_PARAM_INIT;
pid_st pid_pit_yaw_auto = PID_PARAM_INIT;
pid_st pid_pit_pit_aim = PID_PARAM_INIT;
pid_st pid_pit_yaw_aim = PID_PARAM_INIT;

float pid_param[5] = {0.43f, 12000.0f, -60.0f, -12500.0f, 0.0f};

gimbal_status_info_t yaw_status_info;
gimbal_status_info_t pit_status_info;

gimbal_ctrl_info_t gimbal_ctrl_info;

gimbal_ctrl_t pit_ctrl;
gimbal_ctrl_t yaw_ctrl;

float Encoder_Pitch_Buff[MovintAverageForPitchLength];
float Encoder_Pit_Auto_Buff[MovintAverageForPitchLength];
float Encoder_Yaw_Auto_Buff[MovintAverageForYawLength];

bool gyro_offset;

float watch1 = 0, watch2;

void GimbalTask(void)
{
	kalman_filter_init(&kalman_filter_auto_aim, &kalman_filter_init_data);
	
	pid_init(&pid_yaw_position, pid_param[0], 0.0f, 0.0f, POSITION, 5, 12000, 30);
	pid_init(&pid_yaw_speed, pid_param[1], 0.0f, 0.0f, POSITION, 5, 20000, 30);
	
	pid_init(&pid_pit_position, pid_param[2], 0.0f, 0.0f, POSITION, 5, 12000, 30);
	pid_init(&pid_pit_speed, pid_param[3], 0.0f, 0.0f, POSITION, 5, 20000, 30);
	
	pid_init(&pid_pit_pit_aim, -0.00007, 0.0f, -0.005, POSITION, 5, 12000, 30);
	pid_init(&pid_pit_yaw_aim, 0.0149999997, -0.0, 0.0149999997, POSITION, 5, 12000, 30);
	pid_init(&pid_pit_yaw_auto, pid_param[4], 0.0f, 0.0f, POSITION, 5, 12000, 30);
	
	uint32_t gimbal_task_time = osKernelSysTick();
	
	for(;;)
	{
		if((gimbal_ctrl_info.gim_info == CONTROL_MODE) | (gimbal_ctrl_info.gim_info == AUTO_AIM_MODE))
		{
			#if GIMBAL_PID_TEST == 0
			pid_yaw_position.kp = pid_param[0];
			pid_yaw_speed.kp = pid_param[1];
			pid_pit_position.kp = pid_param[2];
			pid_pit_speed.kp = pid_param[3];
			pid_pit_yaw_auto.kp = pid_param[4];
			#endif
		}
		/****************************************保护模式************************************************/
		else if(gimbal_ctrl_info.gim_info == SILENCE_MODE)
		{
			#if GIMBAL_PID_TEST == 0
			pid_yaw_position.kp = 0.0f;
			pid_yaw_speed.kp = 0.0f;
			pid_pit_position.kp = 0.0f;
			pid_pit_speed.kp = 0.0f;
			pid_pit_yaw_auto.kp = 0.0f;
			#endif
		}else if(gimbal_ctrl_info.gim_info == RESET_MODE)
		{
			cali_param.reset_flag = 1;
			save_cali_data();
//			sys_reset();
		}
		/************************************************************************************************/
		mpu_get_data();
		
		imu_AHRS_update();
		/****************************************打符模式************************************************/
		if(gimbal_ctrl_info.gim_info == AUTO_GAIN_MODE)
		{
			MovingAverageForAuto();
			pit_ctrl.output = pid_cal(&pid_pit_speed,
										pit_status_info.gyro,
										pid_cal(&pid_pit_position,pit_status_info.angle,pit_ctrl.ex));
			
			yaw_ctrl.output = pid_cal(&pid_yaw_speed,
										yaw_status_info.gyro,
										pid_cal(&pid_pit_yaw_auto,yaw_status_info.angle,yaw_ctrl.ex));
		}
		/****************************************正常操控模式********************************************/
		else
		{
			if((gimbal_ctrl_info.gim_info == AUTO_AIM_MODE) || (gimbal_ctrl_info.gim_info == AUTO_AIM_MODE_NO))
			{
//				if(gimbal_ctrl_info.auto_aim_f == 0)
//				{
//					gimbal_ctrl_info.auto_aim_f = 1;
//					kalman_predict_auto_aim.xyz_output_data[0] = 0;
//					kalman_predict_auto_aim.xyz_output_data[1] = 0;
//					kalman_predict_auto_aim.xyz_output_data[2] = 0;
//				}
				
//				if(gimbal_ctrl_info.gim_info == AUTO_AIM_MODE)
//				{
//					kalman_predict_auto_aim.xyz_input_data[0] = gimbal_ctrl_info.pit_info;//x
//					kalman_predict_auto_aim.xyz_input_data[1] = gimbal_ctrl_info.yaw_info;//y
//					kalman_predict_auto_aim.xyz_input_data[2] = gimbal_ctrl_info.save_info;//z
//				}
//				else
//				{
//					kalman_predict_auto_aim.xyz_input_data[0] = kalman_predict_auto_aim.xyz_output_data[0];//x
//					kalman_predict_auto_aim.xyz_input_data[1] = kalman_predict_auto_aim.xyz_output_data[1];//y
//					kalman_predict_auto_aim.xyz_input_data[2] = kalman_predict_auto_aim.xyz_output_data[2];//z
//				}
				
//				memcpy(kalman_predict_auto_aim.xyz_last_data, kalman_predict_auto_aim.xyz_output_data, sizeof(kalman_predict_auto_aim.xyz_last_data));
//				
//				getPoint3f(atti.roll, atti.yaw, kalman_predict_auto_aim.xyz_input_data,kalman_predict_auto_aim.xyz_output_data);
//				
//				kalman_predict_auto_aim.xyz_increase_data[0] = kalman_predict_auto_aim.xyz_output_data[0] - kalman_predict_auto_aim.xyz_last_data[0];
//				kalman_predict_auto_aim.xyz_increase_data[1] = kalman_predict_auto_aim.xyz_output_data[1] - kalman_predict_auto_aim.xyz_last_data[1];
//				kalman_predict_auto_aim.xyz_increase_data[2] = kalman_predict_auto_aim.xyz_output_data[2] - kalman_predict_auto_aim.xyz_last_data[2];
//				
//				kalman_filter_calc(&kalman_filter_auto_aim, \
//										kalman_predict_auto_aim.xyz_output_data[0], \
//										kalman_predict_auto_aim.xyz_output_data[1], \
//										kalman_predict_auto_aim.xyz_output_data[2], \
//										kalman_predict_auto_aim.xyz_increase_data[0], \
//										kalman_predict_auto_aim.xyz_increase_data[1], \
//										kalman_predict_auto_aim.xyz_increase_data[2]);
//				
//				memcpy(kalman_predict_auto_aim.xyz_predict_data, kalman_filter_auto_aim.filtered_value, sizeof(kalman_predict_auto_aim.xyz_predict_data));
//				
//				Transform(kalman_predict_auto_aim.xyz_predict_data, BULLET_SPEED, &pit_ctrl.aim_temp, &yaw_ctrl.aim_temp);
//				
//				watch1 = atan2(kalman_predict_auto_aim.xyz_output_data[0] , kalman_predict_auto_aim.xyz_output_data[2]) * 180 / PI *100;
//				watch2 = yaw_ctrl.aim_temp * 100;
//				
//				pit_ctrl.ctrl = pit_ctrl.aim_temp * ANGLE_TO_ENCODER + PITCH_MID;
//				pit_ctrl.ctrl = ctrl_limit(pit_ctrl.ctrl, PITCH_UP, PITCH_DOWN);
//				
//				MovingAverageForPitch();
//				pit_ctrl.ex = pit_ctrl.ctrl;
//				
//				yaw_ctrl.ex = yaw_ctrl.aim_temp;
				
//				pit_ctrl.output = pid_cal(&pid_pit_speed,
//											pit_status_info.gyro,
//											pid_cal(&pid_pit_position,pit_status_info.angle,pit_ctrl.ex));
//											
//				yaw_ctrl.output = pid_cal(&pid_yaw_speed,
//											yaw_status_info.gyro,
//											pid_cal(&pid_yaw_position,yaw_status_info.angle,yaw_ctrl.ex));

				MovingAverageForPitch();
				yaw_status_info.gyro = imu.wz;
				yaw_status_info.angle = atti.yaw;
				
				pit_ctrl.aim_temp = gimbal_ctrl_info.pit_info * 0.01f;//x
				yaw_ctrl.aim_temp = gimbal_ctrl_info.yaw_info * 0.01f;//y
				
				if(gimbal_ctrl_info.gim_info == AUTO_AIM_MODE)
				{
					
					pit_ctrl.ex += pid_cal(&pid_pit_pit_aim, pit_ctrl.aim_temp, 0);
					pit_ctrl.ex = ctrl_limit(pit_ctrl.ex, PITCH_UP, PITCH_DOWN);
					pit_ctrl.ctrl = pit_ctrl.ex;
					
					yaw_ctrl.ex += pid_cal(&pid_pit_yaw_aim, yaw_ctrl.aim_temp, 0);
				}
				else
				{
					pit_ctrl.ctrl = ctrl_limit(pit_ctrl.ctrl, PITCH_UP, PITCH_DOWN);
				}

				yaw_ctrl.output = pid_cal(&pid_yaw_speed,
											yaw_status_info.gyro,
											pid_cal(&pid_yaw_position,yaw_status_info.angle,yaw_ctrl.ex));

				pit_ctrl.output = pid_cal(&pid_pit_speed,
											pit_status_info.gyro,
											pid_cal(&pid_pit_position,pit_status_info.angle,pit_ctrl.ex));
				
			}
			else
			{
				gimbal_ctrl_info.auto_aim_f = 0;
				
				pit_ctrl.ctrl += gimbal_ctrl_info.pit_info / 409600.0f;			
				pit_ctrl.ctrl = ctrl_limit(pit_ctrl.ctrl, PITCH_UP, PITCH_DOWN);
				
				MovingAverageForPitch();
				
				pit_ctrl.ex = pit_ctrl.ctrl;
				
				pit_ctrl.output = pid_cal(&pid_pit_speed,
											pit_status_info.gyro,
											pid_cal(&pid_pit_position,pit_status_info.angle,pit_ctrl.ex));
				
				/************************************************************************************************/
				yaw_ctrl.ctrl = gimbal_ctrl_info.yaw_info  / 8192.0f;
				angle_handle();
				
				yaw_ctrl.output = pid_cal(&pid_yaw_speed,
											yaw_status_info.gyro,
											pid_cal(&pid_yaw_position,yaw_status_info.angle,yaw_ctrl.ex));
			}
			
		}
		send_gimbal_data(yaw_ctrl.output, pit_ctrl.output, trig_out);
		
		osDelayUntil(&gimbal_task_time, GIMBAL_TASK_PERIOD);
	}
}

void angle_handle(void)
{
	yaw_status_info.gyro = imu.wz;
	yaw_status_info.angle = atti.yaw;
	
	yaw_ctrl.ex += yaw_ctrl.ctrl;
}

/**
  * @brief  Encoder Moving Average
  * @param  NULL
  * @retval None
  */
void MovingAverageForAuto(void)
{
	static uint8_t num	=	0;
	static uint8_t cnt;
	
	float Temp1 = 0,Temp2 = 0;
	Encoder_Pit_Auto_Buff[num] = (float)(GimbalMotor.Theta_Real[0] - PITCH_MID);
	Encoder_Yaw_Auto_Buff[num] = (float)(GimbalMotor.Theta_Real[1] - YAW_MID);
	num++;
	if(num	==	MovintAverageForPitchLength)
		num = 0;
	for(cnt	= 0; cnt < MovintAverageForPitchLength; cnt ++)
	{
		Temp1 += Encoder_Pit_Auto_Buff[cnt];
		Temp2 += Encoder_Yaw_Auto_Buff[cnt];
	}
	pit_status_info.angle = Temp1 / MovintAverageForPitchLength;
	yaw_status_info.angle = Temp2 / MovintAverageForYawLength;
	
	pit_status_info.gyro = imu.wx;
	yaw_status_info.gyro = imu.wz;
}

/**
  * @brief  Encoder Moving Average
  * @param  NULL
  * @retval None
  */
void MovingAverageForPitch(void)
{
	static uint8_t num	=	0;
	static uint8_t cnt;
	
	float Temp = 0;
	Encoder_Pitch_Buff[num] = (float)(GimbalMotor.Theta_Real[0] - PITCH_MID) / 8192.0f;
	num++;
	if(num	==	MovintAverageForPitchLength)
		num = 0;
	for(cnt	=	0;cnt <	MovintAverageForPitchLength;cnt++)
		Temp += Encoder_Pitch_Buff[cnt];
	pit_status_info.angle = Temp / MovintAverageForPitchLength;
	pit_status_info.gyro = imu.wx;
}

float ctrl_limit(float Input,float Min,float Max)
{
    if(Input>Max)
        return Max;
    else if(Input<Min)
        return Min;
    return Input;
}

/**
  * @brief  Reset MCU
  * @param  NULL
  * @retval None
  */
void sys_reset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

