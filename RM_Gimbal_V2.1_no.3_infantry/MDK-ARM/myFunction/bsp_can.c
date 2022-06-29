#include "bsp_can.h"
#include "can.h"
#include "bsp_imu.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "RM_Define.h"

/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */
void can_device_init(void)
{
  //can1 &can2 use same filter config
  CAN_FilterConfTypeDef  can_filter;
  static CanTxMsgTypeDef Tx1Message;
  static CanRxMsgTypeDef Rx1Message;

  can_filter.FilterNumber         = 0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  can_filter.BankNumber           = 14;
  can_filter.FilterActivation     = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);
  
  hcan1.pTxMsg = &Tx1Message;
  hcan1.pRxMsg = &Rx1Message;
	
}

void can_receive_start(void)
{
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
}

BLDC_Gimbal_Data GimbalMotor;
BLDC_Motor_Data	TRIGGER_Motor_Data;

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	switch(_hcan->pRxMsg->StdId){
				 case CAN_PIT_MOTOR_ID:{
						GimbalMotor.Theta_Real[0] = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						GimbalMotor.Current_Real[0] = _hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3];
					 
					 #if PIT_8192==1
						if(GimbalMotor.Theta_Real[0] < 4000)
							GimbalMotor.Theta_Real[0] += 8191;
					 #endif
					 
				};break;
				
				case CAN_YAW_MOTOR_ID:{
						GimbalMotor.Theta_Real[1] = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						GimbalMotor.Current_Real[1] = _hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3];	
					
					 #if YAW_8192==1					
						if(GimbalMotor.Theta_Real[1] < 4000)
							GimbalMotor.Theta_Real[1] += 8192;
					 #endif
						
				};break;
				
				case CAN_TRIGGER_MOTOR_ID:{
						TRIGGER_Motor_Data.Theta_Real = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						TRIGGER_Motor_Data.Speed_Real = (uint16_t)(_hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3]);
						TRIGGER_Motor_Data.Current_Real = _hcan->pRxMsg->Data[4]<<8 | _hcan->pRxMsg->Data[5];
				};break;   
				
				case CAN_SHOOT_INFO_ID:{
						shoot_info.friction_CCR = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						shoot_info.shoot_speed = (uint16_t)(_hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3]);
						shoot_info.LASER_FLAG = (uint8_t)(_hcan->pRxMsg->Data[4]);
				};break;   
				
				case CAN_GIMBAL_INFO_ID:{
						gimbal_ctrl_info.pit_info = (int16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						gimbal_ctrl_info.yaw_info = (int16_t)(_hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3]);
						gimbal_ctrl_info.gim_info = (uint8_t)(_hcan->pRxMsg->Data[4]);
						gimbal_ctrl_info.reload_info = (uint8_t)(_hcan->pRxMsg->Data[5]);
						gimbal_ctrl_info.save_info = (int16_t)(_hcan->pRxMsg->Data[6]<<8 | _hcan->pRxMsg->Data[7]);
				};break;
		}
  
  __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_FMP0);

}

/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */
void send_gimbal_data(int16_t yaw, int16_t pit, int16_t trig)
{
  hcan1.pTxMsg->StdId   = 0x1ff;
  hcan1.pTxMsg->IDE     = CAN_ID_STD;
  hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
  hcan1.pTxMsg->DLC     = 8;
  hcan1.pTxMsg->Data[0] = pit >> 8;
  hcan1.pTxMsg->Data[1] = pit;
  hcan1.pTxMsg->Data[2] = yaw >> 8;
  hcan1.pTxMsg->Data[3] = yaw;
  hcan1.pTxMsg->Data[4] = trig >> 8;
  hcan1.pTxMsg->Data[5] = trig;
  hcan1.pTxMsg->Data[6] = 0;
  hcan1.pTxMsg->Data[7] = 0;
  HAL_CAN_Transmit(&hcan1, 10);
}

/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */
void send_gimbal_gyro(float yaw, float pit)
{
  int16_t yaw_atti = (int16_t)(yaw * 100);
  int16_t pit_atti = (int16_t)(pit * 100);
  hcan1.pTxMsg->StdId   = 0x100;
  hcan1.pTxMsg->IDE     = CAN_ID_STD;
  hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
  hcan1.pTxMsg->DLC     = 8;
  hcan1.pTxMsg->Data[0] = yaw_atti >> 8;
  hcan1.pTxMsg->Data[1] = yaw_atti;
  hcan1.pTxMsg->Data[2] = pit_atti >> 8;
  hcan1.pTxMsg->Data[3] = pit_atti;
  hcan1.pTxMsg->Data[4] = 0;
  hcan1.pTxMsg->Data[5] = 0;
  HAL_CAN_Transmit(&hcan1, 10);
}
