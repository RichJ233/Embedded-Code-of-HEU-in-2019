#include "bsp_can.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
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
  static CanTxMsgTypeDef Tx2Message;
  static CanRxMsgTypeDef Rx2Message;

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
  
  can_filter.FilterNumber         = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);
    
  hcan1.pTxMsg = &Tx1Message;
  hcan1.pRxMsg = &Rx1Message;
  hcan2.pTxMsg = &Tx2Message;
  hcan2.pRxMsg = &Rx2Message;
}

void can_receive_start(void)
{
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
}

BLDC_Gimbal_Data GimbalMotor;
BLDC_Motor_Data	Motor_Data;

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	switch(_hcan->pRxMsg->StdId){
				 case CAN_PIT_MOTOR_ID:{
						GimbalMotor.Theta_Real[0] = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						GimbalMotor.Speed_Real[0] = _hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3];
						GimbalMotor.Current_Real[0] = _hcan->pRxMsg->Data[4]<<8 | _hcan->pRxMsg->Data[5];
					 
					 	#if PIT_8192==1
						if(GimbalMotor.Theta_Real[0] < 4000)
							GimbalMotor.Theta_Real[0] += 8191;
						#endif
					 
				};break;
				
				case CAN_YAW_MOTOR_ID:{
						uint16_t encoder_temp;
						encoder_temp = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						GimbalMotor.Speed_Real[1] = _hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3];
						GimbalMotor.Current_Real[1] = _hcan->pRxMsg->Data[4]<<8 | _hcan->pRxMsg->Data[5];
						#if YAW_MID > 4096	
							if(encoder_temp <= YAW_MID_HALF)
								encoder_temp += ENCODER_MAX;
							GimbalMotor.Theta_Real[1] = (int16_t)encoder_temp - 6114;
						#endif
						
						#if YAW_MID < 4096						
							if(encoder_temp >= YAW_MID_HALF)
								encoder_temp -= ENCODER_MAX;
							GimbalMotor.Theta_Real[1] = (int16_t)encoder_temp - YAW_MID;
						#endif
							
						#if YAW_MID == 4096						
							encoder_temp -= YAW_MID;
						#endif
							
				};break;
				
				case CAN_TRIGGER_MOTOR_ID:{
						Motor_Data.Theta_Real[4] = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						Motor_Data.Speed_Real[4] = (uint16_t)(_hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3]);
						Motor_Data.Current_Real[4] = _hcan->pRxMsg->Data[4]<<8 | _hcan->pRxMsg->Data[5];
				};break;   
				
				case CAN_3508_M1_ID:{
						Motor_Data.Theta_Real[0] = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						Motor_Data.Speed_Real[0] = (uint16_t)(_hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3]);
						Motor_Data.Current_Real[0] = (uint16_t)(_hcan->pRxMsg->Data[4]<<8 | _hcan->pRxMsg->Data[5]);
				};break;

				case CAN_3508_M2_ID:{
						Motor_Data.Theta_Real[1] = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						Motor_Data.Speed_Real[1] = (uint16_t)(_hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3]);
						Motor_Data.Current_Real[1] = _hcan->pRxMsg->Data[4]<<8 | _hcan->pRxMsg->Data[5];
				};break;
				
				case CAN_3508_M3_ID:{
						Motor_Data.Theta_Real[2] = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						Motor_Data.Speed_Real[2] = (uint16_t)(_hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3]);
						Motor_Data.Current_Real[2] = _hcan->pRxMsg->Data[4]<<8 | _hcan->pRxMsg->Data[5];
				};break;

				case CAN_3508_M4_ID:{
						Motor_Data.Theta_Real[3] = (uint16_t)(_hcan->pRxMsg->Data[0]<<8 | _hcan->pRxMsg->Data[1]);
						Motor_Data.Speed_Real[3] = (uint16_t)(_hcan->pRxMsg->Data[2]<<8 | _hcan->pRxMsg->Data[3]);
						Motor_Data.Current_Real[3] = _hcan->pRxMsg->Data[4]<<8 | _hcan->pRxMsg->Data[5];
				};break;				
		}
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
  __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
}


void send_gimbal_info(int16_t pit_info, int16_t yaw_info, uint8_t gim_info, uint8_t reload_info,int16_t save_info)
{
  hcan1.pTxMsg->StdId   = CAN_GIMBAL_INFO_ID;
  hcan1.pTxMsg->IDE     = CAN_ID_STD;
  hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
  hcan1.pTxMsg->DLC     = 8;
  hcan1.pTxMsg->Data[0] = pit_info >> 8;
  hcan1.pTxMsg->Data[1] = pit_info;
  hcan1.pTxMsg->Data[2] = yaw_info >> 8;
  hcan1.pTxMsg->Data[3] = yaw_info;
  hcan1.pTxMsg->Data[4] = gim_info;
  hcan1.pTxMsg->Data[5] = reload_info;
  hcan1.pTxMsg->Data[6] = save_info >> 8;
  hcan1.pTxMsg->Data[7] = save_info;
  HAL_CAN_Transmit(&hcan1, 10);
}

void send_shoot_info(int16_t friction_info, int16_t shoot_speed_info, uint8_t laser_info)
{
  hcan1.pTxMsg->StdId   = CAN_SHOOT_INFO_ID;
  hcan1.pTxMsg->IDE     = CAN_ID_STD;
  hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
  hcan1.pTxMsg->DLC     = 8;
  hcan1.pTxMsg->Data[0] = friction_info >> 8;
  hcan1.pTxMsg->Data[1] = friction_info;
  hcan1.pTxMsg->Data[2] = shoot_speed_info >> 8;
  hcan1.pTxMsg->Data[3] = shoot_speed_info;
  hcan1.pTxMsg->Data[4] = laser_info;
  HAL_CAN_Transmit(&hcan1, 10);
}

/**
  * @brief  send calculated current to motor
  * @param  3508 motor ESC id
  */
uint8_t send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  hcan2.pTxMsg->StdId   = 0x200;
  hcan2.pTxMsg->IDE     = CAN_ID_STD;
  hcan2.pTxMsg->RTR     = CAN_RTR_DATA;
  hcan2.pTxMsg->DLC     = 0x08;
  hcan2.pTxMsg->Data[0] = iq1 >> 8;
  hcan2.pTxMsg->Data[1] = iq1;
  hcan2.pTxMsg->Data[2] = iq2 >> 8;
  hcan2.pTxMsg->Data[3] = iq2;
  hcan2.pTxMsg->Data[4] = iq3 >> 8;
  hcan2.pTxMsg->Data[5] = iq3;
  hcan2.pTxMsg->Data[6] = iq4 >> 8;
  hcan2.pTxMsg->Data[7] = iq4;
  return HAL_CAN_Transmit(&hcan2, 10);
}



