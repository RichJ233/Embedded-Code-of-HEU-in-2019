#ifndef __RICHJ_LIBS_H_
#define __RICHJ_LIBS_H_

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef RichJ_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef RichJ_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef RichJ_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void RichJ_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size);
void RichJ_Decode(UART_HandleTypeDef *huart, uint8_t *GyroRrx);

#endif

