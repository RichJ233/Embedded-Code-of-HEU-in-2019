/**
  ******************************************************************************
  * @file    junru_libs.c
  * @author  
  * @version V0.0
  * @date    2016/03/25
  * @brief   
  * 
  ******************************************************************************
  * @file    RichJlib.c
  * @author  
  * @version V1.1
  * @date    2019/07/19
  * @brief   
  * 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "RichJlib.h"
/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

__weak void UART_IdleRxCallback(UART_HandleTypeDef *huart){}

/**
  * @brief  为串口开启没有中断的DMA传输，为了减少中断次数为其他中断空出资源。
  *         代替HAL库的函数(此处在main函数中调用)
  * @param  hdma: 指向DMA_HandleTypeDef结构体的指针，这个结构体包含了DMA流的配置信息.  
  * @retval HAL status
  */
HAL_StatusTypeDef RichJ_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  uint32_t *tmp;
  uint32_t tmp1 = 0;
  
  tmp1 = huart->gState;
  if((tmp1 == HAL_UART_STATE_READY) || (tmp1 == HAL_UART_STATE_BUSY_TX))
  {
    if((pData == NULL ) || (Size == 0)) 
    {
      return HAL_ERROR;
    }
    
    /* Process Locked */
    __HAL_LOCK(huart);
    
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    /* Check if a transmit process is ongoing or not */
    if(huart->gState == HAL_UART_STATE_BUSY_TX)
    {
      huart->gState = HAL_UART_STATE_BUSY_TX_RX;
    }
    else
    {
      huart->gState = HAL_UART_STATE_BUSY_RX;
    }
    
    /* Enable the DMA Stream */
    tmp = (uint32_t*)&pData;
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t*)tmp, Size);
    
    /* Enable the DMA transfer for the receiver request by setting the DMAR bit 
    in the UART CR3 register */
    huart->Instance->CR3 |= USART_CR3_DMAR;
    
    /* Process Unlocked */
    __HAL_UNLOCK(huart);
    
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY; 
  }
}


/**
  * @brief  通过非阻塞方式接收数据，数据长度有最大限度，但是在最大限度之内可以接受任意长度的数据 DMADMA
  * @param  huart: 指向UART_HandleTypeDef结构体的指针，该指针包含了UART的配置信息
  * @param  pData: 指向接受数据缓冲区的指针
  * @param  Size: 可接收数据的最大长度
  * @retval HAL status
  */
HAL_StatusTypeDef RichJ_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//开启串口空闲中断
	
	RichJ_UART_Receive_DMA(huart,pData,Size);//利用DMA接受数据

	return HAL_OK;
}


/**
  * @brief  通过非阻塞方式接收数据，数据长度有最大限度，但是在最大限度之内可以接受任意长度的数据 
  * @param  huart: 指向UART_HandleTypeDef结构体的指针，该指针包含了UART的配置信息
  * @param  Size: 可接收数据的最大长度
  * @retval None
  */
void RichJ_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size)
{
	uint32_t DMA_FLAGS;//根据串口的不同来选择清除不同的DMA标志位
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
		/*清除IDLE标志位*/
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);

    UART_IdleRxCallback(huart);
		//重启DMA
	  __HAL_DMA_DISABLE(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);
		__HAL_DMA_SET_COUNTER(huart->hdmarx,Size);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

/**
  * @brief  通过标志位的方式来解决陀螺收发数据掉帧无法接受完整数据找不到帧头的BUG
  * @param  huart: 指向UART_HandleTypeDef结构体的指针，该指针包含了UART的配置信息
  * @retval None
  */

uint8_t TempBuf =0;
uint8_t ReceiveBuf[11],ReceiveCount = 0;

void RichJ_Decode(UART_HandleTypeDef *huart, uint8_t *GyroRrx)
{
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
	{
		TempBuf = (uint16_t)(huart->Instance->DR & (uint16_t)0x00FFU);
		if((TempBuf == 0x55)&&(ReceiveCount == 0)){
			GyroRrx[ReceiveCount] = TempBuf;
			return;
		}
		if((ReceiveCount < 10)&&(GyroRrx[0] == 0x55))
		{
			ReceiveCount++; 
			GyroRrx[ReceiveCount] = TempBuf;
		}else {
			ReceiveCount = 0;
		}
		if(ReceiveCount == 10)
		{
			UART_IdleRxCallback(huart);
		}
	}
	
}


/**
  * @brief  Sends an amount of data in non blocking mode. 
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef RichJ_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	uint32_t *tmp;
	if((pData == NULL ) || (Size == 0))
	{
	  return HAL_ERROR;
	}

	huart->pTxBuffPtr = pData;
	huart->TxXferSize = Size;
	huart->TxXferCount = Size;

	huart->ErrorCode = HAL_UART_ERROR_NONE;
	huart->gState = HAL_UART_STATE_BUSY_TX;

	/* Enable the UART transmit DMA Stream */
	tmp = (uint32_t*)&pData;
	HAL_DMA_Start_IT(huart->hdmatx, *(uint32_t*)tmp, (uint32_t)&huart->Instance->DR, Size);

	/* Clear the TC flag in the SR register by writing 0 to it */
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);

	/* Enable the DMA transfer for transmit request by setting the DMAT bit
	   in the UART CR3 register */
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);

	return HAL_OK;
}



