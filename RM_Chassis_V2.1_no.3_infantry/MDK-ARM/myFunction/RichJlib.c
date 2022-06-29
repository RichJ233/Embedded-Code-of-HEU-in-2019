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
  * @brief  Ϊ���ڿ���û���жϵ�DMA���䣬Ϊ�˼����жϴ���Ϊ�����жϿճ���Դ��
  *         ����HAL��ĺ���(�˴���main�����е���)
  * @param  hdma: ָ��DMA_HandleTypeDef�ṹ���ָ�룬����ṹ�������DMA����������Ϣ.  
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
  * @brief  ͨ����������ʽ�������ݣ����ݳ���������޶ȣ�����������޶�֮�ڿ��Խ������ⳤ�ȵ����� DMADMA
  * @param  huart: ָ��UART_HandleTypeDef�ṹ���ָ�룬��ָ�������UART��������Ϣ
  * @param  pData: ָ��������ݻ�������ָ��
  * @param  Size: �ɽ������ݵ���󳤶�
  * @retval HAL status
  */
HAL_StatusTypeDef RichJ_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//�������ڿ����ж�
	
	RichJ_UART_Receive_DMA(huart,pData,Size);//����DMA��������

	return HAL_OK;
}


/**
  * @brief  ͨ����������ʽ�������ݣ����ݳ���������޶ȣ�����������޶�֮�ڿ��Խ������ⳤ�ȵ����� 
  * @param  huart: ָ��UART_HandleTypeDef�ṹ���ָ�룬��ָ�������UART��������Ϣ
  * @param  Size: �ɽ������ݵ���󳤶�
  * @retval None
  */
void RichJ_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size)
{
	uint32_t DMA_FLAGS;//���ݴ��ڵĲ�ͬ��ѡ�������ͬ��DMA��־λ
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
		/*���IDLE��־λ*/
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);

    UART_IdleRxCallback(huart);
		//����DMA
	  __HAL_DMA_DISABLE(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);
		__HAL_DMA_SET_COUNTER(huart->hdmarx,Size);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

/**
  * @brief  ͨ����־λ�ķ�ʽ����������շ����ݵ�֡�޷��������������Ҳ���֡ͷ��BUG
  * @param  huart: ָ��UART_HandleTypeDef�ṹ���ָ�룬��ָ�������UART��������Ϣ
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



