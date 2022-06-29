/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "chassis_task.h"
#include "detect_task.h"
#include "super_caps_task.h"
#include "gimbal_task.h"
#include "judgement_task.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId chasTaskHandle;
osThreadId capsTaskHandle;
osThreadId gimTaskHandle;
osThreadId detTaskHandle;
osThreadId judgTaskHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void ChassisTask1(void const * argument);
void SuperCapsTask1(void const * argument);
void GimbalTask1(void const * argument);
void DetectTask1(void const * argument);
void JudgmentTask1(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of chasTask */
  osThreadDef(chasTask, ChassisTask1, osPriorityNormal, 0, 256);
  chasTaskHandle = osThreadCreate(osThread(chasTask), NULL);

  /* definition and creation of capsTask */
  osThreadDef(capsTask, SuperCapsTask1, osPriorityNormal, 0, 128);
  capsTaskHandle = osThreadCreate(osThread(capsTask), NULL);

  /* definition and creation of gimTask */
  osThreadDef(gimTask, GimbalTask1, osPriorityNormal, 0, 128);
  gimTaskHandle = osThreadCreate(osThread(gimTask), NULL);

  /* definition and creation of detTask */
  osThreadDef(detTask, DetectTask1, osPriorityIdle, 0, 128);
  detTaskHandle = osThreadCreate(osThread(detTask), NULL);

  /* definition and creation of judgTask */
  osThreadDef(judgTask, JudgmentTask1, osPriorityNormal, 0, 128);
  judgTaskHandle = osThreadCreate(osThread(judgTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* ChassisTask1 function */
void ChassisTask1(void const * argument)
{
  /* USER CODE BEGIN ChassisTask1 */
  /* Infinite loop */
  for(;;)
  {
		ChassisTask();
  }
  /* USER CODE END ChassisTask1 */
}

/* SuperCapsTask1 function */
void SuperCapsTask1(void const * argument)
{
  /* USER CODE BEGIN SuperCapsTask1 */
  /* Infinite loop */
  for(;;)
  {
		caps_task();
  }
  /* USER CODE END SuperCapsTask1 */
}

/* GimbalTask1 function */
void GimbalTask1(void const * argument)
{
  /* USER CODE BEGIN GimbalTask1 */
  /* Infinite loop */
  for(;;)
  {
		GimbalTask();
  }
  /* USER CODE END GimbalTask1 */
}

/* DetectTask1 function */
void DetectTask1(void const * argument)
{
  /* USER CODE BEGIN DetectTask1 */
  /* Infinite loop */
  for(;;)
  {
		detect_Task();
  }
  /* USER CODE END DetectTask1 */
}

/* JudgmentTask1 function */
void JudgmentTask1(void const * argument)
{
  /* USER CODE BEGIN JudgmentTask1 */
  /* Infinite loop */
  for(;;)
  {
		judgement_unpack();
  }
  /* USER CODE END JudgmentTask1 */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
