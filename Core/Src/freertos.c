/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "i2c.h"
#include "pca9685/pca9685_wrapper.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for chessMove */
osThreadId_t chessMoveHandle;
const osThreadAttr_t chessMove_attributes = {
    .name = "chessMove",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for calcServoPos */
osThreadId_t calcServoPosHandle;
const osThreadAttr_t calcServoPos_attributes = {
    .name = "calcServoPos",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for moveServos */
osThreadId_t moveServosHandle;
const osThreadAttr_t moveServos_attributes = {
    .name = "moveServos",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal3,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void getChessMove(void *argument);
void getServoPos(void *argument);
void moveServo(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  PCA9685_Init();
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of chessMove */
  chessMoveHandle = osThreadNew(getChessMove, NULL, &chessMove_attributes);

  /* creation of calcServoPos */
  calcServoPosHandle = osThreadNew(getServoPos, NULL, &calcServoPos_attributes);

  /* creation of moveServos */
  moveServosHandle = osThreadNew(moveServo, NULL, &moveServos_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_getChessMove */
/**
 * @brief Function implementing the chessMove thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_getChessMove */
void getChessMove(void *argument)
{
  /* USER CODE BEGIN getChessMove */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END getChessMove */
}

/* USER CODE BEGIN Header_getServoPos */
/**
 * @brief Function implementing the calcServoPos thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_getServoPos */
void getServoPos(void *argument)
{
  /* USER CODE BEGIN getServoPos */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END getServoPos */
}

/* USER CODE BEGIN Header_moveServo */
/**
 * @brief Function implementing the moveServos thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_moveServo */
void moveServo(void *argument)
{
  /* USER CODE BEGIN moveServo */

  // initalize servo driver

  /* Infinite loop */
  for (;;)
  {
    PCA9685_SetServoAngle(0, 90.0f); 
    osDelay(1);
  }
  /* USER CODE END moveServo */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
