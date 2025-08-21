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
#include <string.h>
#include <stdbool.h>
#include "i2c.h"
#include "pca9685/pca9685_wrapper.h"
#include "adc.h"
#include "usart.h"
#include "dma.h"
#include "semphr.h"

/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 4096

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// Declare the DMA handle for USART2 TX
extern DMA_HandleTypeDef hdma_usart2_tx;
uint16_t adcBuff[ADC_BUF_LEN];
SemaphoreHandle_t adcSemaphore;
SemaphoreHandle_t uartSemaphore;
QueueHandle_t adcQueue;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name = "defaultTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityNormal,
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

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	adcSemaphore = xSemaphoreCreateBinary();
	uartSemaphore = xSemaphoreCreateBinary();
	configASSERT(adcSemaphore != NULL);
	configASSERT(uartSemaphore != NULL);
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	adcQueue = xQueueCreate(8, sizeof(uint16_t) * ADC_BUF_LEN);
	configASSERT(adcQueue != NULL);
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		osDelay(1000);
	}
	/* USER CODE END StartDefaultTask */
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
	for (;;)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t)adcBuff, ADC_BUF_LEN);

		if (xSemaphoreTake(adcSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
		{
			continue;
		}

		HAL_UART_Transmit(&huart2, (uint8_t *)adcBuff, ADC_BUF_LEN * sizeof(uint16_t), HAL_MAX_DELAY);

		if (xQueueSend(adcQueue, adcBuff, 0) != pdTRUE)
		{
			// Queue full — skip or log
			continue;
		}

		osDelay(1000);
	}
}
/* USER CODE END getServoPos */

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
	uint16_t receivedData[ADC_BUF_LEN];

	// initalize pca9685 driver
	PCA9685_Init();
	// StartIKTask();

	/* Infinite loop */
	for (;;)
	{

		// if (xQueueReceive(adcQueue, receivedData, portMAX_DELAY) != pdTRUE)
		// {
		// 	continue;
		// }

		// char msg[64];
		// int len = snprintf(msg, sizeof(msg),
		// 				   "ADC: %u %u %u %u\r\n",
		// 				   receivedData[0], receivedData[1],
		// 				   receivedData[2], receivedData[3]);
		// // create PID loop

		// Test for now

		PCA9685_SetServoAngle(0, 90.0f);
		osDelay(1000);
	}
	/* USER CODE END moveServo */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(adcSemaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/* USER CODE END Application */
