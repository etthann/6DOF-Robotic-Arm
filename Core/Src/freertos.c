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
#include "ik/ik_solver.h"
#include "ik/arm_params.h"

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
float sign[6] = {+1, -1, +1, +1, +1, +1};
float offset[6] = {90, 90, 90, 90, 90, 90};
float min[6] = {0, 0, 0, 0, 0, 0};
float max[6] = {180, 180, 180, 180, 180, 180};

uint16_t adcBuff[ADC_BUF_LEN];
QueueHandle_t adcBlockPtrQ; // queue of pointers to filled halves
QueueHandle_t angleQ;
SemaphoreHandle_t i2c1Mutex;

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
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void uart_print(const char *msg);
float to_servo_deg(int j, float ik_deg);

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

	// initialize servo driver
	PCA9685_Init(50);
	osDelay(100);
	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	i2c1Mutex = xSemaphoreCreateMutex();
	configASSERT(i2c1Mutex);
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */

	// send pointers
	adcBlockPtrQ = xQueueCreate(4, sizeof(uint16_t *));
	angleQ = xQueueCreate(16, sizeof(float));
	configASSERT(adcBlockPtrQ && angleQ);
	/* USER CODE END RTOS_QUEUES */

	// start DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff, ADC_BUF_LEN);

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
		osDelay(1);
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
		uint16_t *data = NULL;

		// is there data in ADC buff pointer
		if (xQueueReceive(adcBlockPtrQ, &data, portMAX_DELAY) == pdTRUE)
		{
			const size_t halfBuffSize = ADC_BUF_LEN / 2;

			uint32_t acc = 0;
			size_t count = 0;

			// decimate data
			for (int i = 0; i < halfBuffSize; i += 8)
			{
				// sample every 8th value, add them all up to get rough avg
				acc += data[i];
				count++;
			}

			float avgCount = (float)acc / (float)count;

			static float y = 0.0f;
			const float alpha = 0.1f;

			// first-order IIR filter
			// y[n] = y[n-1] + alpha * (x[n] - y[n-1])
			y += alpha * (avgCount - y);

			// map counts -> angle (two-point calibration; fill these)
			// counts c1->θ1, c2->θ2  (e.g., (300, 0°), (3800, 180°))
			const float c1 = 300.0f, c2 = 3800.0f, th1 = 0.0f, th2 = 180.0f;
			const float m = (th2 - th1) / (c2 - c1);
			float angle_deg = m * (y - c1) + th1;

			// print angle to serial com
			char buff[64];
			static uint32_t lastTick = 0;

			// wait 500ms before reading angle again
			if (xTaskGetTickCount() - lastTick > pdMS_TO_TICKS(500))
			{
				lastTick = xTaskGetTickCount();
				snprintf(buff, sizeof(buff), "ADC AVG: %.2f\r\n", angle_deg);
				uart_print(buff);
			}

			// constrain angle
			if (angle_deg < 0)
			{
				angle_deg = 0;
			}

			if (angle_deg > 180)
			{
				angle_deg = 180;
			}

			xQueueSend(angleQ, &angle_deg, 0);
		}
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
	float jointDeg[6];

	float currentAngle = 0;
	float targetAngle = 0;

	Pose targetPose = {
		.x = 100.0f,   // 100mm in X direction
		.y = 50.0f,	   // 50mm in Y direction
		.z = 150.0f,   // 150mm in Z direction
		.yaw = 0.0f,   // 0 degrees yaw
		.pitch = 0.0f, // 0 degrees pitch
		.roll = 0.0f   // 0 degrees roll
	};

	char msg[50];

	/* Infinite loop */
	for (;;)
	{

		if (xQueueReceive(angleQ, &currentAngle, portMAX_DELAY) != pdTRUE)
		{
			continue;
		}

		// Calculate joint angles
		bool ikSuccess = ik6_spherical(&ArmDimensions, &targetPose, jointDeg, true); // elbow up

		if (!ikSuccess)
		{
			continue;
		}

		// for base angle
		targetAngle = jointDeg[0];
		sprintf(msg, "PID output = %.2f\r\n", targetAngle);

		// move servos
		for (int i = 0; i < 6; i++)
		{
			PCA9685_SetServoAngle(i, to_servo_deg(i, jointDeg[i]));
			osDelay(100);
		}

		osDelay(5000);
	}
	/* USER CODE END moveServo */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// don't need to call func this, stm32f4xx_it.c will call it
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		uint16_t *p = &adcBuff[ADC_BUF_LEN / 2];
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(adcBlockPtrQ, &p, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		uint16_t *p = &adcBuff[0];
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(adcBlockPtrQ, &p, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void uart_print(const char *msg)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

float to_servo_deg(int j, float ik_deg)
{
	float d = sign[j] * ik_deg + offset[j];
	if (d < min[j])
		d = min[j];
	if (d > max[j])
		d = max[j];
	return d;
}

/* USER CODE END Application */
