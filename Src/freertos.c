/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

#include "cmsis_os.h"
#include "main.h"
#include "portmacro.h"
#include "rtc.h"
#include "stm32f4xx_hal_rtc.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
osThreadId blink01Handle;
osThreadId blink02Handle;
osThreadId RtcDateHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBlink01(void const *argument);
void StartBlink02(void const *argument);
void StartRtcDate(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of blink01 */
  osThreadDef(blink01, StartBlink01, osPriorityNormal, 0, 128);
  blink01Handle = osThreadCreate(osThread(blink01), NULL);

  /* definition and creation of blink02 */
  osThreadDef(blink02, StartBlink02, osPriorityBelowNormal, 0, 128);
  blink02Handle = osThreadCreate(osThread(blink02), NULL);

  /* definition and creation of RtcDate */
  osThreadDef(RtcDate, StartRtcDate, osPriorityNormal, 0, 128);
  RtcDateHandle = osThreadCreate(osThread(RtcDate), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartBlink01 */
/**
 * @brief  Function implementing the blink01 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlink01 */
void StartBlink01(void const *argument) {
  /* USER CODE BEGIN StartBlink01 */
  /* Infinite loop */
  TickType_t now = xTaskGetTickCount();
  for (;;) {
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
    vTaskDelayUntil(&now, 100);
    printf("Blink01: %u\r\n", now);
  }
  /* USER CODE END StartBlink01 */
}

/* USER CODE BEGIN Header_StartBlink02 */
/**
 * @brief Function implementing the blink02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlink02 */
void StartBlink02(void const *argument) {
  /* USER CODE BEGIN StartBlink02 */
  /* Infinite loop */
  TickType_t now = xTaskGetTickCount();
  for (int i = 0; i < 10; i++) {
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
    vTaskDelayUntil(&now, 100);
    printf("Blink02: %u\r\n", now);
  }

  vTaskDelete(blink01Handle);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, 1);
  printf("Blink01 deleted!\r\n");

  for (;;) {
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
    vTaskDelayUntil(&now, 500);
    printf("Blink02: %u\r\n", now);
  }
  /* USER CODE END StartBlink02 */
}

/* USER CODE BEGIN Header_StartRtcDate */
/**
 * @brief Function implementing the RtcDate thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRtcDate */
void StartRtcDate(void const *argument) {
  /* USER CODE BEGIN StartRtcDate */
  /* Infinite loop */
  RTC_DateTypeDef rtc_date;
  RTC_TimeTypeDef rtc_time;
  TickType_t now = xTaskGetTickCount();
  for (;;) {
    HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
    printf("%d-%02d-%02d ", rtc_date.Year + 2000, rtc_date.Month,
           rtc_date.Date);
    printf("%02d:%02d:%02d\r\n", rtc_time.Hours, rtc_time.Minutes,
           rtc_time.Seconds);
    vTaskDelayUntil(&now, 1000);
  }
  /* USER CODE END StartRtcDate */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
