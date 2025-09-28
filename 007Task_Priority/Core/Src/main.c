/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DWT_CTRL_REG 		(*(volatile uint32_t *)0xE0001000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t volatile nextTaskToBeSuspended_Handle = NULL;
TaskHandle_t led_Handle, gpioA6_Handle, gpioA7_Handle, btn_Handle;
volatile BaseType_t btn_press_status = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void user_led_handler (void *param);
static void gpioA_pin6_handler (void *param);
//static void gpioA_pin7_handler(void *param);
//static void btn_handler(void *param);
static void switch_priority(void);

void btn_interrupt_handler(void);

extern  void SEGGER_UART_init(uint32_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	BaseType_t status;



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  //Enable the Data watch
  DWT_CTRL_REG |= (1 << 0);

  //Enable Segger uart continuous record
  SEGGER_UART_init(500000);

  //Enable Segger and call
  SEGGER_SYSVIEW_Conf();
  //SEGGER_SYSVIEW_Start();

  status = xTaskCreate(user_led_handler, "User_LED", 200, NULL, 3, &led_Handle);
  configASSERT(status == pdPASS);

  nextTaskToBeSuspended_Handle = led_Handle;

  status = xTaskCreate(gpioA_pin6_handler, "portA6_Toggle", 200, NULL, 2, &gpioA6_Handle);
  configASSERT(status == pdPASS);

  //status = xTaskCreate(gpioA_pin7_handler, "portA&_Toggle", 200, NULL, 1, &gpioA7_Handle);
  //configASSERT(status == pdPASS);

  //status = xTaskCreate(btn_handler, "Button Press", 200, NULL, 4, &btn_Handle);
  //configASSERT(status == pdPASS);

  //start the freertos scheduler
  vTaskStartScheduler();

  //if control comes here means there is not enough heap memory to launch the scheduler



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  //GPIO_InitTypeDef GPIO_InitTask = {0};
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|pa6_task_Pin|pa7_task_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin pa6_task_Pin pa7_task_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|pa6_task_Pin|pa7_task_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  //GPIO_InitTask.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitTask.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;

  //HAL_GPIO_Init(GPIOA, &GPIO_InitTask);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#if 0
void btn_interrupt_handler(void){
	 BaseType_t  pxHigherPriorityTaskWoken;
	 pxHigherPriorityTaskWoken = pdFALSE;

	traceISR_ENTER();
	xTaskNotifyFromISR(nextTaskToBeSuspended_Handle, 0, eNoAction, &pxHigherPriorityTaskWoken);

	/* once the isr exists this macro makes the higher priority task which got unblocked to resume on the cpu */
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	traceISR_EXIT();

}
#endif

static void switch_priority(void){

	UBaseType_t  led_priority, gpioA6_priority;
	BaseType_t switch_priority = 0;
	xTaskHandle current;

	/* access the shares variable */
	portENTER_CRITICAL();
	if(btn_press_status){
		btn_press_status = 0;
		switch_priority = 1;
	}
	portEXIT_CRITICAL();

	if(switch_priority){

		/* get priority of tasks */
		led_priority = uxTaskPriorityGet(led_Handle);
		gpioA6_priority = uxTaskPriorityGet(gpioA6_Handle);

		/* get current tak handle. useful to change current tak priority first
		 * as set priority triggers context switching */
		current = xTaskGetCurrentTaskHandle();

		/* switch priority */
		if(current == led_Handle){
			vTaskPrioritySet(led_Handle, gpioA6_priority);
			vTaskPrioritySet(gpioA6_Handle, led_priority);
		}else{
			vTaskPrioritySet(gpioA6_Handle, led_priority);
			vTaskPrioritySet(led_Handle, gpioA6_priority);
		}

	}


}

void btn_interrupt_handler(void){
	traceISR_ENTER();
	btn_press_status = 1;
	traceISR_EXIT();

}


void user_led_handler(void *param){
	//BaseType_t status;

	while(1){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		SEGGER_SYSVIEW_PrintfTarget("user_led_task");
		HAL_Delay(1000);
		switch_priority();
#if 0
		status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(1000));
		if(status == pdPASS){
			portENTER_CRITICAL();
			nextTaskToBeSuspended_Handle = gpioA6_Handle;
			portEXIT_CRITICAL();
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_SET);
			SEGGER_SYSVIEW_PrintfTarget("Delete user_led_task");
			vTaskDelete(NULL);
		}
#endif
	}


}

void gpioA_pin6_handler(void *param){
	//BaseType_t status;

	while(1){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		SEGGER_SYSVIEW_PrintfTarget("gpioa6_task");
		HAL_Delay(100);
		switch_priority();
#if 0
		status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(800));
		//taskYIELD();
		if(status == pdPASS){
			portENTER_CRITICAL();
			nextTaskToBeSuspended_Handle = gpioA7_Handle;
			portEXIT_CRITICAL();
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_SET);
			SEGGER_SYSVIEW_PrintfTarget("Delete gpioa6_task");
			vTaskDelete(NULL);
		}
#endif
	}

}

#if 0
void gpioA_pin7_handler(void *param){
	BaseType_t status;

	while(1){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
		SEGGER_SYSVIEW_PrintfTarget("gpioa7_task");
		status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(400));
		if(status == pdPASS){
			portENTER_CRITICAL();
			nextTaskToBeSuspended_Handle = NULL;
			portEXIT_CRITICAL();
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, GPIO_PIN_SET);
			vTaskDelete(btn_Handle);
			SEGGER_SYSVIEW_PrintfTarget("Delete gpioa6_task and button task");
			vTaskDelete(NULL);
		}

	}

}
#endif

#if 0
void btn_handler(void *param){

	static uint8_t prev_read = 1;
	uint8_t read = 1;
	while(1){

		read = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		if( !read){
			if(prev_read){
				xTaskNotify(nextTaskToBeDeleted_Handle, 0, eNoAction);
			}

		}
		prev_read = read;
		SEGGER_SYSVIEW_PrintfTarget("btn_task");
		vTaskDelay(pdMS_TO_TICKS(10));
		//taskYIELD();
	}

}
#endif
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
