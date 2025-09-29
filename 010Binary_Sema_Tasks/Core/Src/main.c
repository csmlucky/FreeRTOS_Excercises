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
#include "semphr.h"
#include "queue.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx.h"

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

BaseType_t status;
char usr_msg[250]={0};

TaskHandle_t xTaskHandleM=NULL;
TaskHandle_t xTaskHandleE=NULL;

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize both manager and employee task */
xSemaphoreHandle xWork;

/* this is the queue which manager uses to put the work ticket id */
xQueueHandle xWorkQueue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* The tasks to be created. */
static void vManagerTask( void *pvParameters );
static void vEmployeeTask( void *pvParameters );


//static void prvSetupHardware(void);
void printmsg(char *msg);
//static void prvSetupUart(void);
//void prvSetupGpio(void);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //Enable the Data watch
  DWT_CTRL_REG |= (1 << 0);

  //Enable Segger uart continuous record
  //SEGGER_UART_init(500000);

  //Enable Segger and call
  //SEGGER_SYSVIEW_Conf();
  //SEGGER_SYSVIEW_Start();

  sprintf(usr_msg,"Demo of Binary semaphore usage between 2 Tasks \r\n");
  printmsg(usr_msg);

  /* Before a semaphore is used it must be explicitly created.
    * In this example a binary semaphore is created . */
  vSemaphoreCreateBinary(xWork);

  /* The queue is created to hold a maximum of 1 Element. */
  xWorkQueue = xQueueCreate( 1, sizeof( unsigned int ) );

  /* Check the semaphore and queue was created successfully. */
  if( (xWork != NULL) && (xWorkQueue != NULL) ){

	  status = xTaskCreate(vManagerTask, "Manager_Task", 200, NULL, 3, &xTaskHandleM);
	  configASSERT(status == pdPASS);

	  status = xTaskCreate(vEmployeeTask, "Employee_Task", 200, NULL, 2, &xTaskHandleE);
	  configASSERT(status == pdPASS);

	  //start the freertos scheduler
	  vTaskStartScheduler();

  //if control comes here means there is not enough heap memory to launch the scheduler
  }
  sprintf(usr_msg,"Queue/Sema create failed.. \r\n");
  printmsg(usr_msg);

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|pa6_out_Pin|pa7_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin pa6_out_Pin pa7_out_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|pa6_out_Pin|pa7_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  //GPIO_InitTask.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitTask.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;

  //HAL_GPIO_Init(GPIOA, &GPIO_InitTask);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* The tasks to be created. */
static void vManagerTask( void *pvParameters ){

	unsigned int xWorkTicketId;
	portBASE_TYPE xStatus;

	/* The semaphore is created in the 'empty' state, meaning the semaphore must
	 first be given using the xSemaphoreGive() API function before it
	 can subsequently be taken (obtained) */
	xSemaphoreGive( xWork);

	for( ;; )
	{
		/* get a work ticket id(some random number) */
		xWorkTicketId = ( rand() & 0x1FF );

		/* Sends work ticket id to the work queue */
		xStatus = xQueueSend( xWorkQueue, &xWorkTicketId , portMAX_DELAY ); //Post an item on back of the queue

		if( xStatus != pdPASS )
		{
			sprintf(usr_msg,"Could not send to the queue.\r\n");
			printmsg(usr_msg);

		}else
		{
			/* Manager notifying the employee by "Giving" semaphore */
			xSemaphoreGive( xWork);
			/* after assigning the work , just yield the processor because nothing to do */
			taskYIELD();

		}
	}


}

/*********************************************************************************************************/
void EmployeeDoWork(unsigned char TicketId)
{
	/* implement the work according to TickedID */
	sprintf(usr_msg,"Employee task : Working on Ticked id : %d\r\n",TicketId);
	printmsg(usr_msg);
	vTaskDelay(TicketId);
}



/********************************************************************************************************/
static void vEmployeeTask( void *pvParameters ){

	unsigned char xWorkTicketId;
	portBASE_TYPE xStatus;
	/* As per most tasks, this task is implemented within an infinite loop. */
	for( ;; )
	{
		/* First Employee tries to take the semaphore, if it is available that means there is a task assigned by manager, otherwise employee task will be blocked */
		xSemaphoreTake( xWork, 0 );

		/* get the ticket id from the work queue */
		xStatus = xQueueReceive( xWorkQueue, &xWorkTicketId, 0 );

		if( xStatus == pdPASS )
		{
		  /* employee may decode the xWorkTicketId in this function to do the work*/
			EmployeeDoWork(xWorkTicketId);
		}
		else
		{
			/* We did not receive anything from the queue.  This must be an error as this task should only run when the manager assigns at least one work. */
			sprintf(usr_msg,"Employee task : Queue is empty , nothing to do.\r\n");
			printmsg(usr_msg);
		}
	}


}

/***************************************************************************************************************/
void printmsg(char *msg)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

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
