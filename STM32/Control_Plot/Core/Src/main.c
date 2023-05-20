/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "cmsis_os.h"
#include "task.h"
#include "FreeRTOS.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum bool
{
	false,
	true
}bool;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vTaskADC */
osThreadId_t vTaskADCHandle;
const osThreadAttr_t vTaskADC_attributes = {
  .name = "vTaskADC",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for vTaskUSB */
osThreadId_t vTaskUSBHandle;
const osThreadAttr_t vTaskUSB_attributes = {
  .name = "vTaskUSB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for vTaskControl */
osThreadId_t vTaskControlHandle;
const osThreadAttr_t vTaskControl_attributes = {
  .name = "vTaskControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for xFIFOAdcControl */
osMessageQueueId_t xFIFOAdcControlHandle;
const osMessageQueueAttr_t xFIFOAdcControl_attributes = {
  .name = "xFIFOAdcControl"
};
/* Definitions for xFIFOUsartToAdc */
osMessageQueueId_t xFIFOUsartToAdcHandle;
const osMessageQueueAttr_t xFIFOUsartToAdc_attributes = {
  .name = "xFIFOUsartToAdc"
};
/* Definitions for xFIFOAdcUart */
osMessageQueueId_t xFIFOAdcUartHandle;
const osMessageQueueAttr_t xFIFOAdcUart_attributes = {
  .name = "xFIFOAdcUart"
};
/* Definitions for xSemaphoreUARTSend */
osSemaphoreId_t xSemaphoreUARTSendHandle;
const osSemaphoreAttr_t xSemaphoreUARTSend_attributes = {
  .name = "xSemaphoreUARTSend"
};
/* Definitions for xSemaphoreUARTParameterChanged */
osSemaphoreId_t xSemaphoreUARTParameterChangedHandle;
const osSemaphoreAttr_t xSemaphoreUARTParameterChanged_attributes = {
  .name = "xSemaphoreUARTParameterChanged"
};
/* Definitions for xSemaphoreCDCReady */
osSemaphoreId_t xSemaphoreCDCReadyHandle;
const osSemaphoreAttr_t xSemaphoreCDCReady_attributes = {
  .name = "xSemaphoreCDCReady"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void StartTaskADC(void *argument);
void StartTaskUSB(void *argument);
void StartTaskControl(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of xSemaphoreUARTSend */
  xSemaphoreUARTSendHandle = osSemaphoreNew(1, 1, &xSemaphoreUARTSend_attributes);

  /* creation of xSemaphoreUARTParameterChanged */
  xSemaphoreUARTParameterChangedHandle = osSemaphoreNew(1, 1, &xSemaphoreUARTParameterChanged_attributes);

  /* creation of xSemaphoreCDCReady */
  xSemaphoreCDCReadyHandle = osSemaphoreNew(1, 1, &xSemaphoreCDCReady_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xFIFOAdcControl */
  xFIFOAdcControlHandle = osMessageQueueNew (16, sizeof(uint16_t), &xFIFOAdcControl_attributes);

  /* creation of xFIFOUsartToAdc */
  xFIFOUsartToAdcHandle = osMessageQueueNew (16, sizeof(uint16_t), &xFIFOUsartToAdc_attributes);

  /* creation of xFIFOAdcUart */
  xFIFOAdcUartHandle = osMessageQueueNew (16, sizeof(uint16_t), &xFIFOAdcUart_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of vTaskADC */
  vTaskADCHandle = osThreadNew(StartTaskADC, NULL, &vTaskADC_attributes);

  /* creation of vTaskUSB */
  vTaskUSBHandle = osThreadNew(StartTaskUSB, NULL, &vTaskUSB_attributes);

  /* creation of vTaskControl */
  vTaskControlHandle = osThreadNew(StartTaskControl, NULL, &vTaskControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskADC */
/**
* @brief Function implementing the vTaskADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskADC */
void StartTaskADC(void *argument)
{
  /* USER CODE BEGIN StartTaskADC */
  uint16_t ADC_Res;
  HAL_ADC_Start(&hadc1);
  for(;;)
  {
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
		ADC_Res = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	osMessageQueuePut(xFIFOAdcControlHandle, &ADC_Res, 0, pdMS_TO_TICKS(10)); //Send to the Control Task
	osMessageQueuePut(xFIFOAdcUartHandle, &ADC_Res, 0, pdMS_TO_TICKS(10)); //Send to the Comm Task
	osDelay(1);
  }
  /* USER CODE END StartTaskADC */
}

/* USER CODE BEGIN Header_StartTaskUSB */
/**
* @brief Function implementing the vTaskUSB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskUSB */
void StartTaskUSB(void *argument)
{
  /* USER CODE BEGIN StartTaskUSB */
	struct Parameters
	{
	uint16_t samples;
	uint16_t samplingTimeMS;
	}Parameters = {
		  .samples = 100,
		  .samplingTimeMS = 20
	};

	const char TAG[] = "ADC Ready";
	//TickType_t ticks = xTaskGetTickCount();
	uint16_t ADC_Res, index = 0;
	uint16_t numbers[Parameters.samples + 1];
	char Buffer[5];

	CDC_LinkSemaphore(xSemaphoreCDCReadyHandle);

	for(;;)
	{
	  if(osSemaphoreAcquire(xSemaphoreUARTParameterChangedHandle, pdMS_TO_TICKS(10)) == osOK)
	  {
		  //Search for changes on the parameters
	  }
	  else if(osMessageQueueGet(xFIFOAdcUartHandle, &ADC_Res, 0, pdMS_TO_TICKS(10) == osOK))
	  {
		  numbers[index] = ADC_Res;
		  index++;
		  if(index == Parameters.samples)
			  numbers[Parameters.samples + 1] = 0xFFFF; //Set a "flag" to send the plot data
		  else
			  numbers[Parameters.samples + 1] = 0; //Set a 0 to say that the data is not ready
	  }
	  else if(numbers[Parameters.samples + 1] == 0xFFFF)
	  {
		//Means that the data is ready to send
		CDC_Transmit_FS((uint8_t *) TAG, strlen(TAG));
		for(uint16_t i = 0; i < Parameters.samples; i++)
		{
			sprintf(Buffer, "%d", (int) numbers[i]);
			osSemaphoreAcquire(xSemaphoreCDCReadyHandle, 0);
			CDC_Transmit_FS((uint8_t *) Buffer, strlen(Buffer));
		}
		//Reset the array
		for(uint16_t i = 0; i < Parameters.samples+1; i++)
			numbers[i] = 0;
	  }
	  osDelay(1);
	}
  /* USER CODE END StartTaskUSB */
}

/* USER CODE BEGIN Header_StartTaskControl */
/**
* @brief Function implementing the vTaskControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskControl */
void StartTaskControl(void *argument)
{
  /* USER CODE BEGIN StartTaskControl */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(xSemaphoreUARTParameterChangedHandle, 0);
	  osDelay(0);
  }
  /* USER CODE END StartTaskControl */
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
