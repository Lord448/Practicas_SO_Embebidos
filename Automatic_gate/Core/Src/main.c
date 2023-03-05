#include "main.h"

#define USER_DEBUG //It doesn't send the MCU to sleep
#define PULL_UP //It configures the resistor mode: PULL_UP, PULL_DOWN, BOTH_EDGES
#define SAMPLES 35 //It configures the number of samples in the debouncing task
#define MotorVelocity 20 //Percentage value of the power of the motor
#define MaximumCCR 7199 //It establish the maximum counts of the capture compare register
#define FirstTwoPinsMask (uint32_t) 0x03 //Mask for reading two pins

typedef enum bool
{
	false,
	true
}bool;

enum Motor
{
	M_Open,
	M_Close,
	M_Stop
}Motor;

TIM_HandleTypeDef htim1; //Paused Cycle handler @ 1ms, Using ISR, No channels
TIM_HandleTypeDef htim2; //Motor PWM handler @ 10KHz, MaximumCCR = 7199, Channel 1, No ISR

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void Task_Gate(void);
void Task_ReadPins(void);
void Task_WriteMotor(void);
uint16_t Task_Debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#ifdef USER_DEBUG
volatile bool PauseFlag = true;
#endif

const uint32_t CCW = 0b01; //Counter Clockwise
const uint32_t CW = 0b10; //Clock Wise
bool Arrive_Open, Arrive_Close;
bool B_Open, B_Close;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  TIM2 -> CCR1 = (uint32_t) ((MotorVelocity * MaximumCCR) / 100); //Setting the velocity of the motor
  HAL_TIM_Base_Start_IT(&htim1);
  while (1)
  {
	  Task_ReadPins();
	  Task_Gate();
	  Task_WriteMotor();
#ifdef USER_DEBUG
	  while(PauseFlag);
	  PauseFlag = true;
#else
	  HAL_SuspendTick();
	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	  HAL_ResumeTick();
#endif
  }
}

//Tasks
void Task_Gate(void)
{
	enum Stages
	{
		No_Motion_Stage,
		Open_Stage,
		Close_Stage
	}static Stages = No_Motion_Stage;

	switch(Stages)
	{
		case No_Motion_Stage:
			Motor = M_Stop;
			if(B_Open && !Arrive_Open){
				Stages = Open_Stage;
			}
			else if	(B_Close && !Arrive_Close){
				Stages = Close_Stage;
			}
		break;
		case Open_Stage:
			Motor = M_Open;
			if(Arrive_Open){
				Stages = No_Motion_Stage;
			}
		break;
		case Close_Stage:
			Motor = M_Close;
			if(Arrive_Close){
				Stages = No_Motion_Stage;
			}
		break;
	}
}

void Task_ReadPins(void)
{
	enum Reading
	{
		LimitClose = 0b10,
		LimitOpen = 0b01
	}ReadPortA;

	ReadPortA = ((GPIOA -> IDR)>>3)&FirstTwoPinsMask;

	if(ReadPortA == LimitClose)
		Arrive_Close = true;
	else
		Arrive_Close = false;
	if(ReadPortA == LimitOpen)
		Arrive_Open = true;
	else
		Arrive_Open = true;

	if(Task_Debounce(Button_Close_GPIO_Port, Button_Close_Pin) == 1)
		B_Close = true;
	else
		B_Close = false;
	if(Task_Debounce(Button_Open_GPIO_Port, Button_Open_Pin) == 1)
		B_Open = true;
	else
		B_Open = false;
}

void Task_WriteMotor(void)
{
	switch(Motor)
	{
		case M_Open:
			GPIOA -> ODR = (CCW<<1)&FirstTwoPinsMask; //Writing CCW direction to the L293D
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		break;
		case M_Close:
			GPIOA -> ODR = (CW<<1)&FirstTwoPinsMask; //Writing CW direction to the L293D
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		break;
		case M_Stop:
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		break;
	}
}

/**
 * @brief This task debounce a terminal, it is designed to run at 1ms
 * @note  It requires the declaration of the "PULL_DOWN" or the "PULL_UP" symbol for detection edge
 * 		  and the "SAMPLES" symbol for the number of samples
 * @param GPIOx: Port of the terminal
 * @param GPIO_Pin: Pin that you want to debounce
 * @return uint16_t: 1 for activate, 0 for desactivate
 */
uint16_t Task_Debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	enum Stages
	{
		WaitingHigh,
		WaitingLow
	} static Stages = WaitingHigh;

	static uint16_t Lows = 0;
	static uint16_t Highs = 0;

	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 0)
	{
		Lows++;
		Highs = 0;
	}
	else
	{
		Lows = 0;
		Highs++;
	}

	switch(Stages)
	{
		case WaitingHigh:
			if(Highs == SAMPLES)
			{
				Stages = WaitingLow;
#if defined(PULL_UP) || defined(BOTH_EDGES)
				return 1;
#else
				return 0;
#endif
			}
			else
			{
				return 0;
			}
		break;
		case WaitingLow:
			if(Lows == SAMPLES)
			{
				Stages = WaitingHigh;
#if defined(PULL_DOWN) || defined(BOTH_EDGES)
				return 1;
#else
				return 0;
#endif
			}
			else
			{
				return 0;
			}
		break;
		default:
			return 0;
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#ifdef USER_DEBUG
	PauseFlag = false;
#endif
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 35999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L293D_Input1_Pin|L293D_Input2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : L293D_Input1_Pin L293D_Input2_Pin */
  GPIO_InitStruct.Pin = L293D_Input1_Pin|L293D_Input2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LimitSwitch_Close_Pin LimitSwitch_Open_Pin Button_Close_Pin Button_Open_Pin */
  GPIO_InitStruct.Pin = LimitSwitch_Close_Pin|LimitSwitch_Open_Pin|Button_Close_Pin|Button_Open_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
