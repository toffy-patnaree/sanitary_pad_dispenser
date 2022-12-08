/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define TRIG_PIN1 GPIO_PIN_10
#define TRIG_PORT1 GPIOB
#define ECHO_PIN1 GPIO_PIN_1
#define ECHO_PORT1 GPIOA

#define TRIG_PIN2 GPIO_PIN_4
#define TRIG_PORT2 GPIOB
#define ECHO_PIN2 GPIO_PIN_0
#define ECHO_PORT2 GPIOA

uint32_t pMillis1;
uint32_t Value11 = 0;
uint32_t Value21 = 0;
float Distance1  = 0;  // cm
char previous_Detected1 = 'f';
char current_Detected1 = 'f';
char servo1 = '0';

uint32_t pMillis2;
uint32_t Value12 = 0;
uint32_t Value22 = 0;
float Distance2  = 0;  // cm
char previous_Detected2 = 'f';
char current_Detected2 = 'f';
char servo2 = '0';


int button_input = 1;
int state = 0;
//int button_A = 1;
//int button_B = 0;

#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_8
uint8_t RHI11, RHD11, TCI11, TCD11, SUM11;
uint32_t pMillis11, cMillis11;
float tCelsius11 = 0;
float tFahrenheit11 = 0;
float RH11 = 0;

#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_9
uint8_t RH1, RH2, TC1, TC2, SUM22, CHECK22;
uint32_t pMillis22, cMillis22;
float tCelsius22 = 0;
float tFahrenheit22 = 0;
float RH22 = 0;

int x1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int y1=0;



void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT11_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
  HAL_Delay(20);   // wait for 20ms
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
  }
  pMillis11 = HAL_GetTick();
  cMillis11 = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis11 + 2 > cMillis11)
  {
    cMillis11 = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis11 = HAL_GetTick();
    cMillis11 = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis11 + 2 > cMillis11)
    {  // wait for the pin to go high
      cMillis11 = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis11 = HAL_GetTick();
    cMillis11 = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis11 + 2 > cMillis11)
    {  // wait for the pin to go low
      cMillis11 = HAL_GetTick();
    }
  }
  return b;
}

uint8_t DHT22_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT22_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
  microDelay (1300);   // wait for 1300us
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;
  }
  pMillis22 = HAL_GetTick();
  cMillis22 = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis22 + 2 > cMillis22)
  {
    cMillis22 = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT22_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis22 = HAL_GetTick();
    cMillis22 = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis22 + 2 > cMillis22)
    {  // wait for the pin to go high
      cMillis22 = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis22 = HAL_GetTick();
    cMillis22 = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis22 + 2 > cMillis22)
    {  // wait for the pin to go low
      cMillis22 = HAL_GetTick();
    }
  }
  return b;
}
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
  int switch1 = 0;
  int switch2 = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Humidity sensor
	  if(DHT11_Start()) {
		  RHI11 = DHT11_Read(); // Relative humidity integral
	  	  RHD11 = DHT11_Read(); // Relative humidity decimal
	  	  TCI11 = DHT11_Read(); // Celsius integral
	  	  TCD11 = DHT11_Read(); // Celsius decimal
	  	  SUM11 = DHT11_Read(); // Check sum
	  	  if (RHI11 + RHD11 + TCI11 + TCD11 == SUM11) {
	  		  // Can use RHI and TCI for any purposes if whole number only needed
	  	      tCelsius11 = (float)TCI11 + (float)(TCD11/10.0);
	  	      tFahrenheit11 = tCelsius11 * 9/5 + 32;
	  	      RH11 = (float)RHI11 + (float)(RHD11/10.0);
	  	      // Can use tCelsius, tFahrenheit and RH for any purposes
	  	  }
	  }

	  HAL_Delay(1000);

	  if(DHT22_Start())
	      {
	        RH1 = DHT22_Read(); // First 8bits of humidity
	        RH2 = DHT22_Read(); // Second 8bits of Relative humidity
	        TC1 = DHT22_Read(); // First 8bits of Celsius
	        TC2 = DHT22_Read(); // Second 8bits of Celsius
	        SUM22 = DHT22_Read(); // Check sum
	        CHECK22 = RH1 + RH2 + TC1 + TC2;
	        if (CHECK22 == SUM22)
	        {
	          if (TC1>127) // If TC1=10000000, negative temperature
	          {
	            tCelsius22 = (float)TC2/10*(-1);
	          }
	          else
	          {
	            tCelsius22 = (float)((TC1<<8)|TC2)/10;
	          }
	          tFahrenheit22 = tCelsius22 * 9/5 + 32;
	          RH22 = (float) ((RH1<<8)|RH2)/10;
	        }
	      }
	      HAL_Delay(1000);

	  //switch, stepper motor, ultrasonic sensor, servo motor
	  switch1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	  HAL_Delay(10);
	  if (switch1 == 1){

		  //LED1 start here
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		  HAL_Delay(1);

		  //Stepper1 start here
		  for(int i = 0; i<3200; i++){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			  for(int j=0;j<100;j++);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			  HAL_Delay(1);
		  }
		  HAL_Delay(1000);

		  while (current_Detected1 == 'f') {
		  //ultrasonic1 start here
			  HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_SET);  // pull the TRIG pin HIGH
			  __HAL_TIM_SET_COUNTER(&htim1, 0);
			  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
			  HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_RESET);  // pull the TRIG pin low

			  pMillis1 = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
			  // wait for the echo pin to go high
			  while (!(HAL_GPIO_ReadPin (ECHO_PORT1, ECHO_PIN1)) && pMillis1 + 10 >  HAL_GetTick());
			  Value11 = __HAL_TIM_GET_COUNTER (&htim1);

			  pMillis1 = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
			  // wait for the echo pin to go low
			  while ((HAL_GPIO_ReadPin (ECHO_PORT1, ECHO_PIN1)) && pMillis1 + 50 > HAL_GetTick());
			  Value21 = __HAL_TIM_GET_COUNTER (&htim1);

			  Distance1 = (Value21-Value11)* 0.034/2*1.65;
			  HAL_Delay(50);
			  if (Distance1<=10 && Distance1 > 3){
				  current_Detected1 = 't';
			  } else {
				  current_Detected1 = 'f';
			  }
		  }

		  //servo1 start here
		  while (current_Detected1 == 't') {
			  htim2.Instance->CCR1 = 875;
			  HAL_Delay(2000);
			  //check ultrasonic1
			  HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_SET);  // pull the TRIG pin HIGH
			  __HAL_TIM_SET_COUNTER(&htim1, 0);
			  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
			  HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_RESET);  // pull the TRIG pin low

			  pMillis1 = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
			  // wait for the echo pin to go high
			  while (!(HAL_GPIO_ReadPin (ECHO_PORT1, ECHO_PIN1)) && pMillis1 + 10 >  HAL_GetTick());
			  Value11 = __HAL_TIM_GET_COUNTER (&htim1);

			  pMillis1 = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
			  // wait for the echo pin to go low
			  while ((HAL_GPIO_ReadPin (ECHO_PORT1, ECHO_PIN1)) && pMillis1 + 50 > HAL_GetTick());
			  Value21 = __HAL_TIM_GET_COUNTER (&htim1);

			  Distance1 = (Value21-Value11)* 0.034/2*1.65;
			  HAL_Delay(50);

			  if (Distance1<=10 && Distance1 > 3){
			      current_Detected1 = 't';
			  } else {
			  	  current_Detected1 = 'f';
			  }

		  }
		  htim2.Instance->CCR1 = 500;
		  HAL_Delay(50);

		  //reset LED1
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

//		  if (previous_Detected1 == 't' && current_Detected1 == 'f'){
//		  	  servo1 = '0';
//		  	  previous_Detected1 = 'f';
//
//		  }
//		  else {
//			  if (current_Detected1 == 't'){
//				  servo1 = '1';
//			//		  y1+=1;
//				  }
//
//		  	  }

//		  previous_Detected1 = current_Detected1;

	  }

	  switch2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	  HAL_Delay(10);
	  if (switch2 == 1) {

		  //LED2 start here
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		  HAL_Delay(1);

		  //stepper2 start here
		  for(int i = 0; i<3200; i++){
		      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		 	  for(int j=0;j<100;j++);
		 	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		 	  HAL_Delay(1);
		  }
		  HAL_Delay(1000);

		  while(current_Detected2 == 'f'){
			  //ultrasonic 2 start here
			  HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_SET);  // pull the TRIG pin HIGH
			  __HAL_TIM_SET_COUNTER(&htim1, 0);
			  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
			  HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_RESET);  // pull the TRIG pin low

			  pMillis2 = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
			  // wait for the echo pin to go high
			  while (!(HAL_GPIO_ReadPin (ECHO_PORT2, ECHO_PIN2)) && pMillis2 + 10 >  HAL_GetTick());
			  Value12 = __HAL_TIM_GET_COUNTER (&htim1);

			  pMillis2 = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
			  // wait for the echo pin to go low
			  while ((HAL_GPIO_ReadPin (ECHO_PORT2, ECHO_PIN2)) && pMillis2 + 50 > HAL_GetTick());
			  Value22 = __HAL_TIM_GET_COUNTER (&htim1);

			  Distance2 = (Value22-Value12)* 0.034/2*1.65;
			  HAL_Delay(50);
			  if (Distance2<=10 && Distance2 > 3){
				  current_Detected2 = 't';
			  } else {
				  current_Detected2 = 'f';
			  }
		  }

		  //servo2 start here
		  while (current_Detected2 == 't') {
			  htim2.Instance->CCR2 = 875;
			  HAL_Delay(2000);
			  //check ultrasonic2
			  HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_SET);  // pull the TRIG pin HIGH
			  __HAL_TIM_SET_COUNTER(&htim1, 0);
			  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
			  HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_RESET);  // pull the TRIG pin low

			  pMillis2 = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
			  // wait for the echo pin to go high
			  while (!(HAL_GPIO_ReadPin (ECHO_PORT2, ECHO_PIN2)) && pMillis2 + 10 >  HAL_GetTick());
			  Value12 = __HAL_TIM_GET_COUNTER (&htim1);

			  pMillis2 = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
			  // wait for the echo pin to go low
			  while ((HAL_GPIO_ReadPin (ECHO_PORT2, ECHO_PIN2)) && pMillis2 + 50 > HAL_GetTick());
			  Value22 = __HAL_TIM_GET_COUNTER (&htim1);

			  Distance2 = (Value22-Value12)* 0.034/2*1.65;
			  HAL_Delay(50);

			  if (Distance2<=10 && Distance2 > 3){
		 		  current_Detected2 = 't';
		 	  } else {
		 		  current_Detected2 = 'f';
		 		}

		  }
		  htim2.Instance->CCR2 = 500;
		  HAL_Delay(500);

		  //reset LED2
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  htim2.Init.Prescaler = 163;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB4
                           PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
