/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <string.h>
//#include "MY_NRF24.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN   -1000.0f
#define PID_LIM_MAX   1000.0f

#define PID_LIM_MIN_T   0.0f
#define PID_LIM_MAX_T   1000.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PID;

uint8_t steer_package[2] = {0};
uint16_t motion_flag = 0;
uint16_t velocity_cmd = 0;
uint8_t crc_package[2] = {0};
volatile uint8_t  UART1_rxBuffer[12]  = {0};
volatile uint16_t adc_buffer[3] = {0};
uint16_t steer_cmd = 500;
uint16_t brake_cmd = 0;
uint16_t brake_medium_cmd = 625;

float pwm = 0;
float pwm_throttle =0;
uint16_t pwm_throttle_m =0;
uint16_t pwm_m = 0;
uint16_t pwm_m_brake = 0;
uint16_t current_pos = 0;
uint16_t current_brake_pos = 0;
uint8_t direction = 0;
uint16_t adc_val = 0;
uint16_t adc_thr = 0;
uint16_t adc_brake = 0;
uint8_t gear = 1; //1 ileri 0 geri
uint8_t brake = 0; //0 fren yok 1 var
uint16_t current_velocity = 0;

uint8_t flag_start = 0;
uint8_t flag_start_btn = 0;

GPIO_PinState state;
GPIO_PinState state1;
GPIO_PinState start;
GPIO_PinState start_button;
GPIO_PinState gear_manuel;
uint8_t rxdata[4] = {0};
uint8_t emergency_stop = 0;
uint8_t counter = 0;
uint16_t throttle = 0;
uint16_t forward_vel = 0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void drive_right(uint16_t pwm);
void drive_left(uint16_t pwm);
void brake_push(uint16_t pwm);
void brake_pull(uint16_t pwm);
long map(long x, long in_min, long in_max, long out_min, long out_max);
float PID_update(PID *pid, float setpoint, float measurement);
void PID_init(PID *pid);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
uint64_t RxpipeAddrs = 0x11223344AA;
char myRxData[2];
char myAckPayload[32] = "Ack by STMF7!";
char check[2] = {'G','S'};

*/

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */



  PID steer_control = { PID_KP, PID_KI, PID_KD, PID_TAU,PID_LIM_MIN, PID_LIM_MAX,PID_LIM_MIN_INT, PID_LIM_MAX_INT,SAMPLE_TIME_S };

  PID brake_control = { 2.0f, PID_KI, 0.25f, PID_TAU,PID_LIM_MIN, PID_LIM_MAX,PID_LIM_MIN_INT, PID_LIM_MAX_INT,SAMPLE_TIME_S };

  PID throttle_control = { PID_KP, PID_KI, PID_KD, PID_TAU,PID_LIM_MIN_T, PID_LIM_MAX_T,PID_LIM_MIN_INT, PID_LIM_MAX_INT,SAMPLE_TIME_S };

  PID_init(&steer_control);
  PID_init(&brake_control);
  PID_init(&throttle_control);

  //HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, 3);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
/* kablosuz haberlesme */

  /*  NRF24_begin(GPIOD, CSN_Pin, CE_Pin, hspi1);
	nrf24_DebugUART_Init(huart2);

	printRadioSettings();

	NRF24_setAutoAck(true);
	NRF24_setChannel(79);
	NRF24_setPayloadSize(32);
	NRF24_openReadingPipe(1, RxpipeAddrs);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();

	NRF24_startListening();
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  if (counter == 0){

  		  counter = counter + 1;
  		  TIM1 -> CCR3 = 2290;
  		  HAL_Delay(6000);

  	  }


  HAL_UART_Receive_DMA (&huart1, UART1_rxBuffer, 12);
  //HAL_UART_Receive_IT(&huart2, rxdata, 4);

  ADC_ChannelConfTypeDef sConfig = {0};
  start = GPIO_PIN_RESET;

  while (1)
  {

	  //ADC den lineer potun okunuşu ve maplenişi
	  HAL_ADC_Start_IT(&hadc1);
	  HAL_ADC_Start_IT(&hadc2);
	  state1 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
	  //state1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
	  //state = GPIO_PIN_RESET;
	  //state1 = GPIO_PIN_SET;
	  //HAL_UART_Transmit(&huart1, (uint16_t *)current_velocity, 2, 10); // alınan encoder verisi burada kontrol bilgisayarına iletiliyor.
//	  start_button = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
//	  if( start_button == GPIO_PIN_RESET ){
//	 		  flag_start_btn = 1;
//	 		  if(flag_start == 0){
//	 		  start = GPIO_PIN_SET;}
//	 		  else {
//	 			  start = GPIO_PIN_RESET;
//	 		  }
//	 	  }
//	 	  else
//	 	  {
//	 		  if (flag_start_btn == 1)
//	 		  {
//	 			  if(flag_start == 0){ flag_start = 1;}
//	 			  else {flag_start = 0;}
//	 			  flag_start_btn = 0;
//	 		  }
//	 	  }
//	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,start);

	  current_pos = map(adc_val,495,3350,0,1000);
	  current_brake_pos = map(adc_brake,442,4096,0,1000);

	  //pwm update işleminin yapılması ve motorların buna göre sürülmesi

	  //kablosuzdan gelen dur ve git komutları
	  /*if(NRF24_available())
	  		{
	  			NRF24_read(myRxData, 32);
	  			NRF24_writeAckPayload(1, myAckPayload, 32);
	  			myRxData[32] = '\r';
	  			myRxData[32+1] = '\n';
	  			HAL_UART_Transmit(&huart2, (uint8_t *)myRxData, 32+2, 10);


	  			if(myRxData[0]==check[0]){
	  				uint8_t emergency_stop = 0;
	  			}
	  			else if(myRxData[0]==check[1]){
	  			    emergency_stop = 1;
	  				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//frene bas
	  				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);

	  				TIM1->CCR3 = 0;//gazı kes
	  			    TIM2->CCR1 = 8399;
	  				TIM2->CCR2 = 0;
	  				emergency_stop = 1;
	  				velocity_cmd = 0;
	  			}

	  		}*/

	  if(state1)
	  {
		  if(velocity_cmd == 0){

			  motion_flag = 0;
		  		  throttle = 2250;
		  		  TIM1->CCR3 = throttle;
		  		  //TIM2->CCR4 = 8399;

		  		  //buraya throttle için kullanacağın pwm i sıfırla aynı anda gaz fren yapılmaz :D
		  		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);
		  		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);// brake aktüatörüne bağlı mosfeti aktif hale getirerek frene basılmasını sağla
		  		  TIM2->CCR4 = 0;
		  		  TIM2->CCR2 = 8399;
		  	  }


		  //HAL_UART_Receive_DMA (&huart1, UART1_rxBuffer, 12);
		  sConfig.Channel = ADC_CHANNEL_4;
		  		  	  sConfig.Rank = 1;
		  		  	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		  		  	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
		  		  	  {
		  		  	    Error_Handler();
		  		  	  }

	  pwm = PID_update(&steer_control, steer_cmd, current_pos);

	  pwm_m = abs(pwm);

	  pwm_m = map(pwm_m,0,1000,0,8399);

	    	  if(pwm > 0){

	    		  drive_right(pwm_m);

	    	  }
	    	  else{

	    		  drive_left(pwm_m);

	    	  }




//	  pwm = PID_update(&brake_control, brake_cmd, current_brake_pos);
//
//	  pwm_m_brake = abs(pwm);
//
//	  pwm_m_brake = map(pwm_m_brake,0,1000,0,8399);
//
//	    	  if(pwm > 0){
//
//	    		  brake_push(pwm_m_brake);
//
//	    	  }
//	    	  else{
//
//	    		  brake_pull(pwm_m_brake);
//
//	    	  }


  	  forward_vel = parse_velocity_cmd(velocity_cmd);


	  }
	  if((emergency_stop!=1) & (state1 == 1) & velocity_cmd != 0){

		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);// brake i deaktive et
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);
		  //TIM2->CCR1 = 0;
		  //TIM2->CCR2 = 8399;
		  if(motion_flag < 48000)
		  {
		  	  brake_cmd = 0;
		  	  motion_flag += 1;
		  }
		  else{
			  brake_cmd = brake_medium_cmd;
		  }


		  	  pwm = PID_update(&brake_control, brake_cmd, current_brake_pos);

		  	  pwm_m_brake = abs(pwm);

		  	  pwm_m_brake = map(pwm_m_brake,0,1000,0,8399);

		  	    	  if(pwm > 0){

		  	    		  brake_push(pwm_m_brake);

		  	    	  }
		  	    	  else{

		  	    		  brake_pull(pwm_m_brake);

		  	    	  }

		  throttle = map(forward_vel, 13, 57, 3700, 8399);
		  //TIM1 -> CCR3 = 2290;
		  //HAL_Delay(3000);
		  TIM1->CCR3 = throttle;
		  //TIM2->CCR1 = 2000;
		  //TIM2->CCR2 = 0;



	  	  }
	  if(state1== 0){

		    TIM1->CCR2 = 0;
		  	TIM1->CCR1 = 0;


		  	brake_cmd = 200;
		  	pwm = PID_update(&brake_control, brake_cmd, current_brake_pos);

		  		  pwm_m_brake = abs(pwm);

		  		  pwm_m_brake = map(pwm_m_brake,0,1000,0,8399);

		  		    	  if(pwm > 0){

		  		    		  brake_push(pwm_m_brake);

		  		    	  }
		  		    	  else{

		  		    		  brake_pull(pwm_m_brake);

		  		    	  }

		  	if(current_brake_pos < 500){

		  	  sConfig.Channel = ADC_CHANNEL_11;
		  	  sConfig.Rank = 1;
		  	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		  	}
		  	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
		  	  {
		  	    Error_Handler();
		  	  }


		  throttle=2240;

		  /*HAL_ADC_Start(&hadc3);
		  if(HAL_ADC_PollForConversion(&hadc3, 100000) == HAL_OK)
		  {
			  adc_thr = HAL_ADC_GetValue(&hadc3);
		  }
		  HAL_ADC_Stop(&hadc3);*/
		  gear_manuel = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7);

		  if(gear_manuel == GPIO_PIN_SET){

			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_RESET);
		  }
			  else{
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_SET);
			  }






		  pwm_m = 0;
		  throttle = map(adc_thr,0,4096,0,8399);
		  //throttle = 2250;

		  TIM1->CCR3 = throttle;
		  //TIM2->CCR1 = 8399;
		  //TIM2->CCR2 = 0;
		 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);// brake i deaktive et
		 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);

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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_13;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(start_output_GPIO_Port, start_output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : start_input_Pin */
  GPIO_InitStruct.Pin = start_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(start_input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : start_output_Pin */
  GPIO_InitStruct.Pin = start_output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(start_output_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void drive_right(uint16_t pwm){

	/*HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);//sağa sürmek için gerekli yünün enable ını high olan pine bağla*/
	TIM1->CCR2 = 0;
	TIM1->CCR1 = pwm;


	//gerekli yönün pwm pinine bağla

}

void drive_left(uint16_t pwm){

	/*HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);*/
	TIM1->CCR1=0;
	TIM1->CCR2 = pwm;


	//gerekli yönün pwm pinine bağla
}

void brake_push(uint16_t pwm){


	TIM2->CCR4 = 0;
	TIM2->CCR2 = pwm;


	//gerekli yönün pwm pinine bağla

}

void brake_pull(uint16_t pwm){


	TIM2->CCR2 = 0;
	TIM2->CCR4 = pwm;


	//gerekli yönün pwm pinine bağla
}


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  current_pos = adc_buffer[1];
}*/

float PID_update(PID *pid, float setpoint, float measurement){

	float error = setpoint - measurement;


		/*
		* Proportional
		*/
	    float proportional = pid->Kp * error;


		/*
		* Integral
		*/
	    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

		/* Anti-wind-up via integrator clamping */
	    if (pid->integrator > pid->limMaxInt) {

	        pid->integrator = pid->limMaxInt;

	    } else if (pid->integrator < pid->limMinInt) {

	        pid->integrator = pid->limMinInt;

	    }


		/*
		* Derivative (band-limited differentiator)
		*/

	    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
	                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
	                        / (2.0f * pid->tau + pid->T);


		/*
		* Compute output and apply limits
		*/
	    pid->out = proportional + pid->integrator + pid->differentiator;

	    if (pid->out > pid->limMax) {

	        pid->out = pid->limMax;

	    } else if (pid->out < pid->limMin) {

	        pid->out = pid->limMin;

	    }

		/* Store error and measurement for later use */
	    pid->prevError       = error;
	    pid->prevMeasurement = measurement;

		/* Return controller output */
	    return pid->out;

}

void PID_init(PID *pid){


	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;



}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{


	if(state1 == 1)
	{

		adc_brake = HAL_ADC_GetValue(&hadc1);
		if(adc_brake <= 442){
			adc_brake = 442;
		}
		else if (adc_brake >3714){
			adc_brake = 3714;
		}

		adc_val = HAL_ADC_GetValue(&hadc2);


	if (adc_val < 495){
		adc_val = 495;
	}
	else if (adc_val > 3350)
	{
		adc_val = 3350;
	}
	}
	else{
		adc_thr = HAL_ADC_GetValue(&hadc2);
	}
	//HAL_ADC_Start_IT(&hadc2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	for(int i=0;i<12;i++){

		if((rxdata[0]==10) & (rxdata[1]==20) & (rxdata[2]==30) & (rxdata[3]==40)){

			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//frene bas
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);

			TIM1->CCR3 = 0;//gazı kes
			TIM2->CCR4 = 0;
			TIM2->CCR2 = 8399;

			emergency_stop = 1;
			velocity_cmd = 0;

		}

		if((UART1_rxBuffer[i] == 77) & (12-i > 5) & (emergency_stop!=1) & (state1 == 1)){

						steer_package[0] = UART1_rxBuffer[i+1];
						steer_package[1] = UART1_rxBuffer[i+2];
						velocity_cmd = UART1_rxBuffer[i+3];
						//direction = UART1_rxBuffer[i+4];
						brake_medium_cmd = 5*UART1_rxBuffer[i+4];
						crc_package[1] = UART1_rxBuffer[i+5];

						steer_cmd = steer_package[0]<<8|steer_package[1];

						//memset(UART1_rxBuffer,0,sizeof(UART1_rxBuffer));
						if(brake_medium_cmd > 1000) { brake_medium_cmd = 1000; }

						return;
					}
					else{

						//velocity_cmd = 0;

					}

		}

	//HAL_UART_Receive_DMA (&huart1, UART1_rxBuffer, 12);
}

int parse_velocity_cmd(velocity_cmd){ // bu fonksiyon vitesi ayarlamak için

	if(velocity_cmd > 20){

		velocity_cmd = velocity_cmd - 20;					  //0-20 arası ileri 20-30 arası ise geridir.

											//gelen komut bu şekilde parse edilerek gidilmek istenen yön ayarlanır

		if(state1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_RESET);    //vitesin anahtarlamasını 0 a çekerek aracın vitesini geriye alırız
	}
	}
	else{
		//eğer velocity_cmd 0 ise fren köklenecek yoksa pid algoritması ile sürüşe devam edilecek
		if(state1){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_SET);
		}  //0-20 arasına denk geldi ise ileri olacağından aracın vites mosfeti de-aktifleştirilerek 12v lar virleşir ve vites ileri alınır.
		if (velocity_cmd > 20){
					velocity_cmd = 20;
				}
	}
	return velocity_cmd;
}

void accelerate(pwm_m_throttle){//bu fonksiyon gaza basılarak arabanın hızlanmasını sağlar

	TIM1->CCR3 = pwm_m_throttle;

}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
