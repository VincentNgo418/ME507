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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include <string.h>   // for strlen, strcmp, etc.
#include <stdio.h>    // for sprintf
#include <stdlib.h>

#ifdef __cplusplus
}
#endif

#include <ctype.h>
#include "fsm.h"

#ifdef __cplusplus
  FSM fsm;
#endif






#include "../Drivers/BNO055/bno055.h"
#include "../Drivers/BNO055/bno055_hal.h"

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
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t tx_buf[64];
uint8_t rx_buf[64];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static void initialize_IMU(void);
static void log_IMU(void);
static void log_LIDAR(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char log_buf[50] = {0};
struct bno055_t IMU;

//BNO055 initialize readout variables
struct bno055_accel_t accel_data;
struct bno055_euler_t euler_data;
HAL_I2C_StateTypeDef state;
uint8_t op_mode;
uint8_t pow_mode;
uint8_t sys_stat;

//TFLuna initialize readout variables
uint8_t distance_data[2];
uint16_t distance_cm;
uint8_t intensity_data[2];
uint16_t intensity_value;
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
  MX_TIM3_Init();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();

  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */


  // SERVO MOTOR
  HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_4);


  // BASE MOTOR
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);

  // POLOLU 2
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);


  // LAUNCHER MOTOR
  HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_ALL);


  // POLOLU 1
  HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_4);

  // FLYWHEEL
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2500);

  //set BNO055 reset to low
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  //initialize BNO055
  BNO055_setup(&IMU);

  initialize_IMU();
  HAL_UART_Receive_IT(&huart1, rx_buf, 1);

  int16_t last_count1 = 0;
  int16_t last_count2 = 0;



  //set_duty_dual(&Pololu_2, 0, 2500);
  motor_d_set_pos(&Pololu_2, &pos_controller_1, 1000);

  sprintf((char*)log_buf, "Motor Pos: %d Motor goal: %d \r\n", motor_d_get_pos(&Pololu_2), pos_controller_1.setpoint);
  HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t step_counter;
  uint16_t enc_val;


  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

	  HAL_Delay(10);

	  motor_d_update_pos(&Pololu_2, &pos_controller_1);
	  //sprintf((char*)log_buf, "Motor Pos: %d \r\n", enc_val);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);

	  fsm.run();
	  //HAL_UART_Transmit(&huart1, (uint8_t*)"FSM RUNNING\r\n", 13, HAL_MAX_DELAY);
	  /*step_counter += 1;

	  if(step_counter % 10 == 0) {
		  //log_IMU();
		  //log_LIDAR();

		  sprintf((char*)log_buf, "CH1 effort: %d CH2 effort: %d error: %d \r\n",
				  htim3.Instance->CCR1,
				  htim3.Instance->CCR2,
				  pos_controller_1.setpoint-htim4.Instance->CNT);
		  HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 500);
	  }



	  if(step_counter >= 2000) {
		  step_counter = 1;
		  motor_d_set_pos(&Pololu_2, &pos_controller_1, 1500);
		  sprintf((char*)log_buf, "Motor Pos: %d Motor goal: %d \r\n", motor_d_get_pos(&Pololu_2), pos_controller_1.setpoint);
		  HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
	  }

	  if(step_counter == 1000) {
	  		  motor_d_set_pos(&Pololu_2, &pos_controller_1, -1500);
	  		  sprintf((char*)log_buf, "Motor Pos: %d Motor goal: %d \r\n", motor_d_get_pos(&Pololu_2), pos_controller_1.setpoint);
	  		  HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
	  	  }*/


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
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



#include <ctype.h>  // for toupper()
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        char c = rx_buf[0];
        static char cmd_buffer[64];
        static uint8_t cmd_index = 0;

        if (c == '\r' || c == '\n')  // End of command
        {
            cmd_buffer[cmd_index] = '\0';

            // Convert to uppercase for consistent parsing
            for (uint8_t i = 0; i < cmd_index; i++) {
                cmd_buffer[i] = toupper((unsigned char)cmd_buffer[i]);
            }

            // Process full command
            if (cmd_index >= 4)
            {
                // === FSM STATE TRANSITION ===
                if (strncmp(cmd_buffer, "MODE", 4) == 0)
                {
                    uint8_t mode = cmd_buffer[4] - '0';
                    switch (mode) {
                        case 0: fsm.set_state(FSM::S0_INIT); break;
                        case 1: fsm.set_state(FSM::S1_IDLE); break;
                        case 2: fsm.set_state(FSM::S2_MANUAL_STEP_INPUT); break;
                        case 3: fsm.set_state(FSM::S3_MANUAL_TARGET); break;
                        case 4: fsm.set_state(FSM::S4_AUTOMATIC); break;
                        default:
                            HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Mode\r\n", 15, 1000);
                            break;
                    }

                    sprintf((char*)tx_buf, "FSM state: %d\r\n", fsm.get_state());
                    HAL_UART_Transmit(&huart1, tx_buf, strlen((char*)tx_buf), 1000);
                }

                // === MOTOR COMMAND: M1FF / M2FF ===
                else if (cmd_buffer[0] == 'M')
                {
                    if (fsm.get_state() != FSM::S2_MANUAL_STEP_INPUT) {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"Motor command not allowed in this state\r\n", 41, 1000);
                    } else if (cmd_buffer[1] < '1' || cmd_buffer[1] > '3') {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Motor Number\r\n", 23, 1000);
                    } else {
                        uint8_t motor_num = cmd_buffer[1] - '0';
                        char hex_string[3] = { cmd_buffer[2], cmd_buffer[3], '\0' };
                        int8_t duty = (int8_t)strtol(hex_string, NULL, 16);
                        if (duty > 127) duty -= 256;
                        if (duty > 100) duty = 100;
                        if (duty < -100) duty = -100;
                        int16_t pulse = (duty * 4799) / 100;
                        if (pulse < 0) pulse = -pulse;

                        if (motor_num == 1)
                            set_duty(&motor_1, (duty >= 0) ? pulse : 0);
                        else if (motor_num == 2)
                            set_duty(&motor_2, (duty >= 0) ? pulse : 0);
                        //selse if (motor_num == 3)
                        	//set_duty()

                        sprintf((char*)tx_buf, "Motor %d set to duty %d\r\n", motor_num, duty);
                        HAL_UART_Transmit(&huart1, tx_buf, strlen((char*)tx_buf), 1000);
                    }
                }

                // === SERVO COMMAND: S1XX / S2XX ===
                else if (cmd_buffer[0] == 'S')
                {
                    if (fsm.get_state() != FSM::S2_MANUAL_STEP_INPUT) {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"Servo command not allowed in this state\r\n", 41, 1000);
                    } else if (cmd_buffer[1] < '1' || cmd_buffer[1] > '2') {
                        HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Servo Number\r\n", 23, 1000);
                    } else {
                        uint8_t servo_num = cmd_buffer[1] - '0';
                        char hex_string[3] = { cmd_buffer[2], cmd_buffer[3], '\0' };
                        int8_t duty = (int8_t)strtol(hex_string, NULL, 16);
                        if (duty > 127) duty -= 256;
                        if (duty > 100) duty = 100;
                        if (duty < -100) duty = -100;

                        int16_t pulse = duty * (8275 - 1655) / 100 + 1655;
                        if (pulse < 0) {
                            HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Duty\r\n", 15, 1000);
                        }

                        if (servo_num == 1)
                            servo_duty(&servo_1, (duty >= 0) ? pulse : 0);
                        else
                            ; // placeholder for servo_2

                        sprintf((char*)tx_buf, "Servo %d set to duty %d\r\n", servo_num, duty);
                        HAL_UART_Transmit(&huart1, tx_buf, strlen((char*)tx_buf), 1000);
                    }
                }

                // === MOVEMENT COMMANDS (W/A/S/D) ===
                else if (fsm.get_state() == FSM::S2_MANUAL_STEP_INPUT || fsm.get_state() == FSM::S3_MANUAL_TARGET)
                {
                    char dir = cmd_buffer[0];
                    switch (dir) {
                        case 'W': HAL_UART_Transmit(&huart1, (uint8_t*)"Move Up\r\n", 9, 1000); break;
                        case 'A': HAL_UART_Transmit(&huart1, (uint8_t*)"Move Left\r\n", 11, 1000); break;
                        case 'S': HAL_UART_Transmit(&huart1, (uint8_t*)"Move Down\r\n", 11, 1000); break;
                        case 'D': HAL_UART_Transmit(&huart1, (uint8_t*)"Move Right\r\n", 12, 1000); break;
                        default:
                            HAL_UART_Transmit(&huart1, (uint8_t*)"Unknown Direction\r\n", 20, 1000);
                            break;
                    }
                }

                // === INVALID COMMAND ===
                else {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Command\r\n", 18, 1000);
                }
            }
            else {
                HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid Command\r\n", 18, 1000);
            }

            cmd_index = 0; // reset buffer
        }
        else
        {
            if (cmd_index < sizeof(cmd_buffer) - 1)
                cmd_buffer[cmd_index++] = c;
        }

        // Enable next UART RX interrupt
        HAL_UART_Receive_IT(&huart1, rx_buf, 1);
    }
}


void initialize_IMU(void) {
	HAL_Delay(100);
	sprintf((char*)log_buf, "INIT IMU \r\n");
	HAL_UART_Transmit(&huart1,(uint8_t*) log_buf, strlen((char*)log_buf), 1000);
	  if(HAL_I2C_IsDeviceReady (&hi2c3, 0x28 << 1, 10, 500) == HAL_OK) {

	  } else {
		  const int error = HAL_I2C_GetError(&hi2c3);
		  sprintf((char*)log_buf, "Device status error: %d\r\n", error);
		  HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
	  }


	  if(bno055_get_operation_mode(&op_mode) != 0) {
		  sprintf((char*)log_buf, "Failed to read op mode\r\n");
		  HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
	  } else {
		  sprintf((char*)log_buf, "Op mode is: %d \r\n", op_mode);
		  HAL_UART_Transmit(&huart1,(uint8_t*) log_buf, strlen((char*)log_buf), 1000);
	  }


	  switch(bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG)) {
	    	  case BNO055_ERROR:
	    		sprintf((char*)log_buf, "Could not set mode to IMU\r\n");
	    		HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
	    		  break;
	    	  case BNO055_E_NULL_PTR:
	    		sprintf((char*)log_buf, "IMU is a null ptr\r\n");
	    		HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
	    		  break;
	    	  default:
	    		  break;
	  }
	  HAL_Delay(100);
	  switch(bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS)) {
	  	  case BNO055_ERROR:
	  		sprintf((char*)log_buf, "Could not set mode to IMU\r\n");
	  		HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
	  		  break;
	  	  case BNO055_E_NULL_PTR:
	  		sprintf((char*)log_buf, "IMU is a null ptr\r\n");
	  		HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
	  		  break;
	  	  default:
	  		  break;
	}
	  bno055_get_operation_mode(&op_mode);
	  sprintf((char*)log_buf, "Set Op Mode to: %d \r\n", op_mode);
	  HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
	  HAL_Delay(2000);



}

void log_IMU(void) {
	if(HAL_I2C_IsDeviceReady (&hi2c3, 0x29 << 1, 10, 500) == HAL_OK) {

		  } else {
			  const int error = HAL_I2C_GetError(&hi2c3);
			  sprintf((char*)log_buf, "Device status error: %d\r\n", error);
			  HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);
		  }

		//poll the BNO055 for state
		bno055_get_operation_mode(&op_mode);
		bno055_get_power_mode(&pow_mode);
		sprintf((char*)log_buf, "Op Mode: %d Power Mode: %d, \r\n", op_mode, pow_mode);
		HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);

		//poll the BNO055 for data
		state = HAL_I2C_GetState(&hi2c3);
		bno055_read_accel_xyz(&accel_data);
		bno055_read_euler_hrp(&euler_data);
		sprintf((char*)log_buf, "X: %d Y: %d, Z: %d\r\n", euler_data.p, euler_data.h, euler_data.r);
		HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);

}

void log_LIDAR(void) {

	HAL_I2C_Mem_Read(&hi2c2, 0x10 << 1, 0x00, I2C_MEMADD_SIZE_8BIT, distance_data, 2, 500);
	distance_cm = (uint16_t)(distance_data[1] << 8 | distance_data[0]);
	HAL_I2C_Mem_Read(&hi2c2, 0x10 << 1, 0x02, I2C_MEMADD_SIZE_8BIT, intensity_data, 2, 500);
	intensity_value = (uint16_t)(intensity_data[1] << 8 | intensity_data[0]);

	sprintf((char*)log_buf, "distance: %d amp: %d\r\n", distance_cm, intensity_value);
	HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen((char*)log_buf), 1000);


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
