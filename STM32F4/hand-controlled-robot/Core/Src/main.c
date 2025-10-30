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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  MODE_FORWARD = 0,
  MODE_REVERSE = 1,
  MODE_RIGHT = 2,
  MODE_LEFT = 3,
  MODE_STOP = 4
} MotorMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define UART_BUFFER_SIZE 100
#define PACKET_SIZE 5

uint8_t uartRxBuffer[UART_BUFFER_SIZE];
uint8_t rxByte;   
volatile uint16_t rxIndex = 0;  
uint8_t commandBuffer[PACKET_SIZE];     
volatile uint8_t newCommandReceived = 0; 

uint8_t currentMode = MODE_STOP;
uint8_t currentPWM_Right = 0;
uint8_t currentPWM_Left = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void processCommand(uint8_t *data);
void setMotor(GPIO_TypeDef *fwdPort, uint16_t fwdPin, GPIO_TypeDef *revPort, uint16_t revPin, uint32_t channel, uint8_t pwm, uint8_t forward);
void moveForward(uint8_t pwm_right, uint8_t pwm_left);
void moveReverse(uint8_t pwm_right, uint8_t pwm_left);
void rotateRight(uint8_t pwm_right, uint8_t pwm_left);
void rotateLeft(uint8_t pwm_right, uint8_t pwm_left);
void stopMotors(void);
void updateLEDs(void);
void turnOffAllLEDs(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void processCommand(uint8_t *data) {
  if (data[4] != (data[1] ^ data[2] ^ data[3]) || data[1] > MODE_STOP) {
    return; 
  }
  
  currentMode = data[1];
  currentPWM_Right = data[2];
  currentPWM_Left = data[3];
  
  switch (currentMode) {
    case MODE_FORWARD:  moveForward(data[2], data[3]);  break;
    case MODE_REVERSE:  moveReverse(data[2], data[3]);  break;
    case MODE_RIGHT:    rotateRight(data[2], data[3]);  break;
    case MODE_LEFT:     rotateLeft(data[2], data[3]);   break;
    case MODE_STOP:     stopMotors();                   break;
  }
}

void setMotor(GPIO_TypeDef *fwdPort, uint16_t fwdPin, GPIO_TypeDef *revPort, uint16_t revPin, 
              uint32_t channel, uint8_t pwm, uint8_t forward) {
  HAL_GPIO_WritePin(fwdPort, fwdPin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(revPort, revPin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, channel, pwm);
}

void moveForward(uint8_t pwm_right, uint8_t pwm_left) {
  setMotor(RIGHT_FWD_GPIO_Port, RIGHT_FWD_Pin, RIGHT_REV_GPIO_Port, RIGHT_REV_Pin, TIM_CHANNEL_1, pwm_right, 1);
  setMotor(LEFT_FWD_GPIO_Port, LEFT_FWD_Pin, LEFT_REV_GPIO_Port, LEFT_REV_Pin, TIM_CHANNEL_2, pwm_left, 1);
  updateLEDs();
}

void moveReverse(uint8_t pwm_right, uint8_t pwm_left) {
  setMotor(RIGHT_FWD_GPIO_Port, RIGHT_FWD_Pin, RIGHT_REV_GPIO_Port, RIGHT_REV_Pin, TIM_CHANNEL_1, pwm_right, 0);
  setMotor(LEFT_FWD_GPIO_Port, LEFT_FWD_Pin, LEFT_REV_GPIO_Port, LEFT_REV_Pin, TIM_CHANNEL_2, pwm_left, 0);
  updateLEDs();
}

void rotateRight(uint8_t pwm_right, uint8_t pwm_left) {
  setMotor(RIGHT_FWD_GPIO_Port, RIGHT_FWD_Pin, RIGHT_REV_GPIO_Port, RIGHT_REV_Pin, TIM_CHANNEL_1, pwm_right, 0);
  setMotor(LEFT_FWD_GPIO_Port, LEFT_FWD_Pin, LEFT_REV_GPIO_Port, LEFT_REV_Pin, TIM_CHANNEL_2, pwm_left, 1);
  updateLEDs();
}

void rotateLeft(uint8_t pwm_right, uint8_t pwm_left) {
  setMotor(RIGHT_FWD_GPIO_Port, RIGHT_FWD_Pin, RIGHT_REV_GPIO_Port, RIGHT_REV_Pin, TIM_CHANNEL_1, pwm_right, 1);
  setMotor(LEFT_FWD_GPIO_Port, LEFT_FWD_Pin, LEFT_REV_GPIO_Port, LEFT_REV_Pin, TIM_CHANNEL_2, pwm_left, 0);
  updateLEDs();
}

void stopMotors(void) {
  setMotor(RIGHT_FWD_GPIO_Port, RIGHT_FWD_Pin, RIGHT_REV_GPIO_Port, RIGHT_REV_Pin, TIM_CHANNEL_1, 0, 1);
  setMotor(LEFT_FWD_GPIO_Port, LEFT_FWD_Pin, LEFT_REV_GPIO_Port, LEFT_REV_Pin, TIM_CHANNEL_2, 0, 1);
  turnOffAllLEDs();
}

void updateLEDs(void) {
  turnOffAllLEDs();
  
  uint8_t rightFwd = HAL_GPIO_ReadPin(RIGHT_FWD_GPIO_Port, RIGHT_FWD_Pin);
  uint8_t leftFwd = HAL_GPIO_ReadPin(LEFT_FWD_GPIO_Port, LEFT_FWD_Pin);
  
  if (rightFwd == leftFwd) {
    HAL_GPIO_WritePin(rightFwd ? LED_FWD_GPIO_Port : LED_REV_GPIO_Port, 
                      rightFwd ? LED_FWD_Pin : LED_REV_Pin, GPIO_PIN_SET);
    
    if (currentPWM_Right > currentPWM_Left) {
      HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_SET);
    } else if (currentPWM_Left > currentPWM_Right) {
      HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_SET);
    }
  }
  else {
    HAL_GPIO_WritePin(rightFwd ? LED_LEFT_GPIO_Port : LED_RIGHT_GPIO_Port,
                      rightFwd ? LED_LEFT_Pin : LED_RIGHT_Pin, GPIO_PIN_SET);
  }
}

void turnOffAllLEDs(void) {
  HAL_GPIO_WritePin(LED_FWD_GPIO_Port, LED_FWD_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_REV_GPIO_Port, LED_REV_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (rxIndex == 0) {
      if (rxByte == 0xFF) {
        uartRxBuffer[0] = 0xFF;
        rxIndex = 1;
      }
    }
    else {
      uartRxBuffer[rxIndex++] = rxByte;
      
      if (rxIndex == PACKET_SIZE) {
        memcpy(commandBuffer, uartRxBuffer, PACKET_SIZE);
        newCommandReceived = 1;
        rxIndex = 0;
      }
    }
    
    HAL_UART_Receive_IT(&huart2, &rxByte, 1);
  }
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  stopMotors();
  
  HAL_UART_Receive_IT(&huart2, &rxByte, 1);

  HAL_GPIO_WritePin(LED_FWD_GPIO_Port, LED_FWD_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_FWD_GPIO_Port, LED_FWD_Pin, GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(LED_REV_GPIO_Port, LED_REV_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_REV_GPIO_Port, LED_REV_Pin, GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (newCommandReceived) {
      newCommandReceived = 0;
      processCommand(commandBuffer);
    }
    
    HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
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

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RIGHT_FWD_Pin|RIGHT_REV_Pin|LEFT_FWD_Pin|LEFT_REV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_RIGHT_Pin|LED_REV_Pin|LED_LEFT_Pin|LED_FWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RIGHT_FWD_Pin RIGHT_REV_Pin LEFT_FWD_Pin LEFT_REV_Pin */
  GPIO_InitStruct.Pin = RIGHT_FWD_Pin|RIGHT_REV_Pin|LEFT_FWD_Pin|LEFT_REV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RIGHT_Pin LED_REV_Pin LED_LEFT_Pin LED_FWD_Pin */
  GPIO_InitStruct.Pin = LED_RIGHT_Pin|LED_REV_Pin|LED_LEFT_Pin|LED_FWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
