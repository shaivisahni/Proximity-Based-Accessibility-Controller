/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file           : main.c

  * @brief          : Main program body

  ******************************************************************************

  * @attention

  *

  * Copyright (c) 2024 STMicroelectronics.

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



#include "stdio.h"



#include "stm32f4xx_hal.h"



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

UART_HandleTypeDef huart1;

UART_HandleTypeDef huart2;



/* USER CODE BEGIN PV */



/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */



/* USER CODE END PFP */



/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

uint8_t RxData[30];  // Buffer for receiving data

volatile uint8_t dataReceivedFlag = 0;  // Flag to indicate data reception



/* USER CODE END 0 */



/**

  * @brief  The application entry point.

  * @retval int

  */

// Function prototypes for specific actions

void TurnOnLED(void);

void TurnOffLED(void);

void BlinkLED(void);



int main(void)

{

  /* MCU Configuration--------------------------------------------------------*/



  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  HAL_Init();



  /* Configure the system clock */

  SystemClock_Config();



  /* Initialize all configured peripherals */

  MX_GPIO_Init();

  MX_USART2_UART_Init();

  MX_USART1_UART_Init();



  /* USER CODE BEGIN 2 */



  // Enable receiver mode for half-duplex communication

  HAL_HalfDuplex_EnableReceiver(&huart1);



  // Start receiving data in interrupt mode

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));



  /* USER CODE END 2 */



  /* Infinite loop */

  while (1)

      {

          if (dataReceivedFlag)

          {

              dataReceivedFlag = 0;  // Clear the flag



              // Process the received string

              if (strcmp((char *)RxData, "LED_ON") == 0)

              {

            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);  // Turn on LED

              }

              else if (strcmp((char *)RxData, "LED_OFF") == 0)

              {

            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // Turn on LED

              }

              else if (strcmp((char *)RxData, "FAN_ON") == 0)

              {

            	  HAL_GPIO_WritePin(GPIOA, FAN_Pin, GPIO_PIN_SET);  // Turn on LED


              }else if (strcmp((char *)RxData, "FAN_OFF") == 0)

              {
            	  HAL_GPIO_WritePin(GPIOA, FAN_Pin, GPIO_PIN_RESET);

              }else if (strcmp((char *)RxData, "SIGNAL_ON") == 0)

              {


            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

              }else if (strcmp((char *)RxData, "SIGNAL_OFF") == 0)

              {

            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
              }



              // Clear buffer after processing

              memset(RxData, 0, sizeof(RxData));



              // Re-enable UART reception for the next message

              HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));

          }

      }

  }



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)

{

    if (huart->Instance == USART1)

    {

        dataReceivedFlag = 1;  // Set the flag to indicate data reception

        RxData[Size] = '\0';  // Null-terminate the received string

    }

}



// Action functions

void TurnOnLED(void)

{

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);  // Turn on LED

}



void TurnOffLED(void)

{

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // Turn off LED

}



void TurnOnFAN(void)

{



	HAL_GPIO_WritePin(GPIOA, FAN_Pin, GPIO_PIN_SET);  // Turn on LED

}



void ClearBuffer(uint8_t *buffer, uint16_t size)

{

    for (uint16_t i = 0; i < size; i++) {

        buffer[i] = 0;

    }

}





/* USER CODE END 4 */







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

  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 4;

  RCC_OscInitStruct.PLL.PLLN = 84;

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

  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN USART1_Init 2 */



  /* USER CODE END USART1_Init 2 */



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

  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_RCC_GPIOH_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();



  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);



  /*Configure GPIO pin : B1_Pin */

  GPIO_InitStruct.Pin = B1_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);



  /*Configure GPIO pin : PA5 */

  GPIO_InitStruct.Pin = GPIO_PIN_5;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



  GPIO_InitStruct.Pin = GPIO_PIN_6;

   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

   GPIO_InitStruct.Pull = GPIO_NOPULL;

   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



   /*Configure GPIO pin : FAN_Pin */

     GPIO_InitStruct.Pin = FAN_Pin;

     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

     GPIO_InitStruct.Pull = GPIO_NOPULL;

     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



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
