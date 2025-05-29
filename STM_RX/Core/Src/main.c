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
#define usTIM TIM4

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

uint8_t RxData[30];

int indx = 0;

uint8_t TxData[25];

int isClicked = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

void usDelay(uint32_t uSec);

const float speedOfSound = 0.0343 / 2;

charuartBuf[100];

float currentDistance = 0.0f;
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 30);

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		// Keep the button press logic intact if you need it elsewhere
		if (isClicked == 0) {
			isClicked = 1;
		}
	}

	// Replace the button logic with distance logic
	//if (currentDistance < 20.0f && isClicked == 0) {
	//isClicked = 1;  // Set to 1 when the condition is met
	//}

	//shaivi added code//

	/*if (currentDistance >= 20.0f && currentDistance < 40.0f
	 && isClicked == 0) {
	 isClicked = 2; // led off
	 }

	 else if (currentDistance >= 40.0f && currentDistance < 60.0f
	 && isClicked == 0) {
	 isClicked = 3; // fan on
	 }

	 else if (currentDistance >= 60.0f && currentDistance < 80.0f
	 && isClicked == 0) {
	 isClicked = 4; // fan off
	 }

	 else if (currentDistance >= 80.0f && currentDistance < 100.0f
	 && isClicked == 0) {
	 isClicked = 5; // door open
	 }

	 else if (currentDistance >= 100.0f && currentDistance < 120.0f
	 && isClicked == 0) {
	 isClicked = 6; // door close
	 }*/
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN 2 */

	HAL_HalfDuplex_EnableReceiver(&huart1);

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 30);

	/* USER CODE END 2 */

	/* Infinite loop */

	/* USER CODE BEGIN WHILE */

	while (1) {
		// Measure distance
		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
		usDelay(3);
		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
		usDelay(10);
		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

		// Wait for the echo signal
		while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)
			;
		uint32_t numTicks = 0;
		while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET) {
			numTicks++;
			usDelay(2);
		}

		// Calculate distance
		currentDistance = (numTicks * 2.8 * 0.0343) / 2.0;

		// Trigger actions based on distance range
		if (isClicked == 0) {  // Only trigger if no action is in progress
			if (currentDistance < 20.0f) {
				isClicked = 1;  // Trigger logic for LED ON
			} else if (currentDistance >= 20.0f && currentDistance < 40.0f) {
				isClicked = 2;  // Trigger logic for LED OFF
			} else if (currentDistance >= 40.0f && currentDistance < 60.0f) {
				isClicked = 3;  // Trigger logic for FAN ON
			} else if (currentDistance >= 60.0f && currentDistance < 80.0f) {
				isClicked = 4;  // Trigger logic for FAN OFF
			} else if (currentDistance >= 80.0f && currentDistance < 100.0f) {
				isClicked = 5;  // Trigger logic for DOOR OPEN
			} else if (currentDistance >= 100.0f && currentDistance < 120.0f) {
				isClicked = 6;  // Trigger logic for DOOR CLOSE
			}
		}

		// Check and act if `isClicked` is set
		if (isClicked == 1) {
			// Transmit signal and perform action (LED ON)
			int len = sprintf((char*) TxData, "LED_ON");
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit(&huart1, TxData, len, 1000);
			HAL_HalfDuplex_EnableReceiver(&huart1);

			// Provide feedback with an LED
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

			// Reset `isClicked` for the next trigger
			isClicked = 0;  // Allow the next action to be triggered
		} else if (isClicked == 2) {
			// Transmit signal and perform action (LED OFF)
			int len = sprintf((char*) TxData, "LED_OFF");
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit(&huart1, TxData, len, 1000);
			HAL_HalfDuplex_EnableReceiver(&huart1);

			// Provide feedback with an LED
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

			// Reset `isClicked` for the next trigger
			isClicked = 0;
		} else if (isClicked == 3) {
			// Transmit signal and perform action (FAN ON)
			int len = sprintf((char*) TxData, "FAN_ON");
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit(&huart1, TxData, len, 1000);
			HAL_HalfDuplex_EnableReceiver(&huart1);

			// Provide feedback with an LED
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

			// Reset `isClicked` for the next trigger
			isClicked = 0;
		} else if (isClicked == 4) {
			// Transmit signal and perform action (FAN OFF)
			int len = sprintf((char*) TxData, "FAN_OFF");
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit(&huart1, TxData, len, 1000);
			HAL_HalfDuplex_EnableReceiver(&huart1);

			// Provide feedback with an LED
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

			// Reset `isClicked` for the next trigger
			isClicked = 0;
		} else if (isClicked == 5) {
			// Transmit signal and perform action (DOOR OPEN)
			int len = sprintf((char*) TxData, "SIGNAL_ON");
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit(&huart1, TxData, len, 1000);
			HAL_HalfDuplex_EnableReceiver(&huart1);

			// Provide feedback with an LED
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

			// Reset `isClicked` for the next trigger
			isClicked = 0;
		} else if (isClicked == 6) {
			// Transmit signal and perform action (DOOR CLOSE)
			int len = sprintf((char*) TxData, "SIGNAL_OFF");
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit(&huart1, TxData, len, 1000);
			HAL_HalfDuplex_EnableReceiver(&huart1);

			// Provide feedback with an LED
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

			// Reset `isClicked` for the next trigger
			isClicked = 0;
		}

		HAL_Delay(50);  // Small delay to avoid rapid toggling
	}
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}
/* USER CODE BEGIN 4 */

void usDelay(uint32_t uSec) {

	if (uSec < 2)

		uSec = 2;

	usTIM->ARR = uSec - 1; /*sets the value in the auto-reload register*/

	usTIM->EGR = 1; /*Re-initialises the timer*/

	usTIM->SR &= ~1; 		//Resets the flag

	usTIM->CR1 |= 1; 		//Enables the counter

	while ((usTIM->SR & 0x0001) != 1)

		;

	usTIM->SR &= ~(0x0001);

}
/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 84 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_HalfDuplex_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_Pin | TRIG_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_Pin TRIG_Pin */
	GPIO_InitStruct.Pin = LED_Pin | TRIG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ECHO_Pin */
	GPIO_InitStruct.Pin = ECHO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
