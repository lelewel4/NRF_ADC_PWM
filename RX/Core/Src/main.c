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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "nRF24_Defs.h"
#include "nRF24.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DEBUG_PRINTF
//#define DEBUG_AMPLIUTDE_SERWO
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/*---------------PRINTF---------------*/
int __io_putchar(int ch){
  if (ch == '\n') {
    __io_putchar('\r');
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return 1;
}

/*---------------NRF---------------*/
volatile uint8_t nrf24_rx_flag, nrf24_tx_flag, nrf24_mr_flag;
uint8_t Nrf24_Message[NRF24_PAYLOAD_SIZE];
uint8_t Message[32];
uint8_t MessageLength;
int recived_value;

/*---------------PWM---------------*/
uint16_t PWM;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  nRF24_Init(&hspi2);
  nRF24_SetRXAddress(0, "Odb");
  nRF24_SetTXAddress("Nad");
  nRF24_RX_Mode();

  //pomachanie skrzydlem
#ifdef DEBUG_AMPLIUTDE_SERWO
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 1100);
	  HAL_Delay(2000);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 2100);
	  HAL_Delay(2000);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 1100);
	  HAL_Delay(2000);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 2100);
	  HAL_Delay(2000);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 1100);

#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(nRF24_RXAvailible())
	  {
		  nRF24_ReadRXPaylaod(Nrf24_Message);
		  char *endptr;
		        recived_value = strtol((char *)Nrf24_Message, &endptr, 10);
#ifdef DEBUG_PRINTF
		        if (*endptr == '\0')
				{
					printf("Udalo sie odebrac: %i\n\r", recived_value);
				}
		        else
				{
					printf("Error: %s\n\r", (char *)Nrf24_Message);
				}
#endif DEBUG_PRINTF
		        /*----------------SERWO---------------
		       	  Czestowliowsc bazowa PWM 50Hz = okres podstawowy 20ms
		       	  Czas trwania impulsu od 1ms do 2ms

		       	  1000us = 1ms - minimalne wychylenie (0	st.)
		       	  2000us = 2ms - maksymlane wychylenie (90	st.)

		       	  ------------------------------------------------

		       	  W celu wypionowania smigla mozna wprowadzic offset, tak aby smiglo znajdowalo sie w kacie 0st.
		       	  np:
		       	  100us = 0.1ms
		       	  200us = 0.2ms

		       	  -------------------------------------------------

		       	  Wartosci, które można otrzymac z ADC 		|		Wartosci dla PWM
								  0		(min)				|				1000		(0st.)
								  4095	(max)				|				2000		(90st.)

			  	  Z tego powodu nalezy dokonac konwersji danych. Wartosci nalezy podzielic przez 4.095 oraz dodac wartoisc 1000.
		       	  ---------------------------------------*/

		        PWM = 1000 + (int)(recived_value / 4.095);		//konwersja
#ifdef DEBUG_PRINTF
				printf("recived_value: %d,	PWM = %d\n\r", recived_value, PWM);		//weryfikacja konwersji
#endif
				  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, PWM);
				  HAL_Delay(100);

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
