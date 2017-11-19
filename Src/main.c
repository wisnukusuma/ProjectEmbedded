/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include"sensor.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
u_int16_t _md = 0, _d = 0, _m = 0;
u_int8_t lap[3] = { 0, 0, 0 };
u_int8_t _status;
u_int16_t _tO[3] = { 0, 0, 0 };
char krm[16];
char kata[30];
char lcd[16];
uint8_t _enable = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance = TIM2) {
		if (_md == 99) {
			_md = 0;
			if (_d == 59) {
				_m++;
				_d = 0;
			} else {
				_d++;
			}
		} else {
			_md++;
		}
	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  lcd_init();
  lcd_putstr("Lap Counter");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	 HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	  HAL_Delay(100);
	  //lcd_gotoxy(0, 0);
	 // lcd_putstr("Tmbl SS");
		if (SSBtn()) {
				lcd_clear();
				lcd_gotoxy(0, 0);
				if (_enable == 0) {
					_enable = 1;
					HAL_TIM_Base_Start_IT(&htim2);
					lcd_putstr("Started");
				} else {
					_enable = 0;
					HAL_TIM_Base_Stop_IT(&htim2);
					lcd_putstr("Stoped");
				}

			}
			if (RstBtn()) {
				_enable = 0;
				HAL_TIM_Base_Stop_IT(&htim2);
				_md = 0;
				_d = 0;
				_m = 0;
				lap[0] = 0;
				lap[1] = 0;
				lap[2] = 0;
				lcd_clear();
				lcd_gotoxy(0, 0);
				lcd_putstr("Reseted");

			}
			if (_enable) {
				if (Ssr1() == 1 || _tO[0] >= 500) {
					if (_tO[0] < 500) {
						lap[0]++;
						lcd_gotoxy(0, 0);
						sprintf(lcd, "M1 l%2d %2d:%2d:%2d", lap[0], _m, _d, _md);
						HAL_UART_Transmit(&huart1, (uint8_t *) lcd, 15, 10);
						HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", 2, 10);
						lcd_putstr(lcd);
						_tO[0] = 0;
					} else {
						_tO[0] = 0;
						lcd_clear();
						sprintf(krm, "Sensor 1 error");
						HAL_UART_Transmit(&huart1, (uint8_t *) krm, 14, 10);
						HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", 2, 10);
						lcd_gotoxy(0, 0);
						lcd_putstr(krm);
						lcd_gotoxy(0, 1);
						lcd_putstr("Silahkan cek");
					}
				}
				if (Ssr2() || _tO[1] >= 500) {
					if (_tO[1] < 500) {
						lap[1]++;
						lcd_gotoxy(0, 1);
						sprintf(lcd, "M2 l%2d %2d:%2d:%2d", lap[1], _m, _d, _md);
						HAL_UART_Transmit(&huart1, (uint8_t *) lcd, 15, 10);
						HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", 2, 10);
						lcd_putstr(lcd);
						_tO[1] = 0;
					} else {
						lcd_clear();
						_tO[1] = 0;
						lcd_clear();
						sprintf(krm, "Sensor 2 error");
						HAL_UART_Transmit(&huart1, (uint8_t *) krm, 14, 10);
						HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", 2, 10);
						lcd_gotoxy(0, 0);
						lcd_putstr(krm);
						lcd_gotoxy(0, 1);
						lcd_putstr("Silahkan cek");
					}
				}
				if (Ssr3() || _tO[2] >= 500) {
					if (_tO[2] < 500) {
						lap[2]++;
						lcd_gotoxy(0, 0);
						sprintf(lcd, "M3 l%2d %2d:%2d:%2d", lap[2], _m, _d, _md);
						HAL_UART_Transmit(&huart1, (uint8_t *) lcd, 15, 10);
						HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", 2, 10);
						lcd_putstr(lcd);
						_tO[2] = 0;
					} else {
						lcd_clear();
						_tO[2] = 0;
						sprintf(krm, "Sensor 3 error");
						HAL_UART_Transmit(&huart1, (uint8_t *) krm, 14, 10);
						HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", 2, 10);
						lcd_gotoxy(0, 0);
						lcd_putstr(krm);
						lcd_gotoxy(0, 1);
						lcd_putstr("Silahkan cek");
					}
				}
			}
		}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|RW_Pin|EN_Pin|D4_Pin 
                          |D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Ssr3_Pin Ssr2_Pin Ssr1_Pin RstBtn_Pin 
                           SSBtn_Pin */
  GPIO_InitStruct.Pin = Ssr3_Pin|Ssr2_Pin|Ssr1_Pin|RstBtn_Pin 
                          |SSBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin RW_Pin EN_Pin D4_Pin 
                           D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|RW_Pin|EN_Pin|D4_Pin 
                          |D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
