/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "user.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// PWM
uint32_t pwmPeriod = 45000; // ~20khz
// Loop
uint32_t count = 0, count2 = 0;
// HTS221
uint8_t HTS221_calib[16];
uint8_t HTS221_read[5];
uint16_t sens_humidity;
int16_t sens_temperature;
// LIS3MDL
uint8_t LIS3MDL_read[9];
int16_t sens_magnetometer[3];
// LSM6DS0
int16_t gyro_data_available_count = 0;
char init_comm = '[';
char end_comm = ']';
uint8_t LSM6DS0_status_read;
uint8_t LSM6DS0_acce_read[6];
int16_t sens_accelerometer[3];
uint8_t LSM6DS0_gyro_read[6];
int16_t sens_gyroscope[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void execute1Hz(void);
void execute10Hz(void);
void execute100Hz(void);
void execute1000Hz(void);
void toggleLed(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback(void) {
  count++;
  
  // 1000 hz
  execute1000Hz();
  
  // 100 hz
  if(count%10 == 0) {
    execute100Hz();
  }
  
  // 10 hz
  if(count%100 == 0) {
    execute10Hz();
  }
  
  // 1 hz
  if(count % 1000 == 0) {
    count = 0;
    execute1Hz();
  }
}
void getSensors() {
  // Reading sensor
  userReadHTS221(&HTS221_read[0]);
  userReadLIS3MDL(&LIS3MDL_read[0]);
  
  // Interpreting sensors
  userInterpHTS221(&HTS221_read[0], &sens_humidity, &sens_temperature);
  userInterpLIS3MDL(&LIS3MDL_read[0], &sens_magnetometer[0]);
  
  userReadLSM6DS0(&LSM6DS0_acce_read[0], &LSM6DS0_gyro_read[0], &LSM6DS0_status_read);
  userInterpLSM6DS0(&LSM6DS0_acce_read[0], &LSM6DS0_gyro_read[0], &sens_accelerometer[0], &sens_gyroscope[0]);
  
  HAL_UART_Transmit(&huart1, &init_comm, 1, 100);
  HAL_UART_Transmit(&huart1, &LSM6DS0_acce_read[0], 6, 1);
  HAL_UART_Transmit(&huart1, &LSM6DS0_gyro_read[0], 6, 1);
  HAL_UART_Transmit(&huart1, &end_comm, 1, 100);
}
void execute1Hz(void) {
  userSetPWM(&htim3, TIM_CHANNEL_2, pwmPeriod, count2);
  count2 += 10;
  if(count2 > 100) {
    count2 = 0;
  }
}
void execute10Hz(void) {
  getSensors();
  toggleLed();
}
void execute100Hz(void) {
}
void execute1000Hz(void) {
}
void toggleLed(void) {
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  int a = 0;
  int b = 2;
  a = a+b;
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  int a = 0;
  int b = 2;
  a = a+b;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init(pwmPeriod, 50);

  /* USER CODE BEGIN 2 */
  userConfigSens();
  userStartPWM(&htim3, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
