/// \file main.c
/// \brief Example: using interrupt for detect motion or weightlessness (free fall) by LSM303DLHC
/// \copyright &copy; https://github.com/Ilushenko Oleksandr Ilushenko
///	\author Oleksandr Ilushenko
///	\date 2024
#include "main.h"
#include <stdio.h>

#include "log.h"
#include "lsm303dlhc.h"

I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);

volatile uint8_t irq1_ = 0U;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();

  // setup
  HAL_Delay(2000);

  // Init Log
  setlog(&huart1);

  // Accelerometer setup
  if (lsm303_la_setup(&hi2c3, LSM303_ADATARATE_400, 0U, 1U, LSM303_AFS_4G) != HAL_OK) {
    xError("LSM303DLHC Accelerometer Setup Error!\n");
    while (1);
  }
  HAL_Delay(10);

  const float T = 1.0F / 400; // one tact duration

  // === Accelerometer detect motion by INT1 ===
  lsm303_reg_int_cfg_a_t cfg = { 0 };                         // INT1_CFG_A
  cfg.xhe = 1U;                                               // Enabe X high event
  cfg.yhe = 1U;                                               // Enabe Y high event
  cfg.zhe = 1U;                                               // Enabe Z high event
  cfg.aoi6d = LSM303_AOR;                                     // OR combination of interrupt events
  const uint8_t threshould = (uint8_t)(0.05F / lsm303_alsb);  // 0.05g
  const uint8_t duration = (uint8_t)(0.05 / T);               // 50ms
  if (lsm303_la_int1(&hi2c3, cfg.reg, threshould, duration) != HAL_OK) {
    xError("LSM303DLHC Accelerometer Config INT1 Error!\n");
    while (1);
  }

  // === Accelerometer detect weightlessness by INT1 ===
  // lsm303_reg_int_cfg_a_t cfg = { 0 };                       // INT1_CFG_A
  // cfg.xle = 1U;                                             // Enable X for low level
  // cfg.yle = 1U;                                             // Enable Y for low level
  // cfg.zle = 1U;                                             // Enable Z for low level
  // cfg.aoi6d = LSM303_AOR;                                   // OR combination of interrupt events
  // const uint8_t threshould = (uint8_t)(0.2F / lsm303_alsb); // 0.2g
  // const uint8_t duration = (uint8_t)(0.02 / T);             // 20ms
  // if (lsm303_la_int1(&hi2c3, cfg.reg, threshould, duration) != HAL_OK) {
  //   xError("LSM303DLHC Accelerometer Config INT1 Error!\n");
  //   while (1);
  // }

  // === Accelerometer deactivate interrupt by INT1 ===
  // if (lsm303_la_int1(&hi2c3, 0U, 0U, 0U) != HAL_OK) {
  //   xError("LSM303DLHC Accelerometer Deactivate INT1 Error!\n");
  //   while (1);
  // }

  // IRQ test loop
  while (1) {
    if (irq1_ > 1U) {
      irq1_ ^= irq1_; // 0
      xDebug("Interrupt on INT1\n");
    }
  }
  return 0;
}

// Interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (INT1_Pin == GPIO_Pin) {
    lsm303_reg_int_src_a_t src = { 0 };
    lsm303_la_src1(&hi2c3, &src.reg);
    if (src.ia) ++irq1_;
  }
  if (INT2_Pin == GPIO_Pin) {
    // todo
  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c3.Init.Timing = 0x00100D14;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : INT1_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
