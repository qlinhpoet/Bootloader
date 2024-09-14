/*
 * Lib.c
 *
 *  Created on: Sep 6, 2024
 *      Author: Linh
 */
#include "stm32f4xx_hal.h"
#include "Lib.h"

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Config user button */
  RCC->AHB1ENR |= 0x01;  	//IO port A clock enable
  GPIOA->MODER &= ~(0x03);	//PA0 input mode

}

void USART2_Init(void)
{
	//1. set up gpio : PA2: USART2_TX - PA3: USART2_RX
	RCC->AHB1ENR |= 0x01;
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//2. set up uart
	UART_HandleTypeDef UART_Init = {0};
	UART_Init.Instance = USART2;
	UART_Init.Init.BaudRate = 9600;
	UART_Init.Init.Mode = UART_MODE_TX | UART_MODE_RX;
	UART_Init.Init.Parity = UART_PARITY_NONE;
	UART_Init.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART_Init.Init.StopBits = UART_STOPBITS_1;
	UART_Init.Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(&UART_Init);
}

uint8_t UART_Receive(USART_TypeDef* huart)
{
	return huart->DR;
}

void UART_Transmit1byte(USART_TypeDef* huart, uint8_t data)
{
	//send data - both 8, 9 bits
	huart->DR = (uint16_t)data & 0x01FF;
	//TXE - 1:data tranfered to shift register
	while( (huart->SR & (1<<7)) != (1<<7));
}

void UART_Transmit(USART_TypeDef* huart, uint8_t* data, uint32_t datasize)
{
	for(int i=0; i<datasize; i++)
	{
		UART_Transmit1byte(huart, *(data+i));
	}
}

void BlinkLed(uint32_t msecond)
{
	  GPIOD->ODR |= 1<<12;
	  HAL_Delay(msecond);
	  GPIOD->ODR &= ~1<<12;
	  HAL_Delay(msecond);
}

void Error_Handler(void)
{

}

FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG)
{
	FlagStatus bitstatus = RESET;

  if ((USARTx->SR & USART_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

void bootloader_uart_read_data(void)
{

}

void bootloader_jump_to_user_app(void)
{
  
}
