/*
 * Lib.c
 *
 *  Created on: Sep 6, 2024
 *      Author: Linh
 */
#include "stm32f4xx_hal.h"
#include "Lib.h"

#define BL_RX_LEN  200
uint8_t bl_rx_buffer[BL_RX_LEN];

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
	uint8_t Rx_Length = 0;
	uint8_t Rx_Cmd = 0;
	while(1)
	{
		//receive first byte - is length
		HAL_UART_Receive(USART2, bl_rx_buffer, 1, 0xFFFFFFFFU);
		Rx_Length = bl_rx_buffer[0];

		HAL_UART_Receive(USART2, bl_rx_buffer[1], Rx_Length, 0xFFFFFFFFU);
		Rx_Cmd = bl_rx_buffer[1];

		switch (Rx_Cmd)
		{
			case BL_GET_VER :
				bootloader_handle_getver_cmd(bl_rx_buffer);
				break;
			case BL_GET_HELP :
				break;
			case BL_GET_CID :
				break;
			case BL_GET_RDP_STATUS :
				break;
			case BL_GO_TO_ADDR :
				break;
			case BL_FLASH_ERASE :
				break;
			case BL_MEM_WRITE :
				break;
			case BL_EN_RW_PROTECT :
				break;
			case BL_MEM_READ :
				break;
			case BL_READ_SECTOR_P_STATUS :
				break;
			case BL_OTP_READ :
				break;
			case BL_DIS_R_W_PROTECT :
				break;
		}

	}
}

void bootloader_jump_to_user_app(void)
{
	   //declare a function pointer to hold the address of the reset handler of the user app.
	    int (*app_reset_handler)(void);

	    // 1. configure the MSP by reading the value from the base address of the sector 2
	    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	    //This function comes from CMSIS.
	    __set_MSP(msp_value);

	    // 2. fetch reset_handler addr of the user application from FLASH_SECTOR2_BASE_ADDRESS+4
	    uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);
	    app_reset_handler = (void*) resethandler_address;

      //3.Vector Table Relocation to SECTOR 2
      SCB->VTOR = FLASH_SECTOR2_BASE_ADDRESS;

	    //4. jump to reset handler of the user application
	    app_reset_handler();
}

/* boot loader handle fucntion */
void bootloader_handle_getver_cmd(uint8_t* RxBuffer)
{
	uint8_t bl_version;

	//get RxBuffer total length
	uint32_t RxBuffer_Length = RxBuffer[0]+1;

	//host CRC value is stored in last 4 byte of RxBuffer
	uint32_t host_CRC = *(uint32_t *)(RxBuffer + RxBuffer_Length - 4);

	//calculate RxCRC from RxBuffer, dismiss last 4 byte
	uint32_t RxCRC = CRC_Calculation(RxBuffer, RxBuffer_Length-4);

	if(host_CRC == RxCRC)
	{
		//CRC is correct
		bootloader_send_ack(bl_rx_buffer[0],1);
		bl_version=get_bootloader_version();
		bootloader_uart_write_data(&bl_version,1);
	}
	else
	{
		//CRC is wrong
		bootloader_send_nack();
	}

}

uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}

/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
	HAL_UART_Transmit(USART2,pBuffer,len,HAL_MAX_DELAY);

}

uint32_t CRC_Calculation(uint32_t* pInputCRC, uint32_t u32Length)
{
	/*enable clock for CRC*/
	RCC->AHB1ENR |= 1<<12;
	/*Resets the CRC calculation unit and sets the data reg to 0xFFFF FFFF*/
	CRC->CR |= 0x01;
	/*calculate CRC*/
	for(int i=0; i<u32Length; i++)
	{
		CRC->DR = *(pInputCRC+i);
	}
	return CRC->DR;
}

/*This function sends ACK if CRC matches along with "len to follow"*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{

}

void bootloader_send_nack(void)
{

}
