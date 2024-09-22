/*
 * Lib.c
 *
 *  Created on: Sep 6, 2024
 *      Author: Linh
 */
#include "stm32f4xx_hal.h"
#include "FlashLib.h"
#include "Lib.h"

#define BL_RX_LEN  20
uint8_t bl_rx_buffer[BL_RX_LEN];
uint8_t nocmd[] = "no cmd\n";
_Bool inTesting = 1;
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
	UART_Init.Init.WordLength = UART_WORDLENGTH_8B;
	HAL_UART_Init(&UART_Init);
}

void UART_Receive(USART_TypeDef* huart, uint8_t* RxBuffer, uint8_t RxSize)
{
	uint8_t i = 0;
	uint32_t timeout = 0xffff;
	while(i < RxSize && timeout > 0)
	{
		if( (USART2->SR & (0x0020)) == (0x0020) )		//if data is received, clear bit by a read to USART_DR reg
		{
			*(RxBuffer + i) =  huart->DR;
			i++;
			timeout = 0xffff;
		}
		timeout--;
	}
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

void BlinkLed(uint8_t ledNum, uint32_t msecond)
{
	  GPIOD->ODR |= 1<<ledNum;
	  HAL_Delay(msecond);
	  GPIOD->ODR &= ~1<<ledNum;
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
	((UART_HandleTypeDef*)USART2)->RxState = HAL_UART_STATE_READY;
	while(1)
	{
		resetBuffer(bl_rx_buffer, sizeof(bl_rx_buffer));
		//receive first byte - is length
		UART_Receive(USART2, bl_rx_buffer, 1);
		Rx_Length = bl_rx_buffer[0];

		UART_Receive(USART2, &bl_rx_buffer[1], Rx_Length);
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
				bootloader_handle_go_cmd(bl_rx_buffer);
				break;
			case BL_FLASH_ERASE :
				bootloader_handle_flash_erase_cmd(bl_rx_buffer);
				break;
			case BL_MEM_WRITE :
				bootloader_handle_mem_write_cmd(bl_rx_buffer);
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
		//BlinkLed(14, 1000);
		bootloader_uart_write_data(nocmd, sizeof(nocmd));

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
	GPIOD->ODR &= ~1<<15;	//reset bootloader led
	app_reset_handler();
}

/* boot loader handle fucntion */
void bootloader_handle_getver_cmd(uint8_t* RxBuffer)
{
	uint8_t bl_version;

	if(bootloader_verify_CRC(RxBuffer))
	{
		//CRC is correct
		bootloader_send_ack(bl_rx_buffer[0],1);
		bl_version=get_bootloader_version();
		bootloader_uart_write_data("bootloader version 1\n", 21);
		//bootloader_uart_write_data(&bl_version,1);
	}
	else
	{
		//CRC is wrong
		bootloader_send_nack();
		bootloader_uart_write_data(BL_GET_VER, 1);
		bootloader_uart_write_data(" Wrong CRC\n", 11);
	}
}

void bootloader_handle_go_cmd(uint8_t* RxBuffer)
{
	uint32_t go_address = 0;
	void (*ptr_fc)(void);
	if(bootloader_verify_CRC(RxBuffer))
	{
		//CRC is correct
		bootloader_send_ack(bl_rx_buffer[0],1);
		go_address = *(uint32_t*)(RxBuffer + 2);
		if( Addr_valid(go_address) )
		{
			//respond addr is ok
			bootloader_uart_write_data(1, 1);
			//THUMB architect
			ptr_fc = (void*)(go_address + 1);
			//go to addr
			ptr_fc();
		}
		else
		{

			bootloader_uart_write_data(0, 1);
		}

	}
	else
	{
		//CRC is wrong
		bootloader_uart_write_data(" Wrong CRC\n", 11);
		bootloader_send_nack();
	}
}

void bootloader_handle_flash_erase_cmd(uint8_t* RxBuffer)
{
	if(bootloader_verify_CRC(RxBuffer) | inTesting)
	{
		bootloader_uart_write_data("Flash erasing...\n", 16);
		//on led 14 - red
		GPIOD->ODR |= 1<<14;
		Flash_Erase(RxBuffer[2], RxBuffer[3]);
		GPIOD->ODR &= ~(1<<14);
	}
	else
	{
		//CRC is wrong
		bootloader_uart_write_data(" Wrong CRC\n", 11);
		bootloader_send_nack();
	}
}

void bootloader_handle_mem_write_cmd(uint8_t* RxBuffer)
{
	uint8_t payload_len = RxBuffer[6];
	uint32_t mem_address = *((uint32_t *) ( &RxBuffer[2]) );

	if(bootloader_verify_CRC(RxBuffer) | inTesting)
	{
		if( Addr_valid(mem_address) )
		{
			bootloader_uart_write_data("Memory writing...\n", 18);
			//on led 14 - red
			GPIOD->ODR |= 1<<14;
			Flash_Write(mem_address, payload_len, &RxBuffer[7]);
			GPIOD->ODR &= ~(1<<14);
		}
		else
		{

		}
	}
	else
	{
		//CRC is wrong
		bootloader_uart_write_data(" Wrong CRC\n", 11);
		bootloader_send_nack();
	}
}

_Bool bootloader_verify_CRC(uint8_t* RxBuffer)
{
	//get RxBuffer total length
	uint32_t RxBuffer_Length = RxBuffer[0]+1;

	//host CRC value is stored in last 4 byte of RxBuffer
	uint32_t host_CRC = *( (uint32_t *)(RxBuffer + RxBuffer_Length - 4) );

	//calculate RxCRC from RxBuffer, dismiss last 4 byte
	uint32_t RxCRC = CRC_Calculation(RxBuffer, RxBuffer_Length-4);

	return (RxCRC == host_CRC);
}

uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}

_Bool Addr_valid(uint32_t addr)
{
	//so, what are the valid addresses to which we can jump ?
	//can we jump to system memory ? yes
	//can we jump to sram1 memory ?  yes
	//can we jump to sram2 memory ? yes
	//can we jump to backup sram memory ? yes
	//can we jump to peripheral memory ? its possible , but dont allow. so no
	//can we jump to external memory ? yes.
	if(FLASH_BASE <= addr && addr <= FLASH_END) return 1;
	if(SRAM1_BASE <= addr && addr <= SRAM1_END) return 1;
	if(SRAM2_BASE <= addr && addr <= SRAM2_END) return 1;
	if(BKPSRAM_BASE <= addr && addr <= BKPSRAM_END) return 1;
	//can we jump to external memory ? yes...not yet implemented
	return 0;

}

/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
	UART_Transmit(USART2,pBuffer,len);

}

uint32_t CRC_Calculation(uint8_t* pInputCRC, uint32_t u32Length)
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

void resetBuffer(uint8_t* resetBuf, uint8_t len)
{
	for(int i=0; i<len; i++)
	{
		resetBuf[i] = 0;
	}
}

/*This function sends ACK if CRC matches along with "len to follow"*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{

}

void bootloader_send_nack(void)
{

}
