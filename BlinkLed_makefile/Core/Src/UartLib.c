/*
 * UartLib.c
 *
 *  Created on: Sep 22, 2024
 *      Author: Linh
 */
#include "UartLib.h"


typedef struct
{
    uint8_t USART_Mode;
    uint32_t USART_Baud;
    uint8_t USART_NumOfStopBits;
    uint8_t USART_WordLength;
    uint8_t USART_ParityControl;
    uint8_t USART_HWFlowControl;
}USART_Config_t;

void UART_Init(USART_TypeDef* pUSART)
{
	GPIO_USART2_Init();
	//0. USART2 clock enable - RCC select HSI, AHB div1, APB div1 -> USART clk = 16Mhz
	RCC->APB1ENR |= 1<<17;
	//1. USART enable
	pUSART->CR1 |= 1<<13;
	//2. define word length 8 data bits
	pUSART->CR1 &= ~(1<<12);
	//3. number of stop bits = 1
	pUSART->CR2 &= ~(3<<12);
	//4. no DMA
	pUSART->CR3 &= ~(1<<7);
	//5. baudrate 9600
	pUSART->CR1 &= ~(1<<15);	//over8 = 0
	pUSART->BRR = 0x8B;

	//non parity
	pUSART->CR1 &= ~(1<<10);
	//TX enable
	pUSART->CR1 |= 1<<3;
	//RX enable
	pUSART->CR1 |= 1<<2;
	/*00 - hardware flow control disabled*/
	pUSART->CR3 &= ~(3<<8);

	//RXNEIE: RXNE interrupt enable
	pUSART->CR1 |= 1<<5;
	NVIC->ISER[1] |= 1<<6;
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

void GPIO_USART2_Init(void)
{
	/*	PA2: USART2_TX - PA3: USART2_RX	*/
	//enable clock ioPort A
	RCC->AHB1ENR |= 0x01;
	//PA2, PA3 Alternate function
	GPIOA->MODER &= ~0xf0;
	GPIOA->MODER |= 0xA0;
	//PA2, PA3 high output speed
	GPIOA->OSPEEDR &= ~0xf0;
	GPIOA->OSPEEDR |= 0xA0;
	//PA2, PA3 AF7
	GPIOA->AFR[0] &= ~0xff00;
	GPIOA->AFR[0] |= 0x7700;
}

