/*
 * UartLib.h
 *
 *  Created on: Sep 22, 2024
 *      Author: Linh
 */

#ifndef INC_UARTLIB_H_
#define INC_UARTLIB_H_


#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"

void UART_Init(USART_TypeDef* pUSART);
void GPIO_USART2_Init(void);
void UART_Transmit(USART_TypeDef* huart, uint8_t* data, uint32_t datasize);
void UART_Receive(USART_TypeDef* huart, uint8_t* RxBuffer, uint8_t RxSize);
void UART_Transmit1byte(USART_TypeDef* huart, uint8_t data);
#endif /* INC_UARTLIB_H_ */
