/*
 * Lib.h
 *
 *  Created on: Sep 6, 2024
 *      Author: Linh
 */

#ifndef INC_LIB_H_
#define INC_LIB_H_
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void USART2_Init();
void UART_Transmit1byte(USART_TypeDef* huart, uint8_t data);
void UART_Transmit(USART_TypeDef* huart, uint8_t* data, uint32_t datasize);
uint8_t UART_Receive(USART_TypeDef* huart);
void BlinkLed(uint32_t msecond);
void Error_Handler(void);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);

void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);
#endif /* INC_LIB_H_ */
