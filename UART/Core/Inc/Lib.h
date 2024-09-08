/*
 * Lib.h
 *
 *  Created on: Sep 6, 2024
 *      Author: Linh
 */

#ifndef INC_LIB_H_
#define INC_LIB_H_
#include <stdint.h>
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void USART2_Init();
void UART_Transmit1byte(USART_TypeDef* huart, uint8_t data);
void UART_Transmit(USART_TypeDef* huart, uint8_t* data, uint32_t datasize);
uint8_t UART_Receive(USART_TypeDef* huart);
void BlinkLed(uint32_t msecond);
void Error_Handler(void);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);

void bootloader_jump_to_user_app(void);
void bootloader_uart_read_data(void);

// our bootloader commands

//#define  <command name >	<command_code>

//This command is used to read the bootloader version from the MCU
#define BL_GET_VER				0x51

//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP				0x52

//This command is used to read the MCU chip identification number
#define BL_GET_CID				0x53

//This command is used to read the FLASH Read Protection level.
#define BL_GET_RDP_STATUS		0x54

//This command is used to jump bootloader to specified address.
#define BL_GO_TO_ADDR			0x55

//This command is used to mass erase or sector erase of the user flash .
#define BL_FLASH_ERASE          0x56

//This command is used to write data in to different memories of the MCU
#define BL_MEM_WRITE			0x57

//This command is used to enable or disable read/write protect on different sectors of the user flash .
#define BL_EN_RW_PROTECT		0x58

//This command is used to read data from different memories of the microcontroller.
#define BL_MEM_READ				0x59

//This command is used to read all the sector protection status.
#define BL_READ_SECTOR_P_STATUS	0x5A


//This command is used to read the OTP contents.
#define BL_OTP_READ				0x5B


//This command is used disable all sector read/write protection
#define BL_DIS_R_W_PROTECT				0x5C
#endif /* INC_LIB_H_ */
