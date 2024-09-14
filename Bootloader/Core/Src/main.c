#include "main.h"
#include "Lib.h"


uint8_t RxData = 0;
uint8_t TxData[10] = "QuangLinh\n";
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  USART2_Init();		//uart init

  while (1)
  {
	  UART_Transmit(USART2, TxData, sizeof(TxData));

	  if( (USART2->SR & (0x0020)) == (0x0020) )		//if data is received, clear bit by a read to USART_DR reg
	  {
		  RxData = UART_Receive(USART2);
		  UART_Transmit1byte(USART2, RxData);
		  UART_Transmit1byte(USART2, '\n');
	  }

	  BlinkLed(500);
	  if( (GPIOA->IDR & 0x01) == 0x01)		//button pressed
	  {
		  GPIOD->ODR |= 1<<15;
		  bootloader_uart_read_data();
	  }
	  else
	  {
		  GPIOD->ODR &= ~1<<15;
		  bootloader_jump_to_user_app();
	  }
  }

}




#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
