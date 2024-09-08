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

  while(1)
  {
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
