/*
 * FlashLib.h
 *
 *  Created on: Sep 22, 2024
 *      Author: Linh
 */

#ifndef INC_FLASHLIB_H_
#define INC_FLASHLIB_H_

#include "stm32f4xx_hal.h"

typedef enum
{
	notOk = 0,
	Ok
} bool;
bool Flash_Write(volatile uint32_t u32StartAddr, uint32_t u32Length, uint8_t *BufferWrite);
bool Flash_Read(uint32_t SourceAddress, uint8_t * TargetAddressPtr, uint32_t Length);
bool Flash_Erase(uint8_t SectorNum, uint8_t NumofSec);
bool Flash_MassErase(void);
bool Flash_SectorErase(uint8_t SectorNum, uint32_t u32TimeOut);
bool Flash_Unlock(void);
bool Flash_Lock(void);

#endif /* INC_FLASHLIB_H_ */
