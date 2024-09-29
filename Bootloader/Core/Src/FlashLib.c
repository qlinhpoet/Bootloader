/*
 * FlashLib.c
 *
 *  Created on: Sep 22, 2024
 *      Author: Linh
 */

#include "FlashLib.h"

bool Flash_Write(volatile uint32_t u32StartAddr, uint32_t u32Length, uint8_t *BufferWrite)
{
	// Check that no Flash memory operation is ongoing
	while((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);
	//Flash unlock
	Flash_Unlock();

	// PG bit - flash program activated
	FLASH->CR |= 0x01;
	while(u32Length > 0)
	{
		if(u32Length >= 4)
		{
			/*	program size = 32bit	*/
			FLASH->CR &= ~(3 << 8);
			FLASH->CR |= 2 << 8;

			*(volatile uint32_t*)(u32StartAddr) = *(uint32_t *)(BufferWrite);
			u32StartAddr += 4;
			BufferWrite += 4;
			u32Length -= 4;
		}
		else if(u32Length >= 2)
		{
			/*	program size = 16bit	*/
			FLASH->CR &= ~(3 << 8);
			FLASH->CR |= 1 << 8;

			*(volatile uint16_t*)(u32StartAddr) = *(uint16_t *)(BufferWrite);
			u32StartAddr += 2;
			BufferWrite += 2;
			u32Length -= 2;
		}
		else
		{
			/*	program size = 8bit	*/
			FLASH->CR &= ~(3 << 8);

			*(volatile uint8_t*)(u32StartAddr) = *(uint8_t *)(BufferWrite);
			u32StartAddr += 1;
			BufferWrite += 1;
			u32Length -= 1;
		}
	}

	while((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);
	// clear PG bit
	FLASH->CR &= ~0x01;
	Flash_Lock();
	return Ok;
}

bool Flash_Read(uint32_t SourceAddress, uint8_t * DestAddressPtr, uint32_t Length)
{
	while(Length > 0)
	{
		if(Length >= 4)
		{
			*(volatile uint32_t*)DestAddressPtr = *(volatile uint32_t*)SourceAddress;
			SourceAddress += 4;
			DestAddressPtr += 4;
			Length -= 4;
		}
		else if(Length >= 2)
		{
			*(volatile uint16_t*)DestAddressPtr = *(volatile uint16_t*)SourceAddress;
			SourceAddress += 2;
			DestAddressPtr += 2;
			Length -= 2;
		}
		else if(Length >= 1)
		{
			*(volatile uint8_t*)DestAddressPtr = *(volatile uint8_t*)SourceAddress;
			SourceAddress += 1;
			DestAddressPtr += 1;
			Length -= 1;
		}
	}
}

bool Flash_Erase(uint8_t SectorNum, uint8_t NumofSec)
{

	if( SectorNum == 0xff | SectorNum <= 11 )
	{
		if(SectorNum == 0xff)
		{
			//erase whole flash
			Flash_MassErase();
		}
		else
		{
			if(SectorNum + NumofSec > 12)
			{
				//max remaning sector to erase
				NumofSec = 12 - SectorNum;
			}
			for(int i=0; i<NumofSec; i++)
			{
				Flash_SectorErase(SectorNum + i, 0xffffffff);
			}
		}
	}
}

bool Flash_MassErase(void)
{
	uint32_t u32TimeOut = 0xffffffff;
	// 1.Check that no Flash memory operation is ongoing
	while(((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) && (u32TimeOut > 0U))
	{
		/*  Wating for Bsy bit */
		u32TimeOut --;
		if (u32TimeOut == 0)
		{
			//return FLASH_ERRORS_TIMEOUT;
			return notOk;
		}
	}
	// Unlock flash
	(void)Flash_Unlock();
	// 2. MER bit - Erase activated for all user sectors
	FLASH->CR |= 1<<2;
	// 3.Start bit - trigger erase operation
	FLASH->CR |= 1 << 16;
	// 4.Wait for the BSY bit to be cleared
	while((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);
	// 5. clear MER bit
	FLASH->CR &= ~(1<<2);
	return Ok;
}

bool Flash_SectorErase(uint8_t SectorNum, uint32_t u32TimeOut)
{
	// 1.Check that no Flash memory operation is ongoing
	while(((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) && (u32TimeOut > 0U))
	{
		/*  Wating for Bsy bit */
		u32TimeOut --;
		if (u32TimeOut == 0)
		{
			//return FLASH_ERRORS_TIMEOUT;
			return notOk;
		}
	}
	// Unlock flash
	(void)Flash_Unlock();
	// 2. SER bit - Sector erase activated
	FLASH->CR |= 1 << 1;
	// 2. SNB bit - Select sector number to erase
	FLASH->CR &= ~(0x1F << 3);
	FLASH->CR |= SectorNum << 3;
	// 3.Start bit - trigger erase operation
	FLASH->CR |= 1 << 16;
	// 4.Wait for the BSY bit to be cleared
	while((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);
	// 5.Clear SER, SNB bit after erase operation
	FLASH->CR &= ~(1<<1);
	FLASH->CR &= ~(0x1F << 3);
	Flash_Lock();
	return Ok;
}

bool Flash_Unlock(void)
{
	if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
	{
		/* Authorize the FLASH Registers access */
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
		if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
		{
		 return notOk;
		}
		return Ok;
	}
}

bool Flash_Lock(void)
{
	FLASH->CR |= 1<<31;
}


bool Flash_OPT_Unlock(void)
{
	//check OPT Lock bit
	if(FLASH->OPTCR & 0x01 == 0x01)
	{
		/* Authorize the FLASH Registers access */
		FLASH->OPTKEYR = 0x08192A3BU;
		FLASH->OPTKEYR = 0x4C5D6E7FU;
		if(FLASH->OPTCR & 0x01 != RESET)
		{
		 return notOk;
		}
		return Ok;
	}
}

bool Flash_OPT_lock(void)
{
	FLASH->OPTCR |= 0x01;
}

/*
1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
FLASH_SR register
2. Write the desired option value in the FLASH_OPTCR register.
3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
4. Wait for the BSY bit to be cleared
*/
void Flash_RW_Protect(uint16_t SectorDetail)
{
	(void)Flash_OPT_Unlock();
	uint32_t u32TimeOut = 0xffffffff;
	// 1.Check that no Flash memory operation is ongoing
	while(((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) && (u32TimeOut > 0U))
	{
		/*  Wating for Bsy bit */
		u32TimeOut --;
		if (u32TimeOut == 0)
		{
			//return FLASH_ERRORS_TIMEOUT;
			return notOk;
		}
	}
	FLASH->OPTCR &= ~(0xffff << 16);
	FLASH->OPTCR |= ((SectorDetail & 0x0fff) << 16);
	FLASH->OPTCR |= 1<<1;
	while((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY);
	(void)Flash_OPT_Unlock();
}

void Flash_Read_OptByte(uint32_t* pOpt1Value, uint32_t* pOpt2Value)
{
	*pOpt1Value = *((uint16_t*) 0x1fffc000);
	*pOpt2Value = *((uint16_t*) 0x1fffc008);
}

