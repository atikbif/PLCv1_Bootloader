/*
 * flash_interface.c
 *
 *  Created on: 20 дек. 2019 г.
 *      Author: User
 */

#include "flash_interface.h"

static FLASH_EraseInitTypeDef eraseInitStruct;
static const uint16_t first_sector_num = 6;
static uint32_t sector_error = 0;

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

#define FLASH_USER_START_ADDR	((uint32_t)0x8040000)

HAL_StatusTypeDef erase_sector(uint16_t sect_num) {
	eraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	eraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	eraseInitStruct.Sector        = sect_num + first_sector_num;
	eraseInitStruct.NbSectors     = 1;
	return HAL_FLASHEx_Erase(&eraseInitStruct, &sector_error);
}

HAL_StatusTypeDef program_flash(uint32_t addr, uint32_t *ptr, uint16_t cnt) {
	uint16_t i = 0;
	HAL_StatusTypeDef res;
	addr +=  FLASH_USER_START_ADDR;
	while (i < cnt)
	{
		res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *ptr);
	    if (res == HAL_OK)
	    {
	    	addr = addr + 4;
	    	ptr++;i++;
	    }
	   else return res;
	}
	return HAL_OK;
}

void start_application() {
	__disable_irq();
	JumpAddress = *(__IO uint32_t*) (FLASH_USER_START_ADDR + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	__set_MSP(*(__IO uint32_t*) FLASH_USER_START_ADDR);
	Jump_To_Application();
}
