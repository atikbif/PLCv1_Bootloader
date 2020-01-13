/*
 * flash_interface.h
 *
 *  Created on: 20 дек. 2019 г.
 *      Author: User
 */

#ifndef FLASH_INTERFACE_H_
#define FLASH_INTERFACE_H_

#include "main.h"


HAL_StatusTypeDef erase_sector(uint16_t sect_num);
HAL_StatusTypeDef program_flash(uint32_t addr, uint32_t *ptr, uint16_t cnt);
void start_application();

#endif /* FLASH_INTERFACE_H_ */
