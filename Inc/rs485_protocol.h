/*
 * rs485_protocol.h
 *
 *  Created on: 14 но€б. 2019 г.
 *      Author: User
 */

#ifndef RS485_PROTOCOL_H_
#define RS485_PROTOCOL_H_

#include <stdint.h>

void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt);
void rx2_callback(uint8_t* rx_ptr,uint16_t rx_cnt);

#endif /* RS485_PROTOCOL_H_ */
