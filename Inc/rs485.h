/*
 * rs485.h
 *
 *  Created on: 14 но€б. 2019 г.
 *      Author: User
 */

#ifndef RS485_H_
#define RS485_H_

#include <stdint.h>

#define UART_BUF_SISE	512

typedef struct
{
    unsigned char* ptr;     // data pointer
    unsigned short cnt;     // buf length
}buf;

void uart1_scan(void);
void uart2_scan(void);
void send_data_to_uart1(uint8_t *ptr, uint16_t cnt);
void send_data_to_uart2(uint8_t *ptr, uint16_t cnt);
void write_data_to_uart1(buf* data);
void write_data_to_uart2(buf* data);

#endif /* RS485_H_ */
