/*
 * rs485_protocol.c
 *
 *  Created on: 14 но€б. 2019 г.
 *      Author: User
 */

#include "rs485_protocol.h"
#include "rs485.h"
#include <string.h>
#include "main.h"

static uint8_t tmp_rx1_buf[UART_BUF_SISE];
static uint8_t tmp_rx2_buf[UART_BUF_SISE];

void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {
	if(rx_cnt<=UART_BUF_SISE) {
		memcpy(tmp_rx1_buf,rx_ptr,rx_cnt);
		send_data_to_uart1(tmp_rx1_buf,rx_cnt);
	}
}

void rx2_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {
	if(rx_cnt<=UART_BUF_SISE) {
		memcpy(tmp_rx2_buf,rx_ptr,rx_cnt);
		send_data_to_uart2(tmp_rx2_buf,rx_cnt);
	}
}
