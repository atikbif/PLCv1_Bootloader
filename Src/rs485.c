/*
 * rs485.c
 *
 *  Created on: 14 но€б. 2019 г.
 *      Author: User
 */

#include "rs485.h"
#include "main.h"

#define UART_RX_READY_TIME_MS		10

uint8_t rx1_buf[UART_BUF_SISE];
uint8_t tx1_buf[UART_BUF_SISE];
uint16_t rx1_cnt = 0;
uint16_t rx1_tmr = 0;
uint8_t dir1_tmr = 0;

uint8_t rx2_buf[UART_BUF_SISE];
uint8_t tx2_buf[UART_BUF_SISE];
uint16_t rx2_cnt = 0;
uint16_t rx2_tmr = 0;
uint8_t dir2_tmr = 0;

__weak void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {

}

__weak void rx2_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {

}

void uart1_scan(void) {

	if(rx1_cnt && rx1_tmr>=UART_RX_READY_TIME_MS) {
		rx1_callback(rx1_buf,rx1_cnt);
		rx1_cnt = 0;
		rx1_tmr = 0;
	}
}

void uart2_scan(void) {

	if(rx2_cnt && rx2_tmr>=UART_RX_READY_TIME_MS) {
		rx2_callback(rx2_buf,rx2_cnt);
		rx2_cnt = 0;
		rx2_tmr = 0;
	}
}

void send_data_to_uart1(uint8_t *ptr, uint16_t cnt) {
	uint16_t i=0;
	for(i=0;i<cnt;i++) tx1_buf[i] = ptr[i];
	HAL_GPIO_WritePin(RS485_DIR1_GPIO_Port,RS485_DIR1_Pin,GPIO_PIN_SET);
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
	LL_USART_DisableIT_RXNE(USART1);
	while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_7));
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7,(uint32_t)tx1_buf);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7,LL_USART_DMA_GetRegAddr(USART1));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, cnt);
	LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_MEDIUM);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);
	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);
	LL_USART_EnableDMAReq_TX(USART1);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}

void write_data_to_uart1(buf* data) {
	send_data_to_uart1(data->ptr,data->cnt);
}

void send_data_to_uart2(uint8_t *ptr, uint16_t cnt) {
	uint16_t i=0;
	for(i=0;i<cnt;i++) tx2_buf[i] = ptr[i];
	HAL_GPIO_WritePin(RS485_DIR2_GPIO_Port,RS485_DIR2_Pin,GPIO_PIN_SET);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
	LL_USART_DisableIT_RXNE(USART2);
	while(LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_6));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6,(uint32_t)tx2_buf);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6,LL_USART_DMA_GetRegAddr(USART2));
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, cnt);
	LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_4);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_MEDIUM);
	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);
	LL_USART_EnableDMAReq_TX(USART2);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
}

void write_data_to_uart2(buf* data) {
	send_data_to_uart2(data->ptr,data->cnt);
}
