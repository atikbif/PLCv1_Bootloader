/*
 * rs485_protocol.c
 *
 *  Created on: 14 нояб. 2019 г.
 *      Author: User
 */

#include "rs485_protocol.h"
#include "rs485.h"
#include <string.h>
#include "main.h"
#include "crc.h"
#include "flash_interface.h"

static uint8_t tmp_rx1_buf[UART_BUF_SISE];
static uint8_t tmp_rx2_buf[UART_BUF_SISE];
static uint8_t tx1_buf[UART_BUF_SISE];
static uint8_t tx2_buf[UART_BUF_SISE];
uint8_t jump_flag = 0;

uint8_t net_address = 0x01;

void rx_callback(uint8_t* rx_ptr,uint16_t rx_cnt, uint8_t * tx_ptr, void (*send)(uint8_t*,uint16_t)) {
	uint16_t crc=0;
	uint32_t fl_addr=0;
	uint16_t cnt=0;
	static uint32_t fl_buf[128];
	if((rx_cnt>=4) && ((rx_ptr[0]==net_address) || (rx_ptr[0]==0x00)) && (GetCRC16(rx_ptr,rx_cnt)==0)) {
		switch(rx_ptr[1]) {
			case 0xA0:
				tx_ptr[0] = rx_ptr[0];
				tx_ptr[1] = 0xA0;
				// request id
				tx_ptr[2] = rx_ptr[2];
				tx_ptr[3] = rx_ptr[3];
				// device id
				tx_ptr[4] = 0x00;
				tx_ptr[5] = 0x00;
				tx_ptr[6] = 0x00;
				tx_ptr[7] = 0x05;

				crc = GetCRC16((unsigned char*)tx_ptr,8);
				tx_ptr[8]=crc>>8;
				tx_ptr[9]=crc&0xFF;
				send(tx_ptr,10);
				break;
			case 0xE8:
				if(rx_ptr[4]>=0 && rx_ptr[4]<=5) {
					tx_ptr[0] = rx_ptr[0];
					tx_ptr[1] = 0xE8;
					// request id
					tx_ptr[2] = rx_ptr[2];
					tx_ptr[3] = rx_ptr[3];
					if(erase_sector(rx_ptr[4])==HAL_OK) {
						tx_ptr[4] = 0x01;
						crc = GetCRC16((unsigned char*)tx_ptr,5);
						tx_ptr[5]=crc>>8;
						tx_ptr[6]=crc&0xFF;
						send(tx_ptr,7);
					}else {
						tx_ptr[4] = 0x00;
						crc = GetCRC16((unsigned char*)tx_ptr,5);
						tx_ptr[5]=crc>>8;
						tx_ptr[6]=crc&0xFF;
						send(tx_ptr,7);
					}
				}
				break;
			case 0xE9:
				fl_addr = rx_ptr[4];
				fl_addr<<=8;fl_addr|=rx_ptr[5];
				fl_addr<<=8;fl_addr|=rx_ptr[6];
				fl_addr<<=8;fl_addr|=rx_ptr[7];
				cnt = rx_ptr[8];
				cnt<<=8;cnt|=rx_ptr[9];
				tx_ptr[0] = rx_ptr[0];
				tx_ptr[1] = 0xE9;
				// request id
				tx_ptr[2] = rx_ptr[2];
				tx_ptr[3] = rx_ptr[3];
				if((cnt<=512)&&(cnt%4==0))
				{
					memcpy(fl_buf,&rx_ptr[10],cnt);
					if(program_flash(fl_addr,fl_buf,cnt/4)==HAL_OK)
					{
						tx_ptr[4] = 0x01;
						crc = GetCRC16((unsigned char*)tx_ptr,5);
						tx_ptr[5]=crc>>8;
						tx_ptr[6]=crc&0xFF;
						send(tx_ptr,7);
					}else {
						tx_ptr[4] = 0x00;
						crc = GetCRC16((unsigned char*)tx_ptr,5);
						tx_ptr[5]=crc>>8;
						tx_ptr[6]=crc&0xFF;
						send(tx_ptr,7);
					}
				}else {
					tx_ptr[4] = 0x02;
					crc = GetCRC16((unsigned char*)tx_ptr,5);
					tx_ptr[5]=crc>>8;
					tx_ptr[6]=crc&0xFF;
					send(tx_ptr,7);
				}
				break;
			case 0xEA: // jump to the loaded program
				tx_ptr[0] = rx_ptr[0];
				tx_ptr[2] = 0xEA;
				// request id
				tx_ptr[2] = rx_ptr[2];
				tx_ptr[3] = rx_ptr[3];
				tx_ptr[4] = 0x01;
				crc = GetCRC16((unsigned char*)tx_ptr,5);
				tx_ptr[5]=crc>>8;
				tx_ptr[6]=crc&0xFF;
				send(tx_ptr,7);
				jump_flag = 1;
				break;
			case 0xEB:	// вернуть число секторов для стирания
				tx_ptr[0] = rx_ptr[0];
				tx_ptr[1] = 0xEB;
				// request id
				tx_ptr[2] = rx_ptr[2];
				tx_ptr[3] = rx_ptr[3];
				cnt = rx_ptr[4];
				cnt = cnt<<8; cnt|=rx_ptr[5]; // размер программы в килобайтах
				if(cnt%128!=0) cnt+=128;
				cnt=cnt/128;
				tx_ptr[4] = cnt>>8;
				tx_ptr[5] = cnt & 0xFF;
				crc = GetCRC16((unsigned char*)tx_ptr,6);
				tx_ptr[6]=crc>>8;
				tx_ptr[7]=crc&0xFF;
				send(tx_ptr,8);
				break;
			case 0xEC:// restart in boot mode
				tx_ptr[0] = rx_ptr[0];
				tx_ptr[1] = 0xEC;
				// request id
				tx_ptr[2] = rx_ptr[2];
				tx_ptr[3] = rx_ptr[3];
				crc = GetCRC16((unsigned char*)tx_ptr,4);
				tx_ptr[4]=crc>>8;
				tx_ptr[5]=crc&0xFF;
				send(tx_ptr,6);
				break;
			default:
				tx_ptr[0] = rx_ptr[0];
				tx_ptr[1] = 0xA0;
				// request id
				tx_ptr[2] = rx_ptr[2];
				tx_ptr[3] = rx_ptr[3];
				// device id
				tx_ptr[4] = 0x04;
				tx_ptr[5] = 0x00;
				tx_ptr[6] = 0x00;
				tx_ptr[7] = 0x01;

				crc = GetCRC16((unsigned char*)tx_ptr,8);
				tx_ptr[8]=crc>>8;
				tx_ptr[9]=crc&0xFF;
				send(tx_ptr,10);
		}
	}
}

void rx1_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {
	if(rx_cnt<=UART_BUF_SISE) {
		memcpy(tmp_rx1_buf,rx_ptr,rx_cnt);
		rx_callback(tmp_rx1_buf, rx_cnt, tx1_buf, send_data_to_uart1);
	}
}

void rx2_callback(uint8_t* rx_ptr,uint16_t rx_cnt) {
	if(rx_cnt<=UART_BUF_SISE) {
		memcpy(tmp_rx2_buf,rx_ptr,rx_cnt);
		rx_callback(tmp_rx2_buf, rx_cnt, tx2_buf, send_data_to_uart2);
	}
}
