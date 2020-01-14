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
#include "eeprom.h"

static uint8_t tmp_rx1_buf[UART_BUF_SISE];
static uint8_t tmp_rx2_buf[UART_BUF_SISE];
static uint8_t tx1_buf[UART_BUF_SISE];
static uint8_t tx2_buf[UART_BUF_SISE];
uint8_t jump_flag = 0;

uint8_t net_address = 0x01;

#define HOLDR_COUNT 128

#define READ_COILS		1
#define READ_DINPUTS	2
#define READ_HOLD_REGS	3
#define READ_INP_REGS	4
#define WR_SINGLE_COIL	5
#define WR_SINGLE_REG	6
#define WR_MULTI_REGS	0x10
#define WR_MULTI_COIL	0x0F

extern uint16_t ai_type;
extern uint8_t ip_addr[4];
extern uint8_t ip_mask[4];
extern uint8_t ip_gate[4];

extern uint16_t VirtAddVarTab[NB_OF_VAR];

static void modbus_error(unsigned char func, unsigned char code, uint8_t * tx_ptr, void (*send)(uint8_t*,uint16_t)) {
	unsigned short crc=0;
	tx_ptr[0] = net_address;
	tx_ptr[1] = func | 0x80;
	tx_ptr[2] = code;
	crc = GetCRC16((unsigned char*)tx_ptr,3);
	tx_ptr[3]=crc>>8;
	tx_ptr[4]=crc&0xFF;
	send(tx_ptr,5);
}

void rx_callback(uint8_t* rx_ptr,uint16_t rx_cnt, uint8_t * tx_ptr, void (*send)(uint8_t*,uint16_t)) {
	uint16_t crc=0;
	uint32_t fl_addr=0;
	uint16_t cnt=0;
	static uint32_t fl_buf[128];
	uint16_t mem_addr = 0;
	uint16_t tmp=0;
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
			case READ_HOLD_REGS:
				mem_addr = ((unsigned short)rx_ptr[2]<<8) | rx_ptr[3];
				cnt = ((unsigned short)rx_ptr[4]<<8) | rx_ptr[5];
				if(cnt>HOLDR_COUNT || cnt==0) {modbus_error(READ_HOLD_REGS,0x03,tx_ptr,send);break;}
				if(mem_addr+cnt>HOLDR_COUNT) {modbus_error(READ_HOLD_REGS,0x02,tx_ptr,send);break;}

				for(tmp=0;tmp<cnt;tmp++) {
					switch(mem_addr+tmp) {
					case 0:
						tx_ptr[3+tmp*2] = ai_type>>8;
						tx_ptr[4+tmp*2] = ai_type&0xFF;
						break;
					case 1:
						tx_ptr[3+tmp*2] = 0;
						tx_ptr[4+tmp*2] = net_address;
						break;
					case 2:
						tx_ptr[3+tmp*2] = ip_addr[0];
						tx_ptr[4+tmp*2] = ip_addr[1];
						break;
					case 3:
						tx_ptr[3+tmp*2] = ip_addr[2];
						tx_ptr[4+tmp*2] = ip_addr[3];
						break;
					case 4:
						tx_ptr[3+tmp*2] = ip_mask[0];
						tx_ptr[4+tmp*2] = ip_mask[1];
						break;
					case 5:
						tx_ptr[3+tmp*2] = ip_mask[2];
						tx_ptr[4+tmp*2] = ip_mask[3];
						break;
					case 6:
						tx_ptr[3+tmp*2] = ip_gate[0];
						tx_ptr[4+tmp*2] = ip_gate[1];
						break;
					case 7:
						tx_ptr[3+tmp*2] = ip_gate[2];
						tx_ptr[4+tmp*2] = ip_gate[3];
						break;
					default:
						tx_ptr[3+tmp*2] = 0;
						tx_ptr[4+tmp*2] = 0;
						break;
					}
				}
				tx_ptr[0]=rx_ptr[0];
				tx_ptr[1]=READ_HOLD_REGS;
				tx_ptr[2]=cnt*2;
				crc=GetCRC16(tx_ptr,3+cnt*2);
				tx_ptr[3+cnt*2]=crc>>8;
				tx_ptr[4+cnt*2]=crc&0xFF;
				send(tx_ptr,5+cnt*2);
				break;
			case WR_SINGLE_REG:
				mem_addr = ((unsigned short)rx_ptr[2]<<8) | rx_ptr[3];
				cnt = ((unsigned short)rx_ptr[4]<<8) | rx_ptr[5];
				if(mem_addr >= HOLDR_COUNT) {modbus_error(WR_SINGLE_REG,0x02,tx_ptr,send);break;}
				switch(mem_addr) {
					case 0:
						ai_type = cnt;
						EE_WriteVariable(VirtAddVarTab[9],ai_type);
						break;
					case 1:
						net_address = cnt;
						EE_WriteVariable(VirtAddVarTab[2],cnt);
						break;
					case 2:
						ip_addr[0] = cnt>>8;
						ip_addr[1] = cnt & 0xFF;
						EE_WriteVariable(VirtAddVarTab[3],cnt);
						break;
					case 3:
						ip_addr[2] = cnt>>8;
						ip_addr[3] = cnt & 0xFF;
						EE_WriteVariable(VirtAddVarTab[4],cnt);
						break;
					case 4:
						ip_mask[0] = cnt>>8;
						ip_mask[1] = cnt & 0xFF;
						EE_WriteVariable(VirtAddVarTab[5],cnt);
						break;
					case 5:
						ip_mask[2] = cnt>>8;
						ip_mask[3] = cnt & 0xFF;
						EE_WriteVariable(VirtAddVarTab[6],cnt);
						break;
					case 6:
						ip_gate[0] = cnt>>8;
						ip_gate[1] = cnt & 0xFF;
						EE_WriteVariable(VirtAddVarTab[7],cnt);
						break;
					case 7:
						ip_gate[2] = cnt>>8;
						ip_gate[3] = cnt & 0xFF;
						EE_WriteVariable(VirtAddVarTab[8],cnt);
						break;
				}

				tx_ptr[0]=rx_ptr[0];
				tx_ptr[1]=WR_SINGLE_REG;
				tx_ptr[2]=mem_addr>>8;
				tx_ptr[3]=mem_addr&0xFF;
				tx_ptr[4]=cnt>>8;
				tx_ptr[5]=cnt&0xFF;
				crc=GetCRC16(tx_ptr,6);
				tx_ptr[6]=crc>>8;
				tx_ptr[7]=crc&0xFF;
				send(tx_ptr,8);
				break;
			case WR_MULTI_REGS:
				mem_addr = ((unsigned short)rx_ptr[2]<<8) | rx_ptr[3];
				cnt = ((unsigned short)rx_ptr[4]<<8) | rx_ptr[5];
				if(cnt>=128 || cnt==0) {modbus_error(WR_MULTI_REGS,0x03,tx_ptr,send);break;}
				if(mem_addr+cnt>HOLDR_COUNT) {modbus_error(WR_MULTI_REGS,0x02,tx_ptr,send);break;}
				for(tmp=0;tmp<cnt;tmp++) {
					switch(mem_addr+tmp) {
						case 0:
							ai_type = rx_ptr[8+tmp*2] | ((unsigned short)rx_ptr[7+tmp*2]<<8);
							EE_WriteVariable(VirtAddVarTab[9],ai_type);
							break;
						case 1:
							net_address = rx_ptr[8+tmp*2] ;
							EE_WriteVariable(VirtAddVarTab[2],cnt);
							break;
						case 2:
							ip_addr[0] = rx_ptr[7+tmp*2];
							ip_addr[1] = rx_ptr[8+tmp*2];
							EE_WriteVariable(VirtAddVarTab[3],cnt);
							break;
						case 3:
							ip_addr[2] = rx_ptr[7+tmp*2];
							ip_addr[3] = rx_ptr[8+tmp*2];
							EE_WriteVariable(VirtAddVarTab[4],cnt);
							break;
						case 4:
							ip_mask[0] = rx_ptr[7+tmp*2];
							ip_mask[1] = rx_ptr[8+tmp*2];
							EE_WriteVariable(VirtAddVarTab[5],cnt);
							break;
						case 5:
							ip_mask[2] = rx_ptr[7+tmp*2];
							ip_mask[3] = rx_ptr[8+tmp*2];
							EE_WriteVariable(VirtAddVarTab[6],cnt);
							break;
						case 6:
							ip_gate[0] = rx_ptr[7+tmp*2];
							ip_gate[1] = rx_ptr[8+tmp*2];
							EE_WriteVariable(VirtAddVarTab[7],cnt);
							break;
						case 7:
							ip_gate[2] = rx_ptr[7+tmp*2];
							ip_gate[3] = rx_ptr[8+tmp*2];
							EE_WriteVariable(VirtAddVarTab[8],cnt);
							break;
					}
				}
				tx_ptr[0]=rx_ptr[0];
				tx_ptr[1]=WR_MULTI_REGS;
				tx_ptr[2]=mem_addr>>8;
				tx_ptr[3]=mem_addr&0xFF;
				tx_ptr[4]=cnt>>8;
				tx_ptr[5]=cnt&0xFF;
				crc=GetCRC16(tx_ptr,6);
				tx_ptr[6]=crc>>8;
				tx_ptr[7]=crc&0xFF;
				send(tx_ptr,8);
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
