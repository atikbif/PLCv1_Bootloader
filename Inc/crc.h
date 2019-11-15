#ifndef CRC_H_
#define CRC_H_

unsigned int GetCRC16 (unsigned char* ptr,unsigned int cnt);
unsigned char CheckLRC(unsigned char* ptr,unsigned short lng);
unsigned char getLRC(unsigned char* ptr,unsigned short lng);

#endif /* CRC_H_ */
