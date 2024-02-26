/*
 * crc8.h
 *
 *  Created on: Feb 26, 2024
 *      Author: andreanoc
 */

#ifndef CRC8_H_
#define CRC8_H_

uint8_t check_crc(uint8_t recv_crc, char* data, int len);
uint8_t calculate_crc(char* data, int len);

#endif /* CRC8_H_ */
