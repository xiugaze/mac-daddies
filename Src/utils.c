/*
 * manchester_utils.c
 *
 *  Created on: Feb 7, 2024
 *      Author: andreanoc
 *
 */


#include <stdint.h>
#include <stdio.h>
#include "utils.h"

int pair_to_bit(uint8_t pair[]) {
    if(pair[0] == 0 && pair[1] == 1) {
        return 1;
    } else if (pair[0] == 1 && pair[1] == 0) {
        return 0;
    } else {
        return -1;
    }
}

int manchester_decode(uint8_t msg[], int len, char decoded[]) {
    // message length must be divisible by 16 baud == 8 bits == 1 byte
    if(len % 16 != 0) {
        return -1;
    }

    // for each point in the buffer
    int decoded_pos = 0;
    int byte_pos = 7;
    char current_char = '\0';
    for(int i = 0; i < len; i += 2) {
        uint8_t pair[2] = { msg[i], msg[i+1] };
        int bit = pair_to_bit(pair);

        if(bit < 0) {
            return -1;
        }

        current_char |= (bit << byte_pos);

        if(byte_pos == 0) {
            byte_pos = 7;
            decoded[decoded_pos++] = current_char;
            current_char = '\0';
        } else {
            byte_pos--;
        }
    }
    decoded[decoded_pos] = '\0'; // append the null terminator
    return 0;
}

void raise_error(error e) {
	switch(e) {
	case TRANSMISSION_ON_COLLISION:
		printf("Error: attempted transmission on collision line\n");
		break;
	case TRANSMISSION_ON_BUSY:
		printf("Warning: attempting transmission on busy line, waiting...\n");
		break;
	}
}

