#ifndef UTILS_H_
#define UTILS_H_


typedef enum {
	TRANSMISSION_ON_BUSY,
	TRANSMISSION_ON_COLLISION,
} error;


int pair_to_bit(uint8_t pair[]);
int manchester_decode(uint8_t msg[], int len, char decoded[]);
void raise_error(error e);


#endif
