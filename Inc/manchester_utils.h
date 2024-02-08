#ifndef MANCHESTER_UTILS_H_
#define MANCHESTER_UTILS_H_



int pair_to_bit(uint8_t pair[]);

int manchester_decode(uint8_t msg[], int len, char decoded[]);

#endif
