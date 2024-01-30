#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "transmitter.h"

#define BITS_PER_CHAR 8
#define BIT_RATE 1000
#define HALF_BIT_RATE 2000

char userInput[100];
//Assuming a maximum of 100 characters, each represented by 2 half-bits
uint16_t transmissionBuffer[200];


void get_transmission(){

	//Prompt user and grab input
	printf("\nEnter a message: ");
	fgets(userInput, sizeof(userInput), stdin);

	//Getting rid of the newline char
	int len = strlen(input);
	if (len > 0 && input[len - 1] == '\n') {
	     input[len - 1] = '\0';
	}

	//Encode the message and add it to the transmission buffer
	int bufferIndex = 0;
	for (int i = 0; i < len; i++) {
		char currentChar = userInput[i];
		for (int j = BITS_PER_CHAR - 1; j >= 0; j--) {
			transmissionBuffer[bufferIndex++] = (currentChar >> j) & 1;
		}
	}

	transmit_halfbits(transmissionBuffer, strlen(userInput) * BITS_PER_CHAR);
}



void transmit_halfbits(uint16_t buf[], int length){

}
