#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "regs.h"
#include "transmitter.h"
#include "channel_monitor.h"

#define BITS_PER_CHAR 8
#define BIT_RATE 1000
#define HALF_BIT_RATE 2000
#define HALF_BIT_PERIOD 16160/2


char userInput[100];
//Assuming a maximum of 100 characters, each represented by 2 half-bits
// 01110100
uint16_t transmissionBuffer[200];
uint16_t testbuffer[] = {0, 0, 1, 0, 1, 0, 1, 0};
static int transmission_length = -1;
//uint16_t testbuffer[] = {0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0};

static volatile TIMX_16* const tim3 = (TIMX_16*)TIM3;
static volatile RCC* const rcc = (RCC*) RCC_BASE;
static volatile GPIOX* const gpioa = (GPIOX*)GPIOA;
static volatile uint32_t* const nvic_iser = (uint32_t*)NVIC_ISER;


int transmit_halfbits(void);

int get_transmission(void){

	//check if 0 if it is then write 101010101010 a bunch of time set length to whatever and then do an early return

	//Prompt user and grab input
	printf("\nEnter a message: \n");
	fgets(userInput, sizeof(userInput), stdin);

	//Getting rid of the newline char
	int len = strlen(userInput);
	if (len > 0 && userInput[len - 1] == '\n') {
	     userInput[len - 1] = '\0';
	}
	//Encode the message and add it to the transmission buffer
	int bufferIndex = 0;

	char nullstring[] = "";
	if(strcmp(userInput, nullstring)){
		for(int i =0; i < 255; i++){
			uint16_t null_write = 0;
			transmissionBuffer[bufferIndex] = null_write ^ 1;		// first half of Manchester bit
			transmissionBuffer[bufferIndex+1] = null_write ^ 0;   // second half of Manchester bit
			bufferIndex += 2; // advance the pointer twice
		}

		return transmit_halfbits();
	}


	for (int i = 0; i < len; i++) {

		char currentChar = userInput[i];
		int j = (sizeof(char)*8) - 1;
		while(j >= 0) {
			uint16_t to_write = (currentChar >> j) & 1;
			transmissionBuffer[bufferIndex] = to_write ^ 1;		// first half of Manchester bit
			transmissionBuffer[bufferIndex+1] = to_write ^ 0;   // second half of Manchester bit
			bufferIndex += 2; // advance the pointer twice
			j--;
		}
	}

	//transmit_halfbits(transmissionBuffer, strlen(userInput) * BITS_PER_CHAR);

	/*
	 * size of the transmission buffer is the length of the input times the
	 * number of bits in a byte times the number of bits in one baud (Manchester)
	 */
	transmission_length = strlen(userInput) * BITS_PER_CHAR * 2;
	return transmit_halfbits();

}


void transmit_init() {
	rcc->AHB1ENR |= GPIOA_EN;			// enable GPIOA

	// NOTE: do not think this is necessary
//	gpioa->AFRL  |= (0b0010 << 6 * 4); 	// PA6 is AF02 (TIM3_CH1)
//	gpioa->MODER |= (0b10 << 6*2);		// PA6 is in AF mode
	gpioa->MODER  &= ~(0b11 << 6*2);
	gpioa->MODER  |=  (0b01 << 6*2);	// PA6 is in output mode
	gpioa->ODR    |=  (1 << 6);

	// NOTE: not sure if this is necessary either?
	//gpioa->IDR 	  |=  (0b01 << 6); 		// line starts high?


	rcc->APB1ENR |= TIM3_EN;			// enable TIM3

	tim3->CCMR1 &= ~(0b11 << 0);		// clear CC1S bits, tim3_ch1 is in output mode
	//tim3->CCR1 = HALF_BIT_PERIOD;		// interrupt fires on HALF_BIT_PERIOD

	nvic_iser[0] |= (1 << 29);			// TIM3 global interrupt is vector 29
	tim3->CR1 |= 0b01;					// Start the timer
}

/*
 * TIMER3 IRQ is supposed to iterate through the transmission buffer
 * and write the value to the IDR.
 */
void TIM3_IRQHandler(void) {

	tim3->SR=0;

	static int buffer_position = 0;

	if(channel_monitor_get_state() == BUSY || buffer_position == transmission_length) {
	//if(channel_monitor_get_state() == BUSY || buffer_position == 8) {

		tim3->DIER &= ~(0b01 << 1); // disable interrupts
		transmission_length = -1;  	// don't transmit
		buffer_position = 0;	   	// reset the buffer position
		gpioa->ODR |= 1 << 6;		// let the line go high
	}

	tim3->CCR1 += HALF_BIT_PERIOD;  // next interrupt fires last time + 500uS

	// clear whatever's written
	//gpioa->ODR &= ~(0b01 << 6);

	// write the current half-bit to the register
	//gpioa->ODR |= (transmissionBuffer[buffer_position++] | 1) << 6;
	//gpioa->ODR |= (transmissionBuffer[buffer_position++]) << 6;
	gpioa->BSRR = (1 << (6 + 16*(1 - transmissionBuffer[buffer_position++])));
	//gpioa->ODR |= (testbuffer[buffer_position++]) << 6;
}

int transmit_halfbits(void) {
	if(transmission_length < 0) {
		return -1;
	}

	tim3->CCR1 = (tim3->CNT); // trigger on current time + 500uS
	tim3->DIER |= 0b01 << 1;   					// enable interrupts on channel 1
	return 0;
}
