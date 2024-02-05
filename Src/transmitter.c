#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "regs.h"
#include "transmitter.h"

#define BITS_PER_CHAR 8
#define BIT_RATE 1000
#define HALF_BIT_RATE 2000
#define HALF_BIT_PERIOD 17600/2


char userInput[100];
//Assuming a maximum of 100 characters, each represented by 2 half-bits
uint16_t transmissionBuffer[200];
static int transmission_length = -1;

static volatile TIMX_16* const tim3 = (TIMX_16*)TIM3;
static volatile RCC* const rcc = (RCC*) RCC_BASE;
static volatile GPIOX* const gpioa = GPIOA;
static volatile uint32_t* const nvic_iser = (uint32_t*)NVIC_ISER;

int get_transmission(void){

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
	for (int i = 0; i < len; i++) {
		char currentChar = userInput[i];
		for (int j = BITS_PER_CHAR - 1; j >= 0; j--) {
			uint16_t to_write = (currentChar >> j) & 1;
			transmissionBuffer[bufferIndex++] = to_write;
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

	// NOTE: not sure if this is necessary either?
	gpioa->IDR 	  |=  (0b01 << 6); 		// line starts high?


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
	tim3->CCR1 += HALF_BIT_PERIOD;  // next interrupt fires last time + 500uS
	static int buffer_position = 0;

	// clear whatever's written
	gpioa->ODR &= ~(0b01 << 6);

	// write the current half-bit to the register
	gpioa->ODR |= (transmissionBuffer[buffer_position++] | 1) << 6;

	// if transmission is done
	if(buffer_position == transmission_length) {
		tim3->DIER &= (0b01 << 1); // disable interrupts
		transmission_length = -1;  // don't transmit
		buffer_position = 0;	   // reset the buffer position
	}
}

int transmit_halfbits(void) {
	if(transmission_length < 0) {
		return -1;
	}

	tim3->CCR1 = (tim3->CNT + HALF_BIT_PERIOD); // trigger on current time + 500uS
	// enable timer3 interrupts in DIER
	tim3->DIER |= 0b01 << 1;   					// enable interrupts on channel 1
	return 0;

}
