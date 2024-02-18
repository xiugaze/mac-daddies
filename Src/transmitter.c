#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "regs.h"
#include "transmitter.h"
#include "channel_monitor.h"
#include "utils.h"

#define BITS_PER_CHAR 8
#define BIT_RATE 1000
#define HALF_BIT_RATE 2000
#define HALF_BIT_PERIOD 16160/2


char userInput[100];
uint16_t transmissionBuffer[1024];
static int transmission_length = -1;

static volatile TIMX_16* const tim3 = (TIMX_16*)TIM3;
static volatile RCC* const rcc = (RCC*) RCC_BASE;
static volatile GPIOX* const gpioa = (GPIOX*)GPIOA;
static volatile GPIOX* const gpioc = (GPIOX*)GPIOC;
static volatile uint32_t* const nvic_iser = (uint32_t*)NVIC_ISER;

int transmit_halfbits(void);


int transmit(char userInput[], uint8_t dst_addr){


	//Getting rid of the newline char
	int len = strlen(userInput);
	if (len > 0 && userInput[len - 1] == '\n') {
	     userInput[len - 1] = '\0';
	}

	// TODO: variable destination
	Packet* to_send = new_packet(userInput, dst_addr);
	uint8_t packet_buffer[1024];
	int length_bytes = serializePacket(to_send, packet_buffer, 1024);

	int bufferIndex = 0;
	for (int i = 0; i < length_bytes; i++) {

		uint8_t cur = packet_buffer[i];
		int j = (sizeof(char)*8) - 1;
		while(j >= 0) {
			uint16_t to_write = (cur >> j) & 1;
			transmissionBuffer[bufferIndex] = to_write ^ 1;		// first half of Manchester bit
			transmissionBuffer[bufferIndex+1] = to_write ^ 0;   // second half of Manchester bit
			bufferIndex += 2; // advance the pointer twice
			j--;
		}
	}

	transmission_length = length_bytes * 8 * 2;
	return transmit_halfbits();

}


void transmit_init() {

	rcc->AHB1ENR |= (GPIOA_EN | GPIOC_EN); // enable GPIOA

	gpioa->MODER  &= ~(0b11 << 6*2);
	gpioa->MODER  |=  (0b01 << 6*2);	// PA6 is in output mode
	gpioa->OTYPER  |= (1 << 6);
	gpioa->ODR    |=  (1 << 6);

	gpioc->MODER  &= ~(0b11 << 0*2);
	gpioc->MODER  |=  (0b01 << 0*2);

	rcc->APB1ENR |= TIM3_EN;			// enable TIM3

	tim3->CCMR1 &= ~(0b11 << 0);		// clear CC1S bits, tim3_ch1 is in output mode
	//tim3->CCR1 = HALF_BIT_PERIOD;		// interrupt fires on HALF_BIT_PERIOD

	// Configure TOC for retransmit
	tim3->CCMR1 &= ~(0b11 << 8);		// clear CC2S bits, tim4_ch2 is in output mode
	tim3->CCR2 = 0;						// Placeholder


	nvic_iser[0] |= (1 << 29);			// TIM3 global interrupt is vector 29
	tim3->CR1 |= 0b01;					// Start the timer
}

/*
 * TIMER3 IRQ is supposed to iterate through the transmission buffer
 * and write the value to the IDR.
 */
void TIM3_IRQHandler(void) {
	gpioc->BSRR   |=  (1 << 0);		// in the interrupt
	uint32_t status = tim3->SR;
	tim3->SR = 0;

	static int buffer_position = 0;
	static int retransmit_attempts = 0;

	channel_state state = channel_monitor_get_state();


	int transmit_sv = ((status >> 1) & 1) & ((tim3->DIER >> 1) & 1);
	int retransmit_sv = ((status >> 2) & 1) & ((tim3->DIER >> 2) & 1);

	if(transmit_sv) {

		if(state == COLLISION || buffer_position == transmission_length) {

			// If the transmission is over
			tim3->DIER &= ~(0b01 << 1); // disable interrupts

			if(state == COLLISION) {
				if(retransmit_attempts < 10) {
					srand(tim3->CNT);
					tim3->CCR2 = (uint16_t)(tim3->CCR1) + (17600 + (rand() % 17600));
					tim3->DIER |= (0b01 << 2);
					retransmit_attempts++;
				} else {
					retransmit_attempts = 0;
					raise_error(TX_ON_COLLISION);
					printf("Transmission failed: tx on collision");
				}

			} else {
				transmission_length = -1;  	// don't transmit
			}

			buffer_position = 0;	   	// reset the buffer position
			gpioa->ODR |= 1 << 6;		// let the line go high

		} else {
			// Else, we're still transmitting
			tim3->CCR1 += HALF_BIT_PERIOD;  // next interrupt fires last time + 500uS
			gpioa->BSRR = (1 << (6 + 16*(1 - transmissionBuffer[buffer_position++])));
		}
	}

	if(retransmit_sv) {
		tim3->DIER &= ~(0b01 << 2);
		buffer_position = 0;	   	// reset the buffer position
		tim3->DIER |= (0b01 << 1);
	}

	gpioc->BSRR |= (1 << 16);	// out of the interrupt
}

int transmit_halfbits(void) {
	if(transmission_length < 0) {
		return -1;
	}

	if(channel_monitor_get_state() == BUSY) {
		raise_error(TX_ON_BUSY);
	}

	tim3->CCR1 = (tim3->CNT); // trigger on current time + 500uS
	tim3->DIER |= 0b01 << 1;  // enable interrupts on channel 1
	return 0;
}
