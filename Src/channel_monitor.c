/*
 * channel_monitor.c
 *
 *  Created on: Jan 23, 2024
 *      Author: andreanoc
 */

// PB6 and PD12 on AF2


//message is over when idle but packet is the length field +preamble +dest+ add+cr +..


#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <utils.h>
#include "regs.h"
#include "channel_monitor.h"

#define CYCLES_1_1_MS 17600

static volatile RCC *const rcc = (RCC*)RCC_BASE;
static volatile GPIOX *const gpiob = (GPIOX*) GPIOB;
static volatile GPIOX *const gpioa = (GPIOX*) GPIOA;
static volatile GPIOX *const gpioc = (GPIOX*) GPIOC;
static volatile TIMX_16 *tim4 = (TIMX_16*) TIM4;
static volatile uint32_t* const nvic_iser = (uint32_t*)NVIC_ISER;


void ld2_init();
void ld2_toggle();
void channel_monitor_init();
void tim4_init();

void monitor_led_init();
void monitor_led_set(channel_state);

static channel_state state = IDLE;

channel_state channel_monitor_get_state(void) {
	return state;
}


static int receiving = 0;
static int recv_sem = 1;
static int recv_buffer_pos = 1;
static int recv_buffer_size = 0;
static uint8_t recv_buffer[1024];


int recv_set(void) {
	receiving = 1;
	return receiving;
}

int recv_clear(void) {
	receiving = 0;
	return receiving;
}

int recv_status(void) {
	return receiving;
}

void recv_wait() {
	while(recv_sem != 1) {};
	recv_sem = 0;
}

void recv_post() {
	recv_sem = 1;
}

void recv_decode() {
	recv_wait();
	Packet* received = manchester_decode(recv_buffer, recv_buffer_size);
	printf("received: %s from %x\n", received->message, received->header.source_address);
	recv_post();
}

/*
 * TODO:
 * - Implement fudge factor (error%) for 1.1ms
 * - Implement channel monitor leds
 */
void channel_monitor_init(void) {
	monitor_led_init();
	tim4_init();
}

// channel 1: tic
// channel 2: toc
// channel 3: recv oversample
static uint8_t line_state = 0;
void tim4_init(void) {

	/* PB6 is the input pin for TIC on TIM4_CH1 */
	rcc->AHB1ENR |= GPIOB_EN | GPIOC_EN;			// enable GPIOB in RCC
	gpiob->AFRL  |= (0b0010 << 6 * 4); 	// PB6 is AF02 (TIM4_CH1)
	gpiob->MODER |= (0b10 << 6*2);

	gpioc->MODER  &= ~(0b11 << 1*2);
	gpioc->MODER  |=  (0b01 << 1*2);

	/* TIM4 setup */
	rcc->APB1ENR |= TIM4_EN;			// enable TIM4 in RCC

	/* configure TIM4_CH1 as TIC */
	tim4->CCMR1  &= ~(0b11 << 0);		// clear CC1S bits
	tim4->CCMR1  |=  (0b01 << 0);		// tim4_ch1 is in input capture mode

	tim4->CCER &=  ~(1 << 3 | 1 << 1);	// clear edge config, set to rising
	tim4->CCER |=   (1 << 3 | 1 << 1); 	// trigger TIC on either edge

	/* configure TIM4_CH2 as TOC (just free running with an interrupt */
	tim4->CCMR1 &= ~(0b11 << 8);		// clear CC2S bits, tim4_ch2 is in output mode
	tim4->CCR2 = CYCLES_1_1_MS;			// Timeout interrupt fires

	/* configure TIM4_CH3 as TOC for recv sampling */
	tim4->CCMR2 &= ~(0b11 << 0);       // clear CCS3 bits, timer4_ch3 is in output mode
	tim4->CCR3 = CYCLES_1_1_MS; 		// Placeholder, will change before the interrupt is enabled

//	CC1 channel configured as input:
//	This bit determines if a capture of the counter value can actually be done into the input
//	capture/compare register 1 (TIMx_CCR1) or not.
//	0: Capture disabled
//	1: Capture enabled
	tim4->CCER |= (0b01 << 0); // enable input capture
	tim4->DIER |= 0b11 << 1;   // enable TIC interrupts

	nvic_iser[0] |= (1 << 30); // TIM4 global interrupt is in NVIC_ISER0[30]
	tim4->DIER |= (1 << 2);    // Enable timeout interrupt
	tim4->CR1  |= 1; 		   // start the timer

	int i = 1000;
	while(i > 0) { i--; }

    line_state = (gpiob->IDR >> 6) & 0b01;
	monitor_led_set(line_state ? IDLE:COLLISION);

}


void TIM4_IRQHandler(void) {
	gpioc->BSRR    |=  (1 << 1);
	state = IDLE;

	recv_buffer[0] = 1;				// NOTE: hardcoding the first bit as 0,
									// so first symbol is 1 (first recv symbol is falling edge)
	int status = tim4->SR;

	// Timeout TOC Interrupt needs service
	int timout_sv = ((status >> 2) & 0b01) & ((tim4->DIER >> 2) & 1); //
	// TIC interrupt needs service
	int edge_sv = ((status >> 1) & 0b01) & ((tim4->DIER >> 1) & 1);
	// Recv Sample TOC Interrupt needs service
	int oversample_sv = ((status >> 3) & 0b01) & ((tim4->DIER >> 3) & 1);

	// Service the timeout
	if(timout_sv) {
		tim4->DIER &= ~(1 << 2); 	// disable TOC interrupts and
		tim4->DIER &= ~(1 << 3);	// disable sampling interrupts

		if(line_state) {
			state = IDLE;
		} else {
			state = COLLISION;
		}

		if(recv_status()) {
			recv_clear();
			recv_post();
			recv_buffer_size = recv_buffer_pos - (recv_buffer_pos % 16);
			recv_buffer_pos = 1; //possibly move out of loop
		}
	}

	// Service the edge
	if(edge_sv) {
		uint16_t punch = tim4->CCR1;
		line_state = (gpiob->IDR >> 6) & 0b01;		//what line level is at on edge
		tim4->CCR2 = punch + CYCLES_1_1_MS;	//rest to time where edge + 1.1
		tim4->DIER |= (1 << 2);
		state = BUSY;

		// if we're receiving;
		if(recv_status()) {
			tim4->CCR3 = punch + 9000;	// longer than a bit period + change
			tim4->DIER |= (1 << 3); 	// enable ch3 interrupts (oversample)

			recv_buffer[recv_buffer_pos] = line_state;
			recv_buffer_pos++;
		}
	}

	// sample while receiving
	if(oversample_sv && recv_status()) {
		line_state = (gpiob->IDR >> 6) & 0b01;
		recv_buffer[recv_buffer_pos] = line_state;
		recv_buffer_pos++;
	}

	monitor_led_set(state);
	tim4->SR = 0;
	gpioc->BSRR    |=  (1 << 17);
}

void monitor_led_init() {
	rcc->AHB1ENR |= GPIOA_EN;
	gpioa->MODER &= ~(0b1100001111);
    gpioa->MODER |= (0b01 << 0); // setting GPIOA_PIN_0 as output (Green LED)
    gpioa->MODER |= (0b01 << 2); // setting GPIOA_PIN_1 as output (Red LED)
    gpioa->MODER |= (0b01 << 8); // setting GPIOA_PIN_4 as output (Yellow LED)
}

void monitor_led_set(channel_state state) {

	gpioa->ODR &= ~(0b10011);
	switch(state){
	case IDLE:
		gpioa->ODR |= 1 << 0;
		break;
	case BUSY:
	    gpioa->ODR |= 1 << 1;
	    break;
	case COLLISION:
		gpioa->ODR |= 1 << 4;
		break;
	}
}





