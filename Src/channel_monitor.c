/*
 * channel_monitor.c
 *
 *  Created on: Jan 23, 2024
 *      Author: andreanoc
 */

// PB6 and PD12 on AF2

#include <stdint.h>
#include "regs.h"
#include "channel_monitor.h";

#define CYCLES_1_1_MS 17600

static volatile RCC *const rcc = (RCC*)RCC_BASE;
static volatile GPIOX *const gpiob = (GPIOX*) GPIOB;
static volatile GPIOX *const gpioa = (GPIOX*) GPIOA;
static volatile TIMX_16 *tim4 = (TIMX_16*) TIM4;
static volatile uint32_t* const nvic_iser = (uint32_t*)NVIC_ISER;

/*
 * Here's what's gotta happen
 * - TIM4 is gonna fire off every single edge. When an edge happens,
 *   we're going to stop, restart, and start TIM6.
 * - TIM6 will get started by TIM4_IRQ.
 *
 *
 */

void ld2_init();
void ld2_toggle();
void channel_monitor_init();
void tim4_init();

void monitor_led_init();
void monitor_led_set(channel_state);
/*
 * TODO:
 * - Implement fudge factor (error%) for 1.1ms
 * - Implement channel monitor leds
 */
void channel_monitor_init(void) {
	ld2_init();
	tim4_init();
}

// channel 1: tic
// channel 2: toc
void tim4_init(void) {

	/* PB6 is the input pin for TIC on TIM4_CH1 */
	rcc->AHB1ENR |= GPIOB_EN;			// enable GPIOB in RCC
	gpiob->AFRL  |= (0b0010 << 6 * 4); 	// PB6 is AF02 (TIM4_CH1)
	gpiob->MODER |= (0b10 << 6*2);

	/* TIM4 setup */
	rcc->APB1ENR |= TIM4_EN;			// enable TIM4 in RCC

	/* configure TIM4_CH1 as TIC */
	tim4->CCMR1  &= ~(0b11 << 0);		// clear CC1S bits
	tim4->CCMR1  |=  (0b01 << 0);		// tim4_ch1 is in input capture mode

	tim4->CCER &=  ~(1 << 3 | 1 << 1);	// clear edge config, set to rising
	tim4->CCER |=   (1 << 3 | 1 << 1); 	// trigger TIC on either edge

	/* configure TIM4_CH2 as TOC (just free running with an interrupt */
	tim4->CCMR1 &= ~(0b11 << 8);		// clear CC2S bits, tim4_ch2 is in output mode


	tim4->CCR2 = CYCLES_1_1_MS;

//	CC1 channel configured as input:
//	This bit determines if a capture of the counter value can actually be done into the input
//	capture/compare register 1 (TIMx_CCR1) or not.
//	0: Capture disabled
//	1: Capture enabled
	tim4->CCER |= (0b01 << 0); // enable input capture
	tim4->DIER |= 0b11 << 1;   // enable TIC interrupts

	nvic_iser[0] |= (1 << 30); // TIM4 global interrupt is in NVIC_ISER0[30]
	tim4->CR1 |= 1; // start the timer
}

void TIM4_IRQHandler(void) {
	static channel_state state = IDLE;
	tim4->DIER &= ~(0b11 << 1);  // reset CC1IE (Interrupt enable)
	/* interrupt body */


	uint32_t status = tim4->SR;				// read the status register
	tim4->SR = 0; 	        				// clear all flags
	uint32_t tic = (status >> 1) & 0b01; // 1 if from tic
	uint32_t toc = (status >> 2) & 0b01; // 1 if from toc

	if(tic) {
		tim4->CNT = 0;
		state = BUSY;
		toggle_user_led();
	} else if(toc) {
		int line_state = (gpiob->IDR >> 6) & 0b01;
		if(line_state) {
			state = IDLE;
		} else {
			state = COLLISION;
		}
	}

	set_monitor_led(state);
	tim4->DIER |= 0b11 << 1;  // enable interrupts
}

void ld2_init() {
	rcc->AHB1ENR |= GPIOA_EN;
	gpioa->MODER |= (0b01 << 5 * 2);
}

void ld2_toggle(void) {
	gpioa->ODR ^= (1<<5);	// toggle pin 5
}

void monitor_led_init() {
	// TODO;
}
void monitor_led_set(channel_state state) {
	// TODO:
}





