/*
 * channel_monitor.c
 *
 *  Created on: Jan 23, 2024
 *      Author: andreanoc
 */

// PB6 and PD12 on AF2

#include <stdint.h>
#include "regs.h"

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

void user_led_init() {
	rcc->AHB1ENR |= GPIOA_EN;
	gpioa->MODER |= (0b01 << 5 * 2);
}

void channel_monitor_init(void) {

}


void tim4_init(void) {

	/* PB6 is the input pin for TIC */
	rcc->AHB1ENR |= GPIOB_EN;			// enable GPIOB in RCC
	gpiob->AFRL  |= (0b0010 << 6 * 4); 	// PB6 is AF02 (TIM4_CH1)
	gpiob->MODER |= (0b10 << 6*2);

	/* TIM4 setup */
	rcc->APB1ENR |= TIM4_EN;			// enable TIM4 in RCC

	tim4->CCMR1  &= ~(0b11 << 0);		// clear CC1S bits
	tim4->CCMR1  |=  (0b01 << 0);		// tim4 is in input capture mode

	tim4->CCER &=  ~(1 << 3 | 1 << 1);		// clear edge config, set to rising
	tim4->CCER |=   (1 << 3 | 1 << 1); 	// trigger TIC on either edge
	tim4->ARR |= ~(0b0);


//	CC1 channel configured as input:
//	This bit determines if a capture of the counter value can actually be done into the input
//	capture/compare register 1 (TIMx_CCR1) or not.
//	0: Capture disabled
//	1: Capture enabled
	tim4->CCER |= (0b01 << 0); // enable input capture
	tim4->DIER |= 0b01 << 1;   // enable TIC interrupts

	nvic_iser[0] |= (1 << 30); // TIM4 global interrupt is in NVIC_ISER0[30]
}

//void tim6_init(void) {
//	rcc->
//}

void toggle_user_led(void) {
	gpioa->ODR ^= (1<<5);	// toggle pin 5
}




void TIM4_IRQHandler(void) {
	tim4->DIER &= ~(0b01 << 1);  // reset CC1IE (Interrupt enable)
	tim4->SR = 0; 				 // clear all flags
	/* interrupt body */
	toggle_user_led();

	tim4->DIER |= 0b01 << 1;  // enable interrupts

}



