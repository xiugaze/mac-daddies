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

void user_led_init() {
	rcc->AHB1ENR |= GPIOA_EN;
	gpioa->MODER |= (0b01 << 5 * 2);
//	gpioa->ODR |= (0b01 << 5);
}

uint32_t tic_value() {
	// until the CC1IF flag is set
	// CC1IF flag is bit 1 in TIM4_SR
	// TODO: implement a timeout
	while( ((tim4->SR &= ~(0b10)) >> 1) != 0) {}
	return (tim4->CCR1);
}

void channel_monitor_init(void) {

}


void tim4_init(void) {
	rcc->AHB1ENR |= GPIOB_EN;			// enable GPIOB in RCC
	rcc->APB1ENR |= TIM4_EN;			// enable TIM4 in RCC

	tim4->CCMR1  &= ~(0b11 << 0);		// clear CC1S bits
	tim4->CCMR1  |=  (0b01 << 0);		// select TI1

	tim4->CCER   |=  (0b0101 << 1); 	// trigger TIC on either edge

//	CC1 channel configured as input:
//	This bit determines if a capture of the counter value can actually be done into the input
//	capture/compare register 1 (TIMx_CCR1) or not.
//	0: Capture disabled
//	1: Capture enabled

	tim4->CCER |= (0b01 << 0);

	// get rid of this

}




