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
	monitor_led_init();
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


	tim4->CCR2 = CYCLES_1_1_MS;			// Timeout interrupt fires

//	CC1 channel configured as input:
//	This bit determines if a capture of the counter value can actually be done into the input
//	capture/compare register 1 (TIMx_CCR1) or not.
//	0: Capture disabled
//	1: Capture enabled
	tim4->CCER |= (0b01 << 0); // enable input capture
	tim4->DIER |= 0b11 << 1;   // enable TIC interrupts

	nvic_iser[0] |= (1 << 30); // TIM4 global interrupt is in NVIC_ISER0[30]
	tim4->CR1 |= 1; 		   // start the timer
}

void TIM4_IRQHandler(void) {
	static channel_state state = IDLE;
	static int line_state;

	if(((tim4->SR >> 2) & 0b01) & (tim4->DIER >> 2) & 1) {
		tim4->DIER &= ~(1 << 2);

		if(line_state) {
			state = IDLE;
		} else {
			state = COLLISION;
		}
		// disable
	}

	/*
	 * You were right the first time: the first read upstairs will clear
	 * the status register. If there's a pending TIC and a pending TOC, we read
	 * it up there clearing the interrupt
	 */
	if(((tim4->SR >> 1) &  0b01) & (tim4->DIER >> 1) & 1) {
			// edge
		line_state = (gpiob->IDR >> 6) & 0b01;
		//tim4->CNT = 0
		tim4->CCR2 = tim4->CCR1 + CYCLES_1_1_MS;
		tim4->DIER |= (1 << 2);
		state = BUSY;
	}


	monitor_led_set(state);
	tim4->SR = 0;
	//tim4->DIER |= 0b11 << 1;  // enable interrupts
}

/*
void ld2_init() {
	rcc->AHB1ENR |= GPIOA_EN;
	gpioa->MODER |= (0b01 << 5 * 2);
}

void ld2_toggle(void) {
	gpioa->ODR ^= (1<<5);	// toggle pin 5
}
*/

void monitor_led_init() {
	rcc->AHB1ENR |= GPIOA_EN;
	gpioa->MODER &= ~(0b111111);
    gpioa->MODER |= (0b01 << 0); // setting GPIOA_PIN_0 as output (Green LED)
    gpioa->MODER |= (0b01 << 2); // setting GPIOA_PIN_1 as output (Red LED)
    gpioa->MODER |= (0b01 << 8); // setting GPIOA_PIN_2 as output (Yellow LED)
}

void monitor_led_set(channel_state state) {

    // GPIOA_PIN_0 is connected to the green LED,
    // GPIOA_PIN_1 is connected to the red LED,
    // GPIOA_PIN_2 is connected to the yellow LED
	gpioa->ODR &= ~(0b010011);
	switch(state){
	case IDLE:
		gpioa->ODR |= 1 << 0;  // turn on Green LED
		break;
	case BUSY:
	    gpioa->ODR |= 1 << 1;  // turn off Green/Red LED's
	    break;
	case COLLISION:
		gpioa->ODR |= 1 << 4;  // turn off Green/Yellow LED's
		break;

	}
}





