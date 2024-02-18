/*
 * regs.h
 *
 *  Created on: Jan 23, 2024
 *      Author: andreanoc
 */

#ifndef REGS_H_
#define REGS_H_

#define RCC_BASE 0x40023800

#define GPIOA 0x40020000
#define GPIOB 0x40020400
#define GPIOC 0x40020800

#define STK_BASE 0xE000E010
#define TIM2  0x40000000
#define TIM3  0x40000400
#define TIM4  0x40000800

#define CPACR 0xE000ED88
#define NVIC_ISER 0xE000E100

#define GPIOA_EN (1<<0)
#define GPIOB_EN (1<<1)
#define GPIOC_EN (1<<2)
#define TIM2_EN  (1<<0)
#define TIM3_EN  (1<<1)
#define TIM4_EN  (1<<2)

#define FPU_EN (0xF << 20);



typedef struct {
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t LCKR;
	uint32_t AFRL;
	uint32_t AFRH;
} GPIOX;

typedef struct {
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SMCR;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t CCMR1;
	uint32_t CCMR2;
	uint32_t CCER;
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;
	uint32_t RES1;
	uint32_t CCR1;
	uint32_t CCR2;
	uint32_t CCR3;
	uint32_t CCR4;
	uint32_t RES2;
	uint32_t DCR;
	uint32_t DMAR;
} TIMX_16;

typedef struct {
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t AHB3RSTR;
	uint32_t RES0;
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
	uint32_t RSVD1;
	uint32_t RSVD2;
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t AHB3ENR;
	uint32_t RSVD3;
	uint32_t APB1ENR;
	uint32_t APB2ENR;
	uint32_t RSVD4;
	uint32_t RSVD5;
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t AHB3LPENR;
	uint32_t RSVD6;
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
	uint32_t RSVD7;
	uint32_t RSVD8;
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t rsvd9;
	uint32_t rsvd10;
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
	uint32_t PLLISAICFGR;
	uint32_t DCKCFGR;
	uint32_t CKGATENR;
	uint32_t DCKCFGR2;
} RCC;


#endif /* REGS_H_ */
