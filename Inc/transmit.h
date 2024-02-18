#ifndef TRANSMIT_H
#define TRANSMIT_H


int transmit_halfbits(void);

int transmit(void);

void transmit_init();

void TIM3_IRQHandler(void);


#endif // !TRANSMIT
