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
static volatile TIMX_16 *tim2 = (TIMX_16*) TIM2;
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

static channel_state state = IDLE;

channel_state channel_monitor_get_state(void) {
	return state;
}


//function that serializes it (buffer of 1s and 0s, state before transmission)
void serializePacket(Packet *packet, uint8_t *buffer, int buffer_size) {
    //Calculate total packet size // dont needs this??
    //size_t total_size = sizeof(Header) + packet->header.length + sizeof(packet->trailer_crc);

    //Serialize header
    memcpy(buffer, &(packet->header), sizeof(Header));
    //Serialize message
    memcpy(&buffer[5], packet->message, packet->header.length);
    //Serialize trailer CRC
    memcpy(&buffer[5+packet->header.length], &(packet->trailer_crc),sizeof(uint8_t));
    free_packet(packet);

    //Update buffer size (Each byte is 8 bits)
    //*buffer_size = total_size * 8; //dont need this???
}



//function that deserailizes it and returns a struct
		//check whether the address is correct if it isn't break
Packet* deserializePacket(const uint8_t *buffer, size_t buffer_size_bits) {
	//Expected size for header and trailer
	size_t expected_size = sizeof(Header) + sizeof(uint8_t);
    //Check if buffer size is sufficient for header and trailer
    if (buffer_size_bits < expected_size) {
    	printf("Error: Insufficient buffer size for header and trailer. Expected: %u, Actual: %u \n", expected_size, buffer_size_bits);
        return NULL; //Not enough data for header and trailer
    }

    //Adjust buffer size to bytes
  //  size_t buffer_size_bytes = (buffer_size_bits + 7) / 8;

    //Allocate memory for packet
    Packet *packet = (Packet*)malloc(sizeof(Packet));
    if (packet == NULL) {
        return NULL; //Memory allocation failed
    }

    //Deserialize header
	memcpy(&(packet->header), buffer, sizeof(Header));
    char* msg = malloc(sizeof(char)*packet->header.length);
    memcpy(msg,&buffer[5],packet->header.length);
	memcpy(&(packet->trailer_crc), &buffer[4 + packet->header.length], sizeof(uint8_t));


    printf("Deserialized Header: \n");
    printf("Preamble: 0x%02X\n", packet->header.preamble);
    printf("Source Address: 0x%02X\n", packet->header.source_address);
    printf("Destination Address: 0x%02X\n", packet->header.destination_address);
    printf("Length: 0x%02X\n", packet->header.length);
    printf("CRC Flag: 0x%02X\n", packet->header.crc_flag);

    //Check if source and destination addresses are within the acceptable range
    if ((packet->header.destination_address < 0x14 || packet->header.destination_address > 0x17)) {
        free_packet(packet); //Free memory before returning NULL
        return NULL; //Invalid source or destination address
    }


    return packet;
}
//Free message and packet
void free_packet(Packet * packet){
	free(packet->message);
	free(packet);
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
	char msg_buffer[200];
//	for(int i = 0; i < recv_buffer_size; i ++) {
//		printf("%c", recv_buffer[i] == 0 ? '0':'1');
//		if((i+1) % 8 == 0) {
//			printf("\n");
//		}
//
//	}
	manchester_decode(recv_buffer, recv_buffer_size, msg_buffer);
	printf("received: %s\n", msg_buffer);
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

//rx pin int on edge
void tim2_init(void){
	gpiob->MODER  &= ~(0b11 << 8*2);	// PB8 is in input mode (00)
	gpiob->ODR    |=  (1 << 8);

	rcc->APB1ENR |= TIM2_EN;

	// configure TIM2_CH1 as TIC
	tim2->CCMR1  &= ~(0b11 << 0);		// clear CC1S bits
	tim2->CCMR1  |=  (0b01 << 0);		// tim2_ch1 is in input capture mode

	tim2->CCER &= ~(1 << 1);      		// clear CC1P (active on rising edge)
	tim2->CCER |= (1 << 0);        		// enable capture on CC1 pin

	tim2->DIER |= 0b11 << 1;   			// enable TIC interrupts

	nvic_iser[0] |= (1 <<28); 			// TIM2 global interrupt is in NVIC_ISER0[30]
	tim2->CR1 |= 1;					    // Start the timer
}


void TIM2_IRQHandler(void){
	static uint8_t msg[510]; //Assuming maximum message length of 255 bytes
	static int msg_index = 0;
	static uint16_t last_capture = 0;
	if(state == BUSY && tim2->SR &(1<<1)){//Check if the interrupt flag is set
									      //and that we're busy
        //Read captured value
        uint16_t capture_value = tim2->CCR1;
        uint16_t time_elapsed = capture_value - last_capture;

        // Reset the sampling clock using the middle of the bit period
        tim2->CCR1 = last_capture + time_elapsed / 2;

        //Store Manchester bit in the message buffer
        msg[msg_index++] = capture_value;

        //Check if we have received enough bits to decode a byte
        if (msg_index >= 16) {
            char decoded[255]; //Assuming maximum message length of 255 bytes
            if (manchester_decode(msg, 16, decoded) == 0) {
                //Print decoded ASCII text to console
                printf("%s", decoded);
            }
            msg_index = 0; //Reset message index for the next byte
        }

        // Update the last capture value
        last_capture = capture_value;

		tim2->SR &= ~(1<<1);//Clear TIM2_CH1 interrupt flag
	}
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
	tim4->CR1 |= 1; 		   // start the timer
}

void TIM4_IRQHandler(void) {
	state = IDLE;
	static uint8_t line_state;
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
}

void monitor_led_init() {
	rcc->AHB1ENR |= GPIOA_EN;
	gpioa->MODER &= ~(0b1100001111);
    gpioa->MODER |= (0b01 << 0); // setting GPIOA_PIN_0 as output (Green LED)
    gpioa->MODER |= (0b01 << 2); // setting GPIOA_PIN_1 as output (Red LED)
    gpioa->MODER |= (0b01 << 8); // setting GPIOA_PIN_4 as output (Yellow LED)
}

void monitor_led_set(channel_state state) {

    // GPIOA_PIN_0 is connected to the green LED,
    // GPIOA_PIN_1 is connected to the red LED,
    // GPIOA_PIN_2 is connected to the yellow LED
	gpioa->ODR &= ~(0b10011);
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





