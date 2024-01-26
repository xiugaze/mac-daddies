/*
 * channel_monitor.h
 *
 *  Created on: Jan 24, 2024
 *      Author: andreanoc
 */

#ifndef CHANNEL_MONITOR_H_
#define CHANNEL_MONITOR_H_

typedef enum {
	BUSY,
	IDLE,
	COLLISION,
} channel_state;


void user_led_init();
void toggle_user_led();


#endif /* CHANNEL_MONITOR_H_ */
