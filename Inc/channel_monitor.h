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

void channel_monitor_init();


#endif /* CHANNEL_MONITOR_H_ */
