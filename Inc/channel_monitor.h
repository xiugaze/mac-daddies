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
channel_state channel_monitor_get_state(void);
int recv_set(void);
int recv_clear(void);
int recv_status(void);
void recv_wait();
void recv_post();
void recv_decode();


#endif /* CHANNEL_MONITOR_H_ */
