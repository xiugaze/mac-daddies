/*
 * channel_monitor.h
 *
 *  Created on: Jan 24, 2024
 *      Author: andreanoc
 */

#ifndef CHANNEL_MONITOR_H_
#define CHANNEL_MONITOR_H_


#define MAX_MESSAGE_LENGTH 255

#define MAX_MESSAGE_LENGTH 255

//Stuct w all data fields for the message
typedef struct {
    uint8_t preamble;
    uint8_t source_address;
    uint8_t destination_address;
    uint8_t length;
    uint8_t crc_flag;
} Header;

typedef struct {
    Header header;
    uint8_t message[MAX_MESSAGE_LENGTH];
    uint8_t trailer_crc;
} Packet;

typedef enum {
	BUSY,
	IDLE,
	COLLISION,
} channel_state;

void channel_monitor_init();
channel_state channel_monitor_get_state(void);

void serializePacket(const Packet *packet, uint8_t *buffer, size_t *buffer_size);
Packet* deserializePacket(const uint8_t *buffer, size_t buffer_size);


int recv_set(void);
int recv_clear(void);
int recv_status(void);
void recv_wait();
void recv_post();
void recv_decode();


#endif /* CHANNEL_MONITOR_H_ */
