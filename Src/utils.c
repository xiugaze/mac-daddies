/*
 * manchester_utils.c
 *
 *  Created on: Feb 7, 2024
 *      Author: andreanoc
 *
 */


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "utils.h"

int pair_to_bit(uint8_t pair[]) {
    if(pair[0] == 0 && pair[1] == 1) {
        return 1;
    } else if (pair[0] == 1 && pair[1] == 0) {
        return 0;
    } else {
        return -1;
    }
}

Packet* manchester_decode(uint8_t msg[], int len) {
    // message length must be divisible by 16 baud == 8 bits == 1 byte
    if(len % 16 != 0) {
        return NULL;
    }

    uint8_t decoded[1024];

    // for each point in the buffer
    int decoded_pos = 0;
    int byte_pos = 7;
    char current_char = '\0';
    for(int i = 0; i < len; i += 2) {
        uint8_t pair[2] = { msg[i], msg[i+1] };
        int bit = pair_to_bit(pair);

        if(bit < 0) {
            return NULL;
        }

        current_char |= (bit << byte_pos);

        if(byte_pos == 0) {
            byte_pos = 7;
            decoded[decoded_pos++] = current_char;
            current_char = '\0';
        } else {
            byte_pos--;
        }
    }
    decoded[decoded_pos] = '\0'; // append the null terminator


    Packet* p = deserializePacket(decoded, 1024);
    return p;
    return 0;
}

static int error_pos = 0;
static error error_queue[200];
void raise_error(error e) {
	error_queue[error_pos] = e;
	error_pos++;
}

//function that serializes it (buffer of 1s and 0s, state before transmission)
int serializePacket(Packet *packet, uint8_t *buffer, int buffer_size) {
    //Calculate total packet size // dont needs this??
    //size_t total_size = sizeof(Header) + packet->header.length + sizeof(packet->trailer_crc);

    //Serialize header
    memcpy(buffer, &(packet->header), sizeof(Header));
    //Serialize message
    memcpy(&buffer[5], packet->message, packet->header.length);
    //Serialize trailer CRC
    memcpy(&buffer[5+packet->header.length], &(packet->trailer_crc),sizeof(uint8_t));

    int length_bytes = sizeof(Header) +  packet->header.length + sizeof(uint8_t);
    free_packet(packet);

    return length_bytes;

    //Update buffer size (Each byte is 8 bits)
    //*buffer_size = total_size * 8; //dont need this???
}

Packet* test_packet(void) {

	char message[] = "this is a test message!";

	Packet* p = malloc(sizeof(Packet));
	p->header.preamble = 0xAA;
	p->header.source_address = 0x15;
	p->header.destination_address = 0x15;
	p->header.length = strlen(message + 1);
	p->header.crc_flag = 0;

	p->message = malloc(sizeof(uint8_t) * p->header.length);
	strcpy(p->message, message);
	p->trailer_crc = 0;

	return p;
}

Packet* new_packet(char message[], uint8_t destination) {

	Packet* p = malloc(sizeof(Packet));
	p->header.preamble = 0x55;
	p->header.source_address = 0x15;
	p->header.destination_address = destination;
	p->header.length = strlen(message + 1);
	p->header.crc_flag = 0;

	p->message = malloc(sizeof(uint8_t) * p->header.length);
	strcpy(p->message, message);
	p->trailer_crc = 0;

	return p;

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


    //Check if source and destination addresses are within the acceptable range
    if ((packet 	->header.destination_address < 0x14 || packet->header.destination_address > 0x17)) {
    	printf("Packet decoded for address %x, discarded\n", packet->header.destination_address);
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


