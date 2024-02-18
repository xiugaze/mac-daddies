#ifndef UTILS_H_
#define UTILS_H_


typedef enum {
	TRANSMISSION_ON_BUSY,
	TRANSMISSION_ON_COLLISION,
} error;

typedef struct {
    uint8_t preamble;
    uint8_t source_address;
    uint8_t destination_address;
    uint8_t length;
    uint8_t crc_flag;
} Header;

typedef struct {
    Header header;
    char* message;
    uint8_t trailer_crc;
} Packet;



int pair_to_bit(uint8_t pair[]);
Packet* manchester_decode(uint8_t msg[], int len);
void raise_error(error e);

int serializePacket(Packet *packet, uint8_t *buffer, int buffer_size);
Packet* deserializePacket(const uint8_t *buffer, size_t buffer_size);
Packet* new_packet(char message[], uint8_t destination);
void free_packet(Packet * packet);



#endif
