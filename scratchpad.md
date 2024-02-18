typedef struct {
    uint8_t preamble;
    uint8_t source_address;
    uint8_t destination_address;
    uint8_t length;
    uint8_t crc_flag;
} Header;

typedef struct {
    Header header;
    uint8_t *message;
    uint8_t trailer_crc;
} Packet;
