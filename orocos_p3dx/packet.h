#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define PACKET_LEN 256

struct Packet{
    
    uint8_t size;
    uint8_t packet[PACKET_LEN];


    
    uint16_t calculate_checksum();
    bool serialize(uint8_t *data, uint8_t data_size);

};
#endif
