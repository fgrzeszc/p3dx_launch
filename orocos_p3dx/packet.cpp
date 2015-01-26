#include "packet.h"

#include <string.h>


uint16_t Packet::calculate_checksum()
{
    
    uint8_t *buffer = &packet[3];
    int c = 0;
    uint8_t n;

    n = this->size - 5;

    while(n > 1){
        c += ((*buffer) << 8) | *(buffer+1);
        //c = c & 0xffff;
        n -= 2;
        buffer += 2;
    }
    if(n > 0)
        c = c ^ static_cast<int>(*(buffer++));
    return c;
}

bool Packet::serialize(uint8_t* data, uint8_t size)
{
    uint16_t checksum;
    this->size = size + 5;
    this->packet[0] = 0xFA;
    this->packet[1] = 0xFB;

    if(this->size > 198) {
        return false;
    }
    // set byte count: number of cmd/arg bytes + 2 bytes checksumA
    this->packet[2] = size + 2; 

    memcpy(&this->packet[3], data, size);

    checksum = calculate_checksum();
    // set 2 bytes of checksum
    this->packet[3 + size] = checksum >> 8;
    this->packet[3 + size + 1] = checksum & 0xFF;
    
    return true;


}
