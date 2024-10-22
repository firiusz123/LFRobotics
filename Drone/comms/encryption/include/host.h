#ifndef HOST_H
#define HOST_H

#define HOST_ADDRESS_LENGTH 6

#include "buffer.h"

typedef struct
{
    Buffer *key_buffer;
    usi address_length;
    unsigned char hostname[HOST_ADDRESS_LENGTH];
    volatile char connected; // 0 - the controller is not connected,  1 - connected to host, 2 trying to establish connection
    ulli packet_ID;
} Host;

#endif