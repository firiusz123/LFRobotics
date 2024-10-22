#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include "./encryption.h"
#include "./key_generator.h"

#define ulli unsigned long long int
#define usi unsigned short int

#define HOST_ADDRESS_LENGTH 24

typedef struct
{
    Buffer *key_buffer;
    unsigned char *hostname;
    volatile char connected; // 0 - the controller is not connected,  1 - connected to host, 2 trying to establish connection 
    ulli packet_ID;
} Host;

// Might be used
// typedef void (*txFunType)();

// Begins the communication, sends the inital message
// void handshake(ulli);

// Send data, the transmit is a pointer to a function that will be used as a transmitter: that might be for instance some void Send_Packet() for LoRa device
void SendData(char *data, usi dataSize);

// Returns the recieved data in a decoded format
void RecieveData(char *data, usi dataSize);

// Transmits a message, for current implementation it might just print this string
void transmit(Host *host,Message *message);

void transmitKey(Host *host, Buffer *my_key)

#endif