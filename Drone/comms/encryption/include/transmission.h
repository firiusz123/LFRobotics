#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include "./encryption.h"
#include "./key_generator.h"
#include "./key_exchange.h"
#include "./host.h"

#define ulli unsigned long long int
#define usi unsigned short int

// Might be used
// typedef void (*txFunType)();

// Begins the communication, sends the inital message
// void handshake(ulli);

// Send data, the transmit is a pointer to a function that will be used as a transmitter: that might be for instance some void Send_Packet() for LoRa device
void sendData(Host *host, Buffer *my_key, unsigned char *data, usi data_size);

void sendMessage(Host *host, Buffer *my_key, Message *message);

// Returns the recieved data in a decoded format
void recieveData(char *data, usi dataSize);

// Transmits a message, for current implementation it might just print this string
void transmit(Host *host, Message *message);

void transmitKey(Host *host, Buffer *my_key);

unsigned char handshake(Host *host);

// If recieved the key
void handleKey(Host *host);

void printHost(Host *host);

#endif