#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#define ulli unsigned long long int
#define usi unsigned short int

typedef struct
{
    unsigned char *message_body;
    usi message_size;
} Message;

// Might be used
// typedef void (*txFunType)();

// Begins the communication, sends the inital message
void handshake(ulli);

// Send data, the transmit is a pointer to a function that will be used as a transmitter: that might be for instance some void Send_Packet() for LoRa device
void SendData(char *data, usi dataSize);

// Returns the recieved data in a decoded format
void RecieveData(char *data, usi dataSize);

// Transmits a message, for current implementation it might just print this string
void transmit(Message message);

#endif