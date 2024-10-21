#include <stdio.h>
#include "../include/transmission.h"
#include "../include/encryption.h"
#include "../include/key_exchange.h"

void transmit(Message *message)
{
    printf("Transmitting message:\n");
    for (int i = 0; i < message->message_size; i++)
    {
        printf("%02x", message->message_body[i]);
    }
    printf("\nTransmission finished\n");
}

void SendData(char *data, usi data_size)
{
    Message message;
    message.message_body = (char *)malloc(data_size);
    memcpy(data, message.message_body, data_size);
    message.message_size = data_size;
    unsigned char *key;
    Message encrypted = EncryptMessage(&message, key);
    transmit(&encrypted);
}

// void transmit(Message *message,void *function)
// {

// }