#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../include/transmission.h"
#include "../include/encryption.h"
#include "../include/key_exchange.h"
#include "../include/host.h"

void printHost(Host *host)
{
    printf("\nHost: ");
    host->address_length = HOST_ADDRESS_LENGTH;
    for (int i = 0; i < host->address_length; i++)
    {
        printf("%02x", host->hostname[i]);
    }
}

void transmit(Host *host, Message *message)
{
    printf("Transmitting message to host:");
    printHost(host);
    printf("\nPacket number %lld\n", host->packet_ID);
    for (usi i = 0; i < message->message_size; i++)
    {
        printf("%02x", message->message_body[i]);
    }
    host->packet_ID += 1;
    printf("\nTransmission finished\n");
}

// This should return success/error while sending data, there should be some
// Acknowlegement that would say if the connection is accepted, refused or
// Was not possible, here I will just simulate it in sending data
void transmitKey(Host *host, Buffer *my_key)
{
    printf("Transmitting key to host: ");
    printHost(host);
    printf("\nPacket number %lld\n", host->packet_ID);
    for (usi i = 0; i < my_key->key_size; i++)
    {
        printf("%02x", my_key->key[i]);
    }
    host->packet_ID += 1;
    printf("\nTransmission finished\n");
}

unsigned char handshake(Host *host)
{
    Buffer *my_key_buffer = (Buffer *)malloc(sizeof(unsigned char) * AES_KEY_SIZE + sizeof(usi));
    my_key_buffer->key_size = AES_KEY_SIZE; // This stays
    generateKey(213, my_key_buffer, 1);
    transmitKey(host, my_key_buffer);
    // This is for test, in real scenario the code should "await" the key here, if the module
    // is used just to send data, it's fine if it loops here, unless it has to handle
    // multiple hosts at once. If not, the code will have to wait for data until it's
    // received
    host->key_buffer = (Buffer *)malloc(sizeof(unsigned char) * AES_KEY_SIZE + sizeof(usi));
    host->key_buffer->key_size = AES_KEY_SIZE;
    generateKey(21, host->key_buffer, 1);
    return 0x01;
}

void sendData(Host *host, Buffer *my_key, unsigned char *data, usi data_size)
{
    if (!(host->connected & 0x01))
    {
        unsigned char success = handshake(host);
        if (success & 0x02)
        {
            printf("Host not reachable\n");
        }
        else if (success & 0x4)
        {
            printf("Host refused the connection\n");
        }
        else
        {
            printf("Another error has occured while trying to reach the host\n");
        }
    }
    Message message;
    message.message_body = (unsigned char *)malloc(data_size);

    memcpy(message.message_body, data, data_size);
    message.message_size = data_size;

    Message encrypted = EncryptMessage(&message, my_key->key);
    transmit(host, &encrypted);

    free(message.message_body);
    free(encrypted.message_body);
}

void sendMessage(Host *host, Buffer *my_key, Message *message)
{
    sendData(host, my_key, message->message_body, message->message_size);
}
