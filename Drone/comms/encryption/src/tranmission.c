#include <stdio.h>
#include "../include/transmission.h"
#include "../include/encryption.h"
#include "../include/key_exchange.h"
#include "../include/key_generator.h"

void transmit(Host *host, Message *message)
{
    printf("Transmitting message to host:");
    printHost(host);
    for (usi i = 0; i < message->message_size; i++)
    {
        printf("%02x", message->message_body[i]);
    }
    printf("\nTransmission finished\n");
}

void transmitKey(Host *host, Buffer *my_key)
{
    printf("Transmitting key to host:");
    printHost(host);
    for (usi i = 0; i < KEY_BUFFER_SIZE; i++)
    {
        printf("%02x", my_key->key[i]);
    }
    printf("\nTransmission finished\n");
}

void SendData(Host *host, char *data, usi data_size)
{
    Message message;
    message.message_body = (char *)malloc(data_size);
    
    // Correct the argument order in memcpy
    memcpy(message.message_body, data, data_size); 
    
    message.message_size = data_size;

    // Initialize the encryption key properly
    unsigned char *key = getEncryptionKey();  // Assuming this function returns the key

    Message encrypted = EncryptMessage(&message, key);  // Encrypt the message
    transmit(host, &encrypted);  // Transmit the encrypted message

    // Free allocated memory
    free(message.message_body);
    free(encrypted.message_body);  // Assuming EncryptMessage allocates new memory
}
