// main.c (Example usage)
#include "../include/encryption.h"
#include "../include/key_generator.h"
#include "../include/key_exchange.h"
#include "../include/transmission.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifndef AES_KEY_SIZE
#define AES_KEY_SIZE 32
#endif
#define MESSAGE_SIZE 64

int main()
{
    // Define the external host
    Host *host = (Host *)malloc(sizeof(unsigned char) * HOST_ADDRESS_LENGTH + sizeof(Buffer) + sizeof(unsigned char));
    unsigned char tmp[HOST_ADDRESS_LENGTH] = {
        0x6b,
        0xc1,
        0xbe,
        0xe2,
        0x2e,
        0x40,
    };
    for (int i = 0; i < HOST_ADDRESS_LENGTH; i++)
    {
        host->hostname[i] = tmp[i];
    }
    host->address_length = HOST_ADDRESS_LENGTH;
    host->packet_ID = 0;
    handshake(host);

    printf("Setting the key\n");
    // Example key (32 bytes for AES-256)
    unsigned char key[AES_KEY_SIZE] = {
        0x60, 0x3d, 0xeb, 0x10,
        0x15, 0xca, 0x71, 0xbe,
        0x2b, 0x73, 0xae, 0xf0,
        0x85, 0x7d, 0x77, 0x81,
        0x1f, 0x35, 0x2c, 0x07,
        0x3b, 0x61, 0x08, 0xd7,
        0x2d, 0x98, 0x10, 0xa3,
        0x09, 0x14, 0xdf, 0xf4};
    Buffer *my_key_buffer = (Buffer *)malloc(sizeof(unsigned char) * AES_KEY_SIZE + sizeof(usi));
    my_key_buffer->key_size = AES_KEY_SIZE;
    generateKey(2132, my_key_buffer, 1);
    // memcpy(my_key_buffer->key, key, sizeof(char) * 32);
    // unsigned char key[AES_KEY_SIZE] = generateKey()
    unsigned char plaintext_bytes[MESSAGE_SIZE] = {
        0x6b,
        0xc1,
        0xbe,
        0xe2,
        0x2e,
        0x40,
        0x9f,
        0x96,
        0xe9,
        0x3d,
        0x7e,
        0x11,
        0x73,
        0x93,
        0x17,
        0x2a,
        0x6b,
        0xc1,
        0xbe,
        0xe2,
        0x2e,
        0x40,
        0x9f,
        0x96,
        0xe9,
        0x3d,
        0x7e,
        0x11,
        0x73,
        0x93,
        0x17,
        0x2a,
        0x6b,
        0xc1,
        0xbe,
        0xe2,
        0x2e,
        0x40,
        0x9f,
        0x96,
        0xe9,
        0x3d,
        0x7e,
        0x11,
        0x73,
        0x93,
        0x17,
        0x2a,
        0x6b,
        0xc1,
        0xbe,
        0xe2,
        0x2e,
        0x11,
        0x11,
        0x11,
        0x11,
        0x11,
        0x11,
        0x11,
        0x11,
        0x93,
        0x55,
        0x95};
    Message plaintext;
    plaintext.message_size = MESSAGE_SIZE;
    plaintext.message_body = (unsigned char *)plaintext_bytes;
    sendMessage(host, my_key_buffer, &plaintext);

    // Enc/Dec test
    // printf("Starting the encryption\n");
    // printf("Encrypted message:\n");
    // Message encrypted = EncryptMessage(&plaintext, my_key_buffer->key);
    // DecryptMessage(&encrypted, my_key_buffer->key);
    // printf("\n");
    // for (int i = 0; i < plaintext.message_size; i++)
    // {
    //     printf("%02x", (unsigned char)plaintext.message_body[i]);
    // }
    // printf("\n");

    // // Expected decrypted message:
    // // 6b c1 be e2 2e 40 9f 96 e9 3d 7e 11 73 93 17 2a
    // Message decrypted = DecryptMessage(&encrypted, my_key_buffer->key);
    // printf("Decrypted message:\n");
    // for (int i = 0; i < decrypted.message_size; i++)
    // {
    //     printf("%02x ", (unsigned char)decrypted.message_body[i]);
    // }
    // printf("\n");
    // for (int i = 0; i < plaintext.message_size; i++)
    // {
    //     printf("%02x ", (unsigned char)plaintext.message_body[i]);
    // }
    // printf("\n");
    // free(encrypted.message_body);
    // free(decrypted.message_body);

    return 0;
}
