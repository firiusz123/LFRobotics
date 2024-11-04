/*
 * key_generation.c
 *
 *  Created on: Oct 14, 2024
 *      Author: X
 */


#include "key_generator.h"
#include <stdio.h>
#include <time.h>
// Gets current time (we could define a counter similar to millis() in arduino)
unsigned char NRNG()
{
    return 0x34;
}

// USE FOR TESTING ONLY!!!
unsigned char RNGR(ulli seed)
{
    return (unsigned char)(seed * 0x22);
}

// Generates a cryptografic key for encryption
void GenerateKey(ulli seed, Buffer *key_buffer, usi random_mode)
{
    usi key_size = key_buffer->key_size;
    printf("=============== BEGIN KEY ===============\n");
    for (usi key_byte = 0; key_byte < key_size; key_byte++)
    {
        key_buffer->key[key_byte] = RNGR(seed + key_byte * (ulli)time(NULL));
        printf("%02x ", key_buffer->key[key_byte]);
        if (!((key_byte + 1) % 16))
        {
            printf("\n");
        }
    }
    printf("=============== END KEY ===============\n");
}

// Generates a cryptografic key for encryption
void GenerateKeyFuncion(ulli seed, Buffer *key_buffer, usi random_mode, unsigned char (*randomFuncion)())
{
    usi key_size = key_buffer->key_size;
    printf("=============== BEGIN KEY ===============\n");
    for (usi key_byte = 0; key_byte < key_size; key_byte++)
    {
        key_buffer->key[key_byte] = randomFuncion(seed + key_byte * (ulli)time(NULL));
        printf("%02x ", key_buffer->key[key_byte]);
        if (!((key_byte + 1) % 16))
        {
            printf("\n");
        }
    }
    printf("=============== END KEY ===============\n");
}
