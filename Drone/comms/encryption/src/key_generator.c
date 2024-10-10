#include "../include/key_generator.h"
#include <stdio.h>
// Gets current time (we could define a counter similar to millis() in arduino)
unsigned char NRNG()
{
    return 0x34;
}

// USE FOR TESTING ONLY!!!
unsigned char RNG(ulli seed)
{
    return seed * 0x22;
}

// Generates a cryptografic key for encryption
void GenerateKey(ulli seed, Buffer *key_buffer, usi random_mode)
{
    usi key_size = key_buffer->key_size;
    printf("%ld",sizeof(*key_buffer));
    printf("=============== BEGIN KEY ===============\n");
    for (usi key_byte = 0; key_byte < key_size; key_byte++)
    {
        key_buffer->key[key_byte] = 0x22;
        printf("%02x ", key_buffer->key[key_byte]);
    }
    printf("================ END KEY ================\n");
}
