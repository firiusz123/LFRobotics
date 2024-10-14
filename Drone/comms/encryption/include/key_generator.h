#ifndef KEY_GENERATOR_H
#define KEY_GENERATOR_H

#ifndef ulli
#define ulli unsigned long long int
#endif

#ifndef usi
#define usi unsigned short int
#endif

#define BUFFER_SIZE 256

typedef struct
{
    unsigned char key[BUFFER_SIZE]; // ???
    volatile char connection_established; // Size of 1
    usi key_size; // Size of 2 
} Buffer;

// Of course we should avoid using libraries unless absolutely neccessary

// Returns 0 when key has been written in the buffer and is ready to be used
void GenerateKey(ulli seed, Buffer *key_write_buffer, usi random_mode);

void GenerateKeyFuncion(ulli seed, Buffer *key_buffer, usi random_mode, unsigned char (*randomFuncion)());

// Simplest rng
unsigned char RNG(ulli seed);

// Noise rng, here seed is optional
unsigned char NRNG();

#endif