#ifndef KEY_GENERATOR_H
#define KEY_GENERATOR_H

#ifndef ulli
#define ulli unsigned long long int
#endif

#ifndef usi
#define usi unsigned short int
#endif

#include "./buffer.h"

// Of course we should avoid using libraries unless absolutely neccessary

// Returns 0 when key has been written in the buffer and is ready to be used
void generateKey(ulli seed, Buffer *key_write_buffer, usi random_mode);

void generateKeyFuncion(ulli seed, Buffer *key_buffer, usi random_mode, unsigned char (*randomFuncion)());

// Simplest rng
unsigned char RNG(ulli seed);

// Noise rng, here seed is optional
unsigned char NRNG();

#endif