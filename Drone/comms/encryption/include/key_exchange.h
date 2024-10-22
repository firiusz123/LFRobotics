#ifndef KEY_EXCHANGE_H
#define KEY_EXCHANGE_H

#include "./transmission.h"
#include "buffer.h"
#ifndef ulli
#define ulli unsigned long long int
#endif

#ifndef usi
#define usi unsigned short int
#endif

// Multiplies a unsigned char vector by a scalar, writes it to the key pointer
unsigned char *multiplyScalar(unsigned char *key, usi scalar, usi key_size);

void add(unsigned char *first_number, unsigned char *second_number, usi num_size_1, usi num_size_2);

void multiply(unsigned char *key, unsigned char *number, usi key_size, usi number_size);

// Raise key to the power of another huge number (mod 2^256), max 2^256
void fastPower(unsigned char *key, unsigned char *exponent, usi key_size, usi exponent_size);

void fastPowerScalar(unsigned char *key, ulli exponent, usi key_size);
// Similar as the one a
// unsigned char *multiply(ulli scalar, unsigned char key, usi key_size)

void fastPowerBuffer(Buffer *key_buffer, unsigned char *exponent, usi exponent_size);

void fastPowerBufferScalar(Buffer *key_buffer, usi exponent);

#endif