#ifndef KEY_EXCHANGE_H
#define KEY_EXCHANGE_H

#define ulli unsigned long long int
#define usi unsigned short int

#define BUFFER_SIZE 16

typedef struct
{
    unsigned char hostname;
} Host;

typedef struct
{
    unsigned char key[BUFFER_SIZE];
    volatile char connection_established;
    usi key_size;
} Buffer;

// Multiplies a char vector by a scalar, writes it to the key pointer
char *multiplyScalar(char *key, usi scalar, usi key_size);

void add(char *first_number, char *second_number, usi num_size_1, usi num_size_2);

void multiply(char *key, char *number, usi key_size, usi number_size);

// Raisese
void fastPower(char *key, char *exponent, usi key_size, usi exponent_size);

void fastPowerScalar(char *key, ulli exponent, usi key_size);
// Similar as the one a
// char *multiply(ulli scalar, char key, usi key_size)

void fastPowerBuffer(Buffer *key_buffer, char *exponent, usi exponent_size);

void fastPowerBufferScalar(Buffer *key_buffer, usi exponent);

void handshake();

#endif