#include "../include/key_exchange.h"
#include <stdlib.h>
#include <string.h>
// Multiplies a unsigned char vector by a scalar, writes it to the key pointer
ulli scalarPow(ulli num, usi expo)
{
    ulli carry = 1;
    ulli num_buff = num;
    while (expo > 0)
    {
        if (expo % 2 == 1)
        {
            carry *= num_buff;
        }
        num *= num;
        expo /= 2;
    }
    return carry * num;
}

unsigned char *multiplyScalar(unsigned char *key, usi scalar, usi key_size)
{
    unsigned char *result = calloc(key_size, sizeof(unsigned char));
    usi carry = 0;
    for (int i = 0; i < key_size; i++)
    {
        usi mult = (unsigned char)key[i] * scalar + carry;
        result[i] = (unsigned char)(mult & 0xFF);
        carry = (mult >> 8);
    }
    return result;
}

void add(unsigned char *first_number, unsigned char *second_number, usi num_size_1, usi num_size_2)
{
    if (num_size_1 > num_size_2)
    {
        num_size_1 = num_size_2;
    }
    unsigned char carry = 0;
    for (int i = 0; i < num_size_1; i++)
    {
        usi sum = (unsigned char)first_number[i] + (unsigned char)second_number[i] + carry;
        first_number[i] = sum & 0xFF;
        carry = (sum >> 8);
    }
}

// There has to be a faster way...
void multiply(unsigned char *key, unsigned char *number, usi key_size, usi number_size)
{
    if (number_size > key_size)
    {
        number_size = key_size;
    }
    unsigned char *temp_result = calloc(key_size, sizeof(unsigned char));
    unsigned char *tmp;
    for (int i = 0; i < number_size; i++)
    {
        tmp = multiplyScalar(key, (unsigned char)number[i], key_size);
        add(temp_result, tmp, key_size, number_size);
        free(tmp);
    }
    memcpy(key, temp_result, key_size);
    free(temp_result);
}

void fastPowerScalar(unsigned char *key, ulli exponent, usi key_size)
{
    unsigned char *result = calloc(key_size, sizeof(unsigned char));
    result[0] = 1;

    while (exponent > 0)
    {
        if (exponent % 2 == 1)
        {
            multiply(result, key, key_size, key_size);
        }
        multiply(key, key, key_size, key_size);
        exponent /= 2;
    }

    memcpy(key, result, key_size);
    free(result);
}

// I am not sure if this one will be neccessary
void fastPower(unsigned char *key, unsigned char *exponent, usi key_size, usi exponent_size)
{
    usi pointer = 0;
    while (exponent[exponent_size] > 0)
    {
        fastPowerScalar(key, exponent[pointer] * scalarPow(2, pointer), key_size);
    }
}

// Similar as the one a
// unsigned char *multiply(ulli scalar, unsigned char key, usi key_size)

void fastPowerBuffer(Buffer *key_buffer, unsigned char *exponent, usi exponent_size)
{
    fastPower(key_buffer->key, exponent, key_buffer->key_size, exponent_size);
}

void fastPowerBufferScalar(Buffer *key_buffer, usi exponent)
{
    fastPowerScalar(key_buffer->key, exponent, key_buffer->key_size);
}
