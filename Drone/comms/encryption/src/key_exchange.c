#include "../include/key_exchange.h"
#include "../include/key_generator.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// Multiplies a char vector by a scalar, writes it to the key pointer
char *multiplyScalar(char *key, usi scalar, usi key_size)
{
    char *result = calloc(key_size, sizeof(char));
    usi carry = 0;
    for (usi i = 0; i < key_size; i++)
    {
        usi mult = (unsigned char)key[i] * scalar + carry;
        result[i] = (char)(mult & 0xFF);  // Store only the lower byte
        carry = (mult >> 8);              // Get the carry (higher byte)
    }
    return result;
}

void add(char *first_number, char *second_number, usi num_size)
{
    unsigned char carry = 0;
    for (usi i = 0; i < num_size; i++)
    {
        usi sum = (unsigned char)first_number[i] + (unsigned char)second_number[i] + carry;
        first_number[i] = (char)(sum & 0xFF);  // Keep only the lower byte
        carry = (sum >> 8);                    // Carry the overflow (higher byte)
    }
}

void multiply(char *key, char *number, usi key_size, usi number_size)
{
    if (number_size > key_size)
    {
        number_size = key_size;
    }
    
    char *temp_result = calloc(key_size, sizeof(char));
    
    for (usi i = 0; i < number_size; i++)
    {
        char *partial_result = multiplyScalar(key, (unsigned char)number[i], key_size);
        add(temp_result, partial_result, key_size);
        free(partial_result);  // Free allocated memory
    }
    
    memcpy(key, temp_result, key_size);  // Store the result back into `key`
    free(temp_result);
}

void fastPowerScalar(char *key, ulli exponent, usi key_size)
{
    char *result = calloc(key_size, sizeof(char));
    result[0] = 1;  // Set initial value to 1 (since any number raised to power 0 is 1)

    while (exponent > 0)
    {
        if (exponent % 2 == 1)
        {
            multiply(result, key, key_size, key_size);
        }
        multiply(key, key, key_size, key_size);  // Square the base
        exponent /= 2;
    }

    memcpy(key, result, key_size);
    free(result);  // Free allocated memory
}

void fastPowerBuffer(Buffer *key_buffer, char *exponent, usi exponent_size)
{
    fastPower(key_buffer->key, exponent, key_buffer->key_size, exponent_size);
}

void fastPowerBufferScalar(Buffer *key_buffer, usi exponent)
{
    fastPowerScalar(key_buffer->key, exponent, key_buffer->key_size);
}

void printHost(Host *host)
{
    printf("\n");
    for(int i = 0; i < HOST_ADDRESS_LENGTH; i++)
    {
        printf("%02x", host->host[i]);
    }
    printf("\n");
}

void handshake(Host *host, Buffer *my_key)
{
    printf("Connecting to host...");
    printHost(host);     
    transmitKey(host, my_key);
    
    // Assuming foreign_key_buffer is globally declared somewhere else
    foreign_key_buffer.key = getKey(host);  // This function needs to exist elsewhere
}
