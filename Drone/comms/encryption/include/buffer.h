#ifndef BUFFER_H
#define BUFFER_H

#ifndef usi
#define usi unsigned short int
#endif

#define KEY_BUFFER_SIZE 32

typedef struct
{
    unsigned char key[KEY_BUFFER_SIZE];
    volatile char connection_established;
    usi key_size;
} Buffer;

#endif