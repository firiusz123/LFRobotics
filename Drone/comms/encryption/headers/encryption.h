#ifndef ENCRYPTION_H
#define ENCRYPTION_H

typedef struct Message
{
    char *message_body;
    short int message_size;
} Message;

// It's like this for now, we will either settle for specific implementation for each structure,
// or we will just conver the type using casting and this will take size as an argument
void *encrypt_void(void *message, short int message_size /* In bytes */, long long int key);

Message encrypt_message(Message message, long long int key);

void *decrypt_void(void *message, short int message_size /* In bytes */, long long int key);

Message decrypt_message(Message message, long long int key);

#endif