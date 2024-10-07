# Documentation of encryption library

## Message type:
```c
typedef struct Message
{
    char *message_body;
    short int message_size;
} Message;
```

## Message encryption function that uses void structure as an argument, message size in bytes in required

```c
void *encrypt_void(void *message, short int message_size /* In bytes */, long long int key)
```

## Message encryption function that uses uses Message structure as an argument

```c
Message encrypt_message(Message message, long long int key)
```

## Message decryption function that uses void pointer as an argument, message size in bytes in required

```c
void *decrypt_void(void *message, short int message_size /* In bytes */, long long int key)
```

## Message decryption function that uses Message structure as an argument

```c
Message decrypt_message(Message message, long long int key);
```

## Key generation function

### Definition:
```c
long long int generate_key(long long int seed, short int random_mode);
```

### Description:
The function will generate a cryptographic key for later encryption 

## Basic unsafe rng function for key generation, should not be used externally for different purposes and exists solely for initial tests

```c
long long int rng(long long int seed);
```

## Improved rng function for key generation, can be used externally for different purposes

```c
long long int nrng(long long int seed);
```