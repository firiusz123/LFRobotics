#ifndef ENCRYPTION_H
#define ENCRYPTION_H

#ifndef AES_BLOCK_SIZE
#define AES_BLOCK_SIZE 16
#endif

#ifndef ulli
#define ulli unsigned long long int
#endif

#ifndef usi
#define usi unsigned short int
#endif

typedef struct
{
    char *key_body;
    usi key_size;
} Key;

typedef struct
{
    char *message_body;
    usi message_size;
} Message;
// Define AES block size and key size
typedef struct
{
    unsigned char state[4][4];
} AES_State;

// Give the buffer location for key storage, the init will make a handshake
void init(ulli *buffer);

// It's like this for now, we will either settle for specific implementation for each structure,
// or we will just conver the type using casting and this will take size as an argument
Message EncryptMessage(Message *message, unsigned char *originalKey);

void AES_Encrypt(unsigned char *message, unsigned char *RoundKey, unsigned char *encrypted);

Message DecryptMessage(Message *message, unsigned char *originalKey);

void AES_Decrypt(unsigned char *encrypted, unsigned char *RoundKey, unsigned char *decrypted);

void KeyExpansion(unsigned char *key, unsigned char *RoundKey);

void AddRoundKey(AES_State *state, unsigned char *RoundKey);

void SubBytes(AES_State *state);

void ShiftRows(AES_State *state);

void MixColumns(AES_State *state);

void InvSubBytes(AES_State *state);

void InvShiftRows(AES_State *state);

void InvMixColumns(AES_State *state);

unsigned char getSBoxValue(unsigned char num);

unsigned char getSBoxInvert(unsigned char num);

usi PKCS7_Unpadding(unsigned char *padded_data, usi padded_size, unsigned char *data);

usi PKCS7_Padding(unsigned char *data, usi data_len, unsigned char *padded_data, usi padded_size);

#endif