/*
 * encryption.c
 *
 *  Created on: Oct 14, 2024
 *      Author: Hbrt
 */


// encryption.c
#include "encryption.h"
#include "key_generator.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
// AES constants for AES-256
#define AES_KEY_SIZE 32   // 256 bits
#define AES_BLOCK_SIZE 16 // 128 bits
#define AES_NB 4          // Number of columns (32-bit words) comprising the State
#define AES_NK 8          // Number of 32-bit words comprising the Cipher Key
#define AES_NR 14         // Number of rounds

// S-box table
unsigned char sbox[256] = {
    // 0     1    2      3     4    5     6     7      8    9     A      B    C     D     E     F
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,  // 0
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,  // 1
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,  // 2
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,  // 3
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,  // 4
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,  // 5
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,  // 6
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,  // 7
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,  // 8
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,  // 9
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,  // A
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,  // B
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,  // C
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,  // D
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,  // E
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16}; // F

// Inverse S-box table
unsigned char rsbox[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d};

// Rcon table for KeyExpansion (AES-256 requires up to Rcon[15])
unsigned char Rcon[16] = {
    0x00, // Rcon[0] is not used
    0x01, 0x02, 0x04, 0x08,
    0x10, 0x20, 0x40, 0x80,
    0x1B, 0x36, 0x6C, 0xD8,
    0xAB, 0x4D, 0x9A};

// Shifts cyclically to the left by 1 byte
void RotWord(unsigned char *word)
{
    unsigned char temp = word[0];
    word[0] = word[1];
    word[1] = word[2];
    word[2] = word[3];
    word[3] = temp;
}

// Substitution using the sbox lookup table
void SubWord(unsigned char *word)
{
    for (usi i = 0; i < 4; i++)
    {
        word[i] = sbox[word[i]];
    }
}

// KeyExpansion implementation for AES-256
void KeyExpansion(unsigned char *key, unsigned char *RoundKey)
{
    // The expanded key size for AES-256 is AES_NB*(AES_NR+1)*4 = 4*(14+1)*4 = 240 bytes
    // Copy the original key as the first AES_NK words of RoundKey
    for (usi i = 0; i < AES_NK * 4; i++)
    {
        RoundKey[i] = key[i];
    }

    unsigned char temp[4];
    usi i_key = AES_NK * 4;

    while (i_key < AES_NB * (AES_NR + 1) * AES_NB)
    {
        // Copy the previous 4 bytes into temp
        for (usi i = 0; i < 4; i++)
        {
            temp[i] = RoundKey[i_key - 4 + i];
        }

        if (i_key % (AES_NK * 4) == 0)
        {
            RotWord(temp);
            SubWord(temp);
            temp[0] ^= Rcon[i_key / (AES_NK * 4)];
        }
        else if (AES_NK > 6 && (i_key % (AES_NK * 4) == 16))
        {
            SubWord(temp);
        }

        for (usi i = 0; i < 4; i++)
        {
            RoundKey[i_key] = RoundKey[i_key - (AES_NK * 4)] ^ temp[i];
            i_key++;
        }
    }
}

// Generate key for the round
void AddRoundKey(AES_State *state, unsigned char *RoundKey)
{

    for (usi column = 0; column < AES_NB; column++)
    {
        for (usi row = 0; row < 4; row++)
        {
            state->state[row][column] ^= RoundKey[column * 4 + row];
        }
    }
}

// Substitute bytes like in the lookup table
void SubBytes(AES_State *state)
{
    for (usi r = 0; r < 4; r++)
    {
        for (usi c = 0; c < AES_NB; c++)
        {
            state->state[r][c] = sbox[state->state[r][c]];
        }
    }
}

// Invert the substitution bytes
void InvSubBytes(AES_State *state)
{
    for (usi r = 0; r < 4; r++)
    {
        for (usi c = 0; c < AES_NB; c++)
        {
            state->state[r][c] = rsbox[state->state[r][c]];
        }
    }
}

/* Shift rows
   1. ff ee dd cc -> ee dd cc ff
   2. ff ee dd cc -> dd cc ff ee
   3. ff ee dd cc -> cc ff ee dd
*/
void ShiftRows(AES_State *state)
{
    unsigned char temp;

    // Row 1: shift left by 1
    temp = state->state[1][0];
    state->state[1][0] = state->state[1][1];
    state->state[1][1] = state->state[1][2];
    state->state[1][2] = state->state[1][3];
    state->state[1][3] = temp;

    // Row 2: shift left by 2
    temp = state->state[2][0];
    state->state[2][0] = state->state[2][2];
    state->state[2][2] = temp;
    temp = state->state[2][1];
    state->state[2][1] = state->state[2][3];
    state->state[2][3] = temp;

    // Row 3: shift left by 3 (or shift right by 1)
    temp = state->state[3][3];
    state->state[3][3] = state->state[3][2];
    state->state[3][2] = state->state[3][1];
    state->state[3][1] = state->state[3][0];
    state->state[3][0] = temp;
}

/* xtime operation for MixColumns, basically magic, but it does this:
    (each letter is a bit)
    qwertyui -> (wertyui0 XOR ( ( 0000000q AND 00000001 ) * 00011011 ) )
*/
unsigned char xtime(unsigned char x)
{
    return ((x << 1) ^ (((x >> 7) & 1) * 0x1b));
}

// MixColumns step
void MixColumns(AES_State *state)
{
    unsigned char Tmp, Tm, t;
    for (usi c = 0; c < AES_NB; c++)
    {
        t = state->state[0][c];
        Tmp = state->state[0][c] ^ state->state[1][c] ^ state->state[2][c] ^ state->state[3][c];
        Tm = state->state[0][c] ^ state->state[1][c];
        Tm = xtime(Tm);
        state->state[0][c] ^= Tm ^ Tmp;
        Tm = state->state[1][c] ^ state->state[2][c];
        Tm = xtime(Tm);
        state->state[1][c] ^= Tm ^ Tmp;
        Tm = state->state[2][c] ^ state->state[3][c];
        Tm = xtime(Tm);
        state->state[2][c] ^= Tm ^ Tmp;
        Tm = state->state[3][c] ^ t;
        Tm = xtime(Tm);
        state->state[3][c] ^= Tm ^ Tmp;
    }
}

// InvMixColumns step
void InvMixColumns(AES_State *state)
{
    unsigned char a, b, c, d;
    for (usi c_col = 0; c_col < AES_NB; c_col++)
    {
        a = state->state[0][c_col];
        b = state->state[1][c_col];
        c = state->state[2][c_col];
        d = state->state[3][c_col];

        state->state[0][c_col] = UCharMultiply(a, (unsigned char)0x0e) ^ UCharMultiply(b, (unsigned char)0x0b) ^ UCharMultiply(c, (unsigned char)0x0d) ^ UCharMultiply(d, (unsigned char)0x09);
        state->state[1][c_col] = UCharMultiply(a, (unsigned char)0x09) ^ UCharMultiply(b, (unsigned char)0x0e) ^ UCharMultiply(c, (unsigned char)0x0b) ^ UCharMultiply(d, (unsigned char)0x0d);
        state->state[2][c_col] = UCharMultiply(a, (unsigned char)0x0d) ^ UCharMultiply(b, (unsigned char)0x09) ^ UCharMultiply(c, (unsigned char)0x0e) ^ UCharMultiply(d, (unsigned char)0x0b);
        state->state[3][c_col] = UCharMultiply(a, (unsigned char)0x0b) ^ UCharMultiply(b, (unsigned char)0x0d) ^ UCharMultiply(c, (unsigned char)0x09) ^ UCharMultiply(d, (unsigned char)0x0e);
    }
}

/* InvShiftRows step
   1. ff ee dd cc <- ee dd cc ff
   2. ff ee dd cc <- dd cc ff ee
   3. ff ee dd cc <- cc ff ee dd
*/
void InvShiftRows(AES_State *state)
{
    unsigned char temp;

    temp = state->state[1][3];
    state->state[1][3] = state->state[1][2];
    state->state[1][2] = state->state[1][1];
    state->state[1][1] = state->state[1][0];
    state->state[1][0] = temp;

    temp = state->state[2][0];
    state->state[2][0] = state->state[2][2];
    state->state[2][2] = temp;
    temp = state->state[2][1];
    state->state[2][1] = state->state[2][3];
    state->state[2][3] = temp;

    temp = state->state[3][0];
    state->state[3][0] = state->state[3][1];
    state->state[3][1] = state->state[3][2];
    state->state[3][2] = state->state[3][3];
    state->state[3][3] = temp;
}

// Galois Field multiplication (in 256 bit field with + - * ^-1)
unsigned char UCharMultiply(unsigned char x, unsigned char y)
{
    unsigned char result = 0;
    unsigned char temp = x;

    for (usi i = 0; i < 8; i++)
    {
        if (y & 1)
            result ^= temp;
        unsigned char high_bit = temp & 0x80;
        temp <<= 1;
        if (high_bit)
            temp ^= 0x1b; // Reduction polynomial
        y >>= 1;
    }

    return result;
}

// Initialize AES State from message
void InitState(unsigned char *message, AES_State *state)
{
    for (usi i = 0; i < AES_BLOCK_SIZE; i++)
    {
        state->state[i % 4][i / 4] = message[i];
    }
}

// Extract message from AES State
void ExtractMessage(unsigned char *message, AES_State *state)
{
    for (usi i = 0; i < AES_BLOCK_SIZE; i++)
    {
        message[i] = state->state[i % 4][i / 4];
    }
}

// AES Encryption Function (AES-256)
void AES_Encrypt(unsigned char *message, unsigned char *RoundKey, unsigned char *encrypted)
{
    AES_State state;
    InitState(message, &state);

    // Initial AddRoundKey
    AddRoundKey(&state, RoundKey);

    // 13 Main Rounds
    for (usi round = 1; round < AES_NR; round++)
    {
        SubBytes(&state);
        ShiftRows(&state);
        MixColumns(&state);
        AddRoundKey(&state, RoundKey + (round * AES_BLOCK_SIZE));
    }

    // Final Round
    SubBytes(&state);
    ShiftRows(&state);
    AddRoundKey(&state, RoundKey + (AES_NR * AES_BLOCK_SIZE));

    // Extract encrypted message
    ExtractMessage(encrypted, &state);
}

// AES Decryption Function (AES-256)
void AES_Decrypt(unsigned char *encrypted, unsigned char *RoundKey, unsigned char *decrypted)
{
    AES_State state;
    InitState(encrypted, &state);

    // Initial AddRoundKey
    AddRoundKey(&state, RoundKey + (AES_NR * AES_BLOCK_SIZE));

    // 13 Main Rounds
    for (usi round = AES_NR - 1; round > 0; round--)
    {
        InvShiftRows(&state);
        InvSubBytes(&state);
        AddRoundKey(&state, RoundKey + (round * AES_BLOCK_SIZE));
        InvMixColumns(&state);
    }

    // Final Round
    InvShiftRows(&state);
    InvSubBytes(&state);
    AddRoundKey(&state, RoundKey);

    // Extract decrypted message
    ExtractMessage(decrypted, &state);
}

// Encrypt Message
Message EncryptChunk(Message *message, unsigned char *originalKey)
{
    Message encrypted_message;
    encrypted_message.message_size = AES_BLOCK_SIZE;
    encrypted_message.message_body = (unsigned char *)malloc(AES_BLOCK_SIZE);

    unsigned char *RoundKey; // 16 * 15 = 240 bytes for AES-256
    RoundKey = (unsigned char *)malloc(AES_BLOCK_SIZE * AES_BLOCK_SIZE * (+1));
    // Key Expansion
    KeyExpansion(originalKey, RoundKey);

    // // Ensure message size is 16 bytes (AES block size)
    // // Implement padding if necessary (e.g., PKCS#7)
    usi status = PKCS7_Padding(message->message_body, message->message_size, encrypted_message.message_body, message->message_size + (16 - (message->message_size % 16)));
      if (status == -1)
      {
     // Should add some error handling...
          Message empty;
          empty.message_size = 0;
    	  return empty;
      }
    unsigned char plaintext[AES_BLOCK_SIZE] = {0};
    for (usi i = 0; i < AES_BLOCK_SIZE && i < message->message_size; i++)
    {
        plaintext[i] = (unsigned char)message->message_body[i];
    }

    unsigned char ciphertext[AES_BLOCK_SIZE] = {0};
    AES_Encrypt(plaintext, RoundKey, ciphertext);

    // // Prepare encrypted message
    if (encrypted_message.message_body == NULL)
    {
        // Handle malloc failure
        // For now, set size to 0
        encrypted_message.message_size = 0;
        return encrypted_message;
    }
    for (usi i = 0; i < AES_BLOCK_SIZE; i++)
    {
        encrypted_message.message_body[i] = (unsigned char)ciphertext[i];
    }
    free(RoundKey);
    // return *encrypted_message;
    return encrypted_message;
}

// Decrypt Message
Message DecryptChunk(Message *message, unsigned char *originalKey)
{
    Message decrypted_message;
    unsigned char *RoundKey; // 16 * 15 = 240 bytes for AES-256
    RoundKey = (unsigned char *)malloc(AES_BLOCK_SIZE * AES_BLOCK_SIZE * (+1));
    // Key Expansion
    KeyExpansion(originalKey, RoundKey);

    unsigned char ciphertext[AES_BLOCK_SIZE] = {0};
    for (usi i = 0; i < AES_BLOCK_SIZE && i < message->message_size; i++)
    {
        ciphertext[i] = (unsigned char)message->message_body[i];
    }

    unsigned char plaintext[AES_BLOCK_SIZE] = {0};
    AES_Decrypt(ciphertext, RoundKey, plaintext);

    // Prepare decrypted message
    decrypted_message.message_size = AES_BLOCK_SIZE;
    decrypted_message.message_body = (unsigned char *)malloc(AES_BLOCK_SIZE);
    if (decrypted_message.message_body == NULL)
    {
        // Handle malloc failure
        decrypted_message.message_size = 0;
        return decrypted_message;
    }
    for (usi i = 0; i < AES_BLOCK_SIZE; i++)
    {
        decrypted_message.message_body[i] = (char)plaintext[i];
    }
    Message *final_message = (Message *)malloc(8); // Message size
    final_message->message_body = (unsigned char *)malloc(AES_BLOCK_SIZE);
    PKCS7_Unpadding(decrypted_message.message_body, decrypted_message.message_size, final_message->message_body);
    return decrypted_message;
}

// Function to apply PKCS#7 padding
usi PKCS7_Padding(unsigned char *data, usi data_len, unsigned char *padded_data, usi padded_size)
{
    usi pad_len = AES_BLOCK_SIZE - (data_len % AES_BLOCK_SIZE);
    if (pad_len == 0)
        pad_len = AES_BLOCK_SIZE;

    if ((data_len + pad_len) > padded_size)
        return -1; // Not enough space

    memcpy(padded_data, data, data_len);
    for (usi i = data_len; i < data_len + pad_len; i++)
    {
        padded_data[i] = pad_len;
    }

    return data_len + pad_len;
}

// Function to remove PKCS#7 padding
usi PKCS7_Unpadding(unsigned char *padded_data, usi padded_size, unsigned char *data)
{
    if (padded_size == 0 || padded_size % AES_BLOCK_SIZE != 0)
        return -1; // Invalid padding

    unsigned char pad_len = padded_data[padded_size - 1];
    if (pad_len == 0 || pad_len > AES_BLOCK_SIZE)
        return -1; // Invalid padding

    // Verify padding
    for (usi i = padded_size - pad_len; i < padded_size; i++)
    {
        if (padded_data[i] != pad_len)
            return -1; // Invalid padding
    }

    if (pad_len > padded_size)
        return -1; // Invalid padding

    memcpy(data, padded_data, padded_size - pad_len);
    return padded_size - pad_len;
}

Message DecryptMessage(Message *message, unsigned char *key)
{
    usi pointer = 0;
    Message encrypted;
    encrypted.message_size = message->message_size;
    encrypted.message_body = (unsigned char *)malloc(message->message_size);
    while (pointer < message->message_size)
    {
        unsigned char table[16] = {0};
        for (int j = pointer; j < pointer + AES_BLOCK_SIZE; j++)
        {
            if (j > message->message_size)
            {
                table[j % AES_BLOCK_SIZE] = 0;
                continue;
            }
            table[j % AES_BLOCK_SIZE] = message->message_body[j];
        }
        Message block;
        block.message_body = (unsigned char *)table;
        block.message_size = AES_BLOCK_SIZE;
        Message ciphertext = DecryptChunk(&block, key);
        for (usi i = 0; i < AES_BLOCK_SIZE; i++)
        {
            encrypted.message_body[pointer + i] = ciphertext.message_body[i];
            // printf("%02x ", (unsigned char)ciphertext.message_body[i]);
        }
        pointer += AES_BLOCK_SIZE;
    }
    printf("\n");
    return encrypted;
}

Message EncryptMessage(Message *message, unsigned char *key)
{
    usi pointer = 0;
    Message encrypted;
    encrypted.message_size = message->message_size;
    encrypted.message_body = (unsigned char *)malloc(message->message_size);
    while (pointer < message->message_size)
    {
        unsigned char table[16] = {0};
        for (int j = pointer; j < pointer + AES_BLOCK_SIZE; j++)
        {
            if (j > message->message_size)
            {
                table[j % AES_BLOCK_SIZE] = 0;
                continue;
            }
            table[j % AES_BLOCK_SIZE] = message->message_body[j];
        }
        Message block;
        block.message_body = (unsigned char *)table;
        block.message_size = AES_BLOCK_SIZE;
        Message ciphertext = EncryptChunk(&block, key);
        for (usi i = 0; i < AES_BLOCK_SIZE; i++)
        {
            encrypted.message_body[pointer + i] = ciphertext.message_body[i];
            // printf("%02x ", (unsigned char)ciphertext.message_body[i]);
        }
        pointer += AES_BLOCK_SIZE;
    }
    printf("\n");
    return encrypted;
}
