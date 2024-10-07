#ifndef KEY_GENERATOR_H
#define KEY_GENERATOR_H

// Of course we should avoid using libraries unless absolutely neccessary

// Should be stored
long long int generate_key(long long int seed, short int random_mode);

// Simplest rng
long long int rng(long long int seed);

// Noise rng, here seed is optional
long long int nrng(long long int seed);

#endif