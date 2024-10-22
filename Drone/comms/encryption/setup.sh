#!/bin/bash

BASEDIR=$(pwd)

# Error handling here, I don't have time to implement it yet


make clean
if [ ! $? ] ; then
	printf "An error has occured during cleanup"
	exit 1
fi

make 
if [ ! $? ] ; then
	printf "An error has occured during project compilation"
	exit 2
fi

printf "Running the example for testing\n"
./examples/example_shared
