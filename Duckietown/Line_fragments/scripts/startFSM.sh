#!/bin/bash

for LAST_ARG; do true; done
echo $LAST_ARG


# Run the first script with the provided argument
cd ..
echo "RUNNING INSIDE ${LAST_ARG}"
./scripts/container_local_start.sh -d "$LAST_ARG"
cd scripts
# Find the newest active Docker container
# CONTAINER_NAME=$(docker ps --format "{{.Names}}" --sort=starttime | tail -n 1)
CONTAINER_NAME=$(docker ps --format "{{.Names}}" | head -n 1)
# Ensure a container was found
if [ -z "$CONTAINER_NAME" ]; then
    echo "No active Docker container found!"
    exit 1
fi

# Run the main program and debugger in the newest container
./start_main_programme.sh -d -n "$CONTAINER_NAME" -p startFSM.launch
./start_main_programme.sh -d -n "$CONTAINER_NAME" -p startDebugger.launch

