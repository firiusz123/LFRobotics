#!/bin/bash

IMAGE_NAME="duckietown/line_fragments"
# IMAGE_NAME="duckietown/dtf:latest-arm64v8"
RUN_PATH="$(dirname -- "${BASH_SOURCE[0]}")"

RED='\033[0;32m'
NC='\033[0m' # No Color

if [ -v $1 ]
then 
    echo -e "Usage container_duckie_start.sh <duckiebot name>"
    exit 1
fi

echo -e "\n*** Run remote container on duckiebot name: ${RED}${1}${NC} ***\n"

# Set enviromental varaibles
source "${RUN_PATH}/set_env.sh"

# -v /home/student/Desktop/a/LFRobotics/Duckietown/First/DTF/packages:/packages \
# Start container
docker -H "${1}.local" \
    run -it --rm \
    --network host \
    -e VEHICLE_NAME=${1} \
    ${IMAGE_NAME} \
    /bin/bash
