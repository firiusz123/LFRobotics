#!/bin/bash

IMAGE_NAME="duckietown/dtf:latest-amd64"
RUN_PATH="$(dirname -- "${BASH_SOURCE[0]}")"
echo $RUN_PATH
RED='\033[0;32m'
NC='\033[0m' # No Color

RUNDIR=$(dirname -- "$(realpath -- "$0")")
RUNDIR="${DIR}/.."

if [ -n "$1" ]
then 
    echo -e "\n*** Run container on host and connect to duckiebot name: ${RED}${1}${NC} ***\n"

    # Set enviromental varaibles
    source "${RUN_PATH}/set_env.sh" ${1}

    # Start container
    docker run -it --rm \
        -v ${PWD}:/code/catkin_ws/src/DTF \
        -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
        --network host \
        --privileged \
        -e ROS_MASTER_URI=${ROS_MASTER_URI} \
	-e ROS_IP=${ROS_IP} \
        -e ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} \
        -e VEHICLE_NAME=${1} \
        ${IMAGE_NAME} \
	/bin/bash
else
    echo -e "\n*** Run container on host ***\n"

    # Set enviromental varaibles
    source "${RUN_PATH}/set_env.sh"

    # Start container
    docker run -it --rm \
	      -e ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} \
        -v ${PWD}:/code/catkin_ws/src/DTF \
        --network host \
        ${IMAGE_NAME} \
	/bin/bash
fi

#https://stackoverflow.com/questions/25185405/using-gpu-from-a-docker-container
