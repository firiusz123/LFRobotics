#!/bin/bash

IMAGE_NAME="duckietown/main:latest-amd64"
RUN_PATH="$(dirname -- "${BASH_SOURCE[0]}")"
echo $RUN_PATH
RED='\033[0;32m'
NC='\033[0m' # No Color

RUNDIR=$(dirname -- "$(realpath -- "$0")")
RUNDIR="${DIR}/.."

function usage() {
	# TODO - usage
    	echo "Usage TODO"
    	exit 1
}

DETACH=0
while getopts ":d:" opt; do
    case $opt in
        d) DETACH=1 ;;
        \?) usage ;;
    esac
done

for LAST_ARG; do true; done
echo $LAST_ARG


if [ $DETACH -eq 1 ]
then 
if [ -n "$1" ]
then 
    echo -e "\n*** Run container on host and connect to duckiebot name: ${RED}${LAST_ARG}${NC} ***\n"

    # Set enviromental varaibles
    source "${RUN_PATH}/set_env.sh" ${LAST_ARG}

    # Start container
    docker run -itd --rm \
        -v ${PWD}:/code/catkin_ws/src/Main \
        -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
   	-v /tmp/.X11-unix:/tmp/.X11-unix \
   	-v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    	-e DISPLAY=unix$DISPLAY \
        --network host \
        --privileged \
        --runtime=nvidia \
        --gpus all \
        -e ROS_MASTER_URI=${ROS_MASTER_URI} \
	-e ROS_IP=${ROS_IP} \
        -e ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} \
        -e VEHICLE_NAME=${LAST_ARG} \
        ${IMAGE_NAME} \
	/bin/bash
else
    echo -e "\n*** Run container on host ***\n"

    # Set enviromental varaibles
    source "${RUN_PATH}/set_env.sh"

    # Start container
    docker run -itd --rm \
	      -e ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} \
        -v ${PWD}:/code/catkin_ws/src/Main \
        --network host \
        ${IMAGE_NAME} \
	/bin/bash
fi
else
if [ -n "$1" ]
then 
    echo -e "\n*** Run container on host and connect to duckiebot name: ${RED}${LAST_ARG}${NC} ***\n"

    # Set enviromental varaibles
    source "${RUN_PATH}/set_env.sh" ${LAST_ARG}

    # Start container
    docker run -it --rm \
        -v ${PWD}:/code/catkin_ws/src/Main \
        -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
        -e DISPLAY=unix$DISPLAY \
        --network host \
        --privileged \
        --runtime=nvidia \
        --gpus all \
        -e ROS_MASTER_URI=${ROS_MASTER_URI} \
        -e ROS_IP=${ROS_IP} \
        -e ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} \
        -e VEHICLE_NAME=${LAST_ARG} \
        ${IMAGE_NAME} \
        /bin/bash
else
    echo -e "\n*** Run container on host ***\n"

    # Set enviromental varaibles
    source "${RUN_PATH}/set_env.sh"

    # Start container
    docker run -it --rm \
              -e ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} \
        -v ${PWD}:/code/catkin_ws/src/Main \
        --network host \
        ${IMAGE_NAME} \
        /bin/bash
fi
fi

#https://stackoverflow.com/questions/25185405/using-gpu-from-a-docker-container
