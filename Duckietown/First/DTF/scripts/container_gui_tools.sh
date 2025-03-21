#!/bin/bash

IMAGE_NAME="duckietown/dt-gui-tools:daffy-amd64"
RUN_PATH="$(dirname -- "${BASH_SOURCE[0]}")"

RED='\033[0;32m'
NC='\033[0m' # No Color

# If Qt application do not start execute command belov:
xhost +local:root
# https://askubuntu.com/questions/877820/what-are-xhost-and-xhost-si

if [ -v "$1" ]
then 
    echo -e "Usage container_gui_tools.sh <duckiebot name>"
    exit 1
fi

echo -e "\n*** Run container on host computer and connect to duckiebot name: ${RED}${1}${NC} ***\n"

# Set enviromental varaibles
source "${RUN_PATH}/set_env.sh" ${1}

# Start container
docker run -it --rm \
    -v ${PWD}/assets/rqt:/code/catkin_ws/src/dt-gui-tools/rqt \
    -v ${PWD}/rosbag:/code/catkin_ws/src/dt-gui-tools/rosbag \
    -v ${PWD}/scripts:/code/catkin_ws/src/dt-gui-tools/scripts \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    -e DISPLAY=unix$DISPLAY \
    --network host \
    --privileged \
    -e ROS_MASTER_URI=${ROS_MASTER_URI} \
    -e ROS_IP=${ROS_IP} \
    -e VEHICLE_NAME=${1} \
    -e USER_ID=$(id -u) \
    -e GROUP_ID=$(id -g) \
    ${IMAGE_NAME} \
    /bin/bash