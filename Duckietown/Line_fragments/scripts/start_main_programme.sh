#!/bin/bash

function usage() {
    printf "\nUsage: ./start_main_programme.sh [OPTIONS] -n [CONTAINER_NAME] -p [ROSLAUNCH_FILE_NAME]\n"
    printf "\nOptions:\n\n"
    printf "\t-d\tDetach script, run in the background\n"
    printf "\nArguments:\n\n"
    printf "\t-n\tName of the container that the launch will be running in\n"
    printf "\t-p\tPath or name of the roslaunch file\n"
    printf "\n"
    exit 1
}

# Default values
MAIN_LAUNCH_FILE='startFSM.launch'
CONTAINER_NAME=""
DETACH=0

# Parse command-line arguments
while getopts ":dn:p:" opt; do
    case $opt in
        n) CONTAINER_NAME="$OPTARG" ;;
        p) MAIN_LAUNCH_FILE="$OPTARG" ;;
        d) DETACH=1 ;;
        \?) usage ;;
    esac
done

# Check if container name is provided
if [ -z "$CONTAINER_NAME" ]; then
    usage
fi

# Command to execute inside Docker
CMD="python3 /code/catkin_ws/devel/_setup_util.py; source /code/catkin_ws/devel/setup.bash; /opt/ros/noetic/bin/roslaunch ./packages/${MAIN_LAUNCH_FILE}"

# Run Docker command with or without detaching
if [ $DETACH -eq 1 ]; then
    docker exec -itd "${CONTAINER_NAME}" /bin/bash -c "$CMD"
else
    docker exec -it "${CONTAINER_NAME}" /bin/bash -c "$CMD"
fi

