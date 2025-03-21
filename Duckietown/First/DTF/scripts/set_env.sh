#!/bin/bash

echo "Usage: source set_env.sh [duckiebot name]"

if [ -n "$1" ] 
then
    # Set ROS_MASTER_URI 
    ROS_MASTER_URI="http://${1}.local:11311/"
    export ROS_MASTER_URI="${ROS_MASTER_URI}"

    # Set ROS_IP
    export ROS_IP="$(hostname  -I | cut -f1 -d' ')"
else 
    unset ROS_MASTER_URI
    unset ROS_IP
fi

echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "ROS_IP=${ROS_IP}"