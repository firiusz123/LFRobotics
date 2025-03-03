#!/bin/bash

echo "Usage: source set_env.sh [duckiebot name]"

if [ -n "$1" ] 
then
    # Set ROS_MASTER_URI 
    ROS_MASTER_URI="http://${1}.local:11311/"
    export ROS_MASTER_URI="${ROS_MASTER_URI}"

    # Set ROS_IP
    export ROS_IP="$(hostname  -I | cut -f1 -d' ')"
    
    # Set ROS_PACKAGE_PATH
    unset ROS_PACKAGE_PATH
    export ROS_PACKAGE_PATH='/code/catkin_ws/src/DTF/packages/controllers:/code/catkin_ws/src/DTF/packages/datasync:/code/catkin_ws/src/dt-ros-commons/packages/duckietown:/code/catkin_ws/src/dt-ros-commons/packages/duckietown_msgs:/code/catkin_ws/src/dt-ros-commons/packages/duckietown_protocols:/code/catkin_ws/src/dt-ros-commons/packages/ros_commons:/code/catkin_ws/src/dt-ros-commons/packages/ros_http_api:/code/catkin_ws/src/DTF/packages/sensing:/opt/ros/noetic/share:/code/catkin_ws/src/DTF/packages/controllers:/code/catkin_ws/src/DTF/packages/datasync:/code/catkin_ws/src/DTF/packages/line_detector:/code/catkin_ws/src/DTF/packages/poly_fit:/code/catkin_ws/src/DTF/packages/state_controller:/code/catkin_ws/src/DTF/packages/red_line:/code/catkin_ws/src/DTF/packages/fsm'
else 
    unset ROS_MASTER_URI
    unset ROS_IP
    unset ROS_PACKAGE_PATH
fi

echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "ROS_IP=${ROS_IP}"
echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}"
