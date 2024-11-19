#!/bin/bash

echo "Usage: rosbag_data [duckiebot name] [postfix] [duration=30]"

# Return true if a bash variable is unset or set to the empty string
if [ -z "$1" ]
then
    echo "Set duckiebot name"
    exit 1
else
    VEHICLE_NAME=${1}
fi

if [ -z "$2" ]
then
    echo "Set postfix"
    exit 1
else 
    POSTFIX=${2}
fi

if [ -z "$3" ]
then
    DURATION=30
else 
    DURATION=${3}
fi

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )/../rosbag"
DATE="$(date -I)"

# Create log folder if not exist
# mkdir -p ${SCRIPTPATH}

FILE_BAG="${SCRIPTPATH}/${DATE}-${VEHICLE_NAME}-${POSTFIX}.bag"

rosbag record --duration=${DURATION} \
    -O ${FILE_BAG} \
    /${VEHICLE_NAME}/data_sync_node/out/image/compressed \
    /${VEHICLE_NAME}/data_sync_node/out/car_cmd

# If script is run inside container and is set enviroment variable USER_ID and GROUP_ID then check chmod of log files
if [ -n "${USER_ID}" ] && [ -n "${GROUP_ID}" ]
then
    chown -R $USER_ID:$GROUP_ID ${SCRIPTPATH}
fi