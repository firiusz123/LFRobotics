#!/bin/bash
# -u $(id -u):$(id -g)
# --runtime=nvidia

echo "Run spinaker_tensorflow:latest-gpu-jupyter in shell window"

docker run --gpus all -it --rm \
    -v ${PWD}:/code/SpinakerV1 \
    -v ${PWD}/../SpinakerV1Data:/code/SpinakerV1Data \
    spinaker_tensorflow:latest-gpu-jupyter \
    /bin/bash