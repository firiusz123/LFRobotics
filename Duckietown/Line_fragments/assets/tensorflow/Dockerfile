# define base image
FROM tensorflow/tensorflow:latest-gpu-jupyter

ARG USER_ID
ARG GROUP_ID

RUN apt-get update
RUN apt-get install -y python3-opencv nano python3-tk
# apt-get update && apt-get install libgl1

# Uprgrade pip
RUN pip install -U pip

RUN pip install pandas matplotlib scikit-learn opencv-python

RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user

USER user

WORKDIR /code

# build: docker build -t spinaker_tensorflow:latest-gpu-jupyter --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) .

# https://vsupalov.com/docker-shared-permissions/