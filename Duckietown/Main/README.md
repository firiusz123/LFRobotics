# Template: Spinaker v1

The project structure based on [https://github.com/duckietown/template-ros](https://github.com/duckietown/template-roshttps:/) template.

Clone this repository, update information in Dockierfile file, especially `REPO_NAME`, `DESCRIPTION`, `MAINTAINER` and change `DISTRO=ente` to `DISTRO=daffy`.

**Code from directory `package` will be copied, during image create process, into created image in localization `/code/catkin_ws/src/REPO_NAME`**

In order to develop code with running container you have to mounted directory with code to the container using options:

`-v ${PWD}:/code/catkin_ws/src/SpinakerV1`

what is done by automaticly by `container_local_start.sh` script.

**Run all scripts form directory `scripts` from parent directory**

## Using

### Container on host computer

Create local docker image
`dts devel build -f -a amd64`
This command create docker container on host computer (architecture amd64)

Run local container from local docker image (with connect to ROS Master running on localhost)
`scripts/container_local_start.sh <duckiebot name ex. d3>`

### Container on robot

Create remote docke image

`dts devel build -f -H d3.local`

This command create docker container on host computer (architecture arm64v8)

Run remote container from remote docker image (on robot)

`scripts/container_duckiebot_start.sh <duckiebot name ex. d3>`

### Running

Next depend on which controller you want to start, execute command:

* to start neural netowork controller
  `roslaunch packages/startNN.launch`
* to start neural pid controller
  `roslaunch packages/start.launch`

After running one of this container you could check how it works by using `rqt` software.

To do this, in next console window set enviromental variables

`source scripts/set_env.sh <duckiebot name ex. d3>`

next run program `rqt`.

Another way to check how duckiebot work is used `rqt` program running on duckiebot. It can be done by executing command:

`dts start_gui_tools <duckiebot name ex. d3>`

## Login data

From inside container execute command

`roslanuch packages/startDataLog.lanuch veh:=<duckiebot name ex. d3> color_line:=<defaul: yellow>`

In next console window set enviromental variable

`source scripts/set_env.sh <duckiebot name ex. d3>`

and execute logging data command

`rosbag record --duration=60 -O <log filename>.bag /<duckiebot name ex. d3>/data_sync_node/out/image/compressed /<duckiebot name ex. d3>/data_sync_node/out/car_cmd`

The file `<log filename>.bag` will be contain logged data.

Example:

`rosbag record --duration=60 -O 2023-01-23-d3-red1-left-v04.bag /d3/data_sync_node/out/image/compressed /d3/data_sync_node/out/car_cmd `

The file `<2023-01-23-d3-red1-left-v04.bag>.bag` will be contain logged data.

The data from `datasync` topic are synchronised, for each image frame exist one value of angular and linera speed.

## Prepare data for NN

Before we start use our logged data to learn neural network we have to extract them from rosbag file. It can be one by running script:

`scripts/prepareDataForNN.py <rosbag file name 1> <rosbag file name 2> /<duckiebot name ex. d3>/data_sync_node/out/car_cmd /<duckiebot name ex. d3>/data_sync_node/out/image/compressed <folder to save results>`

Example (order of topics have to be preserved):

`./scripts/prepareDataForNN.py ../SpinakerV1Data/2023-06-28-d3-yellow-left-v04.bag ../SpinakerV1Data/2023-06-28-d3-yellow-right-v04.bag /d3/data_sync_node/out/car_cmd /d3/data_sync_node/out/image/compressed  ../SpinakerV1Data/2023-06-28-yellow/`


As results script extract images from rosbag to folder `img/` and create csv file data.csv which contains columns like:

* image file name
* linear velocity `v`
* angular velocity `omega`

## Jupiter scripts

The fastes way to start work with neural netowrk is to use jupter scripts. This script can be run localy (if computer has GPU card) or remotely using Google Colab service.

To run jupyter script localy build image `tensorflow/tensorflow:latest-gpu-jupyter` from official dockerhub. This image allow user to use GPU on local computer which significantly speed up computations (especialy learing of neural network).

To build image for jupyter, tensorflow and gpu support execute command:

`docker build -t spinaker_tensorflow:latest-gpu-jupyter --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) .`

from inside directory `assets/tensorflo` where `Dockerfile` is located.

[https://www.tensorflow.org/install/docker](https://www.tensorflow.org/install/docker)

To start jupyter notebook server with gpu support run script:

`scripts/tf_jupyter_start.sh <port number default 8888>`

To start tensorflow enviroment with gpu support run script:

`scripts/tf_shell_start.sh`
