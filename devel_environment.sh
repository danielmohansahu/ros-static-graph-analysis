#!/usr/bin/bash

set -eo pipefail

# get path to here
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# build and tag docker image
DOCKER_IMAGE="ros-cg-analysis:latest"
docker build -t $DOCKER_IMAGE -f $SCRIPTPATH/Dockerfile .

# drop into a container
docker run \
    -it \
    --rm \
    --ulimit core=-1 \
    --cap-add=SYS_PTRACE \
    --net=host \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -v /tmp/.docker.xauth:/tmp/.docker.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -v $SCRIPTPATH:/workspace \
    $DOCKER_IMAGE \
    byobu

