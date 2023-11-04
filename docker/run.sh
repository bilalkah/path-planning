#!/bin/bash

RED='\033[0;31m'        # red
GREEN='\033[0;32m'      # green
YELLOW='\033[1;33m'     # yellow
BLUE='\033[1;34m'       # blue
CYAN='\033[1;36m'       # cyan
NC='\033[0m'            # no color

IMAGE_NAME="path_planning:latest"
CONTAINER_NAME=planning

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$XAUTH:rw"

 xhost +local:docker

if (docker ps --all | grep -q ${CONTAINER_NAME})
then
    printf "${GREEN}---DOCKER CONTAINER IS VALID---\n${NC}"
    if (docker ps | grep -q ${CONTAINER_NAME})
    then
        printf "${YELLOW}---OPENNING DOCKER CONTAINER---\n${NC}"
        docker exec -it ${CONTAINER_NAME} /bin/bash
    else
        printf "${YELLOW}---STARTING DOCKER CONTAINER---\n${NC}"
        docker start ${CONTAINER_NAME} 1>/dev/null 2>&1
        docker exec -it ${CONTAINER_NAME} /bin/bash
    fi
else
    printf "${RED}---CREATING DOCKER CONTAINER---\n${NC}"
    docker run \
        -it \
        --net=host \
        --privileged \
        --runtime=nvidia \
        --env=DISPLAY \
        --env=NVIDIA_VISIBLE_DEVICES=all \
        --env=NVIDIA_DRIVER_CAPABILITIES=all \
        $VOLUMES \
        --name="${CONTAINER_NAME}" \
        $IMAGE_NAME \
        /bin/bash
fi
