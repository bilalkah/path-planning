#!/bin/bash

RED='\033[0;31m'        # red
GREEN='\033[0;32m'      # green
YELLOW='\033[1;33m'     # yellow
BLUE='\033[1;34m'       # blue
CYAN='\033[1;36m'       # cyan
NC='\033[0m'            # no color

IMAGE_NAME="path_planning:latest"

printf "${RED}---BUILDING THE DOCKERFILE---\n${NC}"

docker build \
    --no-cache \
    --tag $IMAGE_NAME \
    -f Dockerfile .
