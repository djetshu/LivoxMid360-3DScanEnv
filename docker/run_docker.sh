#!/bin/bash

CONTAINER_NAME="livox_mid_360"

# Check if the container exists
if [ $(docker ps -a -q -f name=^/${CONTAINER_NAME}$) ]; then
    echo "Container '${CONTAINER_NAME}' already running. Removing it ..."
    docker rm -f $CONTAINER_NAME
    echo "Container '${CONTAINER_NAME}' has been removed."
else
    echo "Container '${CONTAINER_NAME}' does not exist."
fi

echo "Starting '${CONTAINER_NAME}' container."

SRC_PATH="$(pwd)/livox_mid_360/src"
MOUNT_PATH="/livox_mid_360/livox_mid_360_ws/src"

# Start the container. 
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --net=host \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev:/dev" \
    --volume="${SRC_PATH}:${MOUNT_PATH}" \
    --name=$CONTAINER_NAME \
    $CONTAINER_NAME:stable \
    /bin/bash 