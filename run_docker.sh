#!/usr/bin/env bash
# Sample script to run a ROS command in a Docker container
#
# Usage Example:
# ./run_docker.sh <image_name> <container_name> <commands>

# Define Docker volumes and environment variables
DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="$XAUTHORITY:/home/docker/.Xauthority:ro" \
--volume="/etc/timezone:/etc/timezone:ro" \
--volume="/etc/localtime:/etc/localtime:ro" \
--volume="./package:/home/docker/catkin_ws/src/hello_world:rw"
"
DOCKER_ENV_VARS="
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
--env="DISPLAY" \
--env="XAUTHORITY=/home/docker/.Xauthority" \
--env="QT_X11_NO_MITSHM=1" \
"
DOCKER_EXPOSE_PORTS="
--expose="22" \
"
DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}" "${DOCKER_EXPOSE_PORTS}

# Run the command
docker run -h $1 --rm -it --gpus="all" --ipc=host --device=/dev/dri:/dev/dri --name="$2" $DOCKER_ARGS $1 bash -c "$3"
