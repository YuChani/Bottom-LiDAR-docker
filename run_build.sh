#!/bin/bash
export PROJECT_NAME="bottom-lidar"
# Function to convert to absolute path
to_abs_path() {
    local RELATIVE_PATH=$(echo "`dirname $0`"/$@)
    echo $(realpath $RELATIVE_PATH)
}

# Prompt for user inputs
read -p "Enter username: " USERNAME
read -p "Enter work folder name: " WORK_FOLDER


read -p "Enter Docker image tag (default: latest): " DOCKER_TAG
# if not use latest tag, do not build image
if [ -z "$DOCKER_TAG" ]; then
    DOCKER_TAG="latest"
    DOCKER_BUILD="yes"
fi

# Ensure environment variables are set for Docker Compose
export USERNAME
export WORK_FOLDER
export __ROS_MASTER_URI="http://127.0.0.1:11311"
export __ROS_HOSTNAME="127.0.0.1"
export DOCKER_TAG=${DOCKER_TAG:-latest}

# Set the appropriate Docker Compose file based on the platform
COMPOSE_FILE="./docker-compose/${PROJECT_NAME}_compose.yml"

# Define the commands to be executed
build_command=(
    "docker-compose"
    "--file"
    "${COMPOSE_FILE}"
    "build"
    "${PROJECT_NAME}"
)

up_build_command=(
    "docker-compose"
    "--file"
    "${COMPOSE_FILE}"
    "up"
    "${PROJECT_NAME}"
    "-d"
)

exec_command=(
    "docker"
    "exec"
    "-it"
    "${PROJECT_NAME}"
    "/bin/bash"
    "-c"
    "source /opt/ros/noetic/setup.bash && cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash"
)

stop_command=(
    "docker"
    "stop"
    "${PROJECT_NAME}"
)

# Run the commands
if [ "$DOCKER_BUILD" == "yes" ]; then
    echo "Building Docker image..."
    echo ${build_command[@]}
    "${build_command[@]}"
fi

"${up_build_command[@]}"

"${exec_command[@]}"

"${stop_command[@]}"