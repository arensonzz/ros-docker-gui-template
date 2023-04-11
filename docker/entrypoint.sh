#!/usr/bin/env bash
# Basic entrypoint for ROS Docker containers

# Source ROS 1 Noetic
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source /opt/ros/noetic/setup.bash
echo 'Sourced ROS 1 Noetic'

# Add aliases
echo "alias catkin_make='catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1'" >> ~/.bashrc

# Source the base workspace, if built
if [ -f ~/catkin_ws/devel/setup.bash ]
then
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
    source ~/catkin_ws/devel/setup.bash
    echo 'Sourced catkin_ws base workspace'
    ### Environment variables
    export CMAKE_EXPORT_COMPILE_COMMANDS=1
fi

# Launch new tmux session and attach to it
exec tmux new-session /bin/bash \; set default-shell /bin/bash

# Execute the command passed into this entrypoint
# exec "$@"
