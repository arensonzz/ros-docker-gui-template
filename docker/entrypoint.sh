#!/usr/bin/env bash
# Basic entrypoint for ROS Docker containers

# Source ROS 1 Noetic
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash
echo "Sourced ROS 1 Noetic"

# Source the base workspace, if built
if [ -f ~/catkin_ws/devel/setup.bash ]
then
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/catkin_ws/devel/setup.bash
    echo "Sourced catkin_ws base workspace"
    ### Environment variables
    # export TURTLEBOT3_MODEL=waffle_pi
fi

# Execute the command passed into this entrypoint
exec "$@"
