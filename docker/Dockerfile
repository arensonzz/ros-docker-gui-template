FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# Install system wide packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    xauth \
    git \
    iproute2 \
    sudo

# Create user and add user to the sudoers group
RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo

USER docker

# Create a Catkin workspace and clone repos
RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p ~/catkin_ws/src \
 && cd ~/catkin_ws/src \
 && catkin_init_workspace
 # && git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git \
 
# Build the Catkin workspace
RUN source /opt/ros/noetic/setup.bash \
 && cd ~/catkin_ws \
 && catkin_make
 
# Set the working flder at startup
WORKDIR /home/docker/catkin_ws

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]   

