FROM ubuntu:jammy

ARG DEBIAN_FRONTEND=noninteractive

# Install ros2
RUN apt update && \
    apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt install -y \
        python3-colcon-common-extensions \
        ros-humble-desktop && \
    rm -rf /var/lib/apt/lists/*

# Add source files
ADD     src/turtlefollow /ros2
WORKDIR /ros2

# Build turtlefollow
RUN bash -c 'source /opt/ros/humble/setup.bash; colcon build'

# Source setup file for bash
RUN     echo . /ros2/install/setup.bash >> /root/.bashrc
CMD ["bash"]
