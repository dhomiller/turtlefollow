FROM ubuntu:jammy

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && \
    apt install -y ros-humble-desktop
RUN apt install -y python3-colcon-common-extensions
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN     export uid=1000 gid=1000
RUN     mkdir -p /home/docker_user
RUN     echo "docker_user:x:${uid}:${gid}:docker_user,,,:/home/docker_user:/bin/bash" >> /etc/passwd
RUN     echo "docker_user:x:${uid}:" >> /etc/group
RUN     mkdir /etc/sudoers.d
RUN     echo "docker_user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/docker_user
RUN     chmod 0440 /etc/sudoers.d/docker_user
RUN     chown ${uid}:${gid} -R /home/docker_user 
USER    docker_user 
ENV     HOME=/home/docker_user 

RUN     mkdir /home/docker_user/ros2
WORKDIR /home/docker_user/ros2
ADD     src/turtlefollow /home/docker_user/ros2

RUN bash -c 'source /opt/ros/humble/setup.bash; colcon build'

RUN     echo . /home/docker_user/ros2/install/setup.bash >> /home/docker_user/.bashrc
CMD ["bash"]
