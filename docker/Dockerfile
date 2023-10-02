FROM nvidia/cuda:12.1.1-devel-ubuntu22.04

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO humble

# Get tools
RUN apt update && apt install -y curl tmux iproute2 vim sudo\
				 locales software-properties-common
# ROS Humble Setup
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt upgrade -y
RUN apt update && apt install -y ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-rqt*
RUN apt update && apt install -y ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-moveit python3-vcstool python3-colcon-common-extensions

# Install gazebo garden
RUN apt update && apt install -y lsb-release wget gnupg
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update && apt install -y gz-garden

RUN rm -rf /var/lib/apt/lists/*
 
# Prep env
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV QT_X11_NO_MITSHM=1

# Add user
ENV USERNAME ubuntu
RUN useradd -s /bin/bash -m -G sudo $USERNAME && echo "$USERNAME:$USERNAME" | chpasswd && adduser $USERNAME sudo

USER $USERNAME

RUN mkdir -p /home/$USERNAME/workspace/src
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
