ARG ROS_DISTRO=humble
ARG UBUNTU_VERSION=jammy
FROM ubuntu:${UBUNTU_VERSION}

LABEL maintainer="travka <https://github.com/travka>"
LABEL org.opencontainers.image.description="ROS2 ${ROS_DISTRO} - Multi-platform build"
LABEL org.opencontainers.image.source="https://github.com/travka/ros2-builders"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

WORKDIR /root
