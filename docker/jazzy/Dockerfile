# CI/dev image for ROS 2 Jazzy (001-R15, 001-R17). Base: osrf/ros:jazzy-desktop-full.
# Holds only tooling; sources are mounted/copied at run time (scripts/test-docker.sh). No secrets in the image.
ARG ROS_DISTRO=jazzy
FROM osrf/ros:jazzy-desktop-full
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
RUN apt-get update -qq \
 && apt-get install -y --no-install-recommends git python3-vcstool python3-colcon-common-extensions python3-rosdep \
 && rm -rf /var/lib/apt/lists/*
