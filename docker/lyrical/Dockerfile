# CI/dev image for ROS 2 Lyrical Luth (001-R15, 001-R17). Base: ros:lyrical-ros-base (tag confirmed to exist).
# Holds only tooling; sources are mounted/copied at run time (scripts/test-docker.sh). No secrets in the image.
ARG ROS_DISTRO=lyrical
FROM ros:lyrical-ros-base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
RUN apt-get update -qq \
 && apt-get install -y --no-install-recommends git python3-vcstool python3-colcon-common-extensions python3-rosdep \
 && rm -rf /var/lib/apt/lists/*
