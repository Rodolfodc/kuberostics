# T902 measurement image: ROS 2 + rmw_zenoh + CycloneDDS (for zenoh-bridge-ros2dds) + the bench node.
# Build: docker build --build-arg ROS_DISTRO=lyrical -t t902-bench:lyrical deploy/transport-bench   (009a-R17: ARG ROS_DISTRO)
ARG ROS_DISTRO=lyrical
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-vision-msgs \
      python3-pil iproute2 \
    && rm -rf /var/lib/apt/lists/*
COPY bench.py /opt/bench/bench.py
COPY entrypoint.sh /opt/bench/entrypoint.sh
ENTRYPOINT ["/opt/bench/entrypoint.sh"]
