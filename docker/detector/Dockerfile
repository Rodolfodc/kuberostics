# Cloud detector image (009a-R17, R18). One image per ROS distribution: docker build --build-arg ROS_DISTRO=<jazzy|lyrical> .
# Build context: the repository root (docker build -f docker/detector/Dockerfile --build-arg ROS_DISTRO=jazzy .).
# Holds NO weights and NO secrets: the YOLO weights are mounted at /weights (volume) or fetched at run time; see deploy/README.md.
# Runs as a non-root user; needs no privileges.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
# CPU-only wheels (the local K3s cluster has no GPU). Override with another index for a GPU build.
ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cpu
ENV ROS_DISTRO=${ROS_DISTRO} \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    YOLO_CONFIG_DIR=/tmp/ultralytics \
    DETECTOR_WEIGHTS_DIR=/weights

RUN apt-get update -qq \
 && apt-get install -y --no-install-recommends \
      python3-venv python3-pip python3-colcon-common-extensions python3-rosdep \
      python3-opencv python3-numpy \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-sensor-msgs \
 && rm -rf /var/lib/apt/lists/*

# venv with system site packages: rclpy and cv2 from apt, ultralytics and torch from pip (PEP 668 forbids pip into the system).
COPY docker/detector/requirements.txt /tmp/requirements.txt
RUN python3 -m venv --system-site-packages /opt/venv \
 && /opt/venv/bin/pip install --no-cache-dir --extra-index-url "${PYTORCH_INDEX_URL}" -r /tmp/requirements.txt

# Our packages only (no third-party IA368II code). vacuum_detection hosts the detector executable.
WORKDIR /ws
COPY src/vacuum_interfaces src/vacuum_interfaces
COPY src/vacuum_detection src/vacuum_detection
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon --log-base /tmp/log build --install-base /opt/vacuum --build-base /tmp/build \
      --packages-select vacuum_interfaces vacuum_detection \
 && rm -rf /tmp/build /tmp/log /ws

# rosbag2 tooling: the chaos run (tools/chaos/kill_detector.sh) records and analyses bags inside this image.
RUN apt-get update -qq \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros2bag ros-${ROS_DISTRO}-rosbag2-py ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
 && rm -rf /var/lib/apt/lists/*

COPY docker/detector/entrypoint.sh docker/detector/liveness.sh /opt/detector/
RUN chmod 0755 /opt/detector/entrypoint.sh /opt/detector/liveness.sh \
 && useradd --system --uid 10001 --create-home detector \
 && mkdir -p /weights /tmp/ultralytics && chown detector /weights /tmp/ultralytics
USER 10001
VOLUME ["/weights"]
ENTRYPOINT ["/opt/detector/entrypoint.sh"]
# The `detector` executable (parameter `model`) is delivered by vacuum_detection; the node name is fixed by the contract.
CMD ["ros2", "run", "vacuum_detection", "detector", "--ros-args", "-r", "__node:=detector_cloud"]
