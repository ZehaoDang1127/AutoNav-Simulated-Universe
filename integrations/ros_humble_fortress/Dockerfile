# x86 Linux build profile for the pinned AutoNav Humble/Fortress integration.
# The robot repository's JetPack Dockerfile is ARM64-only.
FROM osrf/ros@sha256:438961e3cd77f142039255d85b8eee0934f0f6f06353a9c142ec5d5a22653842

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    python3-scipy \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-joint-state-publisher \
    ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*
