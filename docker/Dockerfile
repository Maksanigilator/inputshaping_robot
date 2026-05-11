FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        build-essential \
        git \
        cmake \
        python3-pip \
        python3-numpy \
        python3-scipy \
        python3-matplotlib \
        python3-serial \
        python3-colcon-common-extensions \
        ros-humble-rclpy \
        ros-humble-std-msgs \
        ros-humble-geometry-msgs \
        ros-humble-sensor-msgs \
        ros-humble-builtin-interfaces \
        ros-humble-rosbag2 \
        ros-humble-rosbag2-storage-default-plugins \
        # ament_package generates the env hooks (incl. ament_prefix_path.dsv)
        # for ament_python packages at colcon build time. Missing from the
        # stock ros:humble image; without it our package is built but never
        # added to AMENT_PREFIX_PATH and `ros2 launch` cannot find it.
        ros-humble-ament-package \
        ros-humble-ament-index-python \
        udev \
        tmux \
        less \
        vim-tiny \
        ca-certificates \
        ; \
    rm -rf /var/lib/apt/lists/*

# Python deps that are not packaged for Humble (or are nicer from pip).
# Pinned for reproducibility; bump deliberately.
RUN pip3 install --no-cache-dir \
    "flask==3.0.3" \
    "flask-sock==0.7.0" \
    "simple-websocket==1.0.0" \
    "pyserial==3.5"

# Workspace will be bind-mounted from the host so that edits are live without
# rebuilding the image. The colcon build/install dirs are also volumes.
WORKDIR /workspace

# Mark this image as an inputshaping host so scripts can sanity-check.
ENV INPUTSHAPING_DOCKER=1

COPY entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash

ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
