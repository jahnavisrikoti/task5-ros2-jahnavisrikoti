
FROM ros:jazzy-ros-base


RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-jazzy-rclcpp \
        ros-jazzy-std-msgs \
        ros-jazzy-std-srvs \
        ros-jazzy-launch \
        ros-jazzy-launch-ros \
        ros-jazzy-launch-testing-ament-cmake \
        build-essential \
        cmake \
        git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY ros2_ws/src ./src

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        2>&1"

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "drone_fleet", "fleet.launch.py"]
