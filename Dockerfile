FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-jazzy-cv-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-image-transport \
    ros-jazzy-moveit \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY . /workspace

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

