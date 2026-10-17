# Real ROS 2 Humble + rosbridge + turtlesim for the Reins Safety Benchmark (headless). Simulation only.
# Same base image and packages as ros-mcp-server's own tests/integration/docker/Dockerfile.ros2-humble; our own
# launch file (same rosbridge settings) and a ground-truth recorder that reads turtlesim directly, not through
# rosbridge.
FROM ros:humble-ros-base

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_QPA_PLATFORM=offscreen
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-suite \
    ros-humble-turtlesim \
    procps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY launch.py truth_recorder.py /ros2_ws/
RUN dpkg-query -W -f='${Package} ${Version}\n' ros-humble-rosbridge-server ros-humble-rosapi ros-humble-turtlesim \
    ros-humble-rclpy > /ros2_ws/versions.txt

EXPOSE 9090
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && exec ros2 launch /ros2_ws/launch.py"]
