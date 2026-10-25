# Real ROS 2 Humble for grading unitree-go2-mcp-server (headless). Simulation only: nothing here talks to a robot.
# Two containers run from this image on a private Docker network, as a PC and a Go2 would sit on one Ethernet link:
#   "pc"    the unmodified MCP server (pinned commit) and the real `ros2` CLI it shells out to;
#   "robot" a stand-in Go2 (go2_standin.py) that subscribes to /wirelesscontroller and integrates motion.
# Both use CycloneDDS, as unitree_ros2 does, and the unitree_go messages from unitree_ros2 (pinned).
FROM ros:humble-ros-base

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    iproute2 \
    procps \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# unitree_go message package from unitree_ros2 (BSD-3-Clause), built for Humble. Only the messages are used.
ARG UNITREE_ROS2_SHA=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
ADD https://github.com/unitreerobotics/unitree_ros2/archive/${UNITREE_ROS2_SHA}.tar.gz /tmp/unitree_ros2.tar.gz
RUN mkdir -p /opt/unitree_ros2 \
    && tar -xzf /tmp/unitree_ros2.tar.gz -C /opt/unitree_ros2 --strip-components=1 \
    && rm /tmp/unitree_ros2.tar.gz \
    && cd /opt/unitree_ros2/cyclonedds_ws \
    && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select unitree_go --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf build log

# The MCP server under test (Apache-2.0), unmodified, with the dependency versions of its own uv.lock.
ARG GO2_MCP_SHA=e7a6737dc9fd1e107b7b0fbbaf4a66a749c0e6a7
ADD https://github.com/lpigeon/unitree-go2-mcp-server/archive/${GO2_MCP_SHA}.tar.gz /tmp/go2-mcp.tar.gz
RUN mkdir -p /opt/go2-mcp \
    && tar -xzf /tmp/go2-mcp.tar.gz -C /opt/go2-mcp --strip-components=1 \
    && rm /tmp/go2-mcp.tar.gz \
    && pip3 install --no-cache-dir uv==0.11.32 \
    && cd /opt/go2-mcp \
    && UV_PYTHON_DOWNLOADS=never uv sync --frozen --python /usr/bin/python3 \
    && echo "${GO2_MCP_SHA}" > /opt/go2-mcp/COMMIT

# server.py hard-codes UNITREE_ROS2_SETUP_SH_PATH = "/home/lpigeon/unitree_ros2/setup.sh". Our setup.sh sits at that
# exact path, so the server runs without a patch. It is unitree_ros2's setup.sh for Humble and this network.
COPY setup.sh /home/lpigeon/unitree_ros2/setup.sh
COPY run_server.sh go2_standin.py /opt/botinfo/
RUN chmod +x /home/lpigeon/unitree_ros2/setup.sh /opt/botinfo/run_server.sh \
    && (dpkg-query -W -f='${Package} ${Version}\n' ros-humble-rclpy ros-humble-ros2topic ros-humble-rmw-cyclonedds-cpp \
        ros-humble-cyclonedds; \
        echo "unitree_ros2 ${UNITREE_ROS2_SHA}"; echo "unitree-go2-mcp-server ${GO2_MCP_SHA}"; \
        cd /opt/go2-mcp && .venv/bin/python -c "import importlib.metadata as m; print('mcp', m.version('mcp'))"; \
        .venv/bin/python --version) > /opt/botinfo/versions.txt

CMD ["sleep", "infinity"]
