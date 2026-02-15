FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create workspace and copy sources for rosdep
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
COPY . src/hello_vector

# Install all dependencies from package.xml files via rosdep
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Source ROS2 setup in every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc
