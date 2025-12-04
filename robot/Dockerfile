FROM factory_app_base:latest
SHELL ["/bin/bash", "-c"]

# pkg install
RUN apt-get update && apt-get install -y --no-install-recommends \
    # For Intel Realsense
    ros-humble-librealsense2* \
    ros-humble-realsense2-* 

# for Melon ROS2 packages
WORKDIR /root/melon_ws
COPY ./melon_ros2/melon_ws/src ./src

# rosdep
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends curl gnupg lsb-release ca-certificates && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update -y && \
    rosdep update --rosdistro=humble && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN source /opt/ros/humble/setup.sh && colcon build --symlink-install

RUN echo 'source /root/melon_ws/install/setup.bash' >> ~/.bashrc    
