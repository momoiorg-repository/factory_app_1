FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
 git python3-pip vim xterm less \
 ros-humble-rmw-fastrtps-cpp

RUN pip3 install pyquaternion matplotlib transforms3d sympy

RUN pip3 install "numpy<2"
# RUN pip3 install numpy==1.26.4

# Create Colcon workspace with external dependencies
WORKDIR /
RUN mkdir -p /project/lib_ws/src
WORKDIR /project/lib_ws/src
COPY build/pc_dependencies.repos .
RUN vcs import < pc_dependencies.repos

# patch downloaded modules before build
WORKDIR /project/lib_ws/src/pymoveit2
COPY ./project/resource/pymoveit2_setup.py setup.py

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /project/lib_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
 && colcon build --symlink-install

WORKDIR /project
COPY ./project .
WORKDIR /root
COPY ./bin bin
COPY ./dds_config/fastdds.xml fastdds.xml

WORKDIR /root

# Build arguments (default values)
ARG ROS_DOMAIN_ID=80
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

RUN echo '# ROS2 setup' >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /project/lib_ws/install/setup.bash" >> ~/.bashrc && \
    echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}' >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds.xml' >> ~/.bashrc && \
    echo 'PATH=$PATH:/root/bin' >> ~/.bashrc

RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-humble-navigation2 \
 ros-humble-nav2-bringup

RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-humble-dynamixel-sdk ros-humble-ros2-control ros-humble-ros2-controllers \
 ros-humble-gripper-controllers ros-humble-moveit