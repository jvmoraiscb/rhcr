FROM ros:humble

LABEL maintainer="jv.moraiscb@gmail.com"
LABEL version="1.0.0"
LABEL description="ROS2 Slam Package Docker image"

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN apt-get install -y ros-humble-joint-state-publisher ros-humble-xacro ros-humble-slam-toolbox

RUN mkdir -p /ros-workspace/src
COPY ros2-slam/ /ros-workspace/src/ros2-falcon

RUN . /opt/ros/humble/setup.sh && \
cd /ros-workspace && \
rosdep install -i --from-path src --rosdistro humble -y && \
colcon build

CMD . /opt/ros/humble/setup.sh && \
. /ros-workspace/install/local_setup.sh && \
ROS_DOMAIN_ID=42 ros2 launch ros2-slam main.launch.py
