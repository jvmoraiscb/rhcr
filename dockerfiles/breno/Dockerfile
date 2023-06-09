FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN apt-get install -y git

RUN mkdir -p /ros-workspace/src && \
cd /ros-workspace/src && \
git clone https://github.com/allenh1/rplidar_ros.git

RUN . /opt/ros/humble/setup.sh && \
cd /ros-workspace && \
rosdep install -i --from-path src --rosdistro humble -y && \
colcon build

CMD . /opt/ros/humble/setup.sh && \
. /ros-workspace/install/local_setup.sh && \
ros2 launch rplidar_ros rplidar.launch.py