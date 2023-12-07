FROM ros:humble

LABEL maintainer="jv.moraiscb@gmail.com"
LABEL version="1.0.2"
LABEL description="ROS2 Novint Falcon Package Docker image"

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN apt-get install -y git build-essential cmake libusb-1.0.0

RUN cd /tmp && \
git clone https://github.com/libnifalcon/libnifalcon.git && \
cd libnifalcon && \
mkdir build && \
cd build && \
cmake -G "Unix Makefiles" .. && \
make && \
make install && \
/sbin/ldconfig -v

RUN mkdir -p /ros-workspace/src
COPY ros-package/ /ros-workspace/src/ros2-falcon

RUN . /opt/ros/humble/setup.sh && \
cd /ros-workspace && \
rosdep install -i --from-path src --rosdistro humble -y && \
colcon build

CMD . /opt/ros/humble/setup.sh && \
. /ros-workspace/install/local_setup.sh && \
ROS_DOMAIN_ID=42 ros2 run ros2-falcon main
