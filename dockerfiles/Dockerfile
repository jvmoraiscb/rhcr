FROM ros:humble

LABEL maintainer="jv.moraiscb@gmail.com"
LABEL version="0.1"
LABEL description="RHCR ROS2 Package Docker image"

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
COPY rhcr/ /ros-workspace/src/rhcr

RUN . /opt/ros/humble/setup.sh && \
cd /ros-workspace && \
rosdep install -i --from-path src --rosdistro humble -y && \
colcon build

CMD . /opt/ros/humble/setup.sh && \
. /ros-workspace/install/local_setup.sh && \
ros2 run rhcr main