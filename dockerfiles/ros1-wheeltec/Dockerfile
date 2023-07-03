FROM ros:melodic

LABEL maintainer="jv.moraiscb@gmail.com"
LABEL version="1.0.0"
LABEL description="ROS1 Wheeltec Ackermann Package Docker image"

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
apt-get -y install ros-melodic-rosbridge-suite

RUN rosdep update

CMD ROS_MASTER_URI=http://10.10.10.10:11311 && \
. /opt/ros/melodic/setup.sh && \
roslaunch rosbridge_server rosbridge_websocket.launch