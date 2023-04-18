#Download base image ubuntu 18.04
FROM ubuntu:18.04

# LABEL about the custom image

LABEL maintainer="jv.moraiscb@gmail.com"
LABEL version="0.1"
LABEL description="RHCR ROS Package Docker image"

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y git curl build-essential udev iproute2 lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get install -y ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool ros-melodic-rosbridge-suite
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc
RUN rosdep init
RUN rosdep update

#RUN chmod +x /opt/ros/melodic/bin/roslaunch
#CMD ["roslaunch rosbridge_server rosbridge_websocket.launch"]
