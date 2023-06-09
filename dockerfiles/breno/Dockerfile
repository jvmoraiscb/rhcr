FROM ros:noetic

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN apt-get install -y git build-essential

RUN mkdir -p ros-workspace/src

RUN cd ros-workspace/src && \
git clone https://github.com/Slamtec/rplidar_ros.git

RUN . /opt/ros/noetic/setup.sh && \
cd ros-workspace && \
catkin_make

CMD . /ros-workspace/devel/setup.sh && \
roslaunch rplidar_ros rplidar_a2m8.launch