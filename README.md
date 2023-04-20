# RHCR - Remote Haptic Control Robot

## Sections

- [Introduction](#introduction)
- [Methods](#methods)
- [Installation](#installation)
  - [Linux](#linux)
    - [ROS Melodic](#ros-melodic)
    - [Libnifalcon](#libnifalcon)
  - [Windows](#windows)
    - [Unity](#unity)

## Introduction

RHCR is a scientific initiation project carried out by [NTA's Laboratory](https://nta.ufes.br/) at [UFES](https://www.ufes.br/).

Our goal is to promote a user-friendly robot controller using virtual reality elements, creating an environment where the user can see, feel and control a robot even if they are not in the same place.

## Methods

- #### Ackermann Robot

  ![ackermann](/doc/images/ackermann.jpg)

- #### Linux environment

  - **Novint Falcon** as the main controller
    ![novint falcon](/doc/images/falcon.jpg)
    - using the open-source library [libnifalcon](https://github.com/libnifalcon/libnifalcon)
  - **ROS** for communication between all components
    ![ros terminal](/doc/images/ros_terminal.png)
    - using the [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) version (Ubuntu 18.04)

- #### Windows environment

  - **Unity** to simulate the environment
    ![unity project](/doc/images/unity-project.png)
    - using the open-source library [ros-sharp](https://github.com/siemens/ros-sharp)

## Installation

As previously mentioned, the project needs two operating systems, a linux environment to run ROS and Novint Falcon open-source drivers, and a windows environment to run Unity and ROS#.

Below is a step-by-step tutorial on how to prepare each environment:

### Linux

_Recommended version:_ **_Ubuntu 18.04_**

Considering that the system has just been installed, it's a good practice run:

```bash
sudo apt-get update
sudo apt-get upgrade
```

and for the next steps:

```bash
sudo apt-get install git curl build-essential udev iproute2 lsb-release
```

#### ROS Melodic

_A more complete tutorial can be found on their official [website](http://wiki.ros.org/melodic/Installation/Ubuntu)._

First, setup your computer to accept software from packages.ros.org:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

setup your keys:

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

install the package:

```bash
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full
```

setup the environment:

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

install the dependencies:

```bash
sudo apt-get install python-rosdep python-rosinstall python-rosinstall-generator python-wstool ros-melodic-rosbridge-suite
```

and finally, initialize and update rosdep:

```bash
sudo rosdep init
rosdep update
```

#### Libnifalcon

_A more complete tutorial can be found on their official [repository](https://github.com/libnifalcon/libnifalcon)._

First, create a temporary folder:

```bash
mkdir ~/rhcr_temp
cd ~/rhcr_temp
```

clone the libnifalcon repository:

```bash
git clone https://github.com/libnifalcon/libnifalcon.git
cd libnifalcon
```

install the dependecies:

```bash
sudo apt-get install cmake libusb-1.0.0
```

create another temporary folder:

```bash
mkdir build
cd build
```

create the makefile:

```bash
cmake -G "Unix Makefiles" ..
```

run makefile and install libraries in /usr/local/:

```bash
make
sudo make install
```

reload libraries:

```bash
sudo /sbin/ldconfig -v
```

set the udev permission:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="cb48", MODE="0666"' > 99-falcon-rules.rules
sudo cp 99-falcon-rules.rules /etc/udev/rules.d
```

reload the udev rules:

```bash
sudo udevadm control --reload-rules
```

and finally, **unplug and replug the falcon** and delete the temporary folder:

```bash
sudo rm -r ~/rhcr_temp/
cd ~/
```

### Windows

_Under construction_
