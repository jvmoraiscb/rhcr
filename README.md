# RHCR - Remote Haptic Control Robot

RHCR is a scientific initiation project carried out by [NTA's Laboratory](https://nta.ufes.br/) at [UFES](https://www.ufes.br/).

Our goal is to promote a user-friendly robot controller using virtual reality elements, creating an environment where the user can see, feel and control a robot even if they are not in the same place.

## Components

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

### Linux environment

_Recommended version:_ **_Ubuntu 18.04_**

Considering that the system has just been installed, it's a good practice run:

```bash
sudo apt-get update
sudo apt-get upgrade
```

and for the next steps:

```bash
sudo apt-get install -y git curl build-essential udev iproute2 lsb-release
```

#### Installing ROS Melodic

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
sudo apt-get install -y ros-melodic-desktop-full
```

setup the environment:

```bash
printf "\nsource /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

install the dependencies:

```bash
sudo apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool ros-melodic-rosbridge-suite
```

and finally, initialize and update rosdep:

```bash
sudo rosdep init
rosdep update
```

#### Installing libnifalcon

_A more complete tutorial can be found on their official [repository](https://github.com/libnifalcon/libnifalcon)._

First, go to linux temporary folder:

```bash
cd /tmp
```

clone the libnifalcon repository:

```bash
git clone https://github.com/libnifalcon/libnifalcon.git
cd libnifalcon
```

install the dependecies:

```bash
sudo apt-get install -y cmake libusb-1.0.0
```

create a build folder:

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

and finally, **unplug and replug the falcon**

#### Setting up ROS workspace

First, go to linux temporary folder:

```bash
cd /tmp
```

clone this repository:

```bash
git clone https://github.com/jvmoraiscb/rhcr.git
cd rhcr
```

create a ROS workspace folder in home **if it doesn't already exist**:

```bash
mkdir ~/ros-workspace ~/ros-workspace/src
```

copy the ros-package to ros-workspace/src:

```bash
cp -r ros-package ~/ros-workspace/src
```

go to ros-workspace and run catkin make:

```bash
cd ~/ros-workspace
catkin_make
```

and finally, setup the environment:

```bash
printf "\nsource ~/ros-workspace/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Windows environment

_Recommended version:_ **_Windows 10_**

First, visit https://unity.com/download to download and install Unity Hub.

After, visit https://unity.com/releases/editor/archive and find **Unity 2020.3.29** version, then click the Unity Hub button and proceed to install the editor.

Now, download 

## Running

Now that both systems are set up:

- #### In the Linux environment:

  - First, open a terminal and start the websocket:

  ```bash
  roslaunch rosbridge_server rosbridge_websocket.launch
  ```

  - open **another** terminal and run the package:

  ```bash
  rosrun rhcr main
  ```

  - Follow the second terminal instructions to calibrate the Falcon.

- #### In the Windows environment:

  - Just hit unity play button.
