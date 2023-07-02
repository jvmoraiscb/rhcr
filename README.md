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
    - using the [Humble](http://wiki.ros.org/melodic/Installation/Ubuntu) version (Ubuntu 22.04)

- #### Windows environment

  - **Unity** to simulate the environment
    ![unity project](/doc/images/unity-project.png)
    - using the open-source library [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity)

## Installation

As previously mentioned, the project needs two operating systems, a linux environment to run ROS and Novint Falcon open-source drivers, and a windows environment to run Unity and ROS#.

Below is a step-by-step tutorial on how to prepare each environment:

### Linux environment

_Recommended version:_ **_Ubuntu 22.04_**

First, visit https://www.vmware.com to download and install VMWare Workstation Player.

After, visit https://ubuntu.com/download and download the Ubuntu 22.04 image, then open VMWare and proceed to install the OS.

With the system already installed, connect the USB to the virtual machine via:

**Player > Removable Devices > Future Devices FALCON HAPTIC > Connect**

_to facilitate the process and guarantee the functioning of the system, we will use a docker image._

Now, install docker:

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### Windows environment

_Recommended version:_ **_Windows 10_**

First, visit https://unity.com/download to download and install Unity Hub.

After, visit https://unity.com/releases/editor/archive and find **Unity 2020.3.29** version, then click the Unity Hub button and proceed to install the editor.

Now, download this repository, unzip and open **unity-project** folder in Unity (all necessary plugins and libraries are already included in the project).

_Before running the project, it's a good idea to check that both computers are "seeing" each other (just ping them)._

## Running

Now that both systems are set up:

- #### In the Linux environment, run this command and follow the terminal instructions to calibrate the Falcon:

```bash
sudo docker run -it --rm --network host --privileged -v /dev/bus/usb:/dev/bus/usb jvmoraiscb/ros2-falcon
```

- #### In the Windows environment, just hit Unity play button and have fun.
