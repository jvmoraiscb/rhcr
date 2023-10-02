# RHCR - Remote Haptic Control Robot

RHCR is a scientific initiation project carried out by [NTA's Laboratory](https://nta.ufes.br/) at [UFES](https://www.ufes.br/).

Our goal is to promote a user-friendly robot controller using virtual reality elements, creating an environment where the user can see, feel and control a robot even if they are not in the same place.

## Components

-   #### Wheeltec Ackermann Robot

    ![ackermann](/doc/images/ackermann.jpg)

    -   using proprietary ROS 2 version by Wheeltec

-   #### Novint Falcon

    ![falcon](/doc/images/falcon.jpg)

    -   using [libnifalcon](https://github.com/libnifalcon/libnifalcon) library and ROS [Humble](https://docs.ros.org/en/humble/index.html) (Ubuntu 22.04)

-   #### Unity

    ![unity-project](/doc/images/unity-project.png)

    -   using [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity) library

-   #### ROS 2

    ![ros-terminal](/doc/images/ros-terminal.png)

    -   using [navigation](http://wiki.ros.org/navigation) and others ROS packages

## Installation

The project needs two operating systems, a linux virtual machine to run ROS and Novint Falcon open-source drivers, and a windows environment to run Unity and ros2-for-unity library.

Below is a step-by-step tutorial on how to prepare each environment:

### Linux virtual machine

_Recommended version:_ **_Ubuntu 22.04_**

First, visit https://www.vmware.com to download and install VMWare Workstation Player.

After, visit https://ubuntu.com/download and download the Ubuntu 22.04 image, then open VMWare and proceed to install the OS.

The network adapter must be in Bridged mode and **ONLY** the **adapter that the robot is connected to** must be turned on (in our case, the Wi-Fi):

![vmware-network](/doc/images/vmware-network.jpg)

In the end, settings should look like this:

![vmware-config](/doc/images/vmware-config.jpg)

_to facilitate the process and guarantee the functioning of the system, we will use a docker image._

After installation, open a terminal and install docker:

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

And pull the ros2-falcon image:

```bash
sudo docker pull jvmoraiscb/ros2-falcon
```

### Windows environment

_Recommended version:_ **_Windows 10_**

First, visit https://unity.com/download to download and install Unity Hub.

After, visit https://unity.com/releases/editor/archive and find **Unity 2020.3.29** version, then click the Unity Hub button and proceed to install the editor.

Now, download this repository, unzip and open **unity-project** folder in Unity (all necessary plugins and libraries are already included in the project).

## Running

_Before running the project, it's a good idea to check that both computers are "seeing" each other (just ping them)._

### Linux virtual machine

First, connect **Novint Falcon** to vmware:

![falcon-vmware](/doc/images/falcon-vmware.png)

Second, run this command and follow the terminal instructions:

```bash
sudo docker run -it --rm --network host --privileged -v /dev/bus/usb:/dev/bus/usb jvmoraiscb/ros2-falcon
```

### Windows environment

First, disable Windows Firewall:

![windows-firewall](/doc/images/windows-firewall.jpg)

Second, go to Windows Advanced network settings and disable **all adapters that are not connected to the same network as the robot** (same way as in vmware):

![windows-network](/doc/images/windows-network.jpg)

Third, connect to the same network as the robot:

![windows-wifi](/doc/images/windows-wifi.jpg)

Then, open a powershell terminal and connect to robot through ssh protocol:

![wheeltec-ssh](/doc/images/wheeltec-ssh.jpg)

Now, launch default package in the first terminal:

```bash
roslaunch ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

Finally, open **unity-project** and hit play button.
