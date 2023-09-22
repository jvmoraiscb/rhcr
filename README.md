# RHCR - Remote Haptic Control Robot

RHCR is a scientific initiation project carried out by [NTA's Laboratory](https://nta.ufes.br/) at [UFES](https://www.ufes.br/).

Our goal is to promote a user-friendly robot controller using virtual reality elements, creating an environment where the user can see, feel and control a robot even if they are not in the same place.

## Components

-   #### Wheeltec Ackermann Robot

    ![ackermann](/doc/images/ackermann.jpg)

    -   using ROS [Melodic](http://wiki.ros.org/melodic) (Ubuntu 18.04)

-   #### Novint Falcon

    ![novint falcon](/doc/images/falcon.jpg)

    -   using [libnifalcon](https://github.com/libnifalcon/libnifalcon) library and ROS [Humble](https://docs.ros.org/en/humble/index.html) (Ubuntu 22.04)

-   #### Unity

    ![unity project](/doc/images/unity-project.png)

    -   using [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity) and [ros-sharp](https://github.com/siemens/ros-sharp) libraries

-   #### ROS

    ![ros-terminal](/doc/images/ros-terminal.png)

    -   using [rosbridge](http://wiki.ros.org/rosbridge_suite), [navigation](http://wiki.ros.org/navigation) and others ROS packages

## Installation

The project needs two operating systems, a linux virtual machine to run ROS and Novint Falcon open-source drivers, and a windows environment to run Unity and ROS-Unity packages.

Below is a step-by-step tutorial on how to prepare each environment:

### Linux virtual machine

_Recommended version:_ **_Ubuntu 22.04_**

First, visit https://www.vmware.com to download and install VMWare Workstation Player.

After, visit https://ubuntu.com/download and download the Ubuntu 22.04 image, then open VMWare and proceed to install the OS.

The install settings should look like this:

![vmware config](/doc/images/vmware-config.jpg)

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

And finally, open Ros1Holder object and change the rosbridge url to **ws://10.10.10.10:9090**

![ros1-holder](/doc/images/ros1-holder.jpg)

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

Second, connect to robot access point:

![wheeltec-wifi](/doc/images/wheeltec-wifi.jpg)

Third, open two terminals and connect to robot through ssh protocol:

![wheeltec-ssh](/doc/images/wheeltec-ssh.jpg)

Now, launch navigation package in the first terminal:

```bash
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

Then, launch rosbridge package in the second terminal:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

Finally, open **unity-project** and hit play button.
