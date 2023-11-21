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

_Recommended version:_ **_Windows 11_**

First, we need to install ROS2 on Windows.

Before starting the installation, we need to enable script execution in PowerShell with administrator privileges:

```ps1
Set-ExecutionPolicy Unrestricted
```

Now, follow the instructions provided in the official documentation [ROS2 Humble Installation Windows](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html) (if the link is no longer available, this repository has a copy of the installation guide [here](https://github.com/jvmoraiscb/rhcr/blob/main/doc/ros2-humble-windows.md)).

After the installation, we need to unblock the startup script in PowerShell with administrator privileges:

```ps1
Unblock-File C:\dev\ros2_humble\local_setup.ps1
```

Second, we need to install Unity on Windows.

First, visit https://unity.com/download to download and install Unity Hub.

So, visit https://unity.com/releases/editor/archive and find **Unity 2020.3.29** version, then click the Unity Hub button and proceed to install the editor.

Now, download this repository, unzip and save **unity-project** folder somewhere.

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

Then, open a PowerShell terminal and connect to robot through ssh protocol:

![wheeltec-ssh](/doc/images/wheeltec-ssh.jpg)

Now, launch the robot's default package and set ROS_DOMAIN_ID to 42:

```bash
ROS_DOMAIN_ID=42 roslaunch ros2 launch <robot_package> <robot_package_launch>
```

_Replace **<robot_package>** for the robot ros2 package name, and **<robot_package_launch>** for the launch file name._

Finally, open another PowerShell terminal and run:

```ps1
C:\dev\ros2_humble\local_setup.ps1 ; setx ROS_DOMAIN_ID 42 ; Start-Process -FilePath '<path\to\unity>' -ArgumentList '-projectPath "<path\to\unity-project>"'
```

_Replace **<path\to\unity>** for the path to the Unity executable, and **<path\to\unity-project>** for the path to the project folder._
