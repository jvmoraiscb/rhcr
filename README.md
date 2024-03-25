# RHCR - Remote Haptic Control Robot

RHCR is a scientific initiation project carried out by [NTA's Laboratory](https://nta.ufes.br/) at [UFES](https://www.ufes.br/).

Our goal is to promote a user-friendly robot controller using virtual reality elements, creating an environment where the user can see, feel and control a robot even if they are not in the same place.

## Components

-   #### Wheeltec Ackermann Robot

    ![ackermann](/Documentation/images/ackermann.jpg)

    -   using [proprietary ROS 2 version](https://wheeltec.net/) by Wheeltec.

-   #### Meta Quest 2

    ![quest2](/Documentation/images/quest2.jpg)

    -   using [Oculus](https://www.meta.com/quest/setup/) software by Meta.

-   #### Novint Falcon

    ![falcon](/Documentation/images/falcon.jpg)

    -   using [libnifalcon](https://github.com/libnifalcon/libnifalcon) library.

-   #### Unity

    ![unity-project](/Documentation/images/unity-project.png)

    -   using [XR](https://docs.unity3d.com/Manual/XR.html) and [ROS-TCP-Connector](https://github.com/RobotecAI/ros2-for-unity) packages.

-   #### ROS 2

    ![ros-terminal](/Documentation/images/ros-terminal.png)

    -   using [ros2-falcon](https://github.com/jvmoraiscb/ros2-falcon), [navigation2](https://github.com/ros-planning/navigation2), [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) and [ros-tcp-endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) packages.

## Installation

The project needs two operating systems, a linux virtual machine to run ROS2 and connect to Novint Falcon, and a windows environment to run Unity and connect to Quest2.

Below is a step-by-step tutorial on how to prepare each environment:

### Linux virtual machine

_Recommended version:_ **_Ubuntu 22.04_**

First, visit https://www.vmware.com to download and install VMWare Workstation Player.

After, visit https://ubuntu.com/download and download the Ubuntu 22.04 image, then open VMWare and proceed to install the OS.

The network adapter must be in Bridged mode and **ONLY** the **adapter that the robot is connected to** must be turned on (in our case, the Wi-Fi):

![vmware-network](/Documentation/images/vmware-network.jpg)

After installation, install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html):

```bash
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
```

Create a rhcr_ws and clone the packages:
```bash
mkdir -p rhcr_ws/src
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

![falcon-vmware](/Documentation/images/falcon-vmware.png)

Second, run this command and follow the terminal instructions:

```bash
sudo docker run -it --rm --network host --privileged -v /dev/bus/usb:/dev/bus/usb jvmoraiscb/ros2-falcon
```

### Windows environment

First, disable Windows Firewall:

![windows-firewall](/Documentation/images/windows-firewall.jpg)

Second, go to Windows Advanced network settings and disable **all adapters that are not connected to the same network as the robot** (same way as in vmware):

![windows-network](/Documentation/images/windows-network.jpg)

Third, connect to the same network as the robot:

![windows-wifi](/Documentation/images/windows-wifi.jpg)

Then, open a PowerShell terminal and connect to robot through ssh protocol:

![wheeltec-ssh](/Documentation/images/wheeltec-ssh.jpg)

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
