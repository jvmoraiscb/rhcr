# Disclaimer

This is an incomplete installation guide and is provided as a copy of the installation guide available on the official ROS website.

**Use this only if the guide at the following link is no longer available**: [ros2-humble-windows](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html).

If that is the case, all the executables used during the installation can be found here: [ros2-humble-windows-installation-packages](https://1drv.ms/f/s!Ai4mzVGQ5g8wgwsFJWSh_xOhUzI4).

# Installing prerequisites

## Install Chocolatey

Chocolatey is a package manager for Windows, install it by following their installation instructions:

https://chocolatey.org/install

You’ll use Chocolatey to install some other developer tools.

## Install Python

Open a Command Prompt and type the following to install Python via Chocolatey:

```ps1
choco install -y python --version 3.8.3
```

## Install Visual C++ Redistributables

Open a Command Prompt and type the following to install them via Chocolatey:

```ps1
choco install -y vcredist2013 vcredist140
```

## Install OpenSSL

Download the Win64 OpenSSL v1.1.1n OpenSSL installer from [this page](https://slproweb.com/products/Win32OpenSSL.html). Scroll to the bottom of the page and download _Win64 OpenSSL v1.1.1t_. Don’t download the Win32 or Light versions, or the v3.X.Y installers.

Run the installer with default parameters, as the following commands assume you used the default installation directory.

This command sets an environment variable that persists over sessions:

```ps1
setx /m OPENSSL_CONF "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg"
```

You will need to append the OpenSSL-Win64 bin folder to your PATH. You can do this by clicking the Windows icon, typing “Environment Variables”, then clicking on “Edit the system environment variables”. In the resulting dialog, click “Environment Variables”, then click “Path” on the bottom pane, finally click “Edit” and add the path below.

**C:\Program Files\OpenSSL-Win64\bin**

## Install Visual Studio

Install Visual Studio 2019.

If you already have a paid version of Visual Studio 2019 (Professional, Enterprise), skip this step.

Microsoft provides a free of charge version of Visual Studio 2019, named Community, which can be used to build applications that use ROS 2. [You can download the installer directly through this link](https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=Community&rel=16&src=myvs&utm_medium=microsoft&utm_source=my.visualstudio.com&utm_campaign=download&utm_content=vs+community+2019).

Make sure that the Visual C++ features are installed.

An easy way to make sure they’re installed is to select the **Desktop development with C++** workflow during the install.

Make sure that no C++ CMake tools are installed by unselecting them in the list of components to be installed.

## Install OpenCV

Some of the examples require OpenCV to be installed.

You can download a precompiled version of OpenCV 3.4.6 from https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip .

Assuming you unpacked it to **C:\opencv**, type the following on a Command Prompt (requires Admin privileges):

```
setx /m OpenCV_DIR C:\opencv
```

Since you are using a precompiled ROS version, we have to tell it where to find the OpenCV libraries. You have to extend the **PATH** variable to **C:\opencv\x64\vc16\bin**.

## Install dependencies

There are a few dependencies not available in the Chocolatey package database. In order to ease the manual installation process, we provide the necessary Chocolatey packages.

As some chocolatey packages rely on it, we start by installing CMake

```ps1
choco install -y cmake
```

You will need to append the CMake bin folder **C:\Program Files\CMake\bin** to your **PATH**.

Please download these packages from [this](https://github.com/ros2/choco-packages/releases/tag/2022-03-15) GitHub repository.

-   asio.1.12.1.nupkg
-   bullet.3.17.nupkg
-   cunit.2.1.3.nupkg
-   eigen-3.3.4.nupkg
-   tinyxml-usestl.2.6.2.nupkg
-   tinyxml2.6.0.0.nupkg

Once these packages are downloaded, open an administrative shell and execute the following command:

```ps1
choco install -y -s <path\to\downloads> asio cunit eigen tinyxml-usestl tinyxml2 bullet
```

Please replace **<path\to\downloads>** with the folder you downloaded the packages to.

First upgrade pip and setuptools:

```ps1
python -m pip install -U pip setuptools==59.6.0
```

Now install some additional python dependencies:

```ps1
python -m pip install -U catkin_pkg cryptography empy importlib-metadata lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pyyaml rosdistro
```

## Install Qt5

Download the [5.12.X offline installer](https://www.qt.io/offline-installers) from Qt’s website. Run the installer. Make sure to select the **MSVC 2017 64-bit** component under the **Qt** -> **Qt 5.12.12 tree**.

Finally, in an administrator **cmd.exe** window set these environment variables. The commands below assume you installed it to the default location of **C:\Qt**.

```ps1
setx /m Qt5_DIR C:\Qt\Qt5.12.12\5.12.12\msvc2017_64
setx /m QT_QPA_PLATFORM_PLUGIN_PATH C:\Qt\Qt5.12.12\5.12.12\msvc2017_64\plugins\platforms
```

## RQt dependencies

To run rqt_graph you need to [download](https://graphviz.org/download/) and install **Graphviz**. The installer will ask if to add graphviz to PATH, choose to either add it to the current user or all users.

# Downloading ROS 2

Go to the releases page: https://github.com/ros2/ros2/releases

Download the latest package for Windows, e.g., **ros2-humble-\*-windows-release-amd64.zip**.

Unpack the zip file somewhere (we’ll assume **C:\dev\ros2_humble**).
