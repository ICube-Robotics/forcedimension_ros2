# forcedimension_ros2
This stack includes `ros2_control` drivers for Force Dimension SDK compatible haptic interfaces.


***Tested with a Humble ROS distribution only (Ubuntu 22.04 LTS)***

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![CI (humble)](../../actions/workflows/ci-humble.yml/badge.svg?branch=humble)](../../actions/workflows/ci-humble.yml?query=branch:humble)

> **Warning**
>
> The vendor package [fd_sdk_vendor](https://github.com/ICube-Robotics/fd_sdk_vendor.git) is now required!
> Please follow the installation steps carefully.

> **Note**
>
> Although still online, the `foxy` and `galactic` branches are not maintained anymore and might be obsolete.


## Compatible devices
The driver was currently tested on the following haptic devices:
- Force Dimension [Omega.3](https://www.forcedimension.com/products/omega), [Omega.6](https://www.forcedimension.com/products/omega) and [Omega.7](https://www.forcedimension.com/products/omega)
- Novint [Falcon](https://hapticshouse.com/pages/novints-falcon-haptic-device)

## Usage
### Getting Started
***Required setup : Ubuntu 22.04 LTS***

1.  Install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
4. Pull relevant packages, install dependencies (including the vendor pkg [fd_sdk_vendor](https://github.com/ICube-Robotics/fd_sdk_vendor.git)):
    ```shell
    cd ~/ros2_ws
    cd src
    git clone https://github.com/ICube-Robotics/forcedimension_ros2.git
    vcs import . < forcedimension_ros2/forcedimension_ros2.repos
    rosdep install --ignore-src --from-paths . -y -r
    ```
6. Compile and source the workspace by using:
    ```shell
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
### Running the driver

An example launch file is provided with this stack in the `fd_bringup` package. The driver can be run using
```shell
ros2 launch fd_bringup fd.launch.py
```
The device end-effector pose can then be found in the `/fd/ee_pose` and wrench can be set on the `/fd_controller/commands` topic.
Note that __the default launch config is for the Omega 3 device__ (orientation and clutch OFF).

You can test the readings using [plotjuggler](https://plotjuggler.io/) and the force control by requesting a (small) force along X axis:
```bash
ros2 topic pub -r 1000 \
     /fd/fd_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.0, 0.0]"
```

## Practical information

### Initialize usb device
USB devices require `su` privileges to operate unless allowed in udev rules

To declare a new device :
1. run `lsusb -v` which gives
    ```shell
    idVendor = 0x1451 Force Dimension
    idProduct = 0x0301
    ```
2. Create and edit udev rules file
    ```shell
    sudo nano /etc/udev/rules.d/10-omega_3_USB.rules
    ```
    and in the file write
    ```shell
    ATTRS{idProduct}=="[PRODUCT_ID]", ATTRS{idVendor}=="[VENDOR ID]", MODE="666", GROUP="plugdev"
    ```
    **Note**: `[PRODUCT_ID]` is `idProduct` without `0x`, same for `[VENDOR ID]`

3. To apply the new rule run
    ```shell
    sudo udevadm trigger
    ```
4. You can try your setup by running the `HapticDesk` executable from the sdk `fd_hardware/external/sdk-3.14.0/bin` folder. If the haptic device is recognized, you are ready to go.

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://plateforme.icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](mailto:macbednarczyk@gmail.com)
