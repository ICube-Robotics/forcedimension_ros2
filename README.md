# forcedimension_ros2
This stack includes `ros2_control` drivers for Force Dimension SDK compatible haptic interfaces.

## Compatible devices
The driver was currently tested on the following haptic devices:
- Force Dimension [Omega.3](https://www.forcedimension.com/products/omega), [Omega.6](https://www.forcedimension.com/products/omega) and [Omega.7](https://www.forcedimension.com/products/omega)
- Novint [Falcon](https://hapticshouse.com/pages/novints-falcon-haptic-device)

## Usage
### Getting Started
***Required setup : Ubuntu 22.04 LTS***

1.  Install `ros2` packages. The current developpment is based of `ros2 humble`. Installation steps are decribed [here](https://docs.ros.org/en/humble/Installation.html).
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
4. Pull relevant packages, install dependencies by using :
    ```shell
    cd ~/ros2_ws
    git clone https://github.com/ICube-Robotics/forcedimension_ros2.git src/forcedimension_ros2
    git submodule update --init --recursive
    rosdep install --ignore-src --from-paths . -y -r
    ```
5. Download the newest version of Force Dimension [SDK](https://www.forcedimension.com/software/sdk) and copy its content in the `fd_hardware/external/fd_sdk` directory.
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
