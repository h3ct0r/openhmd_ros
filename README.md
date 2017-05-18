# openhmd_ros

Bindings between OpenHMD and ROS. This will be able to put images onto the Oculus Rift and read the IMU information from ROS nodes on Linux.

This module was prepared to be used with the **Oculus Rift CV1**, but it could work with other devices too. It was tested on **Ubuntu 16.04.1 LTS** and **ROS Kinetic**.

## How to use
- After installing all required packages (see requeriments and intall sections)
- Edit publish_stereo.launch file to configure your cameras and location of the image viewer
- Run `roslaunch openhmd_ros oculus.launch ` to launch the oculus node, to acquire IMU data.
- Run `roslaunch openhmd_ros publish_stereo.launch` to launch the vision node. This node gets the camera information and apply the transformations to use it on the oculus. And then it resize the image window and locate it at a proper location.

## Nodes

-**/openhmd/pose**: node to publish the pose of the robot in quaterions and euler angles.

### Requirements
* Openhmd
* cython
* Glew
* libhid

### Install

This package was tested on Ubuntu 16.04.01 and 16.04.02. For installation follow the steps:

* `sudo apt-get install python-dev cython cython3`
* `pip install Cython`
* `sudo apt install libhidapi-dev libhidapi-libusb0 libhidapi-libusb0-dbg`
* `sudo apt install glew-utils libglew-dbg libglew-dev`
* Add these udev rules:

	`echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev"' > /etc/udev/rules.d/83-hmd.rules`
    `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0bb4", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/83-hmd.rules`
    `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="28de", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/83-hmd.rules`
    `udevadm control --reload-rules`
* Clone this repo on your `catkin_ws` folder
* Install the openhmd package from https://github.com/OpenHMD/OpenHMD/
* `sudo ldconfig`
* Install the `usb_cam` module from https://github.com/bosch-ros-pkg/usb_cam
* Go to the folder *python_bindings_openhmd/* and then execute `sudo ./setup.py install`. Verify if the module was correctly installed by executing `from rift import PyRift` in the python console.
* Run a catkin make on top of your catkin_ws: `cd ~/catkin_ws/ && catkin_make`

### What is working

1)  Activate the Oculus CV1 on Linux
2) Get the IMU data from the Oculus
3) Automatically use img topics as images on the oculus screen

### Todos
* General improvements, more documentation
* A better way to show img data on the Oculus on Linux (**critical**)
* Use of the led constelation
