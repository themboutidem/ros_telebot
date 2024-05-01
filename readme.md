# ros_telebot
Telebot is our general-purpose implementation of a controller for a Telepresence Robot. It builds on the ROS Kortex Package from Kinova. This repository contains the ROS Kortex packages as well as the telebot_agent package we developed. 

Before you use our package, first install the Kortex API, documentation for which can be found in the [GitHub Kortex repository](https://github.com/Kinovarobotics/kortex)." Thereafter, follow the instructions below to download and build your workspace.  

## Download links

You can refer to the [Kortex repository "Download links" section](https://github.com/Kinovarobotics/kortex#download-links) to download the firmware package and the release notes.

## Installation

### Setup

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

This package has been tested under ROS Melodic (Ubuntu 18.04).
You can find the instructions to install ROS Kinetic [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and ROS Melodic [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

[Google Protocol Buffers](https://developers.google.com/protocol-buffers/) is used by Kinova to define the Kortex APIs and to automatically generate ROS messages, services and C++ classes from the Kortex API `.proto` files. The installation of Google Protocol Buffers is required by developers implementing new APIs with the robot. However, since we already provide all the necessary generated files on GitHub, this is not required for most end users of the robot.

### Build

These are the instructions to run in a terminal to create the workspace, clone the `ros_telebot` repository and install the necessary ROS dependencies:

        sudo apt install python3 python3-pip
        sudo python3 -m pip install conan
        conan config set general.revisions_enabled=1
        conan profile new default --detect > /dev/null
        conan profile update settings.compiler.libcxx=libstdc++11 default
        mkdir -p catkin_workspace/src
        cd catkin_workspace/src
        git clone https://github.com/themboutidem/ros_telebot.git
        cd ../
        rosdep install --from-paths src --ignore-src -y

Then, to build and source the workspace:

        catkin_make
        source devel/setup.bash

## Contents

Consult the ros_kortex [here](https://github.com/Kinovarobotics/ros_kortex) repository for a detailed description of the ros_kortex packages in this repository.

Below is a description of our telebot_controller package

### telebot_controller
This package contains nodes that subscribe to the topics publishing IMU coordinates and Gripper Values. For more details, please consult the [README](telebot_controller/readme.md) from the package subdirectory.
