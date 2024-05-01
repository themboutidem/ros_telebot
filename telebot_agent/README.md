# telebot_agent

##Overview

This node allows communication between a telebot_controller [composed of an IMU sensor, a potentiometer and two push buttons for mode selection] and a Kinova Gen3 robot.

##Usage

* Ensure both the "controller" and "agent" are connected on the same Virtual Network. We suggest Husarnet's VPN service(https://husarion.com/tutorials/ros-tutorials/6-robot-network/).
* Run the kortex_driver node by following the instructions here in the README(/ros_telebot/kortex_driver)
* Run this terminal command to launch the gripper_subscriber node:
    rosrun telebot_agent right_gripper_subscriber.py
* Run this terminal command to launch the subscriber_mode node:
    rosrun telebot_agent subscriber_mode_listener.py
  
