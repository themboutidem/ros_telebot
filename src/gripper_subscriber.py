#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from gripper_control import GripperControl

def gripper_callback(values):
    rospy.loginfo("Received gripper signal: %f", values.data)
    gripper_control.gripper_action(values)

if __name__ == "__main__":
    rospy.init_node('gripper_subscriber')

    gripper_control = GripperControl()

    if gripper_control.is_init_success:
        rospy.Subscriber("/gripper_signal", Float32, gripper_callback)

        rospy.loginfo("Gripper subscriber node is running.")
        rospy.spin()
    else:
        rospy.logerr("Failed to initialize GripperControl.")
