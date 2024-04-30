#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Float32
from rospy.exceptions import ROSException

current_mode = None
subscriber_process = None

def start_subscriber(subscriber_script):
    global subscriber_process
    if subscriber_process:
        subscriber_process.terminate()  # Terminate the existing subscriber process
    subscriber_process = subprocess.Popen(['rosrun', 'telebot_agent', subscriber_script])

def mode_callback(msg):
    global current_mode
    new_mode = msg.data
    if current_mode != new_mode:
        current_mode = new_mode
        if new_mode == 1:
            start_subscriber('imu_orientation_subscriber.py')
            rospy.loginfo("Opening Orientation Subascriber")
        elif new_mode == 2:
            start_subscriber('imu_orientation_to_position_subscriber.py')
            rospy.loginfo("Opening Orientation to Position Subscriber")
    else:
        # If the mode is the same, kill the current subscriber process
        if subscriber_process:
            subscriber_process.terminate()

def main():
    rospy.init_node('mode_subscriber')
    rospy.Subscriber('/subscriber_mode', Float32, mode_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
