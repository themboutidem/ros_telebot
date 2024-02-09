#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def float32_publisher():
    # Initialize the ROS node with a unique name
    rospy.init_node('gripper_out', anonymous=True)

    # Create a publisher for the Float32 data on the '/float32_topic' topic
    pub = rospy.Publisher('/gripper_signal', Float32, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    # Main loop
    while not rospy.is_shutdown():
        # Get input from the user
        input_value = raw_input("Enter a float value: ")  # For Python 2
        # input_value = input("Enter a float value: ")  # For Python 3

        try:
            # Convert the input to a float
            float_value = float(input_value)

            # Create a Float32 message and set its data
            float32_msg = Float32()
            float32_msg.data = float_value

            # Publish the message
            pub.publish(float32_msg)

        except ValueError:
            print("Invalid input. Please enter a valid float value.")

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        float32_publisher()
    except rospy.ROSInterruptException:
        pass
