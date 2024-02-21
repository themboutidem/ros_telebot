#!/usr/bin/env python
###
# Team Telebot @ Hampton University
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2023 Telebot inc. All rights reserved.
#
###

import sys
import rospy
import time

from kortex_driver.srv import *
from kortex_driver.msg import *
from std_msgs.msg import Float32


class move_arm_subscriber:
    def __init__(self):
        try: 
            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", True)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True
        
    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()
        
        def example_cartesian_waypoint_action(self):
            self.last_action_notif_type = None

            req = ExecuteActionRequest()
            trajectory = WaypointList()

            config = self.get_product_configuration()

            trajectory.waypoints.append(self.FillCartesianWaypoint(0.439,  0.194,  0.448, 90.6, -1.0, 150, 0))
        
            req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
            # Call the service
            rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
                return False
            else:
                return self.wait_for_action_end_or_abort()


        def position_callback(self, values):
            if(values == 0):
                success &= self.example_home_the_robot()
            else:
                succes &= example_cartesian_waypoint_action(self)
    
    if __name__ == "__main__":
        success = self.is_init_success
        rospy.init_node('move_arm_subscriber')

        rospy.Subscriber("/gripper_position", Float32, position_callback)
