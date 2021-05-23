#!/usr/bin/python
#
# Send a value to change the opening of the Robotiq gripper using an action
#

import argparse

import rospy
import actionlib
import control_msgs.msg


def gripper_client(value):

    # Create an action client
    client = actionlib.SimpleActionClient(
        '/gripper_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    # Wait until the action server has been started and is listening for goals
    client.wait_for_server()

    # Create a goal to send (to the action server)
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value   # From 0.0 to 0.8
    goal.command.max_effort = -1.0  # Do not limit the effort
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':

        gripper_value = 0.8
        # Start the ROS node
        rospy.init_node('gripper_command')
        # Set the value to the gripper
        result = gripper_client(gripper_value)

        gripper_value = 0.5
        # Start the ROS node
        rospy.init_node('gripper_command')
        # Set the value to the gripper
        result = gripper_client(gripper_value)
        
