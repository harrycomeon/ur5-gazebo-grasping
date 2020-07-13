#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs

import roslib
import math
import tf     

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
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("manipulator")
#hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
arm_group.set_named_target("home")
plan = arm_group.go()

gripper_value = 0.8
# Start the ROS node
#rospy.init_node('gripper_command')
# Set the value to the gripper
result = gripper_client(gripper_value)

gripper_value = 0.5
# Start the ROS node
#rospy.init_node('gripper_command')
# Set the value to the gripper
result = gripper_client(gripper_value)

#hand_group.set_named_target("open")
#plan = hand_group.go()


pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.15
pose_target.position.y = 0
pose_target.position.z = 1.25
arm_group.set_pose_target(pose_target)
plan = arm_group.go()

pose_target.position.z = 1.08
arm_group.set_pose_target(pose_target)
plan = arm_group.go()

hand_group.set_named_target("grasp")
plan = hand_group.go()

pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()

hand_group.set_named_target("grasp")
plan = hand_group.go()

rospy.sleep(5)
moveit_commander.roscpp_initializer.roscpp_shutdown()
