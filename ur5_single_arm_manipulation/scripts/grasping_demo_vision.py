#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs
import tf

from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from copy import deepcopy

def callback(pose):
    object_position_info = pose.position
    object_orientation_info = pose.orientation
    print object_position_info
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_grasp', anonymous=True)
    robot = moveit_commander.robot.RobotCommander()

    arm_group = moveit_commander.move_group.MoveGroupCommander("manipulator")
    hand_group = moveit_commander.move_group.MoveGroupCommander("gripper") 
   
    arm_group.set_named_target("home_j")
    plan = arm_group.go()

    print("Point 1")

    # Open
    #hand_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05,   9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
    #hand_group.go(wait=True)
    #print("Point 2")
    hand_group.set_named_target("open")
    plan = hand_group.go()
    print("Point 2")

    pose_target = arm_group.get_current_pose().pose

    # Block point top
    pose_target.position.x = object_position_info.x
    pose_target.position.y = object_position_info.y
    pose_target.position.z = object_position_info.z+0.25



    arm_group.set_pose_target(pose_target)
    arm_group.go(wait=True)
    print("Point 3")

    # Block point
    pose_target.position.z = pose_target.position.z-0.26



    arm_group.set_pose_target(pose_target)
    arm_group.go(wait=True)
    print("Point 4")
    rospy.sleep(3)

    hand_group.set_named_target("close")
    plan = hand_group.go()
    print("Point 5")
    rospy.sleep(2)

    pose_target.position.z = pose_target.position.z+0.25
    arm_group.set_pose_target(pose_target)
    plan = arm_group.go()
    print("Point 6")


    pose_target.position.x = 0.5
    arm_group.go()
    rospy.sleep(3)
    print("Point 7")



    hand_group.set_named_target("open")
    plan = hand_group.go()
    print("Point 8")
        
        
    moveit_commander.roscpp_shutdown()

def object_position_sub():
    rospy.Subscriber("/objection_position_pose",Pose,callback,queue_size=10)
if __name__ == "__main__":
    rospy.init_node('object_position_sub_And_grasp_node',anonymous=True)
    object_position_sub()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            print("11")
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
                
        rate.sleep()
