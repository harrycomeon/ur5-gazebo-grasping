#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs
import tf

from moveit_commander import MoveGroupCommander

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
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('moveit_cartesian', anonymous=True)
    cartesian = rospy.get_param('~cartesian', True)
                        
    #set cartesian parameters
    ur5_manipulator = MoveGroupCommander('manipulator')
    ur5_gripper = MoveGroupCommander('gripper')
    ur5_manipulator.allow_replanning(True)
    ur5_manipulator.set_pose_reference_frame('base_link')
    ur5_manipulator.set_goal_position_tolerance(0.01)
    ur5_manipulator.set_goal_orientation_tolerance(0.1)
    end_effector_link = ur5_manipulator.get_end_effector_link()    
    ur5_manipulator.set_named_target('home_j')
    ur5_manipulator.go()
    ur5_gripper.set_named_target('open')
    ur5_gripper.go()

    #get the end effort information
    start_pose = ur5_manipulator.get_current_pose(end_effector_link).pose
    print("The first waypoint:")
    print(start_pose)
    #define waypoints
    waypoints = []   
    waypoints.append(start_pose)

    wpose = deepcopy(start_pose)
    wpose.position.z = object_position_info.z+0.25
    wpose.position.x = object_position_info.x
    wpose.position.y = object_position_info.y
    print("The second waypoint:")
    print(wpose) 
    waypoints.append(deepcopy(wpose))
    print(" ")
    print(waypoints) 


    if cartesian:
        fraction = 0.0   
        maxtries = 100   
        attempts = 0     
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) =  ur5_manipulator.compute_cartesian_path (
                                        waypoints,   
                                        0.01,        
                                        0.0,         
                                        True)              
            attempts += 1
                
                
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            ur5_manipulator.execute(plan)
            rospy.sleep(2)
            rospy.loginfo("Path execution complete.")
            
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

    rospy.sleep(3)
    ur5_gripper.set_named_target("close")
    plan = ur5_gripper.go()
    rospy.sleep(2)
    ur5_manipulator.set_named_target('home_j')
    ur5_manipulator.go()
    rospy.sleep(3)
        
        
    moveit_commander.roscpp_shutdown()
    #moveit_commander.os._exit(0) 

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
