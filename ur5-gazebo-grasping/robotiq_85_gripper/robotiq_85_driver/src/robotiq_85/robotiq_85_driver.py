"""--------------------------------------------------------------------
COPYRIGHT 2015 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed Segway RMP Robotic Platforms is intended and supplied to you, 
the Company's customer, for use solely and exclusively with Stanley Innovation 
products. The software is owned by the Company and/or its supplier, and is 
protected under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   robotiq_85_driver.py

 \brief  Driver for Robotiq 85 communication

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from robotiq_85_gripper import Robotiq85Gripper
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from sensor_msgs.msg import JointState
import numpy as np
import rospy

class Robotiq85Driver:
    def __init__(self):
        self._num_grippers = rospy.get_param('~num_grippers',1)
        self._comport = rospy.get_param('~comport','/dev/ttyUSB0')
        self._baud = rospy.get_param('~baud','115200')

        self._gripper = Robotiq85Gripper(self._num_grippers,self._comport,self._baud)

        if not self._gripper.init_success:
            rospy.logerr("Unable to open commport to %s" % self._comport)
            return
            
        if (self._num_grippers == 1):
            rospy.Subscriber("/gripper/cmd", GripperCmd, self._update_gripper_cmd, queue_size=10)
            self._gripper_pub = rospy.Publisher('/gripper/stat', GripperStat, queue_size=10)
            self._gripper_joint_state_pub = rospy.Publisher('/gripper/joint_states', JointState, queue_size=10)        
        elif (self._num_grippers == 2):
            rospy.Subscriber("/left_gripper/cmd", GripperCmd, self._update_gripper_cmd, queue_size=10)
            self._left_gripper_pub = rospy.Publisher('/left_gripper/stat', GripperStat, queue_size=10)
            self._left_gripper_joint_state_pub = rospy.Publisher('/left_gripper/joint_states', JointState, queue_size=10)
            rospy.Subscriber("/right_gripper/cmd", GripperCmd, self._update_right_gripper_cmd, queue_size=10)
            self._right_gripper_pub = rospy.Publisher('/right_gripper/stat', GripperStat, queue_size=10)
            self._right_gripper_joint_state_pub = rospy.Publisher('/right_gripper/joint_states', JointState, queue_size=10)
        else:
            rospy.logerr("Number of grippers not supported (needs to be 1 or 2)")
            return

        self._seq = [0] * self._num_grippers
        self._prev_js_pos = [0.0] * self._num_grippers
        self._prev_js_time = [rospy.get_time()] * self._num_grippers 
        self._driver_state = 0
        self._driver_ready = False
        
        success = True
        for i in range(self._num_grippers):
            success &= self._gripper.process_stat_cmd(i)
            if not success:
                bad_gripper = i
        if not success:
            rospy.logerr("Failed to contact gripper %d....ABORTING"%bad_gripper)
            return                
                
        self._run_driver()
        
    def _clamp_cmd(self,cmd,lower,upper):
        if (cmd < lower):
            return lower
        elif (cmd > upper):
            return upper
        else:
            return cmd

    def _update_gripper_cmd(self,cmd):
    
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release()

        if (True == cmd.stop):
            self._gripper.stop()
        else:
            pos = self._clamp_cmd(cmd.position,0.0,0.085)
            vel = self._clamp_cmd(cmd.speed,0.013,0.1)
            force = self._clamp_cmd(cmd.force,5.0,220.0)
            self._gripper.goto(dev=0,pos=pos,vel=vel,force=force)
            
    def _update_right_gripper_cmd(self,cmd):
    
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(dev=1,open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release(dev=1)

        if (True == cmd.stop):
            self._gripper.stop(dev=1)
        else:
            pos = self._clamp_cmd(cmd.position,0.0,0.085)
            vel = self._clamp_cmd(cmd.speed,0.013,0.1)
            force = self._clamp_cmd(cmd.force,5.0,220.0)
            self._gripper.goto(dev=1,pos=pos,vel=vel,force=force)
            
    def _update_gripper_stat(self,dev=0):
        stat = GripperStat()
        stat.header.stamp = rospy.get_rostime()
        stat.header.seq = self._seq[dev]
        stat.is_ready = self._gripper.is_ready(dev)
        stat.is_reset = self._gripper.is_reset(dev)
        stat.is_moving = self._gripper.is_moving(dev)
        stat.obj_detected = self._gripper.object_detected(dev)
        stat.fault_status = self._gripper.get_fault_status(dev)
        stat.position = self._gripper.get_pos(dev)
        stat.requested_position = self._gripper.get_req_pos(dev)
        stat.current = self._gripper.get_current(dev)
        self._seq[dev]+=1
        return stat
        
    def _update_gripper_joint_state(self,dev=0):
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = self._seq[dev]
        js.name = ['gripper_finger1_joint']
        pos = np.clip(0.8 - ((0.8/0.085) * self._gripper.get_pos(dev)), 0., 0.8)
        js.position = [pos]
        dt = rospy.get_time() - self._prev_js_time[dev]
        self._prev_js_time[dev] = rospy.get_time()
        js.velocity = [(pos-self._prev_js_pos[dev])/dt]
        self._prev_js_pos[dev] = pos
        return js
        
    def _run_driver(self):
        last_time = rospy.get_time()
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            dt = rospy.get_time() - last_time
            if (0 == self._driver_state):
                for i in range(self._num_grippers):
                    if (dt < 0.5):
                        self._gripper.deactivate_gripper(i)
                    else:
                        self._driver_state = 1
            elif (1 == self._driver_state):
                grippers_activated = True
                for i in range(self._num_grippers):    
                    self._gripper.activate_gripper(i)
                    grippers_activated &= self._gripper.is_ready(i)
                if (grippers_activated):
                    self._driver_state = 2
            elif (2 == self._driver_state):
                self._driver_ready = True
                        
            for i in range(self._num_grippers):
                success = True
                success &= self._gripper.process_act_cmd(i)
                success &= self._gripper.process_stat_cmd(i)
                if not success:
                    rospy.logerr("Failed to contact gripper %d"%i)
                
                else:
                    stat = GripperStat()
                    js = JointState()
                    stat = self._update_gripper_stat(i)
                    js = self._update_gripper_joint_state(i)
                    if (1 == self._num_grippers):
                        self._gripper_pub.publish(stat)
                        self._gripper_joint_state_pub.publish(js)
                    else:
                        if (i == 0):
                            self._left_gripper_pub.publish(stat)
                            self._left_gripper_joint_state_pub.publish(js)
                        else:
                            self._right_gripper_pub.publish(stat)
                            self._right_gripper_joint_state_pub.publish(js)
                            
            r.sleep()

        self._gripper.shutdown()
