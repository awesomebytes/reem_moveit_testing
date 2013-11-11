#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  4 16:15:46 2013

@author: Sam
"""

import sys
import rospy
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState

if __name__=='__main__':

#    roscpp_initialize(sys.argv)
#    rospy.init_node('moveit_py_demo', anonymous=True)
#    
#    robot = RobotCommander()
#    rospy.sleep(1)
#
#    print "Current state:"
#    print robot.get_current_state()
#    
#    # plan to a random location 
#    a = robot.right_arm
#    a.set_start_state(RobotState())
#    r = a.get_random_joint_values()
#    print "Planning to random joint position: "
#    print r
#    p = a.plan(r)
#    print "Solution:"
#    print p
#
#    roscpp_shutdown()
    
    
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("pole")
    scene.remove_world_object("table")
    #scene.remove_world_object("part")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.7
    p.pose.position.y = -0.4
    p.pose.position.z = 1.15
    p.pose.orientation.w = 1.0
    #scene.add_box("pole", p, (0.3, 0.1, 1.0))

    p.pose.position.y = -0.2
    p.pose.position.z = 0.175
    scene.add_box("table", p, (0.5, 1.5, 0.8))

#    p.pose.position.x = 0.6
#    p.pose.position.y = -0.7
#    p.pose.position.z = 0.8
    p.pose.position.x = 0.3 
    p.pose.position.y = -0.3
    p.pose.position.z = 0.9
    scene.add_box("part", p, (0.15, 0.1, 0.3))

    rospy.sleep(1)

    # pick an object
    robot.right_arm.pick("part")
    roscpp_shutdown()
    