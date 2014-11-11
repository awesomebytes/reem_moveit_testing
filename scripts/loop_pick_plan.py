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

import random
import copy


if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo_743568', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    rospy.sleep(1)

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.orientation.w = 1.0

    p.pose.position.x = 0.15
    p.pose.position.y = -0.3
    p.pose.position.z = 0.55

    id_part = 1
    while not rospy.is_shutdown():
        # clean the scene
        scene.remove_world_object("part" + str(id_part - 1))

        # publish a demo scene
        new_p = copy.deepcopy(p)
        new_p.pose.position.x += random.random() * 0.07 * random.choice([1, -1])
        new_p.pose.position.y += random.random() * 0.07 * random.choice([1, -1])
        new_p.pose.position.z += random.random() * 0.07 * random.choice([1, -1])
        scene.add_box("part" + str(id_part), new_p, (0.05, 0.05, 0.05))
        print "Picking " + "part" + str(id_part) +  " at: " + str(new_p.pose.position)
        rospy.sleep(0.5)

        # pick an object
        robot.right_arm.pick("part" + str(id_part))
        id_part += 1
    roscpp_shutdown()
    