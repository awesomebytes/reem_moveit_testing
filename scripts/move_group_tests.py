#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  4 16:15:46 2013

@author: Sam
"""

from moveit_commander import MoveGroupCommander
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point

if __name__ == '__main__':
    rospy.init_node("test78332427")
    rospy.loginfo("Setting movegroup")
    g = MoveGroupCommander("right_arm")
    
    rospy.loginfo("Setting random target")
    #g.set_random_target()
    
    
    
    rospy.loginfo("Go!")
    g.go()
    