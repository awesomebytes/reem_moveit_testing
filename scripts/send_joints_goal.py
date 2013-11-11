#!/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 30 16:21:45 2013

@author: Sam Pfeiffer
"""

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, PositionConstraint, JointConstraint, OrientationConstraint

import actionlib
import rospy

def createJointConstraints():
    joint_positions = [1.7567944054, 1.68571255762, -0.35731008621, 
                       1.06480870567, 1.36986326531, -0.662985424101,
                       -1.31376998814]
    joint_constraints = []
    for joint_num in range(1,8):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'arm_right_' + str(joint_num) + '_joint'
        joint_constraint.tolerance_above = 0.1
        joint_constraint.tolerance_below = 0.1
        joint_constraint.weight = 1.0
        
        joint_constraint.position = joint_positions[joint_num -1]
        joint_constraints.append(joint_constraint)
        print str(joint_constraint)
    return joint_constraints
        

def createJointConstraintsZero():
    joint_positions = [1.7567944054, 1.68571255762, -0.35731008621, 
                       1.06480870567, 1.36986326531, -0.662985424101,
                       -1.31376998814]
    joint_constraints = []
    for joint_num in range(1,8):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'arm_right_' + str(joint_num) + '_joint'
        joint_constraint.tolerance_above = 0.1
        joint_constraint.tolerance_below = 0.1
        joint_constraint.weight = 1.0
        
        joint_constraint.position = 0.0
        joint_constraints.append(joint_constraint)
        print str(joint_constraint)
    return joint_constraints

def createPositionConstraint():
    position_constraints = []
    p = PositionConstraint()
    p.header.frame_id = 'base_link'
    p.link_name = 'arm_right_tool_link'
    p.target_point_offset.x = 0.5
    p.target_point_offset.y = -0.5
    p.target_point_offset.z = 1.0
    p.weight = 1.0
    position_constraints.append(p)
    return position_constraints
    
def createOrientationConstraint():
    orientation_constraints = []
    o = OrientationConstraint()
    o.header.frame_id = 'base_link'
    o.link_name = 'arm_right_tool_link'
    o.orientation.x = 0.0
    o.orientation.y = 0.0
    o.orientation.z = 0.0
    o.orientation.w = 1.0
    o.weight = 1.0
    o.absolute_x_axis_tolerance = 0.1
    o.absolute_y_axis_tolerance = 0.1
    o.absolute_z_axis_tolerance = 0.1
    orientation_constraints.append(o)
    return orientation_constraints

def move_group_client():
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)

    client.wait_for_server()
    goal = MoveGroupGoal()
    goal.request.group_name = "right_arm"
    
    goal_constraints = Constraints()
    goal_constraints.name = 'goal_test'
    
    #goal_constraints.joint_constraints.extend(createJointConstraints())
    #goal_constraints.joint_constraints.extend(createJointConstraintsZero())
    goal_constraints.position_constraints.extend(createPositionConstraint())
    goal_constraints.orientation_constraints.extend(createOrientationConstraint())
    
    goal.request.goal_constraints.append(goal_constraints)
    

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('move_group_client_py')
        result = move_group_client()
        print str(result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"