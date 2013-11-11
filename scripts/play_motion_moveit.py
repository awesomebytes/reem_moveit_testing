#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 11 11:15:46 2013

@author: Sam
"""

import sys
import rospy
import actionlib

from moveit_msgs.msg import MoveGroupGoal, MoveGroupAction, JointConstraint, Constraints, MoveGroupResult, MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

def createJointConstraints(joint_names, joint_positions, tolerances=0.1):
    """Create a JointConstraints message with its joint names and positions with some default tolerances
    @param joint_names names of the joints which will reference to values in param joint_positions
    @param joint_positions values for the joints specified in joint_names
    @param tolerances the tolerance in radians for the joint positions, defaults to 0.1
    @return moveit_msgs/JointConstraints[] message with the joint names and positions"""
    joint_constraints = []
    for joint, pos in zip(joint_names, joint_positions):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint
        joint_constraint.position = pos
        joint_constraint.tolerance_above = tolerances
        joint_constraint.tolerance_below = tolerances
        joint_constraint.weight = 1.0

        joint_constraints.append(joint_constraint)
    return joint_constraints


if __name__=='__main__':
    rospy.init_node('moveit_play_motion', anonymous=True)
    
    # check if params loaded
    global_name = '/play_motion/motions'
    if not rospy.has_param(global_name):
        rospy.logerr("Play motion params not loaded.")
        exit(-1)
    

    # load joints configs of movement by name
    name_of_movement = 'wave'
    movement_params = rospy.get_param(global_name + '/' + name_of_movement)
    joints = movement_params['joints']
    positions_and_times = movement_params['points']
    
    # connect to action server
    rospy.loginfo("Connecting to move_group")
    client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    client.wait_for_server()
    
    # foreach joint config
    step_num = 1
    for position_and_time in positions_and_times:
        rospy.loginfo("Executing step: "  + str(step_num) + "/" + str(len(positions_and_times)))
        step_num += 1 
        # create goal for action server
        mg_g = MoveGroupGoal()
        mg_g.request.group_name = "both_arms"
        mg_g.request.allowed_planning_time = 5.0
        mg_g.request.num_planning_attempts = 3
        c = Constraints()
        joint_constraints = createJointConstraints(joints, position_and_time['positions'], tolerances=0.1)
        c.joint_constraints.extend(joint_constraints)
        mg_g.request.goal_constraints.append(c)
        mg_g.planning_options.plan_only = False
        mg_g.planning_options.replan = True
        mg_g.planning_options.replan_attempts = 3
        mg_g.planning_options.replan_delay = 0.3
        #print mg_g
        # send goal to action server
        client.send_goal(mg_g)
    
        # return result
        client.wait_for_result(rospy.Duration.from_sec(5.0))
        results = client.get_result()
        # handle errors
        #mg_r = MoveGroupResult()
#         if results.error_code.val == 1:
#             print "results:"
#             print results
#         else:
        print moveit_error_dict[results.error_code.val]
            
    
    # Now see if we can concatenate the goals