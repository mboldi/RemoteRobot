#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def go_home(group):
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = - 2 * pi / 3
    joint_goal[2] = pi/2
    joint_goal[3] = pi - pi/3 - pi/2
    joint_goal[4] = 0

    group.go(joint_goal, wait=True)

    group.stop()
    group.clear_pose_targets()

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_robot_n', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("manipulator")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

rospy.loginfo(robot.get_current_state())

go_home(group)

rospy.loginfo(group.get_current_pose().pose)