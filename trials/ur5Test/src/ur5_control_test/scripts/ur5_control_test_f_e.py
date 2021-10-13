#! /usr/bin/python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_control_test', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "ur5_arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame


group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()


print "============ Printing robot state"
print robot.get_current_state()
print ""


# joint_goal = group.get_current_joint_values()
# joint_goal[0] = 0
# joint_goal[1] = -pi/4
# joint_goal[2] = 0
# joint_goal[3] = -pi/2
# joint_goal[4] = 0
# joint_goal[5] = pi/3
#
# group.go(joint_goal, wait=True)
#
# group.stop()

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.2
pose_goal.position.x = -0.2
pose_goal.position.y = 0.1
pose_goal.position.z = 0.5
group.set_pose_target(pose_goal)

plan = group.go(wait=True)

group.stop()

group.clear_pose_targets()