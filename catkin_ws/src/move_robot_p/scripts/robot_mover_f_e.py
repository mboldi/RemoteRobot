#!/usr/bin/python

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Pose
import moveit_commander
import moveit_msgs.msg

from math import pi

modeSetQueue = 'set-mode'
moveRobotQueue = 'move-robot'
setJointsQueue = 'set-joints'
savePosQueue = 'save-pos'

evalMoveQueue = 'evaluate-move'
evalResultQueue = 'evaluate-result'

home_joints = [0, 0, 0, 0, 0]
home_joints[0] = 0
home_joints[1] = - 2 * pi / 3
home_joints[2] = pi / 2
home_joints[3] = pi - pi / 3 - pi / 2
home_joints[4] = pi/2

robot_head_pos = Pose()

def extractHeadPosData(rawData):
    global robot_head_pos



def go_home(group):
    global home_joints

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = home_joints[0]
    joint_goal[1] = home_joints[1]
    joint_goal[2] = home_joints[2]
    joint_goal[3] = home_joints[3]
    joint_goal[4] = home_joints[4]

    group.go(joint_goal, wait=True)

    group.stop()
    group.clear_pose_targets()


def msgDataToVec3(msgData):
    splitData = msgData.lstrip('(').strip(')').split(',')

    moveVec = Vector3()
    moveVec.x = float(splitData[0])
    moveVec.y = float(splitData[1])
    moveVec.z = float(splitData[2])

    return moveVec


def moveRobotWithVector(moveVector):
    global armGroup
    global robot_head_pos

    pose_goal = robot_head_pos

    pose_goal.position.x += moveVector.x
    pose_goal.position.y += moveVector.y
    pose_goal.position.z += moveVector.z

    armGroup.set_pose_target(pose_goal)

    plan = armGroup.go(wait=True)
    armGroup.stop()
    armGroup.clear_pose_targets()

def moveWithVectorCallback(rcvd_msg):
    global armGroup
    global robot_head_pos

    rospy.loginfo("I heard %s", rcvd_msg.data)

    moveVec = msgDataToVec3(rcvd_msg.data)
    rospy.loginfo(moveVec)

    moveRobotWithVector(moveVec)

    rospy.loginfo(armGroup.get_current_pose().pose)
    robot_head_pos = armGroup.get_current_pose().pose


rospy.init_node("robot_mover_n")
moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

armGroup = moveit_commander.MoveGroupCommander('manipulator')

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# go_home(armGroup)

rospy.loginfo(armGroup.get_current_pose().pose)


sub_handle_py = rospy.Subscriber(moveRobotQueue, String, moveWithVectorCallback, queue_size=10)
rospy.spin()
