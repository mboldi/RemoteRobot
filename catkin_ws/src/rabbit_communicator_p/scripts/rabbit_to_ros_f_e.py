#!/usr/bin/env python

import os
import sys
import pika

import rospy
from std_msgs.msg import String

modeSetQueue = 'set-mode'
moveRobotQueue = 'move-robot'
setJointsQueue = 'set-joints'
savePosQueue = 'save-pos'

evalMoveQueue = 'evaluate-move'
evalResultQueue = 'evaluate-result'

setModePubHandle = None
moveRobotPubHandle = None
setJointsPubHandle = None
savePosPubHandle = None
evalMovePubHandle = None
evalResultPubHandle = None

def makeQueueAndConsume(channel, queue, callback):
    channel.queue_declare(queue=queue)
    channel.basic_consume(queue=queue, on_message_callback=callback, auto_ack=True)

def makeQueues(channel):
    global setModePubHandle
    global moveRobotPubHandle
    global setJointsPubHandle
    global savePosPubHandle
    global evalMovePubHandle
    global evalResultPubHandle

    makeQueueAndConsume(channel, modeSetQueue, modeSetCallback)
    setModePubHandle = rospy.Publisher(modeSetQueue, String, queue_size=10)

    makeQueueAndConsume(channel, moveRobotQueue, moveRobotCallback)
    moveRobotPubHandle = rospy.Publisher(moveRobotQueue, String, queue_size=10)

    makeQueueAndConsume(channel, setJointsQueue, setJointsCallback)
    setJointsPubHandle = rospy.Publisher(setJointsQueue, String, queue_size=10)

    makeQueueAndConsume(channel, savePosQueue, savePosCallback)
    savePosPubHandle = rospy.Publisher(savePosQueue, String, queue_size=10)

    makeQueueAndConsume(channel, evalMoveQueue, evalMoveCallback)
    evalMovePubHandle = rospy.Publisher(evalMoveQueue, String, queue_size=10)

    makeQueueAndConsume(channel, evalResultQueue, evalResultCallback)
    evalResultPubHandle = rospy.Publisher(evalResultQueue, String, queue_size=10)


def modeSetCallback(ch, method, properties, body):
    print(" [modeSet] %r" % body)

    global setModePubHandle

    if setModePubHandle is not None:
        modeChangeMsg = String()
        modeChangeMsg.data = body

        setModePubHandle.publish(modeChangeMsg)

        rospy.loginfo('Sent message: ' + modeChangeMsg.data)

def moveRobotCallback(ch, method, properties, body):
    print(" [moveRobot] %r" % body)

    global moveRobotPubHandle

    if moveRobotPubHandle is not None:
        moveRobotMsg = String()
        moveRobotMsg.data = body

        moveRobotPubHandle.publish(moveRobotMsg)

        rospy.loginfo('Sent message: ' + moveRobotMsg.data)

def setJointsCallback(ch, method, properties, body):
    print(" [setJoints] %r" % body)

def savePosCallback(ch, method, properties, body):
    print(" [savePos] %r" % body)

def evalMoveCallback(ch, method, properties, body):
    print(" [evalMove] %r" % body)

def evalResultCallback(ch, method, properties, body):
    print(" [evalResult] %r" % body)



rospy.init_node("rabbit_to_ros_n")

connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()

makeQueues(channel)

print(' [rabbit] Waiting for msgs. To exit press CTRL+C')

channel.start_consuming()

connection.close()
