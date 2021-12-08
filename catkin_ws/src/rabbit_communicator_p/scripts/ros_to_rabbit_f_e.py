#!/usr/bin/env python

import os
import sys
import pika

import rospy
from std_msgs.msg import String

robotStatusQueue = 'robot-status'
evalProductQueue = 'evaluate-req'


def make_queues(channel):
    channel.queue_declare(queue=robotStatusQueue)
    channel.queue_declare(queue=evalProductQueue)

def sendRobotStatus(channel, robotData):
    channel.basic_publish(exchange='',
                          routing_key='hello',
                          body='Hello World!')

def main():



    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()

    make_queues(channel)



    connection.close()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted')
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)