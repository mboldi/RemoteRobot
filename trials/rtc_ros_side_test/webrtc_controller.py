import os
import sys
import pika
import json
import argparse
import asyncio
import logging
import time

from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.signaling import BYE, add_signaling_arguments, create_signaling

class jsonableDesc:
    def __init__(self, newdesc):
        self.type = newdesc.type
        self.sdp = newdesc.sdp

    def toJson(self):
        return json.dumps(self, default=lambda o: o.__dict__)


rabbitConnection = None
rabbitChannel = None


def sendRabbitMsg(msg):
    global rabbitChannel
    rabbitChannel.basic_publish(exchange='',
                                routing_key='rtcConnectionData',
                                body=msg)


def setupRabbit():
    global rabbitChannel, rabbitConnection

    rabbitConnection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    rabbitChannel = rabbitConnection.channel()

    rabbitChannel.queue_declare(queue='rtcConnectionData')

    print(" [rabbit] I am ready!")


def channel_log(channel, t, message):
    print(" [rtc] channel(%s) %s %s" % (channel.label, t, message))


def rtcChannelSend(channel, message):
    channel_log(channel, ">", message)
    channel.send(message)


async def setupRtc():
    pc = RTCPeerConnection()

    channel = pc.createDataChannel("dataTest")
    channel_log(channel, "-", "created by local party")

    @channel.on("message")
    def on_message(message):
        channel_log(channel, "<", message)

    print(pc.localDescription)

    await pc.setLocalDescription(await pc.createOffer())
    print(" [rtc] generated offer: %r" % jsonableDesc(pc.localDescription).toJson())

    sendRabbitMsg(jsonableDesc(pc.localDescription).toJson())


if __name__ == '__main__':
    try:
        setupRabbit()

        asyncio.run(setupRtc())
    except KeyboardInterrupt:
        print('Interrupted')
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
