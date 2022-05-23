#!/usr/bin/python3

import argparse
import asyncio
import logging
import math
import json

import cv2
import numpy
from aiortc.sdp import candidate_from_sdp
from av import VideoFrame

from aiortc import (
    RTCIceCandidate,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder
from aiortc.contrib.signaling import BYE, add_signaling_arguments, create_signaling

import pika

webrtcSignalingQueue = 'webrtc'
webrtcSignalingRespQueue = 'webrtc-resp'

channel: any
stop: False


def object_from_string(message_str):
    message = json.loads(message_str)
    if message["type"] in ["answer", "offer"]:
        return RTCSessionDescription(**message)
    elif message["type"] == "candidate" and message["candidate"]:
        candidate = candidate_from_sdp(message["candidate"].split(":", 1)[1])
        candidate.sdpMid = message["id"]
        candidate.sdpMLineIndex = message["label"]
        return candidate
    elif message["type"] == "bye":
        return BYE


def setupSignaling(respCallback):
    connection = pika.BlockingConnection(pika.ConnectionParameters('192.168.2.101'))
    channel = connection.channel()

    try:
        channel.queue_declare(queue=webrtcSignalingQueue, durable=True)
    except:
        print("WebRTC signaling queue already exists")

    try:
        channel.queue_declare(queue=webrtcSignalingRespQueue, durable=True)
    except:
        print("WebRTC signaling response queue already exists")

    channel.basic_consume(queue=webrtcSignalingRespQueue, on_message_callback=respCallback, auto_ack=True)

    return channel


async def sendSignaling(data):
    global channel

    print("Sending signaling data: " + data)
    channel.basic_publish(exchange='', routing_key=webrtcSignalingQueue, body=data)


async def run(pc, player):
    global stop

    def add_tracks():
        if player and player.audio:
            pc.addTrack(player.audio)

        if player and player.video:
            pc.addTrack(player.video)

    @pc.on("track")
    def on_track(track):
        print("Receiving %s" % track.kind)

    def on_resp_msg(ch, method, properties, body):
        global channel
        global stop

        body = str(body).replace('\\r\\n', ' ')[2:-1]

        print('Signaling response received: ', body)

        typed_obj = object_from_string(body)

        if isinstance(typed_obj, RTCSessionDescription):
            pc.setRemoteDescription(typed_obj)

            if typed_obj.type == "offer":
                # send answer
                add_tracks()
                pc.setLocalDescription(pc.createAnswer())
                sendSignaling(pc.localDescription.sdp)
        elif isinstance(typed_obj, RTCIceCandidate):
            pc.addIceCandidate(typed_obj)
        elif typed_obj is BYE:
            print("Exiting")
            stop = True

    # connect signaling
    global channel
    channel = setupSignaling(on_resp_msg)

    # send offer
    add_tracks()
    await pc.setLocalDescription(await pc.createOffer())
    await sendSignaling(pc.localDescription.sdp)

    # consume signaling
    channel.start_consuming()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Video stream from the command line")
    parser.add_argument("--play-from", help="Read the media from a file and sent it.")

    args = parser.parse_args()

    # create signaling and peer connection
    pc = RTCPeerConnection()

    # create media source
    if args.play_from:
        player = MediaPlayer(args.play_from)
    else:
        player = None

    # run event loop
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(
            run(
                pc=pc,
                player=player
            )
        )
    except KeyboardInterrupt:
        pass
    finally:
        # cleanup
        loop.run_until_complete(pc.close())
