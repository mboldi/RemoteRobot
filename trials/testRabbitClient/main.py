#!/usr/bin/env python

import pika

connection = pika.BlockingConnection(pika.ConnectionParameters('192.168.57.131'))
channel = connection.channel()

channel.queue_declare(queue='set-mode')

channel.basic_publish(exchange='',
                      routing_key='move-robot',
                      body='(0.2,0,0)')



connection.close()