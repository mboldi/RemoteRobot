#!/usr/bin/env python

import pika

connection = pika.BlockingConnection(pika.ConnectionParameters('192.168.57.131'))
channel = connection.channel()

channel.queue_declare(queue='hello')

channel.basic_publish(exchange='',
                      routing_key='hello',
                      body='Hello Dori!')

print(" [x] Sent 'Hello World!'")


connection.close()