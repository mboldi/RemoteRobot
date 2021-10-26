#!/usr/bin/env python
import os
import sys
import pika

def main():
    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()

    # channel.queue_declare(queue='hello')

    # channel.basic_publish(exchange='',
    #                       routing_key='hello',
    #                       body='Hello World!')
    # print(" [x] Sent 'Hello World!'")

    print(' [*] Waiting for hellos. To exit press CTRL+C')

    def callback(ch, method, properties, body):
        print(" [x] %r" % body)

    channel.basic_consume(queue='hello', on_message_callback=callback, auto_ack=True)

    channel.start_consuming()

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