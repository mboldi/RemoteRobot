using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Newtonsoft.Json;
using RabbitMQ.Client;
using RabbitMQ.Client.Events;
using UnityEngine;
using UnityEngine.UI;

public class ControlRobot : MonoBehaviour
{
    private ConnectionFactory factory;
    private IConnection connection;
    private IModel rabbitDataChannel;

    private const string moveChannelName = "move-robot";

    private void ConnectToRabbit(string serverUrl)
    {
        Debug.Log("trying to connect to rabbit");

        factory = new ConnectionFactory() {HostName = serverUrl};
        factory.ClientProperties.Add(new KeyValuePair<string, object>("ClientProvidedName", "UnityControlClient"));


        connection = factory.CreateConnection();
        if (connection == null) return;

        rabbitDataChannel = connection.CreateModel();
        if (rabbitDataChannel == null) return;

        rabbitDataChannel.QueueDeclare(queue: moveChannelName,
            durable: false,
            exclusive: false,
            autoDelete: false,
            arguments: null);

        var consumer = new EventingBasicConsumer(rabbitDataChannel);

        Debug.Log(" [rabbit] Connected to rabbit");
    }

    void Awake()
    {
        ConnectToRabbit("192.168.57.131");
    }

    private string vec3ToString(Vector3 vec)
    {
        return "(" + vec.x + ", " +
               vec.y + ", " +
               vec.z + ")";
    }

    private void sendMoveCommand(Vector3 vec)
    {
        var bodyString = vec3ToString(vec);
        var body = Encoding.UTF8.GetBytes(bodyString);

        rabbitDataChannel.BasicPublish(
            exchange: "",
            routingKey: moveChannelName,
            basicProperties: null,
            body: body);
        
        Debug.Log("Sent message on RabbitMQ: " + bodyString);
    }

    public void moveUpABit()
    {
        sendMoveCommand(new Vector3(0.0f, 0.0f, 0.1f));
    }

    public void moveDownABit()
    {
        sendMoveCommand(new Vector3(0.0f, 0.0f, -0.1f));
    }

    public void moveLeftABit()
    {
        sendMoveCommand(new Vector3(-0.1f, 0.0f, 0f));
    }

    public void moveRightABit()
    {
        sendMoveCommand(new Vector3(0.1f, 0.0f, 0f));
    }

}