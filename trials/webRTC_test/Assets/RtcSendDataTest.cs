using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;
using Unity.WebRTC;
using Unity.WebRTC.Samples;
using UnityEngine;
using UnityEngine.UIElements.Experimental;
using RabbitMQ.Client;
using RabbitMQ.Client.Events;

public class RtcSendDataTest : MonoBehaviour
{
    private RTCPeerConnection pc;
    private RTCDataChannel dataChannel, remoteDataChannel;
    private DelegateOnIceConnectionChange pcOnIceConnectionChange;
    private DelegateOnIceCandidate pcOnIceCandidate;
    private DelegateOnMessage onDataChannelMessage;
    private DelegateOnOpen onDataChannelOpen;
    private DelegateOnClose onDataChannelClose;
    private DelegateOnDataChannel onDataChannel;

    private ConnectionFactory factory;
    private IConnection connection;
    private IModel rabbitDataChannel;

    private string offerString =
        "{\"sdp\": \"v=0\r\no=- 3847174650 3847174650 IN IP4 0.0.0.0\r\ns=-\r\nt=0 0\r\na=group:BUNDLE 0\r\na=msid-semantic:WMS *\r\nm=application 40116 DTLS/SCTP 5000\r\nc=IN IP4 192.168.57.131\r\na=mid:0\r\na=sctpmap:5000 webrtc-datachannel 65535\r\na=max-message-size:65536\r\na=candidate:b61fb8e210cf0cead3dde2ba5ff99959 1 udp 2130706431 192.168.57.131 40116 typ host\r\na=candidate:580b5f32035da7f14a30ea7a8d826c67 1 udp 2130706431 172.17.0.1 46259 typ host\r\na=candidate:0ad8db753210ce9554cf251a9e51e792 1 udp 1694498815 84.0.59.220 63562 typ srflx raddr 172.17.0.1 rport 46259\r\na=candidate:a6117bcc81c73435716a6036dad1289d 1 udp 1694498815 84.0.59.220 63561 typ srflx raddr 192.168.57.131 rport 40116\r\na=end-of-candidates\r\na=ice-ufrag:G6rA\r\na=ice-pwd:uAavsuMvOK6XFmXsTJL2Ju\r\na=fingerprint:sha-256 33:E6:14:7B:32:1D:C5:EF:48:FD:88:5A:05:DB:54:65:8E:2C:E9:1A:A7:94:55:BA:96:1D:23:8D:85:B2:67:86\r\na=setup:actpass\r\n\", \"type\": \"offer\"}";

    private void Awake()
    {
        WebRTC.Initialize(EncoderType.Software, false);

        StartCoroutine(Call());
    }


    private void ConnectToRabbit(string serverUrl)
    {
        Debug.Log("trying to connect to rabbit");

        factory = new ConnectionFactory() {HostName = serverUrl};

        using (connection = factory.CreateConnection())
        using (rabbitDataChannel = connection.CreateModel())
        {
            rabbitDataChannel.QueueDeclare(queue: "rtcConnectionData",
                durable: false,
                exclusive: false,
                autoDelete: false,
                arguments: null);
            
            var consumer = new EventingBasicConsumer(rabbitDataChannel);
            consumer.Received += (model, ea) =>
            {
                var body = ea.Body.ToArray();
                var message = Encoding.UTF8.GetString(body);
                Debug.Log(" [rabbit] Received " + message);
            };
            
            rabbitDataChannel.BasicConsume("rtcConnectionData", true, consumer);
            
            Debug.Log(" [rabbit] Connected to rabbit and listening to rtcConnectionData channel");
        }
    }

    private void Start()
    {
        ConnectToRabbit("192.168.57.131");

        pcOnIceConnectionChange = state => { OnIceConnectionChange(pc, state); };
        pcOnIceCandidate = candidate => { OnIceCandidate(pc, candidate); };
        onDataChannel = channel =>
        {
            dataChannel = channel;
            dataChannel.OnMessage = onDataChannelMessage;
            remoteDataChannel = channel;
            remoteDataChannel.OnMessage = onDataChannelMessage;
        };
        onDataChannelMessage = bytes =>
        {
            Debug.Log($"Incoming message: {System.Text.Encoding.UTF8.GetString(bytes)}");
        };
        onDataChannelOpen = () => { Debug.Log("Data channel opened"); };
        onDataChannelClose = () => { Debug.Log("Data channel closed"); };
    }

    IEnumerator Call()
    {
        var configuration = GetSelectedSdpSemantics();
        pc = new RTCPeerConnection(ref configuration);

        pc.OnIceCandidate = pcOnIceCandidate;
        pc.OnIceConnectionChange = pcOnIceConnectionChange;
        pc.OnDataChannel = onDataChannel;

        var conf = new RTCDataChannelInit();
        dataChannel = pc.CreateDataChannel("testData", conf);
        dataChannel.OnOpen = onDataChannelOpen;
        dataChannel.OnMessage = onDataChannelMessage;
        dataChannel.OnClose = onDataChannelClose;

        var alma = JsonConvert.DeserializeObject<RTCSessionDescription>(offerString);
        Debug.Log($"decoded offer: \n{alma.sdp}");

        var op = pc.CreateOffer();
        yield return op;

        if (!op.IsError)
        {
            Debug.Log(JsonConvert.SerializeObject(op.Desc, new StringEnumConverter()));
        }
        else
        {
            Debug.Log($"Error: {op.Error}");
        }

        /*
        var op = pc.SetRemoteDescription(ref alma);
        yield return op;

        if (!op.IsError)
        {
            Debug.Log($"pc SetRemoteDescription complete");
            
            var op2 = pc.CreateAnswer();
            yield return op2;

            if (!op2.IsError)
            {
                Debug.Log(JsonConvert.SerializeObject(op2.Desc));
            }
            else
            {
                Debug.Log($"Error: {op2.Error}");
            }
            
            dataChannel.Send("alma");
        }
        else
        {
            Debug.Log($"Error: {op.Error}");
        }*/
    }


    RTCConfiguration GetSelectedSdpSemantics()
    {
        RTCConfiguration config = default;
        config.iceServers = new RTCIceServer[]
        {
            new RTCIceServer {urls = new string[] {"stun:stun.l.google.com:19302"}}
        };

        return config;
    }

    void OnIceConnectionChange(RTCPeerConnection pc, RTCIceConnectionState state)
    {
        switch (state)
        {
            case RTCIceConnectionState.New:
                Debug.Log($"pc IceConnectionState: New");
                break;
            case RTCIceConnectionState.Checking:
                Debug.Log($"pc IceConnectionState: Checking");
                break;
            case RTCIceConnectionState.Closed:
                Debug.Log($"pc IceConnectionState: Closed");
                break;
            case RTCIceConnectionState.Completed:
                Debug.Log($"pc IceConnectionState: Completed");
                break;
            case RTCIceConnectionState.Connected:
                Debug.Log($"pc IceConnectionState: Connected");
                break;
            case RTCIceConnectionState.Disconnected:
                Debug.Log($"pc IceConnectionState: Disconnected");
                break;
            case RTCIceConnectionState.Failed:
                Debug.Log($"pc IceConnectionState: Failed");
                break;
            case RTCIceConnectionState.Max:
                Debug.Log($"pc IceConnectionState: Max");
                break;
            default:
                break;
        }
    }

    void Pc1OnIceConnectinChange(RTCIceConnectionState state)
    {
        OnIceConnectionChange(pc, state);
    }

    void OnIceCandidate(RTCPeerConnection pc, RTCIceCandidate candidate)
    {
        pc.AddIceCandidate(candidate);
        Debug.Log($"pc ICE candidate:\n {candidate.Candidate}");
    }
}