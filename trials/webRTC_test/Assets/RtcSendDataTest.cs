using System;
using System.Collections;
using System.Collections.Generic;
using Newtonsoft.Json;
using Unity.WebRTC;
using Unity.WebRTC.Samples;
using UnityEngine;

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

    private string offerString =
        "{\"sdp\": \"v=0\\r\\no=- 3846079074 3846079074 IN IP4 0.0.0.0\\r\\ns=-\\r\\nt=0 0\\r\\na=group:BUNDLE 0\\r\\na=msid-semantic:WMS *\\r\\nm=application 46684 DTLS/SCTP 5000\\r\\nc=IN IP4 192.168.57.131\\r\\na=mid:0\\r\\na=sctpmap:5000 webrtc-datachannel 65535\\r\\na=max-message-size:65536\\r\\na=candidate:b61fb8e210cf0cead3dde2ba5ff99959 1 udp 2130706431 192.168.57.131 46684 typ host\\r\\na=candidate:580b5f32035da7f14a30ea7a8d826c67 1 udp 2130706431 172.17.0.1 46929 typ host\\r\\na=candidate:0ad8db753210ce9554cf251a9e51e792 1 udp 1694498815 84.2.69.6 59349 typ srflx raddr 172.17.0.1 rport 46929\\r\\na=candidate:a6117bcc81c73435716a6036dad1289d 1 udp 1694498815 84.2.69.6 59348 typ srflx raddr 192.168.57.131 rport 46684\\r\\na=end-of-candidates\\r\\na=ice-ufrag:Fbsg\\r\\na=ice-pwd:GzNUTZ7MFOlGuB3aWC1O7S\\r\\na=fingerprint:sha-256 EC:17:2C:53:CD:FB:29:D2:35:9D:B1:6D:87:9D:76:58:08:3B:BF:5C:D3:0F:CD:78:34:9D:50:5A:07:23:97:4F\\r\\na=setup:actpass\\r\\n\", \"type\": \"offer\"}";
    
    private void Awake()
    {
        Debug.Log("I am awake!");
        WebRTC.Initialize(EncoderType.Software, false);

        StartCoroutine(Call());
    }

    IEnumerator Call()
    {
        var configuration = GetSelectedSdpSemantics();

        var op = JsonConvert.DeserializeObject<RTCSessionDescriptionAsyncOperation>(offerString);
        Debug.Log($"decoded offer: \n{op.Desc.sdp}");
    }
    
    IEnumerator OnCreateOfferSuccess(RTCSessionDescription desc)
    {
        Debug.Log($"Offer from pc1\n{desc.sdp}");

        Debug.Log("pc setRemoteDescription start");
        var op2 = pc.SetRemoteDescription(ref desc);
        yield return op2;
        if (!op2.IsError)
        {
            OnSetRemoteSuccess(pc2);
        }
        else
        {
            var error = op2.Error;
            OnSetSessionDescriptionError(ref error);
        }
        Debug.Log("pc2 createAnswer start");
        // Since the 'remote' side has no media stream we need
        // to pass in the right constraints in order for it to
        // accept the incoming offer of audio and video.

        var op3 = pc2.CreateAnswer();
        yield return op3;
        if (!op3.IsError)
        {
            yield return OnCreateAnswerSuccess(op3.Desc);
        }
        else
        {
            OnCreateSessionDescriptionError(op3.Error);
        }
    }
    
    
    RTCConfiguration GetSelectedSdpSemantics()
    {
        RTCConfiguration config = default;
        config.iceServers = new RTCIceServer[]
        {
            new RTCIceServer { urls = new string[] { "stun:stun.l.google.com:19302" } }
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