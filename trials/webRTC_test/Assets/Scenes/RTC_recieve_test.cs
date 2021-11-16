using System.Collections;
using System.Collections.Generic;
using Unity.WebRTC;
using UnityEngine;

public class RTC_recieve_test : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        WebRTC.Initialize();

        var localConn = new RTCPeerConnection();
        var sendChannel = localConn.CreateDataChannel("sendChannel");

        var op1 = localConn.CreateOffer();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
