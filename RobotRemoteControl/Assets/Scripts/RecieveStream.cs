using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using LibVLCSharp;

public class RecieveStream : MonoBehaviour
{
    private LibVLC _libVlc;
    private MediaPlayer _mediaPlayer;
    private Texture2D tex = null;

    private const string sdp = "sdp://v=0\n" +
                               "o=- 0 0 IN IP4 127.0.0.1\n" +
                               "s=No Name\n" +
                               "c=IN IP4 192.168.57.131\n" +
                               "t=0 0\n" +
                               "a=tool:libavformat 58.45.100\n" +
                               "m=video 5004 RTP/AVP 96\n" +
                               "b=AS:2055\n" +
                               "a=rtpmap:96 H264/90000\n" +
                               "a=fmtp:96 packetization-mode=1; sprop-parameter-sets=Z01AKOygUBf8uAtQEBAUAAADAAQACvyAPGDGWA==,aO+G8g==; profile-level-id=4D4028\n";

    private const string sdp_screen = "sdp://v=0\n" +
                                      "o=- 0 0 IN IP4 127.0.0.1\n" +
                                      "s=No Name\n" +
                                      "c=IN IP4 192.168.57.131\n" +
                                      "t=0 0\n" +
                                      "a=tool:libavformat 58.45.100\n" +
                                      "m=video 5004 RTP/AVP 96\n" +
                                      "b=AS:200\n" +
                                      "a=rtpmap:96 MP4V-ES/90000\n" +
                                      "a=fmtp:96 profile-level-id=1";

    private void Awake()
    {
        Core.Initialize(Application.dataPath);

        _libVlc = new LibVLC("--no-osd", "--verbose=2");

        Application.SetStackTraceLogType(LogType.Log, StackTraceLogType.None);

        Play();
    }

    private void Play()
    {
        Debug.Log("Starting stream");

        if (_mediaPlayer == null)
        {
            _mediaPlayer = new MediaPlayer(_libVlc);
        }

        if (!_mediaPlayer.IsPlaying)
        {
            if (_mediaPlayer.Media == null)
            {
                _mediaPlayer.Media = new Media(_libVlc, sdp_screen, FromType.FromLocation);
            }

            _mediaPlayer.Play();
        }
    }

    private void OnDisable()
    {
        _mediaPlayer?.Stop();
        _mediaPlayer?.Dispose();
        _mediaPlayer = null;

        _libVlc?.Dispose();
        _libVlc = null;
    }

    void Update()
    {
        if (tex == null)
        {
            uint i_videoHeight = 0;
            uint i_videoWidth = 0;

            _mediaPlayer.Size(0, ref i_videoWidth, ref i_videoHeight);
            var texptr = _mediaPlayer.GetTexture(out bool updated);
            if (i_videoWidth != 0 && i_videoHeight != 0 && updated && texptr != IntPtr.Zero)
            {
                Debug.Log("Creating texture with height " + i_videoHeight + " and width " + i_videoWidth);
                tex = Texture2D.CreateExternalTexture((int) i_videoWidth,
                    (int) i_videoHeight,
                    TextureFormat.RGBA32,
                    false,
                    true,
                    texptr);
                GetComponent<Renderer>().material.mainTexture = tex;
            } 
        }
        else if (tex != null)
        {
            var texptr = _mediaPlayer.GetTexture(out bool updated);
            if (updated)
            {
                tex.UpdateExternalTexture(texptr);
            }
        }
    }
}