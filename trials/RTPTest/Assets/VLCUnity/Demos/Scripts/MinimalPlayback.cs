using UnityEngine;
using System;
using LibVLCSharp;

/// this class serves as an example on how to configure playback in Unity with VLC for Unity using LibVLCSharp.
/// for libvlcsharp usage documentation, please visit https://code.videolan.org/videolan/LibVLCSharp/-/blob/master/docs/home.md
public class MinimalPlayback : MonoBehaviour
{
    LibVLC _libVLC;
    MediaPlayer _mediaPlayer;
    const int seekTimeDelta = 5000;
    Texture2D tex = null;
    bool playing;

    void Awake()
    {
        Core.Initialize(Application.dataPath);

        _libVLC = new LibVLC("--no-osd", "--verbose=2");

        Application.SetStackTraceLogType(LogType.Log, StackTraceLogType.None);
        //_libVLC.Log += (s, e) => UnityEngine.Debug.Log(e.FormattedLog); // enable this for logs in the editor

        PlayPause();
    }

    public void SeekForward()
    {
        Debug.Log("[VLC] Seeking forward !");
        _mediaPlayer.SetTime(_mediaPlayer.Time + seekTimeDelta);
    }

    public void SeekBackward()
    {
        Debug.Log("[VLC] Seeking backward !");
        _mediaPlayer.SetTime(_mediaPlayer.Time - seekTimeDelta);
    }

    void OnDisable()
    {
        _mediaPlayer?.Stop();
        _mediaPlayer?.Dispose();
        _mediaPlayer = null;

        _libVLC?.Dispose();
        _libVLC = null;
    }

    public void PlayPause()
    {
        Debug.Log("[VLC] Toggling Play Pause !");
        if (_mediaPlayer == null)
        {
            _mediaPlayer = new MediaPlayer(_libVLC);
        }

        if (_mediaPlayer.IsPlaying)
        {
            _mediaPlayer.Pause();
        }
        else
        {
            playing = true;

            if (_mediaPlayer.Media == null)
            {
                // playing remote media
                _mediaPlayer.Media = new Media(_libVLC, "sdp://v=0\n" +
                                                        "o=- 0 0 IN IP4 127.0.0.1\n" +
                                                        "s=No Name\n" +
                                                        "c=IN IP4 192.168.57.131\n" +
                                                        "t=0 0\n" +
                                                        "a=tool:libavformat 58.45.100\n" +
                                                        "m=video 5004 RTP/AVP 96\n" +
                                                        "b=AS:2055\n" +
                                                        "a=rtpmap:96 H264/90000\n" +
                                                        "a=fmtp:96 packetization-mode=1; sprop-parameter-sets=Z01AKOygUBf8uAtQEBAUAAADAAQACvyAPGDGWA==,aO+G8g==; profile-level-id=4D4028\n",
                    FromType.FromLocation);
            }

            _mediaPlayer.Play();
        }
    }

    public void Stop()
    {
        Debug.Log("[VLC] Stopping Player !");

        playing = false;
        _mediaPlayer?.Stop();

        // there is no need to dispose every time you stop, but you should do so when you're done using the mediaplayer and this is how:
        // _mediaPlayer?.Dispose(); 
        // _mediaPlayer = null;
        GetComponent<Renderer>().material.mainTexture = null;
        tex = null;
    }

    void Update()
    {
        if (!playing) return;

        if (tex == null)
        {
            // If received size is not null, it and scale the texture
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